/**
 * HIL Virtual Lab — ECU Electronic Throttle Control Emulator
 * Discrete PID + 2nd-order plant + dual-sensor plausibility + Limp-Home
 * Sample time Δt ≈ 0.016 s (≈60 Hz)
 */

(() => {
  "use strict";

  // ---------- Constants & Plant Parameters ----------
  const DT = 1 / 60;                  // 16.67 ms
  const THETA0 = 10;                  // limp-home mechanical rest angle (deg)
  const THETA_MAX = 90;
  const THETA_MIN = 0;
  const J = 0.018;                    // effective inertia
  const B = 0.12;                     // viscous damping
  const KS_BASE = 1.45;               // base spring stiffness
  const KT = 0.85;                    // torque constant (scaled)
  const LIMP_ANGLE = 14.0;            // Limp-Home target

  // Sensor ranges
  const APP1_MIN = 0.75, APP1_MAX = 4.50;
  const TPS1_MIN = 0.50, TPS1_MAX = 4.50;

  // ---------- State ----------
  const state = {
    pedal: 0,                         // 0–100 %
    kp: 2.40,
    ki: 0.85,
    kd: 0.12,
    springMult: 1.0,
    faultTps2: false,
    faultApp: false,
    // Plant
    theta: 0,                         // actual angle deg
    omega: 0,                         // rad/s (internal)
    // PID
    integral: 0,
    prevError: 0,
    // Mode
    limpHome: false,
    dtc: "P0000",
    // Scope history (last N samples)
    history: { t: [], target: [], actual: [], pwm: [] },
    maxHistory: 300,
    // Log
    log: [],
    runCount: 0,
    // PWM output
    uPwm: 0
  };

  // ---------- DOM ----------
  const $ = (id) => document.getElementById(id);
  const pedalSlider = $("pedalSlider");
  const pedalValue = $("pedalValue");
  const kpInput = $("kp");
  const kiInput = $("ki");
  const kdInput = $("kd");
  const springSlider = $("springSlider");
  const springValue = $("springValue");
  const faultTps2 = $("faultTps2");
  const faultApp = $("faultApp");
  const throttleCanvas = $("throttleCanvas");
  const scopeCanvas = $("scopeCanvas");
  const tCtx = throttleCanvas.getContext("2d");
  const sCtx = scopeCanvas.getContext("2d");

  // ---------- Helpers ----------
  function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  function deg2rad(d) { return d * Math.PI / 180; }
  function rad2deg(r) { return r * 180 / Math.PI; }

  function pedalToTarget(pedalPct) {
    // Linear map 0–100% → 0–90°
    return (pedalPct / 100) * THETA_MAX;
  }

  function voltageAPP1(pedalPct) {
    return APP1_MIN + (pedalPct / 100) * (APP1_MAX - APP1_MIN);
  }

  function voltageTPS1(theta) {
    return TPS1_MIN + (theta / THETA_MAX) * (TPS1_MAX - TPS1_MIN);
  }

  // ---------- Plausibility & Fault Logic ----------
  function checkPlausibility() {
    let dtc = "P0000";
    let limp = false;

    // APP voltages
    let vApp1 = voltageAPP1(state.pedal);
    let vApp2 = 0.5 * vApp1;
    if (state.faultApp) {
      // Force correlation fault
      vApp2 = vApp1 * 0.2; // severe mismatch
    }
    const appErr = Math.abs(vApp1 - 2 * vApp2);
    if (appErr >= 0.35) {
      dtc = "P2138";
      limp = true;
    }

    // TPS voltages
    let vTps1 = voltageTPS1(state.theta);
    let vTps2 = 5.0 - vTps1; // complementary
    if (state.faultTps2) {
      vTps2 = 0.0; // short-to-ground
    }
    const tpsSum = vTps1 + vTps2;
    if (Math.abs(tpsSum - 5.0) >= 0.35) {
      dtc = "P0222";
      limp = true;
    }

    state.dtc = dtc;
    state.limpHome = limp;
    return { vApp1, vApp2, vTps1, vTps2 };
  }

  // ---------- Discrete PID with Anti-Windup ----------
  function computePID(thetaRef, thetaAct) {
    const e = thetaRef - thetaAct;
    // Integral with clamping
    state.integral += e * DT;
    // Anti-windup: clamp integral contribution
    const iTerm = state.ki * state.integral;
    const maxI = 80;
    if (Math.abs(iTerm) > maxI) {
      state.integral = (Math.sign(iTerm) * maxI) / Math.max(state.ki, 1e-6);
    }

    const dTerm = state.kd * (e - state.prevError) / DT;
    state.prevError = e;

    let u = state.kp * e + state.ki * state.integral + dTerm;
    u = clamp(u, -100, 100); // % duty
    return u;
  }

  // ---------- Plant Integration (semi-implicit Euler) ----------
  function stepPlant(uPwm) {
    // Motor torque ≈ (u/100) * KT * scale
    const motorTorque = (uPwm / 100) * KT * 180; // scaled to match spring units
    const Ks = KS_BASE * state.springMult;
    const springTorque = Ks * (state.theta - THETA0);

    // τ = J·α + B·ω + Ks·(θ−θ0)
    // α = (motorTorque - B·ω - springTorque) / J
    const omegaRad = deg2rad(state.omega); // store omega in deg/s for simplicity
    // Re-define: state.omega in deg/s
    const alpha = (motorTorque - B * state.omega - springTorque) / J;

    state.omega += alpha * DT;
    state.theta += state.omega * DT;

    // Mechanical end-stops
    if (state.theta < THETA_MIN) {
      state.theta = THETA_MIN;
      state.omega = 0;
    }
    if (state.theta > THETA_MAX) {
      state.theta = THETA_MAX;
      state.omega = 0;
    }
  }

  // ---------- Main Simulation Step ----------
  function simulationStep() {
    // Read live gains
    state.kp = parseFloat(kpInput.value) || 0;
    state.ki = parseFloat(kiInput.value) || 0;
    state.kd = parseFloat(kdInput.value) || 0;

    const sensors = checkPlausibility();

    let thetaRef;
    if (state.limpHome) {
      thetaRef = LIMP_ANGLE;
      // Freeze integral in limp
      state.integral *= 0.95;
    } else {
      thetaRef = pedalToTarget(state.pedal);
    }

    const u = computePID(thetaRef, state.theta);
    state.uPwm = u;
    stepPlant(u);

    // History for scope
    const tNow = (state.history.t.length ? state.history.t[state.history.t.length - 1] : 0) + DT;
    state.history.t.push(tNow);
    state.history.target.push(thetaRef);
    state.history.actual.push(state.theta);
    state.history.pwm.push(u);
    if (state.history.t.length > state.maxHistory) {
      state.history.t.shift();
      state.history.target.shift();
      state.history.actual.shift();
      state.history.pwm.shift();
    }

    // UI telemetry
    $("telTarget").textContent = thetaRef.toFixed(1) + "°";
    $("telActual").textContent = state.theta.toFixed(1) + "°";
    $("telError").textContent = (thetaRef - state.theta).toFixed(2) + "°";
    $("telPwm").textContent = u.toFixed(1) + "%";
    $("telApp").textContent = `${sensors.vApp1.toFixed(2)} / ${sensors.vApp2.toFixed(2)} V`;
    $("telTps").textContent = `${sensors.vTps1.toFixed(2)} / ${sensors.vTps2.toFixed(2)} V`;
    $("angleLabel").textContent = state.theta.toFixed(1) + "°";

    // Status badge
    const badge = $("ecuStatus");
    const modeLabel = $("modeLabel");
    if (state.limpHome) {
      badge.classList.add("fault");
      $("statusText").textContent = "LIMP-HOME";
      $("dtcText").textContent = "DTC: " + state.dtc;
      modeLabel.textContent = "LIMP-HOME";
      modeLabel.className = "mode-limp";
    } else {
      badge.classList.remove("fault");
      $("statusText").textContent = "NORMAL";
      $("dtcText").textContent = "DTC: P0000";
      modeLabel.textContent = "NORMAL";
      modeLabel.className = "mode-normal";
    }

    drawThrottle();
    drawScope();
  }

  // ---------- Throttle Canvas Drawing ----------
  function drawThrottle() {
    const w = throttleCanvas.width;
    const h = throttleCanvas.height;
    tCtx.clearRect(0, 0, w, h);

    // Bore
    const cx = w / 2;
    const cy = h / 2 - 10;
    const r = Math.min(w, h) * 0.38;

    tCtx.beginPath();
    tCtx.arc(cx, cy, r + 8, 0, Math.PI * 2);
    tCtx.strokeStyle = "#334155";
    tCtx.lineWidth = 10;
    tCtx.stroke();

    tCtx.beginPath();
    tCtx.arc(cx, cy, r, 0, Math.PI * 2);
    tCtx.fillStyle = "#0f172a";
    tCtx.fill();
    tCtx.strokeStyle = "#475569";
    tCtx.lineWidth = 2;
    tCtx.stroke();

    // Airflow arrows (intensity based on opening)
    const openFrac = state.theta / THETA_MAX;
    if (openFrac > 0.05) {
      tCtx.strokeStyle = `rgba(56, 189, 248, ${0.3 + openFrac * 0.5})`;
      tCtx.lineWidth = 2;
      for (let i = -2; i <= 2; i++) {
        const y = cy + i * 18;
        tCtx.beginPath();
        tCtx.moveTo(cx - r - 30, y);
        tCtx.lineTo(cx - r - 8, y);
        tCtx.stroke();
        // arrow head
        tCtx.beginPath();
        tCtx.moveTo(cx - r - 8, y);
        tCtx.lineTo(cx - r - 14, y - 4);
        tCtx.moveTo(cx - r - 8, y);
        tCtx.lineTo(cx - r - 14, y + 4);
        tCtx.stroke();
      }
    }

    // Butterfly plate
    const angleRad = deg2rad(state.theta);
    // Plate is perpendicular to airflow when closed (0° → vertical in this drawing)
    // When θ=0 (closed) plate blocks (horizontal line across), θ=90 fully open (vertical)
    const plateAngle = deg2rad(90 - state.theta); // visual mapping
    const len = r * 0.92;
    const dx = Math.cos(plateAngle) * len;
    const dy = Math.sin(plateAngle) * len;

    tCtx.beginPath();
    tCtx.moveTo(cx - dx, cy - dy);
    tCtx.lineTo(cx + dx, cy + dy);
    tCtx.strokeStyle = "#fb923c";
    tCtx.lineWidth = 10;
    tCtx.lineCap = "round";
    tCtx.stroke();

    // Shaft
    tCtx.beginPath();
    tCtx.arc(cx, cy, 7, 0, Math.PI * 2);
    tCtx.fillStyle = "#f97316";
    tCtx.fill();

    // Labels
    tCtx.fillStyle = "#94a3b8";
    tCtx.font = "12px Segoe UI";
    tCtx.fillText("Intake →", 12, cy - r - 16);
    tCtx.fillText("0° closed", cx + r + 12, cy + 4);
  }

  // ---------- Oscilloscope Drawing ----------
  function drawScope() {
    const w = scopeCanvas.width;
    const h = scopeCanvas.height;
    sCtx.clearRect(0, 0, w, h);

    // Grid
    sCtx.strokeStyle = "#1e293b";
    sCtx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = (h / 4) * i;
      sCtx.beginPath();
      sCtx.moveTo(0, y);
      sCtx.lineTo(w, y);
      sCtx.stroke();
    }
    for (let i = 0; i <= 6; i++) {
      const x = (w / 6) * i;
      sCtx.beginPath();
      sCtx.moveTo(x, 0);
      sCtx.lineTo(x, h);
      sCtx.stroke();
    }

    const n = state.history.t.length;
    if (n < 2) return;

    const yScaleAngle = (h - 20) / 100; // 0–100°
    const yScalePwm = (h - 20) / 200;   // −100..+100 → mapped

    function plot(data, color, scale, offsetY) {
      sCtx.beginPath();
      sCtx.strokeStyle = color;
      sCtx.lineWidth = 2;
      for (let i = 0; i < n; i++) {
        const x = (i / (state.maxHistory - 1)) * w;
        const y = h - 10 - (data[i] + offsetY) * scale;
        if (i === 0) sCtx.moveTo(x, y);
        else sCtx.lineTo(x, y);
      }
      sCtx.stroke();
    }

    plot(state.history.target, "#38bdf8", yScaleAngle, 0);
    plot(state.history.actual, "#4ade80", yScaleAngle, 0);
    // PWM centered
    plot(state.history.pwm, "#fb923c", yScalePwm, 100);
  }

  // ---------- Logging ----------
  function addReading() {
    state.runCount += 1;
    const thetaRef = state.limpHome ? LIMP_ANGLE : pedalToTarget(state.pedal);
    const err = Math.abs(thetaRef - state.theta);
    let condition = "Custom";
    if (state.limpHome) {
      condition = state.dtc === "P0222" ? "TPS2 Short-to-GND Fault" : "APP1/APP2 Sync Fault";
    } else if (state.pedal === 0) condition = "Idle Baseline";
    else if (state.pedal === 50) condition = "Mid-Throttle Step";
    else if (state.pedal === 100) condition = "Wide-Open Throttle";
    else if (state.pedal === 15) condition = "Part-Load Cruise";

    const entry = {
      run: state.runCount,
      condition,
      pedal: state.pedal.toFixed(1),
      target: thetaRef.toFixed(1),
      actual: state.theta.toFixed(1),
      error: err.toFixed(2),
      dtc: state.dtc,
      mode: state.limpHome ? "LIMP-HOME" : "NORMAL"
    };
    state.log.push(entry);
    renderLog();
  }

  function renderLog() {
    const tbody = $("logBody");
    tbody.innerHTML = "";
    state.log.forEach((row) => {
      const tr = document.createElement("tr");
      tr.innerHTML = `
        <td>${row.run}</td>
        <td>${row.condition}</td>
        <td>${row.pedal}</td>
        <td>${row.target}</td>
        <td>${row.actual}</td>
        <td>${row.error}</td>
        <td class="${row.dtc === "P0000" ? "dtc-ok" : "dtc-fault"}">${row.dtc}</td>
        <td class="${row.mode === "NORMAL" ? "mode-normal" : "mode-limp"}">${row.mode}</td>
      `;
      tbody.appendChild(tr);
    });
  }

  function exportCSV() {
    if (!state.log.length) {
      alert("No readings to export. Click “Add Reading” first.");
      return;
    }
    const header = "Run,Test Condition,Pedal %,Target °,Actual °,Error °,DTC,ECU Mode\n";
    const rows = state.log.map(r =>
      `${r.run},"${r.condition}",${r.pedal},${r.target},${r.actual},${r.error},${r.dtc},${r.mode}`
    ).join("\n");
    const blob = new Blob([header + rows], { type: "text/csv" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `HIL_Throttle_Log_${new Date().toISOString().slice(0, 19).replace(/:/g, "-")}.csv`;
    a.click();
    URL.revokeObjectURL(url);
  }

  function clearLog() {
    state.log = [];
    state.runCount = 0;
    renderLog();
  }

  function resetHIL() {
    state.theta = 0;
    state.omega = 0;
    state.integral = 0;
    state.prevError = 0;
    state.uPwm = 0;
    state.limpHome = false;
    state.dtc = "P0000";
    faultTps2.checked = false;
    faultApp.checked = false;
    state.faultTps2 = false;
    state.faultApp = false;
    state.history = { t: [], target: [], actual: [], pwm: [] };
    pedalSlider.value = 0;
    state.pedal = 0;
    pedalValue.textContent = "0%";
    document.querySelectorAll(".btn-step").forEach(b => b.classList.remove("active"));
  }

  // ---------- Event Listeners ----------
  pedalSlider.addEventListener("input", () => {
    state.pedal = parseFloat(pedalSlider.value);
    pedalValue.textContent = state.pedal.toFixed(1) + "%";
    document.querySelectorAll(".btn-step").forEach(b => {
      b.classList.toggle("active", parseFloat(b.dataset.pct) === state.pedal);
    });
  });

  document.querySelectorAll(".btn-step").forEach(btn => {
    btn.addEventListener("click", () => {
      const pct = parseFloat(btn.dataset.pct);
      state.pedal = pct;
      pedalSlider.value = pct;
      pedalValue.textContent = pct + "%";
      document.querySelectorAll(".btn-step").forEach(b => b.classList.remove("active"));
      btn.classList.add("active");
    });
  });

  springSlider.addEventListener("input", () => {
    state.springMult = parseFloat(springSlider.value);
    springValue.textContent = state.springMult.toFixed(1) + "×";
  });

  faultTps2.addEventListener("change", () => {
    state.faultTps2 = faultTps2.checked;
  });
  faultApp.addEventListener("change", () => {
    state.faultApp = faultApp.checked;
  });

  $("btnAddReading").addEventListener("click", addReading);
  $("btnExport").addEventListener("click", exportCSV);
  $("btnClearLog").addEventListener("click", clearLog);
  $("btnReset").addEventListener("click", resetHIL);

  // ---------- Main Loop ----------
  function loop() {
    simulationStep();
    requestAnimationFrame(loop);
  }

  // Kick off
  resetHIL();
  requestAnimationFrame(loop);
})();
    
