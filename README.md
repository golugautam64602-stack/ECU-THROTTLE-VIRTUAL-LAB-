# ECU-THROTTLE-VIRTUAL-LAB-
A virtual Hardware-in-the-Loop (HIL) emulation project for simulating ECU throttle responses, accelerator pedal inputs, throttle control, engine RPM, and fault detection
new experince
# HIL Virtual Lab — ECU Electronic Throttle Control

Browser-based Hardware-in-the-Loop emulator for Electronic Throttle Control (ETC).

## How to Run

1. Open `index.html` in any modern browser (Chrome / Edge / Firefox).
2. No server or build step required — pure HTML + CSS + JS.

Or serve locally:

```bash
cd hil_virtual_lab
python -m http.server 8080
# then open http://localhost:8080
```

## Features

- **Accelerator Pedal** slider + step buttons (Idle 0% / Half 50% / WOT 100%)
- **Live PID gains** (Kp, Ki, Kd) with anti-windup
- **Return-spring stiffness** multiplier
- **Fault injection**
  - TPS2 Short-to-GND → DTC P0222 → Limp-Home (14°)
  - APP1/APP2 Mismatch → DTC P2138 → Limp-Home
- **2D Throttle body** animation
- **Real-time oscilloscope** (Target θ, Actual θ, PWM)
- **Experiment log** + one-click **CSV export**
- Discrete PID at ≈60 Hz (Δt = 0.016 s)
- 2nd-order electromechanical plant model

## Default PID (report baseline)

| Gain | Value |
|------|-------|
| Kp   | 2.40  |
| Ki   | 0.85  |
| Kd   | 0.12  |

## Lab Procedure (matches report)

1. Launch lab → verify Kp=2.4, Ki=0.85, Kd=0.12 and DTC P0000.
2. Apply Idle / Half / WOT steps and watch scope.
3. When steady → **Add Reading**.
4. Enable fault switches → confirm Limp-Home.
5. **Export CSV**.

## Files

| File        | Purpose              |
|-------------|----------------------|
| index.html  | Structure & UI layout |
| style.css   | Dark engineering theme |
| script.js   | Plant + PID + faults + drawing |
| README.md   | This file            |


