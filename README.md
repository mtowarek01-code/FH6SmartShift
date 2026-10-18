# 🏎️ FH6SmartShift (v1.1.0) — Virtual Dyno, Dragy Timer & Shift Telemetry System for Forza Horizon 6 & Forza Motorsport

[![Platform](https://img.shields.io/badge/Platform-SimHub-blue.svg)](https://www.simhubdash.com/)
[![Games](https://img.shields.io/badge/Games-Forza%20Horizon%206%20%7C%20Forza%20Motorsport-orange.svg)](#)
[![Framework](https://img.shields.io/badge/.NET%20Framework-4.8-purple.svg)](https://dotnet.microsoft.com/)
[![Language](https://img.shields.io/badge/C%23-7.3-success.svg)](https://docs.microsoft.com/en-us/dotnet/csharp/)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-brightgreen.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Buy Me A Coffee](https://img.shields.io/badge/Buy%20Me%20a%20Coffee-ffdd00?logo=buy-me-a-coffee&logoColor=black)](https://buymeacoffee.com/mtowarek01d)

**Author:** mtowarek01  
**Version:** v1.1.0 (Dual-Game FH6 & Forza Motorsport Engine, Virtual Dyno, Dragy Timer, F1 Shift Beep, Tuning Advisor & WPF UI)

---

## 📖 About the Project

**FH6SmartShift** is an advanced, real-time (60 Hz) C# telemetry plugin for **SimHub** and a dedicated racing dashboard suite engineered for the **Forza Horizon 6** and **Forza Motorsport** physics engines.

Unlike basic scripts that only average rev-limiter bounce RPM, **FH6SmartShift** constructs a live **401-band Virtual Dyno** (`50 RPM` resolution up to `20,000 RPM`). By capturing raw UDP telemetry from the game — including engine power (`Power`), torque (`Torque`), boost pressure (`Boost`), wheel rotation speeds (`WheelRotationSpeed`), longitudinal and lateral tire slip (`TireSlipRatio`, `TireSlipAngle`), and suspension travel — the plugin autonomously learns transmission gear ratios and calculates the exact mathematical power curve intersection point between consecutive gears (Gears 1–10).

In **v1.1.0**, the project expands into a complete dual-game race-engineering suite featuring automatic **Forza Horizon 6 / Forza Motorsport detection**, a **Built-In "Dragy" Acceleration Timer** (`0–100 km/h`, `100–200 km/h`, `0–200 km/h`), an **F1 / GT3 Audio Shift Beep**, **Custom Car & Build Naming** (`✏️ Name Car` / `FH6SmartShift_CarList.json`), **Active Gear Color/Status Delegates** (`CurrentGearColor`, `CurrentGearStatusText`), a **Native SimHub WPF Settings Control Center**, an **Automated Gearbox Tuning Advisor**, and **Single-Car Profile Export/Merge** inside the interactive HTML Garage Inspector (`FH6SmartShift_GarageReport.html`).

---

## 🆕 What's New in v1.1.0

1. **🏁 Dual-Game Engine (*Forza Horizon 6* & *Forza Motorsport*):** Automatically detects the active game in SimHub (`[FH6SmartShift.ActiveGameMode]`). Prefixes *Forza Motorsport* vehicle keys with `FM_` to keep Horizon (`[FH]`) and Motorsport (`[FM]`) databases cleanly separated, while automatically tightening tire slip filters for track racing in *Forza Motorsport*.
2. **🎯 Active Gear Delegates (`CurrentGearColor` & `CurrentGearStatusText`):** Directly exposes the learning status label and Hex color code of the currently engaged gear (`1–10`) so you can bind a dynamic border, frame, or background box without nested NCalc `if()` statements.
3. **🏷️ Custom Car & Build Naming (`✏️ Name Car` & `FH6SmartShift_CarList.json`):** Because raw UDP telemetry only provides a numeric `CarOrdinal` ID, you can assign custom names to specific cars or individual tuning setups directly in the HTML Garage Report (`✏️ Name Car`) and save them to `FH6SmartShift_CarList.json`. Exposed in SimHub via `[FH6SmartShift.CurrentCarName]`.
4. **🔊 F1 / GT3 Audio Shift Beep (`ToggleShiftBeep`):** Optional background audio cue (`2650 Hz`, `60 ms`) triggered the exact instant all 12 LEDs lock at `ShiftLightFlash` (`0.40 s` cooldown, zero frame-time impact).
5. **⏱️ Built-In "Dragy" Performance Timer:** Sub-frame linearly interpolated acceleration timer measuring **`0–100 km/h`**, **`100–200 km/h`** (standing or rolling), and **`0–200 km/h`**, automatically saving Personal Best (PB) records per car/tune in the JSON database.
6. **🔧 Automated Gearbox Tuning Advisor:** Embedded in `FH6SmartShift_GarageReport.html`, automatically analyzing each gear transition (`N -> N+1`) against `RpmAtMaxTorque`, `RpmAtMaxPower`, and `BoostMap` to flag overly wide ratios, turbo lag drops (`< 72%` peak boost), or overly tight low-gear steps.
7. **🖥️ Native SimHub WPF Control Center (`IWPFSettings`):** Dedicated SimHub settings panel with live telemetry/Dragy status, an interactive **Driver Reaction-Time Lead Buffer Slider (`0.10 s – 0.40 s`)**, Audio Shift Beep checkbox, and one-click memory/report actions.
8. **🌐 Single-Car Export & Database Merge:** Export individual calibrated cars (`📤 Export Car`) to share with the community or merge external `.json` profiles (`📥 Import / Merge JSON`) directly inside the HTML Garage Inspector.
9. **⚡ Smooth 12-LED Shift Engine, Multi-Warning Pill & 3-Color Slide Recovery System:** Sequential 12-LED bar (`LedActiveCount` `1..12`) with step hysteresis, a multi-function warning indicator (`SLIP` / `OVRST!` / `BOTTOM!`), and a real-time slip-derivative bar (`SlideSeverity`, `SlideBarColor`, `SlideStateText`) indicating whether a slide is **Deepening** (🔴 Red), **Holding** (🟠 Orange), or **Recovering** (🟢 Green).

---

## ⚙️ Architecture & Actual Operating Logic

* **401-Band Virtual Dyno (`PowerMap`, `TorqueMap`, `BoostMap`):** Sampling takes place near wide-open throttle (`Accel >= 240` on a `0–255` scale, i.e., **~94% throttle**), at power levels above `5000 W`, after at least 2 consecutive clean WOT frames (`_cleanWotFrames >= 2`). The range up to `20,000 RPM` is divided into 401 bands in `50 RPM` increments, updated with an asymmetric EMA filter (`alpha = 0.35` for higher power readings, `0.08` for lower readings) and protected by automatic 1–2 band pocket healing (`HealEngineMapPockets`).
* **Power Curve Intersection Algorithm (`EvaluateAllMathIntersections`):** Every `0.15 s` (when data has changed), the algorithm compares smoothed engine power in the current gear (`pNow` at `r` RPM) against power after upshifting to the next gear (`pNext` at landing RPM `r * dropRatio`). If the engine is turbocharged (`Boost > 0.1`) and expected boost at landing RPM drops below `70%` of current boost, `pNext` is penalized by a `0.85` multiplier. A shift point is confirmed as mathematically optimal `(P)` when, for `r >= RpmAtMaxPower`, the condition **`pNext >= pNow * 1.015`** (1.5% wheel-power advantage) is met across **5 consecutive 25 RPM steps** (or 2 steps near the mapped ceiling), and sample density in the active gear landing zone is at least **78%**.
* **Fast & Bulletproof Gear Ratio Learning (`RpmPerRadS`):** The `CurrentEngineRpm / avgWheelSpeed` ratio is recorded when `Accel >= 75.0`, with all wheels grounded, no suspension rebound spike, no braking (`Brake < 5.0`), slip across all wheels `< 0.08`, wheel speed `> 16 rad/s`, and engine speed `> 1400 RPM`. Samples are stored in a 25-element circular buffer. **Median calculation and ratio confirmation occur once 12 samples are collected (~0.18 s) with an interquartile spread `(p75 - p25) / median < 1.2%`**. The system enforces global ratio monotonicity (`IsRatioMonotonicAcrossAllGears`, requiring a minimum `3.5%` step per gear across all known gears `1..10`) and automatically resets transmission ratios upon detecting a tuning change.
* **Hidden Limiter Detection (`GhostLimiterRpm`):** Detects power cuts (`powerRatio < 0.85`, RPM drop `_currentRpmAccel < -150 RPM/s`, or an RPM wall `|_currentRpmAccel| < 60 RPM/s` near the mapped ceiling for `0.05 s`) occurring before nominal `EngineMaxRpm`. After hitting the same band twice (`±150 RPM`), the plugin locks `GhostLimiterRpm` and purges contaminated bands (`CleanUpContaminatedBands`). Automatically unlocks if the engine later revs past `GhostLimiterRpm + 150 RPM` for `0.10 s`.
* **Multi-Layer Physics Filters (`filtersOk`):** Dyno learning is paused whenever any of the following conditions occur:
  1. Any wheel leaves the ground (`NormalizedSuspensionTravel <= 0.02`).
  2. Driving into a puddle (`WheelInPuddleDepth > 0.01` on any wheel).
  3. Vehicle damage (`Damage1 > 0.05` or `Damage2 > 0.05`).
  4. Sudden **rear** suspension rebound (`|ΔSuspensionTravelMetersRear| > 0.08 m` between frames for RL or RR).
  5. Surface vibration (`SurfaceRumble > 0` on any wheel).
  6. Clutch or handbrake engagement (`Clutch >= 2.0` or `HandBrake >= 2.0`), or post-shift stabilization cooldown (`0.25 s` upshift, `0.45 s` downshift, `0.12 s` clutch).
  7. Exceeding the `tractionOk` threshold:
     * **Forza Horizon 6 (`FH6`):** driven wheel longitudinal slip `>= 0.60` (gears 1–3) or `>= 0.38` (gears 4+), or slip angle `>= 0.25 rad`.
     * **Forza Motorsport (`FM`):** driven wheel longitudinal slip `>= 0.42` (gears 1–3) or `>= 0.26` (gears 4+), or slip angle `>= 0.18 rad`.
* **Downshift & Hill-Climb Assistant (`Downshift` & `_climbDownshiftActive`):** The safe downshift threshold (`DownshiftTriggerRpm`) is raised proportionally to braking force (up to `+600 RPM`) and dynamic front-axle dive (`DbgChassisDive`, up to `+250 RPM`), capped by a hard ceiling of `(actualLimiter - 150) * dropFromPrev`. An independent `_climbDownshiftActive` algorithm suggests a downshift under load when `Accel >= 225`, speed trend is `<= +1.0 km/h/s`, power in the lower gear is higher by at least **5%** (`pLand > pNow * 1.05`), post-downshift RPM is at least `300 RPM` below the lower gear's shift point, and driven wheel slip is `< 0.15`.
* **Sub-Frame "Dragy" Performance Timer (`UpdateDragyTimers`):** Uses linear interpolation between 60 Hz UDP frames (`frac = (targetSpeed - prevSpeed) / (currentSpeed - prevSpeed)`) to measure `0–100 km/h` (armed below `1.5 km/h`), `100–200 km/h` (triggered automatically upon crossing `100.0 km/h` under throttle), and `0–200 km/h`.
* **Zero-Stutter Persistence & HTML Report Generation:** Disk saving takes place asynchronously every `15 s` minimum (when data has changed): the plugin performs a deep copy of all car dictionaries (`CloneCarStore`) inside a `lock` statement and offloads both JSON serialization and `FH6SmartShift_GarageReport.html` generation to a background `Task.Run` thread.

---

## 🖥️ Virtual Dyno, Dragy & Gearbox Inspector (`FH6SmartShift_GarageReport.html`)

Every time the plugin saves telemetry data (or when you click **Open Virtual Dyno & Tuning Report** in SimHub / trigger `FH6SmartShift.OpenDynoReport`), it generates an interactive HTML application in:
`C:\Program Files (x86)\SimHub\PluginsData\Common\FH6SmartShift_GarageReport.html`

### Key Features of the Garage Inspector:
1. **Vehicle Selector & Custom Car Naming (`✏️ Name Car` & `💾 CarList.json`):** Switch between all saved cars/tunes on the dropdown list (prefixed with `[FH]` or `[FM]` and formatted by default as `Car #Ordinal | PI | Drivetrain | Gauge RPM`). Click **`✏️ Name Car`** to assign your own custom name to a specific car or build, and click **`💾 CarList.json`** to save your custom dictionary to `PluginsData\Common\`.
2. **Profile Sharing (`📤 Export Car` & `📥 Import / Merge JSON`):** Export any single car profile (`FH6_CarProfile_<Key>.json`) to share with friends, or import/merge external profiles and download the combined database (`💾 Save Merged DB`).
3. **One-Click Engine & Dyno Summary Banner:** Displays peak power (HP & kW), peak torque (Nm), max boost pressure (psi & bar), and detected `GhostLimiterRpm` with a **`📋 Copy Summary`** button:
   > *Example:* `Engine & Dyno Profile — [FH] Car #4122 (4122_PI700_DT2_R8000): Peak power of 675.0 HP (503.3 kW) at 7100 RPM, peak torque of 734.8 Nm at 6300 RPM, 41.03 psi (~2.83 bar) of boost, and a detected hidden rev limiter (GhostLimiterRpm) at 7600.7 RPM (on an 8000 RPM gauge cluster).`
4. **Dragy Performance Records Cards:** Displays saved Personal Best times for **`0–100 km/h`**, **`100–200 km/h`**, and **`0–200 km/h`**.
5. **Complete Gears 1–10 Shift Table & Automated Gearbox Tuning Advisor:** Displays each gear's ratio (`RpmPerRadS`), percentage RPM drop (`dropRatio`), next gear landing RPM (`~RPM`), saved JSON threshold, physical shift target (`OptimalShiftRpm`), status badge, and automated ratio tuning recommendations beneath the table.
6. **Interactive 401-Band Dyno Chart:** Plots smoothed **Engine Power (HP, Green)** and **Engine Torque (Nm, Blue)** across the RPM range.

---

## 📱 Included SimHub Dashboards (Classic & v1.1 Optimal Layout)

The repository includes both **Classic** and **Optimal MoTeC-Style** dashboard layouts (`KM/H` and `MPH` variants) scaled cleanly for landscape mobile screens (`1334×513` / `980×377`) and PC overlays.

### 1. Classic Telemetry Layout (`FH6SmartShift_Dashboard_EN.jpg`)

![FH6SmartShift Classic Dashboard Overview](./FH6SmartShift_Dashboard_EN.jpg)

* **12-Segment Predictive Shift Light Bar (Top):** Scales dynamically to your car's calculated shift point (`ShiftTriggerRpm`), including a real-time reaction-time buffer based on RPM acceleration.
* **Central Gear, Speed & RPM Cluster:** High-contrast gear readout flanked by vehicle speed (`KM/H` or `MPH`) and live engine RPM.
* **Gears 1–10 Calibration Matrix (Right):** Real-time per-gear learning status (`(-)` Red, `(R)` Blue, `(L?)` Orange, `(P)` / `(L)` Green).
* **Live Mapping Progress (`100.0`), Clock (`18:33`), `SLIP` Warning & Bottom Bar:** Essential peripheral indicators at the bottom of the screen.

---

### 2. Enhanced MoTeC-Style Layout (`FH6SmartShift_Optimal_Layout_v1.1.jpg`)

![FH6SmartShift Optimal Layout v1.1](./FH6SmartShift_Optimal_Layout_v1.1.jpg)

The updated layout refines visual hierarchy and exposes the new v1.1.0 telemetry properties:

1. **Smooth 12-LED Bar (Zero Skipping):** Bound to `[FH6SmartShift.LedActiveCount]` (`1..12`) and `[FH6SmartShift.ShiftLightFlash]`. Lights up all 12 LEDs sequentially (Green 1–4, Yellow 5–8, Red 9–11, Blue/Red 12) with step hysteresis and WOT anti-flicker protection.
2. **Compact Dyno Status Pill & Small Clock (`DYNO OK 100%` + `18:33`):** Combines `[FH6SmartShift.EngineMapStatus]` and `[FH6SmartShift.LiveMappingProgress]` into a status badge alongside a compact system clock.
3. **Downshift & Hill-Climb Assistant (`▼ DOWN`):** Bound to `[FH6SmartShift.IsDownshiftSuggested]` and `[FH6SmartShift.IsDownshiftSafe]`. Illuminates green to the left of the speed readout during safe braking downshifts or when the hill-climb assistant detects a `>= 5%` wheel-power gain in the lower gear.
4. **Live Shift Target Pill (`SHIFT @ 7520`):** Placed directly beneath the live `ENGINE RPM` readout, displaying `[FH6SmartShift.OptimalShiftRpm]` so you always know the exact target RPM for the currently engaged gear.
5. **Multi-Function Warning Pill (`SLIP` / `OVRST!` / `BOTTOM!`) & 3-Color Slide Recovery System:**
   * **Warning Pill:** Combines `[FH6SmartShift.IsGripLost]` (Yellow `SLIP`), `[FH6SmartShift.PowerOversteerWarning]` (Red `OVRST!`), and `[FH6SmartShift.AeroBottomingWarning]` (Blue `BOTTOM!` above `200 km/h`).
   * **3-Color Slide Recovery Bar:** Bound to `SlideSeverity`, `SlideBarColor`, `SlideStateText`, and `BodySlipAngle`:
     * 🔴 **Red (`▲ DEEPENING`):** Slip angle and wheelspin are increasing — ease off the throttle or add counter-steer!
     * 🟠 **Orange (`● HOLDING`):** Car is maintaining a steady drift/slip angle.
     * 🟢 **Green (`◄ RECOVERING`):** Slip angle is decreasing and tires are regaining grip — safe to straighten the wheel and apply full throttle!
6. **Gears 1–10 Calibration Matrix (`GEAR MATRIX (1–10)`) & Active Gear Frame:** Cleanly separated two-column matrix on the right side of the screen, plus direct support for `[FH6SmartShift.CurrentGearColor]` and `[FH6SmartShift.CurrentGearStatusText]` to highlight the active gear.

| Matrix Label | Color | Hex Code | Actual Condition in C# Code |
| :--- | :--- | :--- | :--- |
| **`N: (P)`** | 🟢 Green | `#4CAF50` | **Peak Optimal (`IsConfirmed && IsMathOptimal`):** Confirmed power curve intersection with gear `N+1` (`pNext >= 1.015 * pNow` across 5 steps) and active landing zone density `>= 78%`. |
| **`N: (L)`** | 🟢 Green | `#4CAF50` | **Limiter Confirmed (`IsConfirmed && !IsMathOptimal`):** Confirmed optimal shift right at the true/ghost rev limiter (no intersection before cutoff, or top gear confirmed). |
| **`N: (L?)`** | 🟠 Orange | `#F39C12` | **Edge / Unconfirmed Limit (`TriggerIsEdge`):** Provisional trigger near the mapped ceiling while the engine map is still being populated. |
| **`N: (P?)`** | 🟡 Yellow | `#F1C40F` | **Unconfirmed Trigger (`PowerPeakCaught && ShiftTriggerRpm > 0`):** Gear has a saved threshold, awaiting full confirmation. |
| **`N: (R)`** | 🔵 Blue | `#3498DB` | **Ratio Locked (`RpmPerRadS > 0`):** Gear ratio learned (`>= 12` tight-cluster IQR samples), waiting for gear `N+1` ratio or WOT pull to limiter. |
| **`N: (-)`** | 🔴 Red | `#E74C3C` | **Unmapped / Paused:** Gear has no learned ratio yet, memory was reset, or game is paused/in menu (`IsRaceOn == 0`). |

---

## 📂 Repository Contents & File Paths

```text
├── FH6SmartShift.cs                       # Full C# source code of the plugin (v1.1.0 - FH6 & FM)
├── FH6SmartShift.csproj                   # .NET Framework 4.8 (C# 7.3 + WPF) project file
├── FH6SmartShift.dll                      # Compiled library ready to drop into SimHub
├── FH6SmartShift_GarageReport.html        # Interactive Virtual Dyno, Dragy & Tuning Inspector
├── FH6SmartShift_Dashboard_EN.jpg         # Annotated classic dashboard infographic
├── FH6SmartShift_Optimal_Layout_v1.1.jpg  # Annotated v1.1 MoTeC-style dashboard layout
├── DashTemplates.zip                      # Classic SimHub dashboards (KM/H & MPH)
├── FH6SmartShift_Optimal.zip              # Optimal v1.1 MoTeC-style SimHub dashboard (1334x513)
├── README.md                              # Online repository documentation (v1.1.0)
└── README.pdf                             # Printable user manual & technical reference
