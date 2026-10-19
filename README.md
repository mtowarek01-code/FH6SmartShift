# 🏎️ FH6SmartShift (v1.1.1) — Virtual Dyno, Dragy Timer & Shift Telemetry System for Forza Horizon 6 & Forza Motorsport

[![Platform](https://img.shields.io/badge/Platform-SimHub-blue.svg)](https://www.simhubdash.com/)
[![Games](https://img.shields.io/badge/Games-Forza%20Horizon%206%20%7C%20Forza%20Motorsport-orange.svg)](#)
[![Framework](https://img.shields.io/badge/.NET%20Framework-4.8-purple.svg)](https://dotnet.microsoft.com/)
[![Language](https://img.shields.io/badge/C%23-7.3-success.svg)](https://docs.microsoft.com/en-us/dotnet/csharp/)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-brightgreen.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Buy Me A Coffee](https://img.shields.io/badge/Buy%20Me%20a%20Coffee-ffdd00?logo=buy-me-a-coffee&logoColor=black)](https://buymeacoffee.com/mtowarek01d)

**Author:** mtowarek01  
**Version:** v1.1.1 (Dual-Game FH6 & Forza Motorsport Engine, 100% Dyno Map Gate, Dragy Timer, F1 Shift Beep, Tuning Advisor & WPF UI)

---

## 📖 About the Project

**FH6SmartShift** is an advanced, real-time (60 Hz) C# telemetry plugin for **SimHub** and a dedicated racing dashboard suite engineered for the **Forza Horizon 6** and **Forza Motorsport** physics engines.

Unlike basic scripts that only average rev-limiter bounce RPM, **FH6SmartShift** constructs a live **401-band Virtual Dyno** (`50 RPM` resolution up to `20,000 RPM`). By capturing raw UDP telemetry from the game — including engine power (`Power`), torque (`Torque`), boost pressure (`Boost`), wheel rotation speeds (`WheelRotationSpeed`), longitudinal and lateral tire slip (`TireSlipRatio`, `TireSlipAngle`), and suspension travel — the plugin autonomously learns transmission gear ratios and calculates the exact mathematical power curve intersection point between consecutive gears (Gears 1–10).

---

## 🎯 Best Practice: How to Calibrate the Engine Map (100% Dyno) & Gears

To get a **100% repeatable Virtual Dyno curve**, accurate peak torque readings, and verified **Gearbox Tuning Advisor** recommendations, follow this 2-step calibration method on a flat straight:

### Why a Standing 0–200 km/h Launch Is Not Enough for the Dyno Map
During a hard standing launch (`0–200 km/h`), **1st gear** typically experiences heavy wheelspin (which the plugin's traction gate filters out to protect your data), and upshifts into **Gears 2–6** land high in the rev range (typically above `4,800–5,500 RPM`). As a result, the entire mid-range (`2,500–4,800 RPM` — where peak torque usually lives) is skipped, leaving the dyno map around `55–65%` complete.

### ✅ Step 1: The 3rd or 4th Gear Dyno Pull (2,500 RPM to Limiter)
1. Shift into **3rd or 4th gear** while cruising at low RPM (**below `2,500 RPM`**).
2. Press and hold **100% Wide-Open Throttle (WOT)** cleanly all the way to the **rev limiter**.
3. **Why this works best:** A higher gear loads the engine smoothly without wheelspin, gives turbochargers time to reach 100% boost pressure before hitting the mid-range, and sweeps every single `50 RPM` band from `2,500 RPM` to redline in one continuous pass.
4. Watch the dashboard badge switch to **`DYNO OK 100%`**. Reaching `100%` automatically unlocks the verified **Virtual Dyno Curve** and **Automated Gearbox Tuning Advisor** inside `FH6SmartShift_GarageReport.html`.

### ✅ Step 2: Lock All Gear Ratios (Gears 1–10)
1. Accelerate cleanly through each gear (**Gear 1 up to your top gear**) for at least `0.35 s` per gear without spinning the tires.
2. As each gear ratio locks, its slot in the dashboard matrix turns **Blue `(R)`**.
3. Once the next gear (`N+1`) is locked and the dyno map is populated, the plugin automatically calculates the power curve intersection and turns each gear **Green `(P)`** (Mathematical Peak Shift) or **Green `(L)`** (Rev-Limiter Optimal Shift).

---

## 🖥️ FH6SmartShift — Dyno, Dragy & Gearbox Tuning Inspector (`FH6SmartShift_GarageReport.html`)

Every time the plugin saves telemetry data (or when you click **📊 Open Virtual Dyno & Tuning Report (HTML)** in the SimHub WPF Settings tab / trigger `FH6SmartShift.OpenDynoReport`), it generates an interactive standalone browser application in:
`C:\Program Files (x86)\SimHub\PluginsData\Common\FH6SmartShift_GarageReport.html`

![FH6SmartShift — Dyno, Dragy & Gearbox Tuning Inspector](./FH6SmartShift%20—%20Dyno,%20Dragy%20&%20Gearbox%20Tuning%20Inspector.jpg)

![FH6SmartShift — 401-Band Virtual Dyno Curve](./FH6SmartShift%20—%20Dyno,%20Dragy%20&%20Gearbox%20Tuning%20Inspector%202.jpg)

### Detailed Breakdown of the Garage Inspector Modules:

1. **Top Control Bar (Profile Selector, Custom Naming & Database Sharing):**
   * **Vehicle & Build Dropdown (`carSelect`):** Switch between all saved profiles across *Forza Horizon 6* (`[FH]`) and *Forza Motorsport* (`[FM]`), formatted with Car Name, PI Class, Drivetrain (`FWD` / `RWD` / `AWD`), and Gauge Cluster RPM (e.g., `Car #1006 | PI800 | AWD | Gauge 10000 RPM (1006_PI800_DT2_R10000)`).
   * **`✏️ Name Car` & `💾 CarList.json`:** Because raw Forza UDP telemetry only transmits a numeric `CarOrdinal` ID, click **`✏️ Name Car`** to assign a custom name to any car or specific tune, and click **`💾 CarList.json`** to save your dictionary to `PluginsData\Common\`.
   * **`📤 Export Car` & `📥 Import / Merge JSON`:** Export any single calibrated car build (`FH6_CarProfile_<Key>.json`) to share with other tuners, or import and merge external `.json` profiles into your database (`💾 Save Merged DB`).
2. **Engine & Dyno Profile Summary Banner (`📋 Copy Summary`):**
   * Displays map completion status (`[DYNO MAP: 100% COMPLETE]`), peak power in HP and kW with exact RPM, peak torque in Nm with exact RPM, maximum turbo boost in psi and bar, and detected hidden rev limiters (`GhostLimiterRpm`) with a one-click **`📋 Copy Summary`** button:
   > *Example:* `Engine & Dyno Profile — Car #1006 (1006_PI800_DT2_R10000): Peak power of 628.8 HP (468.9 kW) at 9000 RPM, peak torque of 583.8 Nm at 6200 RPM, 4.20 psi (~0.29 bar) of boost, and a detected hidden rev limiter (GhostLimiterRpm) at 9481.0 RPM (on an 10000 RPM gauge cluster).`
3. **Dragy Acceleration Telemetry Cards (`Best PB`):**
   * Displays saved Personal Best acceleration times calculated via 60 Hz sub-frame linear interpolation:
     * **⏱️ `0–100 KM/H (BEST PB)`** — Standing launch telemetry (e.g., `2.25 s`).
     * **🚀 `100–200 KM/H (BEST PB)`** — Rolling or standing mid-range pull (e.g., `4.26 s`).
     * **🏁 `0–200 KM/H (BEST PB)`** — Full standing sprint (e.g., `6.53 s`).
4. **Complete Gears 1–10 Shift & Ratio Table:**
   * **`Gear Ratio (RpmPerRadS)`:** Locked mechanical ratio (`EngineRPM / WheelRadS`, e.g., `138.76` in Gear 1 down to `28.11` in Gear 10).
   * **`RPM Drop on N+1 Upshift (dropRatio)`:** Ratio step (`nextRatio / currentRatio`) and percentage RPM drop on upshift (e.g., `0.678 (-32.2%)` on `1 -> 2`, down to close-ratio `0.927 (-7.3%)` on `9 -> 10`).
   * **`Next Gear Landing RPM`:** Exact engine RPM immediately after upshifting at the target RPM (e.g., `~6376 RPM` on `1 -> 2`, `~8734 RPM` on `9 -> 10`).
   * **`Saved JSON Threshold (Rpm)` & `Physical Shift Target (OptimalShiftRpm)`:** Raw trigger stored in JSON and the exact unbuffered physical shift target (e.g., `9401 RPM` for limiter-optimal gears `1–7` & `10`, and `9448 RPM` / `9423 RPM` for math-peak optimal gears `8` & `9`).
   * **`Dashboard Status`:** Color-coded calibration badge matching the SimHub dashboard (`(P)` Math Peak Optimal, `(L)` Limiter Confirmed, `(L?)` Edge/Unconfirmed, `(P?)` Unconfirmed Trigger, `(R)` Ratio Locked, `(-)` Unmapped).
5. **Automated Gearbox Tuning Advisor (100%-Map Gated Telemetry Analysis):**
   * Once the engine map reaches 100%, automatically evaluates every gear transition (`N -> N+1`) against your engine's torque curve, power curve, and boost curve:
     * ✅ **`Powerband Alignment`:** Confirms how many gear transitions land squarely between Peak Torque and Peak Power (e.g., `9 gear transition(s) land squarely inside the optimal power window between Peak Torque (6200 RPM) and Peak Power (9000 RPM)`).
     * 🐌 **`Turbo Lag Risk`:** Flags gear transitions where landing RPM drops turbo boost below `72%` of peak boost (e.g., `Turbo Lag Risk on Gear 1 -> 2: Boost pressure at ~6376 RPM is only 2.1 psi (vs 4.2 psi peak). Shorten Gear 2 to keep the turbo spooled!`).
     * ⚠️ **`Too Wide / Power Drop / Very Close Ratio`:** Flags overly wide gear steps (landing below `90%` of Peak Torque RPM or losing `> 22%` peak power) and overly tight low-gear spacing (`dropRatio > 0.895`).
6. **401-Band Virtual Dyno Curve Canvas (`Power HP & Torque Nm`):**
   * Plots smoothed **Engine Power (HP, Green)** and **Engine Torque (Nm, Blue)** across the entire RPM range (`1,000–10,000+ RPM`) without artificial straight-line bridging across unmapped gaps.

---

## 🧱 Core Features (v1.0 Foundation — Built-In Telemetry Systems)

These foundational systems form the core physics and mathematical engine of **FH6SmartShift**:

1. **📊 401-Band Live Virtual Dyno (`PowerMap`, `TorqueMap`, `BoostMap`):**
   * Samples engine output at 60 Hz across 401 discrete `50 RPM` bands (from `0` to `20,000 RPM`) during Wide-Open Throttle (`Accel >= 240`, `> 5000 W`).
   * Uses asymmetric Exponential Moving Average (EMA) filtering to reject transient dips while capturing true peak output, accompanied by a live 20-block coverage indicator (`MapCoverageBar` `[████████░░░░░░░░░░░░]`) and percentage readout (`LiveMappingProgress`).
2. **🧮 Mathematical Power Curve Intersection Engine (Gears 1–10):**
   * Evaluates every gear transition (`N -> N+1`) by comparing smoothed power in the current gear (`pNow`) against landing power in the next gear (`pNext` at `r * dropRatio`).
   * **Turbo Boost-Drop Compensation:** Penalizes `pNext` by `0.85` if upshifting drops boost pressure below `70%` of current boost, preventing premature shifts into turbo lag.
   * Automatically classifies each gear in real time: Red `(-)` (Unmapped), Blue `(R)` (Ratio Locked), Orange `(L?)` (Provisional Ceiling), Yellow `(P?)` (Unconfirmed Peak), Green `(P)` (Confirmed Mathematical Peak Intersection), or Green `(L)` (Confirmed Limiter Optimal).
3. **⚙️ Autonomous Gear Ratio Learning & Auto-Reset on Re-Tuning:**
   * Computes mechanical ratios (`CurrentEngineRpm / avgWheelSpeed`) for AWD, RWD, and FWD drivetrains using a 25-sample circular buffer and interquartile range (IQR) cluster filtering.
   * Enforces global ratio monotonicity across Gears 1–10 and automatically detects when you change transmission sliders in the tuning menu (`RatioMismatchCount`), resetting stale ratios without wiping your engine dyno map.
4. **👻 Hidden Rev Limiter ("Ghost Limiter") Detection & Auto-Unlock:**
   * Many upgraded or engine-swapped cars in Forza cut fuel/ignition hundreds of RPM before the visual gauge `EngineMaxRpm`.
   * The plugin detects power cuts (`powerRatio < 0.85`), sudden RPM deceleration (`< -150 RPM/s`), or RPM walls near the ceiling, locks `GhostLimiterRpm` after 2 confirmed hits, and purges contaminated upper bands (`CleanUpContaminatedBands`).
   * If you later upgrade cams/valves and rev past `GhostLimiterRpm + 150 RPM` for `0.10 s`, it automatically breaks and unlocks the old ceiling (`GhostLimiterBreakAccumulator`).
5. **⚡ Dynamic Driver Reaction-Time Lead Buffer:**
   * Tracks real-time engine RPM acceleration (`_currentRpmAccel` in `RPM/s`) and triggers the shift light bar *ahead* of the physical target (`ShiftTriggerRpm = realShiftRpm - liveBuffer`, scaled up to `1500 RPM` in Gears 1–2 and `600 RPM` in Gears 3+) so your physical reaction time lands right on the optimal shift point without hitting the limiter.
6. **🔽 Smart Downshift & Hill-Climb Assistant (`IsDownshiftSafe` & `IsDownshiftSuggested`):**
   * **Braking & Chassis Dive Compensation:** Calculates safe downshift thresholds (`93%` landing multiplier, `88%` max cap) and dynamically raises `DownshiftTriggerRpm` under braking (up to `+600 RPM`) and dynamic front-axle dive (`DbgChassisDive`, up to `+250 RPM` relative to a self-calibrating static sag baseline `_staticChassisDiveBase`).
   * **Hill-Climb / Load Downshift Prompt (`_climbDownshiftActive`):** Suggests a downshift at WOT when vehicle acceleration stagnates (`_speedTrendKmhPerS <= +1.0 km/h/s`) and the lower gear provides at least **`+5%` more wheel power** (`pLand > pNow * 1.05`) with safe RPM headroom.
7. **🛡️ 7-Layer Physics & Surface Contamination Filter (`filtersOk`):**
   * Automatically pauses dyno sampling during wheelspin (`tractionOk`), airborne wheels (`NormalizedSuspensionTravel <= 0.02`), puddles (`WheelInPuddleDepth > 0.01`), rumble strips/kerbs (`SurfaceRumble > 0`), rear suspension rebound spikes (`|ΔSuspensionTravelMetersRear| > 0.08 m`), vehicle damage (`Damage1/2 > 0.05`), clutch/handbrake engagement, and post-shift settle windows (`0.25 s` upshift, `0.45 s` downshift, `0.15 s` clutch).
8. **🏎️ Chassis, Suspension & Traction Warnings:**
   * **Grip Loss (`IsGripLost`):** Triggers above `1.25 G` combined acceleration or `> 0.30` driven wheel slip.
   * **Power Oversteer (`PowerOversteerWarning` & `OversteerSeverity`):** Alerts RWD/AWD drivers when rear combined tire slip exceeds `0.60` under throttle (`0.0–1.0` severity scale).
   * **High-Speed Aero Bottoming (`AeroBottomingWarning`):** Triggers when any damper hits `>= 98%` compression above `200 km/h`, helping tuners dial in ride height and spring stiffness.
   * **Body Slip Angle (`BodySlipAngle`) & Top Gear Detection (`IsTopGear`):** Real-time chassis drift angle in degrees and automatic top/overdrive gear identification.
9. **💾 Per-Build Profile Persistence, Legacy Migration & Daily Backups:**
   * Stores independent profiles keyed by `{Ordinal}_PI{PI}_DT{DT}_R{Rpm}` (so an A800 RWD build and an S1-900 AWD build of the same car never overwrite each other).
   * Uses lock-cloned asynchronous background saving (`Task.Run`) for zero frame stutter and maintains **3 rolling daily backups** (`.bak_yyyyMMdd`) in `PluginsData\Common\`.

---

## 🆕 What's New in v1.1.0 & v1.1.1 (Latest Expansion)

Building on the v1.0 foundation, versions **v1.1.0** and **v1.1.1** introduce:

1. **🏁 Dual-Game Engine (*Forza Horizon 6* & *Forza Motorsport*):**
   * Automatically detects the active game in SimHub (`[FH6SmartShift.ActiveGameMode]` returns `"FH6"` or `"FM"`).
   * Prefixes *Forza Motorsport* vehicle keys with `FM_` to keep Horizon (`[FH]`) and Motorsport (`[FM]`) databases cleanly separated, while tightening tire slip gates for track racing in *Forza Motorsport*.
2. **🔒 100% Dyno Map Verification Gate & Turbo Spool Filter (v1.1.1):**
   * Tracks full-range mapping progress from `~2,500–3,000 RPM` (`min(3000, 0.45 * RpmAtMaxPower)`) to `actualLimiter - 250 RPM`.
   * Locks the HTML Virtual Dyno chart and Gearbox Tuning Advisor until the map reaches **100% completion** (with a `👁️ Toggle Partial Curve` button for live diagnostics), eliminating fake straight-line bridging across unmapped mid-range RPM bands.
   * **Turbo Spool-Up Protection (`MaxBoost`):** Rejects low-boost transient samples above `2,800 RPM` (`< 78%` peak boost) when a turbocharger is still spooling up after a gear shift, preventing jagged torque spikes, and automatically heals 1–2 band pockets (`HealEngineMapPockets`).
3. **🛡️ Self-Healing Gear Ratio Progression (v1.1.1):**
   * Enforces a `0.35 s` post-upshift stabilization window and a `1.2%` interquartile spread (`10+` samples) before locking gear ratios.
   * **Instant Upward Self-Correction:** Since wheelspin during acceleration always inflates wheel speed and *underestimates* `RpmPerRadS`, if the plugin later observes a higher mechanical ratio (`> +2.5%`) under full grip (`< 4%` slip), it immediately overwrites the slip-contaminated value.
   * **Geometric Progression Sanitizer (`SanitizeGearProgression`):** Automatically detects and purges any middle gear whose recorded ratio creates an impossible deep drop (`< 0.635`) followed by an abnormally flat step (`> 0.875`).
4. **📊 Interactive HTML Garage Inspector & Automated Gearbox Tuning Advisor (`FH6SmartShift_GarageReport.html`).**
5. **⏱️ Built-In Sub-Frame "Dragy" Performance Timer (`0–100`, `100–200`, `0–200 km/h`)** with Personal Best persistence.
6. **🔊 F1 / GT3 Audio Shift Beep (`ToggleShiftBeep` — Off by Default):**
   * Optional background audio cue (`2650 Hz`, `60 ms`) triggered the exact instant all 12 LEDs lock at `ShiftLightFlash` (`0.40 s` cooldown, zero frame-time impact). Disabled by default; enable anytime in the SimHub WPF Settings tab or via action `FH6SmartShift.ToggleShiftBeep`.
7. **🏷️ Custom Car & Build Naming (`✏️ Name Car` & `FH6SmartShift_CarList.json`).**
8. **🌐 Single-Car Export (`📤 Export Car`) & Database Merge (`📥 Import / Merge JSON`).**
9. **🖥️ Native SimHub WPF Control Center (`IWPFSettings`)** with an interactive **Driver Reaction-Time Lead Buffer Slider (`0.10 s – 0.40 s`)**.
10. **🎯 Active Gear Delegates (`CurrentGearColor`, `CurrentGearStatusText`), Smooth 12-LED Engine (`LedActiveCount` `1..12`) & 3-Color Slide Recovery Bar (`▲ DEEPENING`, `● HOLDING`, `◄ RECOVERING`).**

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

The updated layout refines visual hierarchy and exposes the v1.1.1 telemetry properties:

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
| **`N: (R)`** | 🔵 Blue | `#3498DB` | **Ratio Locked (`RpmPerRadS > 0`):** Gear ratio learned (`>= 10` tight-cluster IQR samples), waiting for gear `N+1` ratio or WOT pull to limiter. |
| **`N: (-)`** | 🔴 Red | `#E74C3C` | **Unmapped / Paused:** Gear has no learned ratio yet, memory was reset, or game is paused/in menu (`IsRaceOn == 0`). |

---

## 📂 Repository Contents & File Paths

```text
├── FH6SmartShift.cs                                                    # Full C# source code of the plugin (v1.1.1 - FH6 & FM)
├── FH6SmartShift.csproj                                                # .NET Framework 4.8 (C# 7.3 + WPF) project file
├── FH6SmartShift.dll                                                   # Compiled library ready to drop into SimHub
├── FH6SmartShift_GarageReport.html                                     # Interactive Virtual Dyno, Dragy & Tuning Inspector
├── FH6SmartShift — Dyno, Dragy & Gearbox Tuning Inspector.jpg          # Annotated Gears 1-10 Table, Dragy & Tuning Advisor screenshot
├── FH6SmartShift — Dyno, Dragy & Gearbox Tuning Inspector 2.jpg        # Annotated 401-Band Virtual Dyno Curve screenshot
├── FH6SmartShift_Dashboard_EN.jpg                                      # Annotated classic dashboard infographic
├── FH6SmartShift_Optimal_Layout_v1.1.jpg                               # Annotated v1.1 MoTeC-style dashboard layout
├── DashTemplates.zip                                                   # Classic SimHub dashboards (KM/H & MPH)
├── FH6SmartShift_Optimal.zip                                           # Optimal v1.1 MoTeC-style SimHub dashboard (1334x513)
├── README.md                                                           # Online repository documentation (v1.1.1)
└── README.pdf                                                          # Printable user manual & technical reference
