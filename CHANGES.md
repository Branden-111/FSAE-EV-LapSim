# Changes from upstream (Optimum-Mindstorm)

**Upstream:** https://github.com/OxxoCodes/Optimum-Mindstorm - Jonathan Vogel, Clemson Formula SAE, GPL-3.0
**This work:** Branden Colaco, SPCE Racing Electric, 2025-2026

This document lists what was modified, added or left unchanged relative to the upstream snapshot, so the extent of the modifications is clear. The upstream main script `Lap_Sim.m` became `Lap_Sim_EV.m`.

Values are shown as *upstream -> this project*. Units follow the code (ft, lb, ft/s unless stated).

---

## 1. Powertrain: combustion -> electric

| Item | Change |
|---|---|
| Torque curve | `engineSpeed`/`engineTq`: 80-point engine curve (6200-14100 rpm, about 32-54 Nm) -> 212-point motor curve (364-7500 rpm, 80.7 Nm peak, about 32 kW peak at about 4180 rpm) |
| Gearing | Six gears `[33/12 ... 29/24]`, primary `76/36`, final `40/12` -> single ratio `gear = 8.5`, `primaryReduction = 1`, `finalDrive = 1` |
| Differential lock | `T_lock` 90 -> 100 |
| `Powertrainlapsim.m` | Gear-selection `while` loop disabled; `gearSelect = 1`; starting rpm 15000 -> 7000; rpm clamped to 6900 if above 7000 (motor table limit); tire radius fixed at `0.2286` m instead of derived from `tire_radius` |
| `lap_time.m` | `VMAX = top_speed.top_speed` -> `VMAX = top_speed`; shift-time error term `err` commented out |

The upstream shift logic in `lap_information*.m` still exists; with one ratio it is bypassed rather than removed.

## 2. Tire models: both Pacejka MF5.2

| Item | Change |
|---|---|
| Longitudinal force | Spline (`csaps`) fit to Hoosier R25B test data evaluated with `fnval(..., full_send_x)` -> new function `MF52_Fx_fcn.m` driven by new coefficient structure `B` in `A1654run21_MF52_Fx.mat` (`FZ0 = 1080`) |
| Lateral force | `MF52_Fy_fcn.m` unchanged; coefficient vector `A` in `A1654run21_MF52_Fy_12.mat` replaced with a new fit; `FZ0` in `A1654run21_MF52_Fy_GV12.mat` -150 -> -1080 |
| Friction scaling | `sf_x = 0.6`, `sf_y = 0.47` **unchanged from upstream** |
| Generation scripts | `tools/createLongitudinalMatFile*.m`, `createTireParamMatFile.m`, `create_A_mat_file.m` build the `.mat` files |

## 3. Vehicle parameters (previous SPCE car)

| Parameter | Upstream | This project |
|---|---|---|
| Weight `W` | 660 lb | 738.549 lb |
| Front weight distribution | 50 % | 48.51 % |
| LLTD (front) | 51.5 % | 54.56 % |
| CG height | 13.2 in | 0.9229 ft |
| Wheelbase | 60.5 in | 5.151 ft |
| Front / rear track | 46 / 44 in | 4.078 / 3.937 ft |
| Roll gradient front / rear | 0 / 0 | 0.4268 / 0.6402 deg/g |
| Pitch gradient | 0 | 0.597 deg/g |
| Ride rate front / rear | 180 / 180 lb/in | 227.6 / 145.2 lb/in |
| Camber compensation front / rear | 10 / 20 | -2.390 / -0.318 |
| Caster front / rear | 0 / 4.1568 deg | 2.03 / 2.06 deg |
| KPI front / rear | 0 / 0 deg | 7.87 / 7.12 deg |
| Centre of pressure (front) | 48 % | 48.51 % |
| `track_width` | not set in main script | 9.842 |
| Aero `Cl`, `Cd` | 0.0418, 0.0184 | **unchanged** |

## 4. Steering and cornering model

- Upstream used a single Ackermann angle `delta = l/R`. This project uses **separate inner and outer front-wheel steer angles** (`theta_i`, `theta_o`) from power-law fits against turn radius (`a_i, p_i, c_i`, `a_o, p_o`).
- Front slip angle is now per wheel (`a_fi`, `a_fo`); front camber from kingpin/caster uses the per-wheel steer angle.
- Induced drag from front-tire lateral force is computed per wheel instead of from one mean angle.
- Yaw-balance iteration steps `theta_i`, `theta_o` together (`dthetai`, `dthetao`) rather than `delta`.
- Iteration guards (`bran` counter with `break` at 1000/2000/5000/6000 iterations) added to the cornering solver loops to prevent non-terminating loops.

## 5. g-g-V grid

| | Upstream | This project |
|---|---|---|
| Acceleration speeds | 15:5:130 ft/s | 1:5:90 ft/s |
| Braking speeds | 15:5:130 ft/s | 15:5:80 ft/s |
| Corner radii | 15:10:155 ft | 13:10:155 ft |

## 6. Tracks

- Endurance: `Endurance_Coordinates_1.xlsx` (2019 Michigan) -> `FB2026EnduranceCoordinatesNew.xlsx` (Kari Motor Speedway).
- Autocross: `Autocross_Coordinates_2.xlsx` -> `FB2026AutoCrossCoordinatesNew.xlsx`.
- Preloaded Michigan racing lines (`endurance_racing_line.mat`, `autocross_racing_line.mat`) replaced by a centreline vector `ones(1,150)*0.5`. The optimiser sections remain, commented out.

## 7. Scoring (Section 17)

Michigan 2019 formulas replaced with Formula Bharat-style constants and clamping:

| Event | Max pts | Fastest time | Slowest-time factor |
|---|---|---|---|
| Endurance | 325 | 2054 s | 1.1 |
| Autocross | 100 | 121.303 s | 1.25 |
| Skidpad | 75 | 6.91 s | 1.25 |
| Acceleration | 75 | 5.5 s | 1.5 |

Endurance time is extrapolated as `lap_time / 1.5 * 22`. A typo where acceleration fell back to the skidpad maximum was corrected (`Pmaxsk` -> `Pmaxac`).

## 8. New analysis and output

- **Section 20:** 3D g-g-V surface plot.
- **Sections 21-22:** yaw-balance and slip-angle plots (commented, available to enable).
- **Section 23:** formatted score summary.
- **Section 24:** energy consumption. Integrates tractive force (inertia + aero drag + rolling resistance) over distance, applies drivetrain efficiency when motoring and a regen factor of 0.6 when braking, reports lap kWh, average kW, extrapolated 22 km endurance kWh and a power trace.
- **Section 18:** `VX_min(1)`, `VX_max(1)` added so load-case extraction works when several samples share the extreme value.
- `lap_information.m`: bounds-checked velocity colouring in the track plot; plot titles updated for FB 2026.

## 9. Housekeeping

- Fixed a stray token (`F` glued to a comment) in the induced-drag block of the first cornering loop.
- Removed leftover edit notes and stray comment text.
- Modification notices added to modified upstream files (GPL-3.0 section 5a).
- Original author banner restored in `Lap_Sim_EV.m`.

## Not carried over

Upstream files not used by `Lap_Sim_EV.m` and omitted from this repository: Michigan track spreadsheets and racing lines, Hoosier FX/FY `.mat` files, `Lap_Sim_CSAPS.m`, `myvars.mat`, and the original `Lap_Sim.m`. They remain available in the upstream repository.

## Development history (not included)

Intermediate working versions were consolidated into `Lap_Sim_EV.m`: `Lap_Sim.m` (initial vehicle data), `Lap_SimChanges.m` (split inner/outer steer), `Lap_Sim_Combined_Slip.m`, `Lap_Sim_SlipNew.m` (experimental inline brake-disc model, since removed), `Lap_sim_EnergyCalc.m` (energy analysis), `LapsimNewMotor.m` (alternative motor: 100 Nm plateau, ratio 5.18) and `Lap_Sim_Latest.m` (final base).
