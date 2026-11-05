# FSAE-EV-LapSim

A MATLAB quasi-steady-state **lap-time and points simulator for an electric Formula Student / Formula Bharat car**, built by adapting an open-source Formula SAE lap simulator to an EV powertrain, Pacejka MF5.2 tires and the Kari Motor Speedway layout.

Developed for **SPCE Racing Electric** (Sardar Patel College of Engineering). Track layouts are Formula Bharat 2026 (Kari Motor Speedway); the vehicle data is from the team's **previous car** and should be updated for the current one.

> **This is a derivative work.** It is based on [Optimum-Mindstorm](https://github.com/OxxoCodes/Optimum-Mindstorm), a lap simulator written by **Jonathan Vogel** for **Clemson Formula SAE**, released under GPL-3.0. The solver architecture (g-g-V envelope, track discretisation, forward/backward velocity pass) is his. The EV adaptation, tire-model changes, vehicle and track data, scoring and energy analysis are described in [`CHANGES.md`](CHANGES.md). See [`NOTICE.md`](NOTICE.md) for full attribution.

---

## What it does

1. Builds a **g-g-V performance envelope** (acceleration, braking and cornering capacity versus speed) from the vehicle and tire model.
2. Discretises the **endurance** and **autocross** tracks from gate coordinates and drives the car around them at the limit of the envelope.
3. Computes **acceleration and skidpad** times directly from the envelope.
4. Converts event times to **Formula Bharat-style points**.
5. Estimates **energy consumption (kWh)** per lap and over the endurance distance, including a simple regen model.
6. Outputs velocity/acceleration traces, speed-coloured track maps, a 3D g-g-V surface and structural load cases.

## What changed relative to the upstream simulator

| Area | Upstream (Clemson, 2019, combustion) | This project (EV, Formula Bharat 2026) |
|---|---|---|
| Powertrain | Engine torque curve, 6-speed gearbox, shift logic | Motor torque-speed curve (212 points, 364-7500 rpm), single fixed ratio |
| Longitudinal tire | Spline fit to Hoosier R25B test data | Pacejka MF5.2 `Fx` (`MF52_Fx_fcn.m`) |
| Lateral tire | MF5.2 `Fy` | MF5.2 `Fy`, new coefficient set |
| Vehicle | Clemson 2019 car | SPCE previous car: mass, CG, wheelbase, track, LLTD, roll/pitch gradients, ride rates, camber/caster/KPI |
| Steering | Single Ackermann angle `delta = L/R` | Separate inner/outer wheel steer angles (fitted vs. radius), per-wheel slip angle and induced drag |
| Tracks | 2019 Michigan | Kari Motor Speedway endurance and autocross |
| Scoring | 2019 Michigan formulas | Formula Bharat-style scoring constants |
| Extras | none | Energy/regen analysis, 3D g-g-V plot, score summary |

Details, with before/after values, are in [`CHANGES.md`](CHANGES.md).

## Repository layout

```
Lap_Sim_EV.m                  Main script: run this
lap_information.m             Endurance lap integration
lap_information_sprint.m      Autocross lap integration
lap_time.m / lap_time_sprint.m  Objective functions for racing-line optimisation
track_curvature*.m            Track constraint functions for the optimiser
Powertrainlapsim.m            Motor + single-ratio drivetrain force model
MF52_Fx_fcn.m, MF52_Fy_fcn.m  Pacejka MF5.2 longitudinal / lateral force
A1654run21_MF52_*.mat         Tire coefficient files (Fx struct B; Fy vector A + scaling)
FB2026*CoordinatesNew.xlsx    Track gate coordinates (outside / inside edges)
arclength.m, curvature.m, circumcenter.m   Geometry helpers
tools/                        Scripts that generate the .mat coefficient files
docs/upstream/                Original documentation from the upstream project
```

The core files are kept in one flat folder on purpose: the main script loads data files by name, so everything needs to be on the MATLAB path together.

## Requirements

- MATLAB (developed and run in MATLAB; add the release you used here)
- Curve Fitting Toolbox (`csaps`, `fnval`)
- Optimization Toolbox, only if you re-enable racing-line optimisation (`fmincon`)

## Quick start

```matlab
cd FSAE-EV-LapSim
Lap_Sim_EV
```

The script prints a score summary and an energy summary to the command window and opens the plots. Generating the g-g-V envelope is the slow step.

## Key inputs (edit in `Lap_Sim_EV.m`)

| Section | What to change |
|---|---|
| 1 | Tire files, longitudinal/lateral friction scaling factors `sf_x`, `sf_y`, tire radius |
| 2 | Motor speed/torque vectors, gear ratio, drivetrain efficiency |
| 3-4 | Mass, weight distribution, CG height, wheelbase, track widths, LLTD, roll/pitch gradients, ride rates, camber, caster, KPI |
| 5 | Aero coefficients `Cl`, `Cd`, centre of pressure |
| 7, 12 | Track coordinate files |
| 17 | Event scoring constants |
| 24 | Rolling-resistance coefficient, regen efficiency |

## Vehicle parameters used (previous car)

| Parameter | Value |
|---|---|
| Mass (car + driver) | 738.5 lb (335 kg) |
| Front weight distribution | 48.51 % |
| CG height | 0.923 ft (281 mm) |
| Wheelbase | 5.151 ft (1570 mm) |
| Track (front / rear) | 4.078 ft / 3.937 ft (1243 mm / 1200 mm) |
| LLTD (front) | 54.56 % |
| Roll gradient (front / rear) | 0.427 / 0.640 deg/g |
| Pitch gradient | 0.597 deg/g |
| Ride rate (front / rear) | 227.6 / 145.2 lb/in |
| Drivetrain | Single ratio 8.5:1, 0.85 efficiency |
| Tire radius | 0.2286 m |
| Motor | 80.7 Nm peak torque, 364-7500 rpm |

The simulator works internally in imperial units (ft, lb, ft/s), as in the upstream code.

## Known limitations

This is a simplified steady-state model and has not been validated against logged car data.

- `sf_x = 0.6` and `sf_y = 0.47` are **inherited from the upstream project**, where they were tuned to Clemson's test data. They have not been re-correlated for this car.
- Aero coefficients `Cl` and `Cd` are **inherited from upstream**; only the centre of pressure was changed.
- The racing line is the **track centreline** (`0.5` at every gate). The optimiser is still present but disabled.
- Energy results depend on assumed values: rolling resistance `C_rr = 0.015`, regen capture efficiency 0.6, and a single 0.85 drivetrain efficiency.
- Event scoring constants (best times, maximum points, the endurance 1.5/22 scaling) should be re-checked against the current Formula Bharat rulebook before results are quoted.
- The upstream gear-shift logic still exists in the lap integration code. It is bypassed for a single ratio rather than removed.
- `lap_time_sprint.m` still uses the upstream `top_speed.top_speed` form. Apply the same one-line change made in `lap_time.m` before re-enabling autocross line optimisation.
- Vehicle parameters (mass, CG, geometry, gradients, ride rates, motor curve) describe the team's previous car. Update Sections 2-5 of `Lap_Sim_EV.m` for the current car before using results for design decisions.

## Data provenance

- Tire coefficient files `A1654run21_*` are carried over from the upstream repository; its `MF52_Fy_fcn.m` identifies the fitted data as Tire Testing Consortium data. The `Fx` structure and replacement `Fy` vector were generated for this project with the scripts in `tools/`.
- Vehicle parameters and motor curve are SPCE Racing Electric data for the previous car; track coordinates are Formula Bharat 2026 / Kari Motor Speedway.

## License

Because this project modifies GPL-3.0 software, it is distributed under the **GNU General Public License v3.0**. See [`LICENSE`](LICENSE). Original copyright remains with Jonathan Vogel for the upstream portions; modifications are by Branden Colaco, 2025-2026.

## Acknowledgements

- **Jonathan Vogel** and **Clemson Formula SAE** for the original lap simulator.
- **John D'Errico** for `arclength.m`.
- **SPCE Racing Electric** for vehicle, powertrain and track data.
