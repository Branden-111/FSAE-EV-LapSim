# Attribution and third-party notices

## Upstream project

This repository is a modified version of **Optimum-Mindstorm**, a lap simulation package for Clemson University Formula SAE.

- Original author: **Jonathan Vogel** (Clemson Formula SAE), 2019
- Upstream repository: https://github.com/OxxoCodes/Optimum-Mindstorm
- License: GNU General Public License v3.0 (copy in `LICENSE`)

Files that originate upstream and are **unchanged**: `arclength.m`, `circumcenter.m`, `curvature.m`, `track_curvature.m`, `track_curvature_sprint.m`, `lap_time_sprint.m`, `MF52_Fy_fcn.m`, and everything in `docs/upstream/`.

Files that originate upstream and have been **modified** (each carries a modification notice at the top): `Lap_Sim_EV.m` (from `Lap_Sim.m`), `Powertrainlapsim.m`, `lap_time.m`, `lap_information.m`, `lap_information_sprint.m`.

Files that are **new** in this project: `MF52_Fx_fcn.m`, `A1654run21_MF52_Fx.mat`, `FB2026*CoordinatesNew.xlsx`, everything in `tools/`. The tire coefficient files `A1654run21_MF52_Fy_*.mat` exist upstream under the same names, with contents replaced here.

## Third-party code

- `arclength.m` - John D'Errico (MATLAB File Exchange, release 1.0, 2010). The author credit in the file header is retained. Refer to its File Exchange page for its license terms.

## Tire data

The upstream `MF52_Fy_fcn.m` describes its fit as being of tire data provided by the Tire Testing Consortium. Use of such data is governed by the consortium's terms. Anyone reusing the coefficient files should check those terms.

## Modifications

Modifications are by Branden Colaco, 2025-2026, for SPCE Racing Electric. A full list is in `CHANGES.md`, satisfying the GPL-3.0 requirement (section 5a) to state that the work was modified and when.
