<!--
SPDX-FileCopyrightText: 2026 Uwe Fechner, Bart van de Lint
SPDX-License-Identifier: MIT
-->
### Changed
- BREAKING: the quaternion written to `SysState` is `KA` (aft-right-up, against ENU),
  the convention KiteUtils 0.13 stores. The model itself is unchanged: `kite_ref_frame`
  and `calc_orient_quat` stay `KS`, and `update_sys_state!` converts at the boundary.
- BREAKING: `roll`, `pitch` and `yaw` are no longer written to `SysState`, which
  dropped them in KiteUtils 0.13. `orient_euler(s)` still reports them, against
  NED, and `euler_KS(ss.orient)` recovers them from a state or a log.
- `turn_rates` is `KA`, so the z component has the opposite sign to before.
- `calc_heading(s)` passes the quaternion rather than Euler angles, skipping a
  round trip through `quat2euler`. The angle is unchanged.
- Requires KiteUtils 0.13 and Julia 1.12 or 1.13: the WinchModels and AtmosphericModels
  releases that accept KiteUtils 0.13 no longer support Julia 1.11.
