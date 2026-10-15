<!--
SPDX-FileCopyrightText: 2026 Uwe Fechner
SPDX-License-Identifier: CC-BY-4.0
-->

# Code review of `src/`

Review of the files in `src/` at commit `6fd4c1fe` (2026-10-16), with a focus on the recent changes to `src/`. Findings 1–4 were checked against the code; the others were not individually verified.

## Correctness

1. **Fixed:** **Compression damping is scaled by the stiffness in KPS3** (`src/KPS3.jl:388`). In the compression branch of `calc_spring_force(::KPS3)`, the code computes `k2 * ((norm1 - L) + c * spring_vel)`, which multiplies the damping term by `0.05 * k`. It should almost certainly be `k2 * (norm1 - L) + c * spring_vel`, which is what KPS4 does. A slack segment closing at 1 m/s gets a damping force that is orders of magnitude too large. The bug dates back to at least v0.8, but `spring_forces` now also reports this value in `SysState.spring_force`. Fixing it may shift reference results.

2. **Kite-spring damping jumps at the unstressed length in KPS4** (`src/KPS4.jl:353-361`). A stretched kite spring uses `rel_damping * c`, a compressed one plain `c`. With `rel_damping = 6`, the force is discontinuous at `norm1 == l_0`, which hurts the step control of the DAE solver. The lower compression stiffness (`rel_compr_stiffness`) does not cause this: the elastic force is zero at `norm1 == l_0` in both branches, so it only has a kink (its slope changes from `k` to `0.25 * k`). The damping force at that point is `rel_damping * c * spring_vel` on one side and `c * spring_vel` on the other, so the force jumps by `5 * c * spring_vel` whenever a moving kite spring passes its unstressed length. This behaviour dates back to the original code, where the factor was hard-coded as `6.0` (made configurable in `bd96eb8b`); it was not introduced by the recent refactor into `calc_spring_force`. This may be intentional, but should be a deliberate choice.

3. **`ss.time` holds the start time of the step** (`src/KiteModels.jl:562`). `next_step!` sets `s.t_0 = integrator.t` before stepping, so `update_sys_state!` stamps the state at t + dt with time t, while the rates are computed with `integrator.t`. Logs plotted against time are shifted by one step. Either use `integrator.t`, or document the convention.

4. **`spring_forces(::KPS3)` mixes two states** (`src/KPS3.jl:650`). It takes positions from `s.pos`, which may hold the last trial state of the solver, and velocities from `integrator.u`. This is only consistent if `update_caches!` was called just before, as `update_sys_state!` does. A direct call of the exported function can return forces that belong to neither state.

5. **The accepted-state fix covers only KPS3** (`src/KiteModels.jl:987`). `next_step!` calls `update_kite_state!` only for KPS3. For KPS4, `calc_height(s)` still reads `s.pos` from the last trial residual, so the wind at the start of each step is evaluated at a trial height. A general fix is to refresh the state from `integrator.u` (or call `update_caches!`) for every model before `set_v_wind_ground!`.

## Cleanups

1. **Duplicated state-vector index layout** (`src/KPS3.jl:279`). `update_kite_state!`, `particle_velocities` and `update_acc_kite!` all compute indices like `3*(P-1) + 3*(i-2)` by hand. A change of the layout must be made in all three places. A shared helper would remove the duplication.

2. **Wasted work with `damping=false`** (`src/KPS3.jl:650`). `spring_forces(::KPS3; damping=false)`, used by `calc_pre_tension`, still computes all particle velocities and `spring_vel` for every segment and then discards them.

3. **Hard-coded constants** (`src/KPS4.jl:856`, `src/KPS4.jl:361`, `src/KPS3.jl:387`). The bridle breaking force `4000.0` N and the compression stiffness factors `0.1` (KPS4) and `0.05` (KPS3) are magic numbers. The coding style asks for named constants or values from the settings; the bridle limit in particular cannot be configured per kite.

4. **Type check instead of dispatch** (`src/KiteModels.jl:683`). The shared function `update_rates!` branches on `s isa KPS3` to update `turn_rates` and `orient_last`, fields that only KPS3 has. A small per-model method, with a no-op for KPS4, would be cleaner.

5. **Inconsistent accessor for the elevation** (`src/KPS3.jl:706`). `find_steady_state_inner` reads `s.set.elevations[1]`, while `steady_state_offsets`, `find_steady_state!` and `clear!` read `s.set.elevation`. Both give the same value today, but the steady-state code should use one accessor.
