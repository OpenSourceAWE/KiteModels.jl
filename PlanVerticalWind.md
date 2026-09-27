<!--
SPDX-FileCopyrightText: 2026 Uwe Fechner
SPDX-License-Identifier: MIT
-->

# Implement feature: vertical wind

## Goal

Allow a vertical wind component (updraft/downdraft) at the kite, so that the apparent wind gets a
nonzero z-velocity. Fully backward compatible: the default of `0.0` reproduces today's behavior
exactly.

## Original request

> I'm using KiteModels (v0.6.17, via a Python/Julia setup) and I'd like to inject a vertical wind
> component at the kite — e.g. to simulate updrafts/gusts with a nonzero z-velocity in the apparent
> wind. Right now `set_v_wind_ground!` forces the vertical component to zero. Would you be open to
> adding an optional keyword (default 0.0) that sets the vertical component, plumbed through
> `next_step!`? It's fully backward-compatible, and since `v_wind_kite` returns `s.v_wind`, the
> component would also show up in the logged state automatically.

The request is valid: `set_v_wind_ground!` rewrites `s.v_wind` at the start of every `next_step!`
(`src/KiteModels.jl:695`), so a caller cannot inject the component from outside — an API change is
required. The report is against v0.6.17; the relevant line still exists verbatim in 0.11.x, but a
turbulence branch was added above it in the meantime, which changes the implementation (see
decision 1).

## Design decisions

### 1. Add to, do not overwrite, the vertical component

`set_v_wind_ground!` (`src/KiteModels.jl:220-236`) has two branches. With `use_turbulence > 0` it
delegates to `AtmosphericModels.calc_turbulent_wind`, and the Mann field is a full three-component
field — its `w` component is passed straight through, so `s.v_wind[3]` is *already* nonzero there.
A plain `s.v_wind[3] = v_wind_vert` would therefore silently delete the vertical turbulence.

Use `s.v_wind[3] += v_wind_vert` after the `if`/`else`, so the value is a mean updraft superimposed
on whatever the wind model already produced. In the laminar branch the third component is zero, so
`+=` and `=` coincide there.

### 2. Store it in the model struct, don't thread a keyword through every call site

`set_v_wind_ground!` has six call sites — `next_step!` (`src/KiteModels.jl:695`), `init!`
(`src/KiteModels.jl:623`), `src/KPS3.jl:488`, `src/KPS3.jl:562`, `src/KPS4.jl:713` and
`src/KPS4.jl:763`. None of them are inside `residual!`, so either approach is technically safe, but
a keyword on `next_step!` alone would leave all the initialization and steady-state paths at zero
and require touching five more signatures to change that.

Instead, follow the pattern `next_step!` already uses for `set_speed`/`set_torque`/`bearing`: add a
defaulted field to both model structs, have `next_step!` write the keyword into it, and let
`set_v_wind_ground!` read it. Both structs are `@with_kw mutable struct`, so a defaulted field is
backward compatible.

### 3. Phase 1 is kite-only; the tether stays horizontal

The vertical component is applied to `s.v_wind` (the wind at the kite), not to `v_wind_tether`.
This is a deliberate, documented limitation, not an oversight:

- **KPS3** would honor a vertical `v_wind_tether`: it is written only in `clear!` (`src/KPS3.jl:154`)
  and in `set_v_wind_ground!`, and read by `calc_drag` (`src/KPS3.jl:207`).
- **KPS4** would not. `inner_loop!` recomputes `s.v_wind_tether` per spring from `s.v_wind_gnd`
  (`src/KPS4.jl:453`, called at `src/KPS4.jl:489`), and `v_wind_gnd` is horizontal by construction
  (`src/KiteModels.jl:224`). Anything written upstream is discarded. (The same overwrite already
  discards the *turbulent* tether wind for KPS4 — a pre-existing quirk, out of scope here.)

Applying it to KPS3 only would make the two models disagree. Phase 1 therefore leaves the tether
alone in both, and documents it. See "Possible phase 2" below.

### 4. Sign convention: positive is up

In this code `z` is up — `calc_height` is `pos_kite(s)[3]` and heights are positive. The NED frame
mentioned in the docs applies to *orientation*, not to these vectors, which invites the opposite
reading. The docstrings must state explicitly that a positive `v_wind_vert` is an updraft.

### 5. Leave `v_wind_gnd` horizontal

`upwind_dir(v_wind_gnd)` is an `atan` over components 1 and 2 (`src/KiteModels.jl:238-245`). The
ground wind vector must keep its `0.0` third component; only `s.v_wind` is touched.

## Implementation steps

1. **`src/KPS4.jl`** — add to the struct, next to `v_wind_tether`:
   ```julia
   "vertical wind velocity at the kite, positive up [m/s]"
   v_wind_vert::S = 0.0
   ```
2. **`src/KPS3.jl`** — the same field (note KPS3 is parameterized `{S, T, P}`, KPS4 is
   `{S, T, P, Q, SP}`; the field only needs `S`). Reset it in `clear!` alongside the other wind
   fields.
3. **`src/KiteModels.jl`, `set_v_wind_ground!`** — after the `if`/`else` and before `s.rho = ...`:
   ```julia
   s.v_wind[3] += s.v_wind_vert
   ```
   Update the docstring: new behavior, sign convention, and the note that the tether wind is
   unaffected.
4. **`src/KiteModels.jl`, `next_step!`** — add the keyword `v_wind_vert=0.0` and assign
   `s.v_wind_vert = v_wind_vert` next to the existing `s.sync_speed = set_speed` assignments, i.e.
   before the `set_v_wind_ground!` call. Document the keyword in the docstring's parameter list.
5. **Docs** — mention the keyword where `next_step!` and the wind model are described, and add a
   `CHANGELOG.md` entry under the next version.

Because the assignment happens once per `next_step!`, the updraft is constant across the step,
consistent with how `s.v_wind` is already handled.

## What comes for free

`v_wind_kite(s)` returns `s.v_wind` (`src/KiteModels.jl:211`), `update_sys_state!` copies it with
`ss.v_wind_kite .= s.v_wind` (`src/KiteModels.jl:552`), and KiteUtils logs `v_wind_kite` as a full
`MVector{3}` (`_logger.jl:58`, `_log.jl:48`). The vertical component therefore appears in the system
state and in saved logs with no further work — the original request is right about this.

The aerodynamics also pick it up directly: the kite-point apparent wind is `s.v_wind - v_*` at
`src/KPS4.jl:369` and `src/KPS3.jl:220`.

## Tests

Add to `test/test-kps4.jl` (and a KPS3 equivalent in `test/test-kps3.jl`):

1. Default `v_wind_vert` is `0.0` and `s.v_wind[3] == 0.0` — the backward-compatibility guarantee.
2. With `v_wind_vert = 1.0` and `use_turbulence == 0`, `s.v_wind[3] ≈ 1.0` after `next_step!`, and
   the horizontal components are unchanged.
3. **Regression for decision 1**: with `use_turbulence > 0`, `s.v_wind[3]` equals the turbulent
   vertical component *plus* `v_wind_vert`, not `v_wind_vert` alone.
4. `s.v_wind_gnd[3] == 0.0` and `upwind_dir(s)` is unaffected.
5. A short simulation with a nonzero updraft stays finite and the logged `v_wind_kite[3]` carries
   the value.

## Possible phase 2 (not in scope)

Extend the vertical component to the tether: set it in `v_wind_tether` for KPS3, and for KPS4 pass
it into `inner_loop!` so it survives the per-spring recomputation. Worth doing only if a use case
needs updrafts large enough to matter for tether drag; it also invites fixing the related quirk that
KPS4 discards the turbulent tether wind.
