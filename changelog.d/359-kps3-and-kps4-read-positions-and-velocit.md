### Changed
- BREAKING: KPS3 and KPS4 read the positions and velocities of the particles from the state of the integrator. After `next_step!`, `s.pos`, `s.vel`, `s.vel_kite` and every getter built on them (`pos_kite`, `calc_elevation`, `tether_length`, `SysState`, …) give the accepted state of the time step instead of the last state the solver tried. Before `init!` they give the state of `init`, `find_steady_state!` or the last call of `residual!`. `s.pos`, `s.vel` and `s.vel_kite` are read-only now; KPS3 gains `s.vel`.
### Fixed
- KPS3: the elevation of the kite used for the correction of the steering by gravity is that of the state the residual is evaluated for, not that of the previous call of the residual.
