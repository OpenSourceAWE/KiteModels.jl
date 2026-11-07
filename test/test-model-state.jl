# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using KiteModels, KiteUtils, LinearAlgebra, Test
using KiteModels: init!

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))

# point i of the particles of the state vector u, whose points start after offset
state_point(u, offset, i) = i == 1 ? zeros(3) : u[offset+3i-5:offset+3i-3]

@testset "the getters read the accepted state of the integrator, $Model" for Model in (KPS3, KPS4)
    set = deepcopy(load_settings("system.yaml"))
    s = Model(KCU(set))
    integrator = init!(s; prn=false)
    for _ in 1:5
        next_step!(s, integrator; set_speed=0, dt=1/set.sample_freq)
    end
    u = integrator.u
    P = length(s.pos)
    i_kite = Model == KPS4 ? P - 2 : P
    pos = [state_point(u, 0, i) for i in 1:P]
    vel = [state_point(u, 3 * (P - 1), i) for i in 1:P]
    @test s.pos == pos
    @test s.vel == vel
    @test s.vel_kite == vel[i_kite]
    @test pos_kite(s) == pos[i_kite]
    @test calc_elevation(s) == KiteUtils.calc_elevation(pos[i_kite])
    @test tether_length(s) ≈ sum(norm(pos[i+1] - pos[i]) for i in 1:set.segments)
end
