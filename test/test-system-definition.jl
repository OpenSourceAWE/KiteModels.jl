# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using KiteModels, KiteUtils, Test, YAML
using KiteModels: init!

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))

"The pair of point indices each segment of `definition` joins."
segment_points(definition) = [segment.points for segment in definition.segments]

@testset "KPS4 definition holds the points and springs the model integrates" begin
    kps4 = KPS4(load_settings("system.yaml"))
    integrator = init!(kps4; delta=0.001, prn=false)
    definition = system_definition(kps4)
    segments = kps4.set.segments
    @test definition.metadata.n_points == length(kps4.pos) == segments + 5
    @test [point.pos_ENU for point in definition.points] == kps4.pos
    @test [segment.l0 for segment in definition.segments] ==
          [spring.length for spring in kps4.springs]
    @test only(definition.tethers).segments == 1:segments
    @test only(definition.tethers).end_point == segments + 1
    @test only(definition.winches).winch_point == 1
    @test sum(point.extra_mass for point in definition.points) ≈
          kps4.set.mass + kps4.set.kcu_mass
    next_step!(kps4, integrator; set_speed=0, dt=1 / kps4.set.sample_freq)
    @test segment_points(definition) == [(spring.p1, spring.p2) for spring in kps4.springs]
end

@testset "KPS3 definition is one tether from the winch to the kite" begin
    kps3 = KPS3(load_settings("system.yaml"))
    init!(kps3; delta=0.001)
    definition = system_definition(kps3)
    segments = kps3.set.segments
    @test definition.metadata.n_points == length(kps3.pos) == segments + 1
    @test segment_points(definition) == [(i, i + 1) for i in 1:segments]
    @test only(definition.tethers).segments == 1:segments
    @test definition.points[end].extra_mass ≈ kps3.set.mass + kps3.set.kcu_mass
end

@testset "a saved log carries the KPS4 definition under topology" begin
    kps4 = KPS4(load_settings("system.yaml"))
    init!(kps4; delta=0.001, prn=false)
    logger = Logger(length(kps4.pos), 1)
    log!(logger, SysState(kps4))
    path = mktempdir()
    save_log(logger, "topology"; path, metadata=topology_metadata(kps4))
    document = YAML.load(load_log("topology"; path).metadata["topology"])
    @test structure_document(SystemDefinition(document)) ==
          structure_document(system_definition(kps4))
end
