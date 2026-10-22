# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using KiteModels, KiteUtils, Test
using KiteModels: init!

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))

"The pair of point indices each segment of `definition` joins."
segment_points(definition) = [segment.points for segment in definition.segments]

"The `parameters` of the material `segment` of `definition` is made of."
parameters(definition, segment) = definition.materials[segment.material].parameters

"The axial stiffness times rest length [N] of `segment`, from its `linear` material."
unit_stiffness(definition, segment) =
    parameters(definition, segment)["youngs_modulus"] * π * segment.diameter^2 / 4

@testset "KPS4 definition holds the points and springs the model integrates" begin
    kps4 = KPS4(load_settings("system.yaml"))
    init!(kps4; prn=false)
    definition = system_definition(kps4)
    segments = kps4.set.segments
    @test definition.metadata.n_points == length(kps4.pos) == segments + 5
    @test segment_points(definition) == [(spring.p1, spring.p2) for spring in kps4.springs]
    @test [point.pos_ENU for point in definition.points] == kps4.pos
    @test [point.fixed for point in definition.points] == (1:length(kps4.pos) .== 1)
    @test [segment.rest_length for segment in definition.segments] ==
          [spring.length for spring in kps4.springs]
    @test only(definition.tethers).segments == 1:segments
    @test only(definition.tethers).end_point == segments + 1
    @test only(definition.winches).winch_point == 1
    @test [unit_stiffness(definition, segment) for segment in definition.segments] ≈
          [spring.axial_stiffness * spring.length for spring in kps4.springs]
    @test sum(point.extra_mass for point in definition.points) ≈
          kps4.set.mass + kps4.set.kcu_mass
    @test all(parameters(definition, segment)["density"] == 0
              for segment in definition.segments[(segments + 1):end])
    @test all(material.law == "linear" for material in definition.materials)
    @test definition.metadata.connectivity_sha == connectivity_sha(definition)
    kcu = segments + 1
    kite_springs = kps4.springs[(segments + 1):end]
    @test [segment.name for segment in definition.segments] ==
          [[Symbol("tether_$i") for i in 1:segments];
           [Symbol(kcu in (spring.p1, spring.p2) ? "bridle_$i" : "strut_$i")
            for (i, spring) in enumerate(kite_springs)]]
    @test count(startswith("strut") ∘ string,
                segment.name for segment in definition.segments) == 6
end

@testset "KPS3 definition is one tether from the winch to the kite" begin
    kps3 = KPS3(load_settings("system.yaml"))
    init!(kps3)
    definition = system_definition(kps3)
    segments = kps3.set.segments
    @test definition.metadata.n_points == length(kps3.pos) == segments + 1
    @test segment_points(definition) == [(i, i + 1) for i in 1:segments]
    @test only(definition.tethers).segments == 1:segments
    @test [segment.name for segment in definition.segments] ==
          [Symbol("tether_$i") for i in 1:segments]
    @test definition.points[end].extra_mass ≈ kps3.set.mass + kps3.set.kcu_mass
end

@testset "a saved log carries the KPS4 definition under topology" begin
    kps4 = KPS4(load_settings("system.yaml"))
    init!(kps4; prn=false)
    logger = Logger(length(kps4.pos), 1; segments=n_springs(kps4))
    log!(logger, SysState(kps4))
    path = mktempdir()
    definition = system_definition(kps4)
    save_log(logger, definition, "topology"; path)
    @test structure_document(KiteGeometry.definition(load_log("topology"; path))) ==
          structure_document(definition)
end
