# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

# Test the heading calculation for a kite at various elevations, azimuths,
# and pitch angles. The heading is computed from the kite's orientation as
# Euler angles, its elevation and its azimuth_east.

using KiteUtils
using KiteModels
using KitePodModels: KCU

using LinearAlgebra
using Test

using Pkg
if isfile(joinpath(@__DIR__, "Project.toml")) && dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
    using StaticArrays
end

function calc_heading2(s::KiteModels.AKM)
    calc_heading(orient_euler(s), calc_elevation(s), calc_azimuth_east(s))
end

"""
    create_kite_model(x, y, z, pos)

Create a kite model with a given kite reference frame and kite position.

x, y, z:    Kite reference frame in ENU coordinates
pos:        Kite position in ENU coordinates
"""
function create_kite_model(x, y, z, pos)
    kcu::KCU = KCU(se())
    s::KPS4 = KPS4(kcu)

    s.x = x
    s.y = y
    s.z = z

    s.pos[end-2][begin] = pos[begin]
    s.pos[end-2][begin+1] = pos[begin+1]
    s.pos[end-2][begin+2] = pos[begin+2]
    s
end

@testset verbose=true "Test heading..." begin

# Kite at an elevation of 45 degrees and with 0 roll pitch yaw and azimuth. Heading is then 180 degrees"
@testset "elevation 45" begin
    s = create_kite_model((0, 1, 0), (1, 0, 0), (0, 0, -1), # Orientation
                          (0, sqrt(2)/2, sqrt(2)/2))        # Pos ENU
    heading = rad2deg(calc_heading2(s))
    @test isapprox(heading,         180,    atol=1e-4, rtol=1e-4)
end
@testset "elevation 60" begin
    s = create_kite_model((0, 1, 0), (1, 0, 0), (0, 0, -1), # Orientation
                          (0, 0.5, sqrt(3)/2))              # Pos ENU
    heading = rad2deg(calc_heading2(s))
    @test isapprox(heading,         180,    atol=1e-4, rtol=1e-4)
end
# Kite at an elevation of -60 degrees and with 0 roll pitch yaw and azimuth. Heading is then 0 degrees"
@testset "elevation -60" begin
    s = create_kite_model((0, 1, 0), (1, 0, 0), (0, 0, -1), # Orientation
                          (0, 0.5, -sqrt(3)/2))             # Pos
    heading = rad2deg(calc_heading2(s))
    @test wrap2pi(deg2rad(heading)) ≈ 0     atol=deg2rad(1e-4)
end
# Kite at an azimuth of 45 degrees and with 0 roll pitch yaw and elevation. Heading is then 90 degrees"
@testset "azimuth 45" begin
    s = create_kite_model((0, 1, 0), (1, 0, 0), (0, 0, -1), # Orientation
                          (-sqrt(2)/2, sqrt(2)/2, 0))       # Pos
    heading = rad2deg(calc_heading2(s))
    @test isapprox(heading,         90,     atol=1e-4, rtol=1e-4)
end
# Kite at an azimuth of 60 degrees and with 0 roll pitch yaw and elevation. Heading is then 90 degrees"
@testset "azimuth 60" begin
    s = create_kite_model((0, 1, 0), (1, 0, 0), (0, 0, -1), # Orientation
                          (-sqrt(3)/2, 1/2, 0))             # Pos
    heading = rad2deg(calc_heading2(s))
    @test isapprox(heading,         90,     atol=1e-4, rtol=1e-4)
end
# Kite at an azimuth of -60 degrees and with 0 roll pitch yaw and elevation. Heading is then 270 degrees"
@testset "azimuth_north -60" begin
    s = create_kite_model((0, 1, 0), (1, 0, 0), (0, 0, -1), # Orientation
                           (sqrt(3)/2, 1/2, 0))             # Pos
    heading = rad2deg(calc_heading2(s))
    @test isapprox(heading,         270,    atol=1e-4, rtol=1e-4)
end
# Kite at an azimuth of 60 degrees and with 45 degrees pitch, 0 roll yaw and elevation. Heading is then 41 degrees"
@testset "pitch 45 azimuth_north 60" begin
    s = create_kite_model((0, sqrt(2)/2, sqrt(2)/2), (1, 0, 0), (0, sqrt(2)/2, -sqrt(2)/2),   # Orientation
                          (-sqrt(3)/2, 1/2, 0))                                               # Pos
    heading = rad2deg(calc_heading2(s))
    @test isapprox(heading,         40.8933946491309,    atol=1e-4, rtol=1e-4)
end
end
nothing
