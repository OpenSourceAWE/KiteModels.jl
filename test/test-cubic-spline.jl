# SPDX-FileCopyrightText: 2026 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end

using Dierckx, KiteModels, KitePodModels, KiteUtils, Test
using KiteModels: CubicSpline

# Points at which a CubicSpline is compared with the Dierckx.Spline1D it was created from:
# all knots, points close to the knots, points between the knots, both ends of the data and
# points outside of both ends.
function test_points(x)
    lo, hi = extrema(x)
    span = hi - lo
    pts = Float64[]
    append!(pts, x)
    for i in 1:length(x)-1
        append!(pts, range(x[i], x[i+1]; length=7))
    end
    for xi in x
        push!(pts, prevfloat(xi), nextfloat(xi), xi - 1e-9 * span, xi + 1e-9 * span)
    end
    append!(pts, [lo - 1e3 * span, lo - span, lo - 1e-3 * span, prevfloat(lo)])
    append!(pts, [nextfloat(hi), hi + 1e-3 * span, hi + span, hi + 1e3 * span])
    pts
end

# Compare a CubicSpline with the Dierckx spline it was created from, with a tolerance close to
# machine precision relative to the magnitude of the coefficients
function compare(spline::Spline1D, x)
    sp = CubicSpline(spline)
    tol = 1e-13 * max(1.0, maximum(abs, spline.c))
    for xi in vcat(test_points(x), spline.t)
        @test isapprox(sp(xi), spline(xi); atol=tol)
    end
    # beyond both ends, the value at the nearest end is returned
    @test sp(minimum(x) - 1.0e6) == sp(spline.t[1])
    @test sp(maximum(x) + 1.0e6) == sp(spline.t[end])
    sp
end

@testset "CubicSpline" begin
    set = load_settings("system.yaml")

    @testset "lift and drag coefficients from the settings" begin
        compare(Spline1D(set.alpha_cl, set.cl_list), set.alpha_cl)
        compare(Spline1D(set.alpha_cd, set.cd_list), set.alpha_cd)
    end

    @testset "minimal spline with four samples" begin
        x = [0.0, 1.0, 3.0, 4.5]
        y = [1.0, -2.0, 0.5, 3.0]
        spline = Spline1D(x, y)
        @test length(spline.t) == 8
        sp = compare(spline, x)
        # interpolating spline: exact at the samples
        for (xi, yi) in zip(x, y)
            @test isapprox(sp(xi), yi; atol=1e-13)
        end
    end

    @testset "non-uniform samples" begin
        for n in (5, 6, 11, 40)
            # irregular, but deterministic spacing and values
            x = cumsum(0.1 .+ abs.(sin.(1.7 .* (1:n)))) .* 10 .- 50
            y = 10 .* cos.(2.3 .* (1:n) .^ 1.3)
            compare(Spline1D(x, y), x)
        end
    end

    @testset "smoothing spline" begin
        x = collect(range(-10.0, 30.0; length=30))
        y = sin.(x ./ 5) .+ 0.05 .* sin.(7.1 .* x)
        spline = Spline1D(x, y; s=0.1)
        @test length(spline.t) < length(x) + 4
        compare(spline, x)
    end

    @testset "argument types" begin
        sp = CubicSpline(Spline1D(set.alpha_cl, set.cl_list))
        @test sp(5) === sp(5.0)
        @test sp(5.0f0) ≈ sp(5.0)
        @test sp(5.0) isa Float64
    end

    @testset "unsupported splines are rejected" begin
        x = collect(0.0:1.0:6.0)
        y = x .^ 2
        for k in (1, 2, 4, 5)
            @test_throws ArgumentError CubicSpline(Spline1D(x, y; k))
        end
        for bc in ("extrapolate", "zero", "error")
            @test_throws ArgumentError CubicSpline(Spline1D(x, y; bc))
        end
        @test CubicSpline(Spline1D(x, y; bc="nearest")) isa CubicSpline
    end

    @testset "conversion on assignment" begin
        kcu = KCU(set)
        cl_spline = Spline1D(set.alpha_cl, set.cl_list)
        cd_spline = Spline1D(set.alpha_cd, set.cd_list)
        for model in (KPS3, KPS4)
            s = model(kcu)
            @test s.calc_cl isa CubicSpline
            @test s.calc_cd isa CubicSpline
            # assign new splines; they must be converted to CubicSpline
            s.calc_cl = Spline1D(set.alpha_cl, 2 .* set.cl_list)
            s.calc_cd = Spline1D(set.alpha_cd, 3 .* set.cd_list)
            @test s.calc_cl isa CubicSpline
            @test s.calc_cd isa CubicSpline
            for alpha in (-5.0, 0.0, 7.5, 20.0)
                @test isapprox(s.calc_cl(alpha), 2 * cl_spline(alpha); atol=1e-12)
                @test isapprox(s.calc_cd(alpha), 3 * cd_spline(alpha); atol=1e-12)
            end
            @test_throws ArgumentError (s.calc_cl = Spline1D(set.alpha_cl, set.cl_list; k=2))
            @test_throws ArgumentError (s.calc_cd = Spline1D(set.alpha_cd, set.cd_list; bc="zero"))
        end
    end
end
nothing
