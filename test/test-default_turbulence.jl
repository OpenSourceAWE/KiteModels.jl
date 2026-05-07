# SPDX-FileCopyrightText: 2022, 2024, 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if !("Test" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using Test, KiteModels, KiteUtils.YAML

function write_gui_default(path::AbstractString; turbulence=0.0)
    open(path, "w") do io
        println(io, "gui:")
        println(io, "    project: hydra20_600.yml")
        println(io, "    default_turbulence: $(turbulence)")
    end
end

function write_gui_without_turbulence(path::AbstractString)
    open(path, "w") do io
        println(io, "gui:")
        println(io, "    project: hydra20_600.yml")
    end
end

function write_project_config(path::AbstractString; use_turbulence=nothing)
    open(path, "w") do io
        println(io, "name: test")
        if !isnothing(use_turbulence)
            println(io, "overwrite:")
            println(io, "    use_turbulence: $(use_turbulence)")
        end
    end
end

_old_data_path = KiteUtils.get_data_path()
try
    @testset "default_turbulence config" begin
        mktempdir() do tmpdir
            KiteUtils.set_data_path(tmpdir)

            gui_yaml = joinpath(tmpdir, "gui.yaml")
            gui_yaml_default = gui_yaml * ".default"
            write_gui_default(gui_yaml_default; turbulence=0.15)

            @test !isfile(gui_yaml)
            @test KiteModels.get_default_turbulence() ≈ 0.15
            @test isfile(gui_yaml)

            dict = YAML.load_file(gui_yaml)
            @test dict["gui"]["project"] == "hydra20_600.yml"
            @test Float64(dict["gui"]["default_turbulence"]) ≈ 0.15

            @test KiteModels.set_default_turbulence(0.35) ≈ 0.35

            updated = YAML.load_file(gui_yaml)
            @test updated["gui"]["project"] == "hydra20_600.yml"
            @test Float64(updated["gui"]["default_turbulence"]) ≈ 0.35
            @test KiteModels.get_default_turbulence() ≈ 0.35

            rm(gui_yaml)
            write_gui_without_turbulence(gui_yaml)
            @test KiteModels.set_default_turbulence(0.45) ≈ 0.45

            inserted = YAML.load_file(gui_yaml)
            @test inserted["gui"]["project"] == "hydra20_600.yml"
            @test Float64(inserted["gui"]["default_turbulence"]) ≈ 0.45
            @test KiteModels.get_default_turbulence() ≈ 0.45
        end
    end
finally
    KiteUtils.set_data_path(_old_data_path)
end

nothing