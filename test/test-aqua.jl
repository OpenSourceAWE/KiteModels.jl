# SPDX-FileCopyrightText: 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end

using Aqua, KiteModels, Test
@testset "Aqua.jl" begin
    Aqua.test_all(
      KiteModels;
      stale_deps=(ignore=[:PyCall, :CodecXz, :REPL],), # CodecXz is used during precompilation only
      deps_compat=(ignore=[:PyCall],),                 # PyCall is needed for CI to recompile Python
      piracies=false,                                  # the norm function is doing piracy for performance reasons
      persistent_tasks=false                           # disable persistent tasks test due to Julia 1.10 issues
    )
end
nothing
