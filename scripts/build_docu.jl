# SPDX-FileCopyrightText: 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

# Build and display the html documentation locally.

using Pkg

_docs_project = joinpath(@__DIR__, "..", "docs")
if !isfile(joinpath(dirname(Pkg.project().path), "make.jl"))
    Pkg.activate(_docs_project)
end
using LiveServer; servedocs(launch_browser=true)
