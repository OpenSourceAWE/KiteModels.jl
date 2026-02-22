# SPDX-FileCopyrightText: 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

# build and display the html documentation locally
# you must have installed the package LiveServer in your global environment

using Pkg

function globaldependencies()
    projectpath = Pkg.project().path
    basepath, _ = splitdir(projectpath)
    Pkg.activate()
    globaldependencies = keys(Pkg.project().dependencies)
    Pkg.activate(basepath)
    globaldependencies
end

if !("Documenter" ∈ keys(Pkg.project().dependencies))
    using Pkg
    Pkg.activate("docs")
end
using LiveServer; servedocs(launch_browser=true)
