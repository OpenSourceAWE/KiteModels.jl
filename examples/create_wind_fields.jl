# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using Timers; tic()
using KiteModels

let
    set::Settings = deepcopy(load_settings("system.yaml"))
    am::AtmosphericModel = AtmosphericModel(set)
    new_windfields(am::AtmosphericModel; prn=true)
end
toc()
nothing