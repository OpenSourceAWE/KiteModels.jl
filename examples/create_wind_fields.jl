 using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
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