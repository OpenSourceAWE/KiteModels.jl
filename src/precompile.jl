# Copyright (c) 2024 Uwe Fechner, Bart van de Lint
# SPDX-License-Identifier: MIT

@setup_workload begin
    # Putting some things in `@setup_workload` instead of `@compile_workload` can reduce the size of the
    # precompile file and potentially make loading faster.
    path = dirname(pathof(@__MODULE__))
    set_data_path(joinpath(path, "..", "data"))

    set = load_settings("system.yaml")
    set.kcu_diameter = 0.0
    set4 = deepcopy(set)
    kps4_::KPS4 = KPS4(KCU(set4))
    kps3_::KPS3 = KPS3(KCU(set))
    ver = "$(VERSION.major).$(VERSION.minor)_"

    @assert ! isnothing(kps4_.wm)
    @compile_workload begin
        # all calls in this block will be precompiled, regardless of whether(
        # they belong to your package or not (on Julia 1.8 and higher)
        integrator = KiteModels.init!(kps3_; delta=0.001, stiffness_factor=0.035, prn=false)
        integrator = KiteModels.init!(kps4_; delta=0.001, stiffness_factor=0.035, prn=false)
        nothing
    end
end