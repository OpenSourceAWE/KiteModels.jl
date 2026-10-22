# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    system_definition(s::KPS4)

The `SystemDefinition` of the four-point model: its points where they are now, and as
segments the springs it integrates, at full stiffness. Segments are named `tether_i`, and
the massless kite springs `bridle_i` where they end at the KCU and `strut_i` where they join
two kite points. Call it after `init!` for the initial pose.
"""
function system_definition(s::KPS4)
    tether_segments = s.set.segments
    extra_masses = zeros(length(s.pos))
    extra_masses[tether_segments + 1] = s.set.kcu_mass
    extra_masses[(tether_segments + 2):end] .= s.masses[(tether_segments + 2):end]
    materials = Material[]
    segments = Segment[]
    for (i, spring) in enumerate(s.springs)
        in_tether = i <= tether_segments
        diameter = (in_tether ? s.set.d_tether : s.set.d_line) / 1000
        density = in_tether ? s.set.rho_tether : 0.0
        material = line_material!(materials, in_tether ? "tether" : "kite_line", density,
                                  spring.axial_stiffness * spring.length, diameter)
        push!(segments, Segment(; name=spring_name(s, i, spring),
                                points=(Int(spring.p1), Int(spring.p2)),
                                rest_length=spring.length, diameter, material))
    end
    return assemble_system_definition(s, materials, segments, extra_masses)
end

"""
    spring_name(s::KPS4, i, spring)

The segment name of `spring`, the `i`-th of `s.springs`: `tether_i` for a tether spring,
then, numbered from 1, `bridle_j` for a kite spring ending at the KCU and `strut_j` for one
joining two kite points.
"""
function spring_name(s::KPS4, i, spring)
    tether_segments = s.set.segments
    i <= tether_segments && return "tether_$i"
    kcu = tether_segments + 1
    role = kcu in (spring.p1, spring.p2) ? "bridle" : "strut"
    return "$(role)_$(i - tether_segments)"
end

"""
    system_definition(s::KPS3)

The `SystemDefinition` of the one-point model: its points where they are now, the tether
segments between them at full stiffness, and the kite and KCU mass on the last point. Call
it after `init!` for the initial pose.
"""
function system_definition(s::KPS3)
    extra_masses = zeros(length(s.pos))
    extra_masses[end] = s.set.mass + s.set.kcu_mass
    diameter = s.set.d_tether / 1000
    materials = Material[]
    material = line_material!(materials, "tether", s.set.rho_tether,
                              s.axial_stiffness * s.segment_length, diameter)
    segments = [Segment(; name="tether_$i", points=(i, i + 1), rest_length=s.segment_length,
                        diameter, material)
                for i in 1:s.set.segments]
    return assemble_system_definition(s, materials, segments, extra_masses)
end

"""
    line_material!(materials, name, density, unit_stiffness, diameter)

The name of the `linear` material among `materials` of `density` [kg/m³] that gives a
line of `diameter` [m] the axial stiffness times rest length `unit_stiffness` [N], added
as `name` where none matches.
"""
function line_material!(materials, name, density, unit_stiffness, diameter)
    area = π * diameter^2 / 4
    parameters = Dict{String, Any}("density" => density,
                                   "youngs_modulus" => unit_stiffness / area)
    for material in materials
        material.parameters == parameters && return material.name
    end
    push!(materials, Material(; name, law="linear", parameters))
    return name
end

"""
    assemble_system_definition(s::AbstractKiteModel, materials, segments, extra_masses)

The `SystemDefinition` of `s` with its `segments`, made of `materials`, the first
`s.set.segments` of which are the tether from the winch at point 1 to point
`s.set.segments + 1`.
"""
function assemble_system_definition(s::AbstractKiteModel, materials, segments, extra_masses)
    n_points = length(s.pos)
    tether_segments = s.set.segments
    points = [Point(; name=string(i), fixed=i == 1, pos_ENU=s.pos[i],
                    extra_mass=extra_masses[i])
              for i in 1:n_points]
    tether = Tether(; name="tether", start_point=1, end_point=tether_segments + 1,
                    segments=collect(1:tether_segments))
    winch = Winch(; name="winch", tethers=[1], winch_point=1, gear_ratio=s.set.gear_ratio,
                  drum_radius=s.set.drum_radius)
    tables = (; materials, points, segments, tethers=[tether], winches=[winch])
    name = string(nameof(typeof(s)))
    draft = SystemDefinition(; metadata=Metadata(; name, n_points), tables...)
    metadata = Metadata(; name, n_points, connectivity_sha=connectivity_sha(draft))
    return SystemDefinition(; metadata, tables...)
end
