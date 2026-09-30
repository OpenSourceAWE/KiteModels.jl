# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    system_definition(s::KPS4)

The `SystemDefinition` of the four-point model: its points where they are now, and as
segments the tether and bridle springs it integrates. Call it after `init!` for the
initial pose.
"""
function system_definition(s::KPS4)
    tether_segments = s.set.segments
    extra_masses = zeros(length(s.pos))
    extra_masses[tether_segments + 1] = s.set.kcu_mass
    extra_masses[(tether_segments + 2):end] .= s.masses[(tether_segments + 2):end]
    bridle_segments = length(s.springs) - tether_segments
    diameters = [fill(s.set.d_tether, tether_segments); fill(s.set.d_line, bridle_segments)]
    segments = [Segment(; name=string(i), points=(Int(spring.p1), Int(spring.p2)),
                        l0=spring.length, diameter=diameters[i] / 1000,
                        density=s.set.rho_tether,
                        unit_stiffness=spring.axial_stiffness * spring.length)
                for (i, spring) in enumerate(s.springs)]
    return system_definition(s, segments, extra_masses)
end

"""
    system_definition(s::KPS3)

The `SystemDefinition` of the one-point model: its points where they are now, the tether
segments between them, and the kite and KCU mass on the last point. Call it after `init!`
for the initial pose.
"""
function system_definition(s::KPS3)
    extra_masses = zeros(length(s.pos))
    extra_masses[end] = s.set.mass + s.set.kcu_mass
    segments = [Segment(; name=string(i), points=(i, i + 1), l0=s.segment_length,
                        diameter=s.set.d_tether / 1000, density=s.set.rho_tether,
                        unit_stiffness=s.axial_stiffness * s.segment_length)
                for i in 1:s.set.segments]
    return system_definition(s, segments, extra_masses)
end

"""
    system_definition(s::AbstractKiteModel, segments, extra_masses)

The `SystemDefinition` of `s` with its `segments`, the first `s.set.segments` of which are
the tether from the winch at point 1 to point `s.set.segments + 1`.
"""
function system_definition(s::AbstractKiteModel, segments, extra_masses)
    n_points = length(s.pos)
    tether_segments = s.set.segments
    metadata = Metadata(string(nameof(typeof(s))), "", "", "1.0.0", "structure_schema.yml",
                        n_points, "")
    points = [Point(; name=string(i), type=i == 1 ? STATIC : DYNAMIC, pos_ENU=s.pos[i],
                    extra_mass=extra_masses[i])
              for i in 1:n_points]
    tether = Tether(; name="tether", start_point=1, end_point=tether_segments + 1,
                    segments=collect(1:tether_segments))
    winch = Winch(; name="winch", tethers=[1], winch_point=1, gear_ratio=s.set.gear_ratio,
                  drum_radius=s.set.drum_radius)
    return SystemDefinition(; metadata, points, segments, tethers=[tether], winches=[winch])
end

"""
    topology_metadata(s::AbstractKiteModel)

The table metadata that carries the structure document of `system_definition(s)` in a
log, under the key `topology`: pass it as `save_log(logger, name; metadata)`.
"""
function topology_metadata(s::AbstractKiteModel)
    return Dict("topology" => YAML.write(structure_document(system_definition(s))))
end
