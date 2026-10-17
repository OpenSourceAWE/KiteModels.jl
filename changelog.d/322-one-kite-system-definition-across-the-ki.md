<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: MIT
-->
### Added
- `system_definition(s)` builds a KiteGeometry `SystemDefinition` of a KPS3 or KPS4 model: its points, the springs it integrates as segments, the tether and the winch. Each segment names a `linear` material that holds its density and Young's modulus. KPS4's kite springs are named `bridle_i` where they end at the KCU and `strut_i` where they join two kite points.
- `save_log(logger, system_definition(s), name)` stores that definition in the log, and `KiteGeometry.definition(load_log(name))` reads it back.
- KiteGeometry is re-exported.
