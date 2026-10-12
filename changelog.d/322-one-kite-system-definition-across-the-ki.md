<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: MIT
-->
### Added
- `system_definition(s)` builds a KiteGeometry `SystemDefinition` of a KPS3 or KPS4 model: its points, the springs it integrates as segments, the tether and the winch.
- `save_log(logger, system_definition(s), name)` stores that definition in the log, and `KiteGeometry.definition(load_log(name))` reads it back.
- KiteGeometry is re-exported.
