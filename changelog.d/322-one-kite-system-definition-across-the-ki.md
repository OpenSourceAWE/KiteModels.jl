<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: MIT
-->
### Added
- `system_definition(s)` builds a KiteGeometry `SystemDefinition` of a KPS3 or KPS4 model: its points, the springs it integrates as segments, the tether and the winch.
- `topology_metadata(s)` carries that definition in a log: `save_log(logger, name; metadata)` writes it under the key `topology`.
- KiteGeometry is re-exported.
