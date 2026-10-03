<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: MIT
-->
### Changed
- `bin/install` only installs: it copies `Manifest-v<major>.toml.default`, instantiates and precompiles with the Julia on the PATH. It no longer runs the tests, resolves, calls `juliaup add`/`juliaup default`, adds Revise to the global environment or appends an alias to `~/.bashrc`; select another Julia with `JULIAUP_CHANNEL`.
- `bin/update_default_manifest` updates the manifest of the Julia on the PATH instead of switching the `juliaup` default.
