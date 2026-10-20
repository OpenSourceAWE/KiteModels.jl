<!--
SPDX-FileCopyrightText: 2026 Uwe Fechner
SPDX-License-Identifier: MIT
-->

# System image build: memory and thread rules

This file describes how the system image build checks memory and chooses the number of
compilation threads (`JULIA_IMAGE_THREADS`). It is the
specification: change the rules here first, then update the scripts to match.

## Scripts

| Shell script            | Julia script                | Used for                                                                                           |
|-------------------------|-----------------------------|----------------------------------------------------------------------------------------------------|
| `bin/create_sys_image`  | `test/create_sys_image.jl`  | this repository                                                                                    |
| `bin/create_sys_image2` | `test/create_sys_image2.jl` | user projects; `copy_bin()` installs them as `bin/create_sys_image` and `test/create_sys_image.jl` |

The memory and thread code of each pair must be identical. The shell script runs first and
starts Julia with the Julia script.

## Definitions

- **RAM**: physical memory.
- **Swap**:
  - Linux: `SwapTotal` from `/proc/meminfo`.
  - Windows: the size the pagefile can grow to. For a pagefile without a maximum ("System
    managed", or `MaximumSize` 0) Windows allows up to max(3 × RAM, 4 GB), but at most 1/8 of
    the volume. If this can't be determined, the current pagefile size is used.
  - macOS: the free space of the volume macOS creates its swap files on
    (`/System/Volumes/VM`, or `/` if that doesn't exist) minus 5 GB, at least 0.
- **Total**: RAM + swap, in GB.
- **GB**: 1024³ bytes. The shell scripts work in MB (1024² bytes); a 16 GB machine reports
  a little less than 16384 MB, so "16 GB" is tested as 15000 MB.

## Rules

### 1. Shell script

1. Linux only, before doing anything else: if the total < 30 GB (30720 MB), stop with an
   error ("Increase your swap file and retry"). On Windows and macOS the swap can grow, so
   the Julia script checks this.
2. If `JULIA_IMAGE_THREADS` is already set, keep it in all later steps.
3. Read RAM (MB):
   - Linux: `MemTotal` from `/proc/meminfo`
   - macOS: `sysctl -n hw.memsize`
   - Windows: `Win32_ComputerSystem.TotalPhysicalMemory`
4. Minimum RAM: 15 GB (15000 MB) on Linux and Windows, 7 GB (7000 MB) on macOS, so that
   Macs with 8 GB of RAM can build the image. Below that, stop with an error, before
   instantiating or precompiling anything. If RAM can't be read, warn and continue.
5. If `JULIA_IMAGE_THREADS` is not set, on all platforms:
   - RAM < 27 GB (26674 MB on Linux, 27000 MB on macOS and Windows) → 2 threads, or 1 if the
     machine has fewer than 4 CPU threads (`getconf _NPROCESSORS_ONLN`).
   - RAM unknown → 1 thread.

### 2. Julia script

1. Compute the total (RAM + swap) as defined above.
2. Total < 30 GB: stop with an error on all platforms. The message says what to increase:
   the pagefile (Windows), free disk space (macOS) or the swap file (Linux).
3. 30 GB ≤ total < 34 GB: use 1 thread on all platforms, also if the shell script chose 2.
   A value set by the caller is kept.
4. Otherwise, if `JULIA_IMAGE_THREADS` is set (by the caller or by the shell script), keep it.
5. Otherwise (total ≥ 34 GB), on all platforms: `clamp(Sys.CPU_THREADS ÷ 2, 1, 16)` threads.

### Resulting behaviour

Without `JULIA_IMAGE_THREADS` set by the caller:

| Platform       | Condition                                 | Result                                      |
|----------------|-------------------------------------------|---------------------------------------------|
| Linux, Windows | RAM < 15 GB                               | error                                       |
| macOS          | RAM < 7 GB                                | error                                       |
| all            | total < 30 GB                             | error                                       |
| all            | 30 GB ≤ total < 34 GB                     | 1 thread                                    |
| all            | RAM ≥ minimum, RAM < 27 GB, total ≥ 34 GB | 2 threads (1 with fewer than 4 CPU threads) |
| all            | RAM ≥ 27 GB, total ≥ 34 GB                | half the CPU threads, at most 16            |

## Where the numbers come from

Peak memory demand (RAM used + swap used, without counting zram pages twice, including the
operating system) from the AWEMeta.jl GitHub workflows, on GitHub runners with 16 GB of RAM
and 36 GB of swap (zram + swap file), 4 CPU threads:

| Image                                 | Workflow | Julia  | Threads | Peak (GB)  |
|---------------------------------------|----------|--------|---------|------------|
| KiteModels#main (`create_sys_image`)  | KM2      | 1.13.2 | 1       | 25.6, 26.3 |
| KiteModels#main (`create_sys_image2`) | KM3      | 1.13.2 | 1       | 25.0, 24.8 |
| KiteModels release                    | KM4      | 1.13.2 | 1       | 25.6, 24.9 |
| KiteModels release                    | KM4      | 1.12.7 | 1       | 22.6, 22.2 |
| Tethers#main                          | TE1      | 1.13.2 | 1       | 25.1, 23.8 |

(Runs of 2026-10-19 and 2026-10-20; GB = 1024 MB.)

Tethers.jl also measured (AWEMeta TE2/TE3 and a local run) about 30 GB with two image
threads, and about the same up to 16 threads. More than two threads cost no extra memory, so
the thread count doesn't grow with the memory.

- With one thread the KiteModels.jl image needs about the same as the Tethers.jl image: at
  most 26.3 GB, on Julia 1.13. Julia 1.12 needs about 3 GB less.
- 30 GB minimum: the one-thread peak plus a few GB of margin. For KiteModels.jl the margin is
  about 3.7 GB.
- There are no measurements of the KiteModels.jl image with more than one thread: the shell
  script forced one thread on the 16 GB runners.

No GC heap-size hint is set. `--heap-size-hint` on the command line of the shell script
would limit only the Julia process that drives PackageCompiler, not the processes it starts
for the precompile script and for compiling the image, where the peak memory demand occurs:
child processes don't inherit the flag (checked on Julia 1.12.7 and 1.13.2). The environment
variable `JULIA_HEAP_SIZE_HINT` would be inherited, but its effect on the build hasn't been
measured.
