# SPDX-FileCopyrightText: 2022, 2024, 2025, 2026 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if ! ("PackageCompiler" ∈ keys(Pkg.project().dependencies))
    @info "Installing PackageCompiler ..."
    Pkg.add("PackageCompiler")
end
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    @info "Installing MakieControlPlots ..."
    Pkg.add("MakieControlPlots")
end
@info "Loading packages ..."
using KiteUtils, KitePodModels, KiteModels, MakieControlPlots
using PackageCompiler

@info "Creating sysimage ..."
push!(LOAD_PATH,joinpath(pwd(),"src"))

using Base64

# Upper limit of RAM + pagefile in MB that the pagefile can grow to. A pagefile without a
# maximum ("System managed", or a setting with MaximumSize 0) grows up to 3 x RAM or 4 GB,
# whichever is larger, but at most 1/8 of the size of its volume.
const PAGEFILE_MAX_PS = raw"""
$cs = Get-CimInstance Win32_ComputerSystem
$ram = [math]::Floor($cs.TotalPhysicalMemory / 1MB)
function AutoMax($drive) {
    $vol = Get-CimInstance Win32_LogicalDisk -Filter "DeviceID='$drive'"
    [math]::Min([math]::Max(3 * $ram, 4096), [math]::Floor($vol.Size / 1MB / 8))
}
$max = 0
if ($cs.AutomaticManagedPagefile) {
    $max = AutoMax $env:SystemDrive
} else {
    foreach ($s in @(Get-CimInstance Win32_PageFileSetting)) {
        if ($s.MaximumSize -gt 0) { $max += $s.MaximumSize } else { $max += AutoMax $s.Name.Substring(0, 2) }
    }
}
$ram + $max
"""

function total_ram_swap_gb()
    if Sys.iswindows()
        # MEMORYSTATUSEX layout (x64): dwLength(u32), dwMemoryLoad(u32), then 7 x UInt64
        buf = zeros(UInt8, 64)
        unsafe_store!(Ptr{UInt32}(pointer(buf)), UInt32(64))
        ok = ccall((:GlobalMemoryStatusEx, "kernel32"), stdcall, Cint, (Ptr{UInt8},), buf)
        ok == 0 && error("GlobalMemoryStatusEx failed")
        # ullTotalPageFile: RAM + the pagefile at its current size, which may still grow
        current = unsafe_load(Ptr{UInt64}(pointer(buf) + 24)) / 1_073_741_824  # -> GiB
        max_gb = try
            # -EncodedCommand (Base64 of UTF-16LE) avoids quoting problems on the command line
            script = Base64.base64encode(reinterpret(UInt8, transcode(UInt16, PAGEFILE_MAX_PS)))
            parse(Float64, strip(read(`powershell.exe -NoProfile -EncodedCommand $script`, String))) / 1024
        catch e
            @warn "Could not determine the maximum pagefile size, using its current size" exception=e
            0.0
        end
        max(current, max_gb)
    elseif Sys.isapple()
        # macOS has no fixed swap: it creates swap files in the VM volume as needed, until
        # the disk is full, so count the free space of that volume minus 5 GB as swap
        vm = isdir("/System/Volumes/VM") ? "/System/Volumes/VM" : "/"
        swap = max(0, diskstat(vm).available - 5 * 1_073_741_824)
        (Sys.total_memory() + swap) / 1_073_741_824  # -> GiB
    else
        info = read("/proc/meminfo", String)
        memtotal = parse(Int, match(r"MemTotal:\s+(\d+)", info).captures[1])   # kB
        swaptotal = parse(Int, match(r"SwapTotal:\s+(\d+)", info).captures[1]) # kB
        (memtotal + swaptotal) / 1_048_576  # -> GiB
    end
end

# The rules for the memory checks and the number of image threads are documented in
# docs/sysimage.md
let total = total_ram_swap_gb()
    @info "Total RAM + swap: $(round(total; digits=1)) GB"
    # Measured peak demand with one image thread: about 26 GB, plus a few GB of margin
    if total < 30
        error("At least 30 GB of RAM + swap is needed to create a system image even with " *
              "one thread, but only $(round(total; digits=1)) GB is available. " *
              (Sys.iswindows() ? "Increase the maximum size of the pagefile and retry." :
               Sys.isapple()   ? "Free some disk space and retry." :
                                 "Increase your swap file and retry."))
    end

    # Set by the shell script if it chose the number of threads itself; otherwise a value
    # of JULIA_IMAGE_THREADS was set by the caller
    auto = get(ENV, "SYSIMAGE_THREADS_AUTO", "false") == "true"
    if haskey(ENV, "JULIA_IMAGE_THREADS") && !auto
        @info "JULIA_IMAGE_THREADS already set to $(ENV["JULIA_IMAGE_THREADS"]), leaving as is"
    elseif total < 34
        # Measured peak of RAM + swap with two threads: about 30 GB
        ENV["JULIA_IMAGE_THREADS"] = "1"
        @info "Setting JULIA_IMAGE_THREADS=1 based on $(round(total; digits=1)) GB total memory"
    elseif haskey(ENV, "JULIA_IMAGE_THREADS")
        @info "Using JULIA_IMAGE_THREADS=$(ENV["JULIA_IMAGE_THREADS"]) as chosen by the shell script"
    else
        # Measured peak of RAM + swap: about 26 GB with one thread, 30 GB with two, and
        # about the same up to 16 threads, so more than two threads cost no extra memory.
        # Use Julia's default of CPU_THREADS / 2, capped at 16, the most measured.
        threads = clamp(Sys.CPU_THREADS ÷ 2, 1, 16)
        ENV["JULIA_IMAGE_THREADS"] = string(threads)
        @info "Setting JULIA_IMAGE_THREADS=$threads based on $(round(total; digits=1)) GB total memory"
    end
end

GC.gc(true)
let mem = Sys.free_memory() / 1024^2
    @info "Free memory: $(round(mem; digits=1)) MB"
    @info "JULIA_IMAGE_THREADS: $(ENV["JULIA_IMAGE_THREADS"])"
end

PackageCompiler.create_sysimage(
    [:KiteUtils, :KitePodModels, :KiteModels, :MakieControlPlots];
    sysimage_path="kps-image_tmp.so",
    include_transitive_dependencies=true,
    precompile_execution_file=joinpath("test", "test_for_precompile.jl"),
    # PackageCompiler 2.4.1+ defaults to the multi-target CPU list of the Julia binary on
    # Julia 1.13+, which compiles every function several times and needs more than twice
    # the memory and time (JuliaLang/julia#63141); build for this machine only, as on 1.12
    cpu_target="native"
)
