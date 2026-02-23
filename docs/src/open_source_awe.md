```{=latex}
\begin{center}
\begingroup
\Large\bfseries\linespread{1.2}\selectfont
KiteModels.jl as example for an OpenSourceAWE project - a workshop\par
\endgroup
\end{center}
```

## Introduction
For the progress of airborne wind energy report it is important that new researchers can build on top of the results
of previous researchers. While scientific publications serve this purpose, it is also very beneficial if software
is available that implements existing models and controllers. The purpose of the [OpenSourceAWE](https://github.com/OpenSourceAWE) organization is to collect such open source software and make it available in a well documented, modular
and maintainable way.

In this workshop we will have a deeper look at one of the core OpenSourceAWE packages, the [KiteModels.jl](https://github.com/OpenSourceAWE/KiteModels.jl) package. It provides a one-point and a four-point kite power system model with a segmented tether and two different type of ground stations [@Fechner2015]. The package provides a large number of examples. Running these examples does not require any programming knowledge.

The goal of the workshop is, that you get an overview which open source software for airborne wind energy systems exist,
and how you can contribute to this software ecosystem.

## Preparation
If possible, try to do the following steps before attending the workshop:

- Make sure you have the software "Git" installed. You can download it from the [Git website](https://git-scm.com/install/windows). During installation, select VSCode (or your preferred editor) as editor and select bash as your preferred terminal.
- Install [Julia 1.11 and VSCode](https://ufechner7.github.io/2024/08/09/installing-julia-with-juliaup.html). VSCode is optional, but it is nice to read the examples and the configuration files. You can also use any other IDE or editor if you prefer something else.

## Workshop
### Install KiteModels.jl from git
On Linux, make sure that Python3 and Matplotlib are installed:

```bash
sudo apt install python3-matplotlib
```

Make sure that `ControlPlots.jl` works as explained in the [installation instructions](https://github.com/aenarete/ControlPlots.jl?tab=readme-ov-file#installation).

Before installing `KiteModels.jl` it is suggested to create a new folder, for example like this:

```bash
mkdir awe
cd awe
```

Then add KiteModels from git:

```bash
git clone https://github.com/OpenSourceAWE/KiteModels.jl.git
cd KiteModels.jl
```

Then, run the install script:

```bash
cd bin
./install
cd ..
```

You can now launch Julia:

```bash
./bin/run_julia
```

and run one of the two example menus:

```julia
menu() # or menu2()
```

### The examples
The following examples are provided when you type `menu()`:

```julia
"bench = include(\"bench.jl\")"
"bench_4p = include(\"bench_4p.jl\")"
"test_init_1p = include(\"test_init_1p.jl\")"
"test_init_4p = include(\"test_init_4p.jl\")"
"plot_cl_cd_plate = include(\"plot_cl_cd_plate.jl\")"
"plot_side_cl = include(\"plot_side_cl.jl\")"
"compare_kps3_kps4 = include(\"compare_kps3_kps4.jl\")"
"reel_out_1p = include(\"reel_out_1p.jl\")"
"reel_out_4p = include(\"reel_out_4p.jl\")"
"reel_out_4p_torque_control = include(\"reel_out_4p_torque_control.jl\")"
"simulate_simple = include(\"simulate_simple.jl\")"
"simulate_steering = include(\"simulate_steering.jl\")"
"steering_test_1p = include(\"steering_test_1p.jl\")"
"steering_test_4p = include(\"steering_test_4p.jl\")"
"calc_spectrum = include(\"calc_spectrum.jl\")"
"plot_spectrum_ = include(\"plot_spectrum.jl\")"
"calculate_rotational_inertia = include(\"calculate_rotational_inertia.jl\")"
"quit"
```

## Outlook

## References
