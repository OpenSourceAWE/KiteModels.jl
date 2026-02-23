# KiteModels.jl as example for an OpenSourceAWE project - a workshop

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

Linux, Windows and MacOS are supported.

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
```

The following examples are available if you type `menu2()`:

```julia
"plot_cl_cd_plate = include(\"plot_cl_cd_plate.jl\")"
"plot_side_cl = include(\"plot_side_cl.jl\")"
"steering_test_4p = include(\"steering_test_4p.jl\")"
"calculate_rot_inertia = include(\"calculate_rotational_inertia.jl\")"
"show_kite = include(\"../examples_3d/show_kite.jl\")"
"parking_4p = include(\"../examples_3d/parking_4p.jl\")"
"auto_parking_4p = include(\"../examples_3d/auto_parking_4p.jl\")"
"parking_wind_dir = include(\"../examples_3d/parking_wind_dir.jl\")"
"build_docu = include(\"../scripts/build_docu.jl\")"
```

## Configuration
The configuration happens using YAML files. The file `system.yaml` in the data folder decides which configuration
to use:

```yaml
system:
    sim_settings: "settings.yaml"  # simulator settings
```

This is the default `settings.yaml` file:

```{=latex}
\small
```

```yaml
system:
    log_file: "data/log_8700W_8ms" # filename without extension  [replay only]
                                   #   use / as path delimiter, even on Windows 
    log_level:      2              # 0: no logging 
    time_lapse:   1.0              # relative replay speed
    sim_time:   100.0              # simulation time             [sim only]
    segments:       6              # number of tether segments
    sample_freq:   20              # sample frequency in Hz
    zoom:        0.03              # zoom factor for the system view
    kite_scale:   3.0              # relative zoom factor for the 4 point kite
    fixed_font: ""                 # name or filepath+filename of alternative fixed pitch font, e.g. Liberation Mono

initial:
    elevations: [70.8]               # initial elevation angle   [deg]
    elevation_rates: [0.0]           # initial elevation rate  [deg/s]
    azimuths: [0.0]                  # initial azimuth angle     [deg]
    azimuth_rates: [0.0]             # initial azimuth rate    [deg/s]
    headings: [0.0]                  # initial heading angle     [deg]
    heading_rates: [0.0]             # initial heading rate    [deg/s]
    # three values are only needed for RamAirKite, for KPS3 and KPS4 use only the first value
    l_tethers: [150.0]               # initial tether lengths      [m]
    kite_distances: [51.0]           # initial kite distances     [m]
    v_reel_outs: [0.0]               # initial reel out speeds   [m/s]
    depowers: [25.0]                 # initial depower settings    [%]
    steerings: [0.0]                 # initial steering settings   [%]

solver:
    abs_tol: 0.0006        # absolute tolerance of the DAE solver [m, m/s]
    rel_tol: 0.001         # relative tolerance of the DAE solver [-]
    solver: "DFBDF"        # DAE solver, IDA or DFBDF or DImplicitEuler
    linear_solver: "GMRES" # can be GMRES or LapackDense or Dense (only for IDA)
    max_order: 4           # maximal order, usually between 3 and 5 (IDA and DFBDF)
    max_iter:  200         # max number of iterations of the steady-state-solver

steering:
    c0:       0.0          # steering offset   -0.0032           [-]
    c_s:      2.59         # steering coefficient one point model; 2.59 was 0.6; TODO: check if it must be divided by kite_area
    c2_cor:   0.93         # correction factor one point model
    k_ds:     1.5          # influence of the depower angle on the steering sensitivity
    delta_st: 0.02         # steering increment (when pressing RIGHT)
    max_steering: 16.834   # max. steering angle of the side planes for four point model [degrees]
    cs_4p:  1.0            # correction factor for the steering coefficient of the four point model

depower:
    alpha_d_max:    31.0   # max depower angle                            [deg]
    depower_offset: 23.6   # at rel_depower=0.236 the kite is fully powered [%]

kite:
    model: "data/kite.obj" # 3D model of the kite
    physical_model: "KPS4" # name of the kite model to use (KPS3 or KPS4)
    version: 1             # version of the model to use
    mass:  6.2             # kite mass incl. sensor unit [kg]
    area: 10.18            # projected kite area         [m²]
    rel_side_area: 30.6    # relative side area           [%]
    height: 2.23           # height of the kite           [m]
    alpha_cl:  [-180.0, -160.0, -90.0, -20.0, -10.0,  -5.0,  0.0, 20.0, 40.0, 90.0, 160.0, 180.0]
    cl_list:   [   0.0,    0.5,   0.0,  0.08, 0.125,  0.15,  0.2,  1.0,  1.0,  0.0,  -0.5,   0.0]
    alpha_cd:  [-180.0, -170.0, -140.0, -90.0, -20.0, 0.0, 20.0, 90.0, 140.0, 170.0, 180.0]
    cd_list:   [   0.5,    0.5,    0.5,   1.0,   0.2, 0.1,  0.2,  1.0,   0.5,   0.5,   0.5]

kps4:
    width:         5.77     # width of the kite                      [m]
    alpha_zero:    4.0      # should be 4 .. 10                [degrees]
    alpha_ztip:   10.0      #                                  [degrees]
    m_k:           0.2      # relative nose distance; increasing m_k increases C2 of the turn-rate law
    rel_nose_mass: 0.47     # relative nose mass
    rel_top_mass:  0.4      # mass of the top particle relative to the sum of top and side particles
    smc:           0.0      # steering moment coefficient                 [-]
    cmq:           0.0      # pitch rate dependant moment coefficient     [-]
    cord_length:   2.0      # average aerodynamic cord length of the kite [m]

kps4_3l:
    radius: 2.0                    # the radius of the circle shape on which the kite lines, viewed 
                                    #     from the front                                              [m]
    bridle_center_distance: 4.0     # the distance from point the center bridle connection point of 
                                    #     the middle line to the kite                                 [m]
    middle_length:          1.5     # the cord length of the kite in the middle                       [m]
    tip_length:             0.62     # the cord length of the kite at the tips                         [m]
    min_steering_line_distance: 1.0 # the distance between the left and right steering bridle         [m]
                                    #     line connections on the kite that are closest to each other [m]
    width_3l:               4.1    # width of the kite                                               [m]
    aero_surfaces:          3      # the number of aerodynamic surfaces to use per mass point        [-]
    
bridle:
    d_line:    2.5            # bridle line diameter                                                 [mm]
    l_bridle: 33.4            # sum of the lengths of the bridle lines                                [m]
    h_bridle:  4.9            # height of bridle                                                      [m]
    rel_compr_stiffness: 0.25 # relative compression stiffness of the kite springs                    [-]
    rel_damping: 6.0          # relative damping of the kite spring (relative to main tether)         [-]

kcu:
    kcu_model: "KCU1"            # name of the kite control unit model, KCU1 or KCU2
    kcu_mass: 8.4                # mass of the kite control unit                       [kg]
    kcu_diameter: 0.4            # diameter of the KCU for drag calculation            [m]
    cd_kcu: 0.0                  # drag coefficient of the KCU                         [-]  
    power2steer_dist: 1.3        #                                                     [m]
    depower_drum_diameter: 0.069 #                                                     [m]
    tape_thickness: 0.0006       #                                                     [m]
    v_depower: 0.075             # max velocity of depowering in units per second (full range: 1 unit)
    v_steering: 0.2              # max velocity of steering in units per second   (full range: 2 units)
    depower_gain: 3.0            # 3.0 means: more than 33% error -> full speed
    steering_gain: 3.0

tether:
    d_tether:  4           # tether diameter                 [mm]
    cd_tether: 0.958       # drag coefficient of the tether
    damping: 473.0         # unit damping coefficient        [Ns]
    c_spring: 614600.0     # unit spring constant coefficient [N]
    rho_tether:  724.0     # density of Dyneema           [kg/m³]
    e_tether: 55000000000.0 # axial tensile modulus of Dyneema (M.B. Ruppert) [Pa]
                           # SK75: 109 to 132 GPa according to datasheet

winch:
    winch_model: "AsyncMachine" # or TorqueControlledMachine
    max_force: 4000        # maximal (nominal) tether force; short overload allowed [N]
    v_ro_max:  8.0         # maximal reel-out speed                          [m/s]
    v_ro_min: -8.0         # minimal reel-out speed (=max reel-in speed)     [m/s]
    drum_radius: 0.1615    # radius of the drum                              [m]
    max_acc: 4.0           # maximal acceleration of the winch               [m/s²]
    gear_ratio: 6.2        # gear ratio of the winch                         [-]   
    inertia_total: 0.204   # total inertia, as seen from the motor/generator [kgm²]
    f_coulomb: 122.0       # coulomb friction                                [N]
    c_vf: 30.6             # coefficient for the viscous friction            [Ns/m]
    p_speed: 10000.0       # proportional gain of the winch speed controller [-]
    i_speed:  2500.0       # integral gain of the winch speed controller     [-]

environment:
    v_wind: 9.51             # wind speed at reference height          [m/s]
    upwind_dir: -90.0        # upwind direction                        [deg]
    temp_ref: 15.0           # temperature at reference height         [°C]
    height_gnd: 0.0          # height of groundstation above see level [m]
    h_ref:  6.0              # reference height for the wind speed     [m]

    rho_0:  1.225            # air density at zero height and 15 °C    [kg/m³]
    alpha:  0.08163          # exponent of the wind profile law
    z0:     0.0002           # surface roughness                       [m]
    profile_law: 3           # 1=EXP, 2=LOG, 3=EXPLOG
    # the following parameters are for calculating the turbulent wind field using the Mann model
    use_turbulence: 0.0      # turbulence intensity relative to Cabauw, NL
    v_wind_gnds: [3.483, 5.324, 8.163] # wind speeds at ref height for calculating the turbulent wind field [m/s]
    avg_height: 200.0        # average height during reel out          [m]
    rel_turbs:   [0.342, 0.465, 0.583] # relative turbulence at the v_wind_gnds
    i_ref: 0.14              # is the expected value of the turbulence intensity at 15 m/s.
    v_ref: 42.9              # five times the average wind speed in m/s at hub height over the full year    [m/s]
                             # Cabauw: 8.5863 m/s * 5.0 = 42.9 m/s
    height_step: 2.0         # use a grid with 2m resolution in z direction                                 [m]
    grid_step:   2.0         # grid resolution in x and y direction                                         [m]               
```

```{=latex}
\normalsize
```

## Questions
After this workshop, you should be able to answer the following questions:

- What is the meaning of parking mode of operation?
- Can rigid wing AWE systems operate in parking mode?
- Which winch models are currently available?
- How can the constants C1 and C2 of the turn-rate law be determined/ validated using this software?
- How does the tether force depend on the wind speed?
- How does the wind speed depend on the height in this model? How can that be configured?
- What is the advantage and disadvantage of the one-point model compared to the four-point model?
- How many tether segments are needed for a good accuracy in parking mode?
- Why is it less important to have many tether segments during reel-out compared to reel-in?
- Which kind of initial conditions are supported by this model?
- Why is the high stiffness of the differential algebraic (DAE) system a problem, and how does Julia handle this issue?

## Outlook

## References
