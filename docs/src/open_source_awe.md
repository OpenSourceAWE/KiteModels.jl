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

In this workshop we will have a deeper look at one of the core OpenSourceAWE packages, the [KiteModels.jl](https://github.com/OpenSourceAWE/KiteModels.jl) package. It provides a one-point and a four-point kite power system model with a segmented tether and two different type of ground stations [@Fechner2015]. The package provides a large number of examples. Running these examples does
not require any programming knowledge.

The goal of the workshop is, that you get an overview which open source software for airborne wind energy systems exist,
and how you can contribute to this software ecosystem.

## Preparation
If possible, try to do the following steps before attending the workshop:

- Make sure you have the software "Git" installed. You can download it from the [Git website](https://git-scm.com/install/windows). During installation, select VSCode (or your preferred editor) as editor and select bash as your preferred terminal.
- Install [Julia 1.11 and VSCode](https://ufechner7.github.io/2024/08/09/installing-julia-with-juliaup.html). VSCode is optional, but it is nice to read the examples and the configuration files. You can also use any other IDE or editor if you prefer something else.

<!-- The wind farm layout, using six turbine groups is shown in Fig. \ref{fig:windfarm_6T}:

![Wind farm NordseeOne\label{fig:windfarm_6T}](windfarm_6T.png){width=100%} -->

## References
