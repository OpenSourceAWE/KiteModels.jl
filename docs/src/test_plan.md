## Testing the develop branch before merging it
On  AMD Ryzen 7 7840U on power.

### Pass criteria for the performance:
First run:
- simplifying the system      < 45s

Second run:
- system initialized          < 15s
- total time without plotting < 35s

## Results on Desktop, without SymbolicAWEModels 22.09.2025
MTK: 9.73.0
SymbolicUtils: v3.25.1
First run
- simplifying: 26.3 s
- Info: Total time without plotting: 124 s
Second run
- system initialized: 5.6s
- total time without plotting: 26.4 s

**Note:** Performance tests for SymbolicAWEModel have been moved to the [SymbolicAWEModels.jl](https://github.com/OpenSourceAWE/SymbolicAWEModels.jl) repository.

### NLSolve test
```
include("mwes/mwe_26.jl")
include("mwes/mwe_26.jl")
include("mwes/mwe_26.jl")
```
Each time the force, that is printed must be > 9000 N.

### Force precompilation
./bin/force_precompile