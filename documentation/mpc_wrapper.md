# MPC wrapper node

The node `avt_341_mpc_planner_node` is a wrapper around the MPC planner available at [TulgaErsal/AVT-341-MPC](https://github.com/TulgaErsal/AVT-341-MPC). The planner is written in Julia and relies on the optimal control library [NLOptControl](https://github.com/JuliaMPC/NLOptControl.jl) for solving the optimal control problem.

The ROS node uses the Julia C API to interface with the planner. 

## Setting up the Julia C API

The planner requires the Julia C API from Julia version 1.5.4. To provide the bindings, one must:

1. Download [version 1.5.4 of the Julia binaries](https://julialang-s3.julialang.org/bin/linux/x64/1.5/julia-1.5.4-linux-x86_64.tar.gz) from the Julia website.
2. Extract the archive and copy the contens to the location of your choice (e.g. `/opt/julia`, after applying required permissions).
3. Add the Julia executable to `PATH` (e.g. appending it to the `PATH` of the current session with `export PATH=${PATH}:/opt/julia/bin/julia`).
4. Add the Julia library to your linker. This step depends on your build setup - you must ensure the Julia library `libjulia.so` is available under `LD_LIBRARY_PATH` (e.g. by issuing `export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/julia/lib`).

## Building the planner

> **WARNING: Moving the stack install folder**  
> If you are moving the stack from the original install directory, you will have to repeat the build or, alternatively, provide the parameter `~julia_module_path` with the absolute path to the Julia MPC module `mpc_planner.jl`.