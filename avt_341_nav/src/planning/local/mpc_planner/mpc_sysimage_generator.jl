"""
@file mpc_sysimage_template.jl

@brief Script for the Julia sysimage generation.

@date 10/12/2023

@author Dario Sirangelo (dsi@mpe.au.dk)
        Aarhus University (DK)
        Department of Mechanical and Production Engineering
        Section Mechatronics & Dynamics
"""

# Install the required dependencies
import Pkg
Pkg.add("PackageCompiler")
Pkg.add(url="https://github.com/JuliaMPC/NLOptControl.jl")
Pkg.add("JuMP")
Pkg.add(Pkg.PackageSpec(;name="Ipopt", version="0.7.0"))

using PackageCompiler

# Create directory if not already existing
output_dir=dirname(ENV["JULIA_SYSIMAGE_OUTPUT_PATH"])
if !isdir(output_dir)
    mkdir(output_dir)
end

# Generate the sysimage.
create_sysimage([:NLOptControl,:JuMP,:Ipopt],
                sysimage_path=ENV["JULIA_SYSIMAGE_OUTPUT_PATH"],
                precompile_execution_file=ENV["JULIA_SYSIMAGE_TEMPLATE_PATH"])

# Exit the REPL.
exit()