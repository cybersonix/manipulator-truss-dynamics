#!/bin/bash
export LD_LIBRARY_PATH=$HOME/miniconda3/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/chrono/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/urdf_install/lib:$LD_LIBRARY_PATH

# ./build/bin/docking
./build/bin/relative_motion_elimination
