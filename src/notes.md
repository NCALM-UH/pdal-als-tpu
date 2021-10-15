1. The interpolation is most efficient when the point cloud is sorted by time.
2. I need to check that there are point normals already computed if the incidence angle error option is being used.
3. I am ignoring the pitch values supplied by SRI's trajectory for now. I need to understand how they are computed first:
    * Are they "clean" with respect to a laser scanner with a constant forward/back laser emission angle?
    * Do they make sense for a sensor that flies with a consistent non-zero pitch value?
    * Does the heading account for a sensor/aircraft that is "crabbing"?
    * CURRENT ASSUMPTIONS:
        * No sensor/aircraft crab
        * Zero pitch
        * No forward/back laser emission angle
4. Talk with Craig about what a full model should look like.
5. What is the best way to get all the measurement uncertainties entered?
6. Need to add a UAV and ALS generic parameter set selection.


## Updated SRI Trajectory Install Instructions
Current instructions direct you to install dependencies into a Conda environment:
```
conda install -c conda-forge ceres-solver pdal
conda install -c saedrna geographiclib
```

...but then proceed to direct you to install the plugin outside of the Conda environment:
```
INSTALL_DIR=/tmp/pdal-filters
mkdir BUILD
cd BUILD
cmake -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR ..
make -j4
make install
export PDAL_DRIVER_PATH=$INSTALL_DIR/lib:$PDAL_DRIVER_PATH
export SRI_TRAJECTORY_CONFIG_DIR=$INSTALL_DIR/resources/trajectory
pdal --drivers | grep trajectory
```

It is desirable to keep everything inside a single Conda environment:
```
mkdir BUILD
cd BUILD
cmake -D CMAKE_INSTALL_PREFIX=$CONDA_PREFIX ..
make -j4
make install
conda env config vars set SRI_TRAJECTORY_CONFIG_DIR="${CONDA_PREFIX}/resources/trajectory"
pdal --drivers | grep trajectory
```