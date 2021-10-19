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
4. Add option for users to point to a JSON file for the uncertainty set.
5. We assume heading has already been adjusted for wander angle
6. We assume trajectory attitude angles are in degrees (not radians).
6. We assume beam divergence is a 1/e^2 (not 1/e) definition.
7. Need to add a UAV and ALS generic parameter set selection.
8. Rework my symbolic estimated comps to use all active rotations.
9. Add option for constant forward/back laser angle (e.g., Optech Titan 532 and 1550 channels). Subtract the value from the trajectory recovered by SRI's filter.



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