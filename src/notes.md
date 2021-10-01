1. The interpolation is most efficient when the point cloud is sorted by time.
2. We will not create valid TPU for points with times outside the trajectory.
    * Assign obvious bad value, like -9999?
    * Or not store that point?
3. It is cleaner to install SRI's trajectory filter into a Conda environment. Need to show the steps and where things get stored

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
export SRI_TRAJECTORY_CONFIG_DIR=$CONDA_PREFIX/resources/trajectory
pdal --drivers | grep trajectory
```