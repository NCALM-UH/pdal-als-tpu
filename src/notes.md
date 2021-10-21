1. The interpolation is expected to be most efficient when the point cloud is sorted by time.
2. I am ignoring any forward/back scan angle for now in terms of inverting/recovering them.
    * However, we know two of the Titan channels have a fixed forward/back angle.
    * But the SRI trajectory will interpret that as pitch. Would like to address that first. Could just subtract a user-supplied constant forward/back angle from the trajectory pitch. But would be better, IMO, to add an option to SRI's trajectory filter to do this so the trajectory pitch values are correct. We could then recover the forward/back scan angle in the observation inversion function.
    * The forward/back scan angle is included (all zero values for now) in order to insert beam divergence error in a direction orthogonal to the scanning direction.
    * We do not estimate/add/insert an observation uncertainty on the forward/back scan angle as it almost perfectly correlated with boresight pitch and any uncertainty should be contained therein.
4. Add option for users to point to a JSON file for the uncertainty set.
5. We assume heading has already been adjusted for wander angle
6. We assume trajectory attitude angles are in degrees (not radians).
7. We assume beam divergence is a 1/e^2 (not 1/e) definition.
8. Need to add a UAV and ALS generic parameter set selection.




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

We also need nlohmann's json library
```
conda install -c conda-forge nlohmann_json
```
