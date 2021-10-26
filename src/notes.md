1. The interpolation is expected to be most efficient when the point cloud is sorted by time.
2. I am ignoring any forward/back scan angle for now in terms of inverting/recovering them.
    * However, we know two of the Titan channels have a fixed forward/back angle.
    * But the SRI trajectory will interpret that as pitch. Would like to address that first. Could just subtract a user-supplied constant forward/back angle from the trajectory pitch. But would be better, IMO, to add an option to SRI's trajectory filter to do this so the trajectory pitch values are correct. We could then recover the forward/back scan angle in the observation inversion function.
    * The forward/back scan angle is included (all zero values for now) in order to insert beam divergence error in a direction orthogonal to the scanning direction.
    * We do not estimate/add/insert an observation uncertainty on the forward/back scan angle as it almost perfectly correlated with boresight pitch and any uncertainty should be contained therein.
3. We assume heading has already been adjusted for wander angle
4. We assume trajectory attitude angles are in degrees (not radians).
5. Need to tell users that beam divergence is a 1/e^2 (not 1/e) definition.
6. Need to add a UAV and ALS generic parameter set selection.
8. Need to profile the code for slow spots.
9. Note that IMU angles typically rotate you into local level. For this approximation, we are estimating "IMU angles" that rotate you into a projected XY coordinate system
10. What we like about the combination of active rotations, an NED local level definition, and IMU axes orientation of X-forward, Y-right, Z-down.
    * Natural Ordering: roll, pitch, heading aligns with R1(Rx), R2(Ry), R3(Rz); i.e., R1 is a function of roll, R2 is a function of pitch, and R3 is a function of heading
    * Natural signs: roll, pitch, and heading angles are used in the rotation matrices without negation
    * Equality of heading and yaw: Since the Z axis is downward, heading is equal to yaw (no negation necessary)
11. What we don't like:
    * We need to change our local level basis from NED to ENU before adding the GNSS/trajectory XYZ in the lidar equation. This adds an extra rotation matrix to the equation.


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

We also need a yaml library
1. `conda install -c conda-forge yaml-cpp`
2. You'll need to add that to your CMakeLists file.
