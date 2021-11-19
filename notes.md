## Notes


* The trajectory interpolation is expected to be most efficient when the point cloud is sorted by time.
* Forward/Back scan angle:
    * The SRI trajectory will interpret a forward/back scan angle as pitch.
    * The forward/back scan angle is solved in order to insert beam divergence error in a direction orthogonal to the scanning direction.
    * We do not apply an observation uncertainty on the forward/back scan angle as it almost perfectly correlated with boresight pitch and any uncertainty should be contained therein.
* We assume heading has already been adjusted for wander angle
* We assume trajectory attitude angles are in degrees (not radians).
* Note that IMU angles typically rotate you into local level. For this approximation, we are estimating "IMU angles" that rotate you into a projected XY coordinate system in which both the trajectory and point cloud are defined.
* Combination of active rotations, an NED local level definition, and IMU axes orientation of X-forward, Y-right, Z-down
    * What we like:
        * Natural Ordering: roll, pitch, heading aligns with R1(Rx), R2(Ry), R3(Rz); i.e., R1 is a function of roll, R2 is a function of pitch, and R3 is a function of heading
        * Natural signs: roll, pitch, and heading angles are used in the rotation matrices without negation
        * Equality of heading and yaw: Since the Z axis is downward, heading is equal to yaw (no negation necessary)
    * What we don't like:
        * We need to change our local level basis from NED to ENU before adding the GNSS/trajectory XYZ in the lidar equation. This adds an extra rotation matrix to the equation.


## Titan Channels
* C1 = 1550nm = 3.5 degrees forward
* C2 = 1064nm = 0 degrees (nadir)
* C3 = 532nm = 7 degrees forward


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

It is desirable, to me, to keep everything inside a single Conda environment:
```
mkdir BUILD
cd BUILD
cmake -D CMAKE_INSTALL_PREFIX=$CONDA_PREFIX ..
make -j4
make install
conda env config vars set SRI_TRAJECTORY_CONFIG_DIR="${CONDA_PREFIX}/resources/trajectory"
pdal --drivers | grep trajectory
```

