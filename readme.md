# Generating Per-Point Total Propagated Uncertainty (TPU)

Generating total propagated uncertainty (TPU), also referred to as "propagated error", for airborne laser scanning (ALS) point cloud data requires knowledge of the ALS measurements necessary to compute the ground coordinates, the measurement uncertainties (expressed as standard deviations), and the ALS sensor model. The ALS measurements (or "observations") consist of: 

- Lidar range
- Scanner angle
- Sensor location (e.g., llh, xyz)
- Sensor attitude (roll, pitch, heading)
- Axes misalignment between the scanner and IMU (boresight roll, pitch, heading)
- Relative location between the scanner and IMU (lever arm xyz)

Note that each measurement also needs a corresponding estimated uncertainty. The ALS sensor model refers to items such as the inertial motion unit (IMU), scanner, and local level reference frame definitions, and rotation types and orders for the boresight and IMU angles. Together, the measurement and sensor model information enable us to generate the point cloud coordinates using the standard ALS ground coordinate equation:

![](./img/LidarEqn.svg)

We use the ALS ground coordinate equation to propagate the measurement uncertainties into covariance matrices for each ground point via the General Law Of Propagation of Variance (GLOPOV).

The above summary should, hopefully, make obvious the challenge that geospatial practitioners face when per-point ALS TPU is desired for a point cloud dataset. Neither the ALS observations nor the sensor model information is typically available, and even if it were, a non-trivial algorithm would need to be developed to generate the TPU.


## ALS TPU with PDAL

To address - in part - the challenge outlined above, a plugin filter have been developed for the [Point Data Abstraction Library](https://pdal.io/) (PDAL) that enable generation of per-point ALS TPU estimates from lidar point data alone. The first plugin, `filters.sritrajectory`, generates an estimate of the ALS sensor trajectory, which provides six (technically five, as roll is not estimated) of the necessary ALS observations: sensor location and attitude (X, Y, Z, Roll, Pitch, Heading). Given trajectory information and a generic ALS sensor model definition, the lidar range and scanner angle measurements for each point can be inverted. This inversion and the ultimate TPU computation is performed in the second PDAL plugin, `filters.als_tpu`. The remaining observations - boresight angles and lever arm displacements - are set equal to zero. These values are typically quite small, so a zero value has little impact on the final TPU values.

For each observation necessary to generate a ground coordinate from the lidar equation, including those set to zero, a corresponding uncertainty must be provided. Many of these are assumed static for a given ALS collection campaign (boresight angles, lever arm displacements, sensor location and attitude), or we only have knowledge of a single estimate provided by the sensor manufacturer (lidar range, scanner angle). However, the influences of a non-zero and range-dependent laser beam width and the incidence angle at which the laser beam intercepts the ground surface contribute non-static uncertainties to the scan angle and lidar range that must be individually estimated for each instance of the lidar equation. 

If you are familiar with lidar TPU, the above discussion will be familiar - and probably inspire points of contention. If you are not familiar with lidar TPU, references are provided at the end of this document. Note that the `sritrajectory` filter is not yet open source (but should be soon). The source code for the `als_tpu` filter is still being refined, but is available on [Github](https://github.com/pjhartzell/pdal-als-tpu).


Example TPU pipeline.

```json
[
    {
        "type": "readers.las",
        "filename": "111.laz",
        "tag": "cloud"
    },
    {
        "type": "filters.sritrajectory",
        "tag": "trajectory"
    },
    {
        "type": "filters.als_tpu",
        "yaml_file": "Titan-C2-1064nm.yml",
        "inputs": [
            "cloud",
            "trajectory"
        ]
    },
    {
        "type": "writers.las",
        "minor_version": 4,
        "extra_dims": "all",
        "filename": "111-tpu.laz"
    }
]
```