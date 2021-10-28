# ALS TPU from Point Clouds

## Motivation
Generating total propagated uncertainty (TPU), also referred to as propagated error, for airborne laser scanning (ALS) xyz coordinate triplets requires knowledge of the ALS observations (and their uncertainties) and the sensor model. By ALS observations we mean the measurements necessary to populate the standard lidar equation: 

    * Lidar range
    * Scanner angle
    * Sensor location (e.g., llh, xyz)
    * Sensor attitude (roll, pitch, heading)
    * Axes misalignment between the scanner and IMU (boresight roll, pitch, heading)
    * Relative location between the scanner and IMU (lever arm xyz)

The above list amounts to 14 measurements (and uncertainties) necessary for each xyz ground coordinate. By sensor model we mean things like the IMU, scanner, and local level reference frame definitions, and rotation types and orders for the boresight and IMU angles. These requirements should, hopefully, make obvious the challenge that geospatial practitioners face when per-point ALS TPU is desired for a point cloud dataset. Neither the ALS observations nor sensor model information is typically available, and even if it were, a non-trivial algorithm will need to be developed to generate the TPU.

## An ALS TPU Plugin for PDAL
To address the above, two plugin filters have been developed for the Point Data Abstraction Library (PDAL) that enable generation of per-point ALS TPU estimates from lidar point data alone. The first filter, `filters.sritraj`, generates an estimate of the ALS sensor trajectory, which provides six of the necessary ALS observations: sensor location (llh or xyz) and sensor attitude (roll, pitch heading). Combined with a generic ALS sensor model definition, the estimated trajectory data enables the lidar range and scanner angle to be inverted for each lidar point. This inversion and the ultimate TPU computation is performed in the second PDAL filter, `filters.als_tpu`. The remaining observations - boresight angles and lever arm displacements - are set equal to zero. These values are typically quite small, so a zero value has little impact on the final TPU values.

For each observation necessary to generate a ground coordinate from the lidar equation, including those set to zero, a corresponding uncertainty must be provided. Many of these are assumed static for a given ALS collection campaign (boresight angles, lever arm displacments, sensor location and attitude), or we only have knowledge of a single estimate provided by the sensor manufacturer (lidar range, scanner angle). However, the influences of a non-zero and range-dependent laser beam width and the incidence angle at which the laser beam intercepts the ground surface contribute non-static uncertainties to the scan angle and lidar range that must be individually estimated for each instance of the lidar equation. 

If you are familiar with lidar TPU, the above discussion will make sense - and probably inspire points of contention. Otherwise, see the reference list at the end of this document for resources for more complete discussions of lidar TPU and its nuances.

## Example Workflow
With the introductory remarks out of the way, let's walk through a real-world example of generating per-point TPU for a lidar point cloud dataset. The data consists of 
