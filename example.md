# ALS TPU from Point Clouds

## Motivation

Generating total propagated uncertainty (TPU), also referred to as propagated error, for airborne laser scanning (ALS) xyz coordinate triplets requires knowledge of the ALS observations (and their uncertainties) and the sensor model. By ALS observations we mean the measurements necessary to populate the standard airborne lidar equation: 

    - Lidar range
    - Scanner angle
    - Sensor location (e.g., llh, xyz)
    - Sensor attitude (roll, pitch, heading)
    - Axes misalignment between the scanner and IMU (boresight roll, pitch, heading)
    - Relative location between the scanner and IMU (lever arm xyz)

The above list amounts to 14 measurements (and uncertainties) necessary for each xyz ground coordinate. By sensor model we mean things like the IMU, scanner, and local level reference frame definitions, and rotation types and orders for the boresight and IMU angles. These requirements should, hopefully, make obvious the challenge that geospatial practitioners face when per-point ALS TPU is desired for a point cloud dataset. Neither the ALS observations nor sensor model information is typically available, and even if it were, a non-trivial algorithm will need to be developed to generate the TPU.

## An ALS TPU Plugin for PDAL

To address the above, two plugin filters have been developed for the Point Data Abstraction Library (PDAL) that enable generation of per-point ALS TPU estimates from lidar point data alone. The first filter, `filters.sritraj`, generates an estimate of the ALS sensor trajectory, which provides six of the necessary ALS observations: sensor location (llh or xyz) and sensor attitude (roll, pitch, heading). Combined with a generic ALS sensor model definition, the estimated trajectory data enables the lidar range and scanner angle to be inverted for each lidar point. This inversion and the ultimate TPU computation is performed in the second PDAL filter, `filters.als_tpu`. The remaining observations - boresight angles and lever arm displacements - are set equal to zero. These values are typically quite small, so a zero value has little impact on the final TPU values.

For each observation necessary to generate a ground coordinate from the lidar equation, including those set to zero, a corresponding uncertainty must be provided. Many of these are assumed static for a given ALS collection campaign (boresight angles, lever arm displacments, sensor location and attitude), or we only have knowledge of a single estimate provided by the sensor manufacturer (lidar range, scanner angle). However, the influences of a non-zero and range-dependent laser beam width and the incidence angle at which the laser beam intercepts the ground surface contribute non-static uncertainties to the scan angle and lidar range that must be individually estimated for each instance of the lidar equation. 

If you are familiar with lidar TPU, the above discussion will make sense - and probably inspire points of contention. If you are not familiar with lidar TPU and wish to have a better grasp on what `filters.als_tpu` is producing, review the references provided at the end of this document.

## Example Workflow

With introductory remarks out of the way, let's walk through a real-world example of generating per-point TPU for a lidar point cloud dataset. We have been provided with 19 tiles of ALS data collected over the University of Houston campus in LAS file format and our colleague has requested TPU on the data inside an area of interest, which they have defined with a KML file. 

### 1. Prerequisites

1. We'll be using a Bash terminal for much of this work. If you are a Windoze user like me and want to follow along, set up WSL2 on your machine and install an Ubuntu distribution.
2. We'll be using PDAL and Entwine CLI applications on the Bash terminal. One way to get those applications on your machine is via Conda. You'll need to install Miniconda into your WSL2 Ubuntu distibution. Once that is done, create a Conda environment and install PDAL and Entwine into it via conda-forge: 
    - `conda create -n pdal`
    - `conda activate pdal`
    - `conda install -c conda-forge pdal`
    - `conda install -c conda-forge entwine`
3. We'll also be using CloudCompare and QGIS to visualize the point clouds.

### 2. Data Exploration

The first thing we want to do is visualize the data to get some context. We can use CloudCompare or QGIS to do this. However, there are a few snags.  First, the provided LAS files do not contain a coordinate reference system (CRS). While this is not a deal breaker, it's bad form and prevents us from overlaying the provided KML boundary data. So let's use PDAL's `translate` command to add a CRS and, while we are at it, convert the files to compressed LAZ format. Based on metadata found elsewhere, we know the datum is WGS84, the projection is UTM Zone 15N, and the elevations are ellipsoidal. We'll store the LAZ files in a new directory and leave the original LAS tiles alone after this. My current directory structure is this:

```
(pdal) pjhartze@GSE-10:/mnt/f/uh$ tree -d
.
└── tiles
    └── las
```

Let's convert to LAZ while adding the correct CRS. We'll use GNU parallel to, well, run things in parallel because it's faster.

```
(pdal) pjhartze@GSE-10:/mnt/f/uh$ mkdir ./tiles/laz
(pdal) pjhartze@GSE-10:/mnt/f/uh$ ls ./tiles/las/*.las | parallel -j+0 pdal translate {} ./tiles/laz/{/.}.laz --readers.las.override_srs=epsg:32615
(pdal) pjhartze@GSE-10:/mnt/f/uh$ tree -d
.
└── tiles
    ├── las
    └── laz
```

The second issue is that loading millions of points into CloudCompare or QGIS can take a long time and make viewing laggy. I'd prefer to load the data into QGIS so we can easily overlay the KML area of interest file. For this, we'll convert our LAZ data into an Entwine Point Tile (EPT) index, which will enable us to view the point cloud data inside QGIS without loading all the points at once. This will take some time, but better to burn time once than every time you want to view the data.

```
(pdal) pjhartze@GSE-10:/mnt/f/uh$ entwine build -i ./tiles/laz -o ./tiles/ept/
(pdal) pjhartze@GSE-10:/mnt/f/uh$ tree -d
.
└── tiles
    ├── ept
    │   ├── ept-data
    │   ├── ept-hierarchy
    │   └── ept-sources
    ├── las
    └── laz
```

Now we can very quickly view this data inside QGIS by going to the Data Source Manager > Point Cloud and browsing to the point to the newly created `ept-json` file. We can color by the Z coordinate and overlay the KML area of interest (black box in this case).

![](img/cloud-aoi-elevation.png)

Ah! We see that our colleague has requested virtually the entire data collect. Therefore, we'll generate TPU for all the data and clip to their desired boundary at the end. There is also some funky data in there - what's up with the blue trapezoidal looking things? If we go to View > New 3D Map View, and look at the data obliquely, we can see the trapezoids appear to be clusters of mid-air returns. If we select the identify tool and click one these artifact points, we find that they do not have a classification value, i.e., their classification is zero. We'll use this knowledge to remove these points during our processing.

![](img/cloud-aoi-elevation-oblique.png)

With some context on how much of the data needs to be processed (all of it) and knowledge of some artifacts that will need to be removed, we are ready to move on. 

### 3. Surface Normals

In order to include the influence of the laser ray to ground surface incidence angle in the TPU estimation, we need to a surface normal for each point. However, we need to remove the in-air point returns discovered during data exploration before computing the normals. We can use PDAL to accomplish both tasks.

```
(pdal) pjhartze@GSE-10:/mnt/f/uh$ mkdir ./tiles/laz-normal
(pdal) pjhartze@GSE-10:/mnt/f/uh$ ls ./tiles/laz/*.laz | parallel -j+0 pdal translate {} ./tiles/laz-normal/{/.}-normal.laz range normal '--filters.range.limits="Classification![0:0]"' '--filters.normal.knn=64' '--writers.las.minor_version=4' '--writers.las.extra_dims="NormalX=float,NormalY=float,NormalZ=float"'
(pdal) pjhartze@GSE-10:/mnt/f/uh$ tree -d
.
└── tiles
    ├── ept
    │   ├── ept-data
    │   ├── ept-hierarchy
    │   └── ept-sources
    ├── las
    ├── laz
    └── laz-normal
```

### 4. Flightline Extraction

We'll use the the `PointSourceId` field contains the flightline number, so we can Each point is taggThere are a few ways to First merge all tiles
```
(pdal) pjhartze@GSE-10:/mnt/f/uh$ mkdir merged
(pdal) pjhartze@GSE-10:/mnt/f/uh$ tree -d
.
├── merged
└── tiles
    ├── ept
    │   ├── ept-data
    │   ├── ept-hierarchy
    │   └── ept-sources
    ├── las
    ├── laz
    └── laz-normal
(pdal) pjhartze@GSE-10:/mnt/f/uh$ echo '{ "pipeline": [ "./tiles/laz-normal/*.laz", { "type":"writers.las", "filename":"./merged/normal.laz", "minor_version":4, "extra_dims":"all" } ] }' | pdal pipeline --stdin
```

We know from our data exploration that the `PointSourceId` field contains the flightline numbers. We can get a list of the flightlines with PDAL's `info` command:
```
(pdal) pjhartze@GSE-10:/mnt/f/uh$ pdal info ./merged/normal.laz --enumerate "PointSourceId"
{
  "file_size": 4439599570,
  "filename": "./merged/normal.laz",
  "now": "2021-10-30T22:10:00-0400",
  "pdal_version": "2.3.0 (git-version: 43197e)",
  "reader": "readers.las",
  "stats":
  {
    "bbox":
    {
      "EPSG:4326":
      {
        "bbox":
        {
          "maxx": -95.3130282,
          "maxy": 29.73713743,
          "maxz": 219.95,
          "minx": -95.36775693,
          "miny": 29.70779386,
          "minz": -113.71
        },
        "boundary": { "type": "Polygon", "coordinates": [ [ [ -95.367090313483885, 29.707793855243175, -113.71 ], [ -95.367756928931357, 29.736180546330861, -113.71 ], [ -95.313679625043022, 29.737137428570652, 219.95 ], [ -95.313028200965121, 29.708749642515375, 219.95 ], [ -95.367090313483885, 29.707793855243175, -113.71 ] ] ] }
      },
      "native":
      {
        "bbox":
        {
          "maxx": 276234.09,
          "maxy": 3291900.42,
          "maxz": 219.95,
          "minx": 271000.36,
          "miny": 3288752.38,
          "minz": -113.71
        },
        "boundary": { "type": "Polygon", "coordinates": [ [ [ 271000.35999999998603, 3288752.379999999888241, -113.71 ], [ 271000.35999999998603, 3291900.42, -113.71 ], [ 276234.090000000025611, 3291900.42, 219.95 ], [ 276234.090000000025611, 3288752.379999999888241, 219.95 ], [ 271000.35999999998603, 3288752.379999999888241, -113.71 ] ] ] }
      }
    },
    "statistic":
    [
      {
        "average": 273576.1627,
        "count": 355211315,
        "maximum": 276234.09,
        "minimum": 271000.36,
        "name": "X",
        "position": 0,
        "stddev": 1231.919901,
        "variance": 1517626.643
      },
      {
        "average": 3290219.814,
        "count": 355211315,
        "maximum": 3291900.42,
        "minimum": 3288752.38,
        "name": "Y",
        "position": 1,
        "stddev": 563.8214184,
        "variance": 317894.5918
      },
      {
        "average": -12.57677251,
        "count": 355211315,
        "maximum": 219.95,
        "minimum": -113.71,
        "name": "Z",
        "position": 2,
        "stddev": 5.215434469,
        "variance": 27.2007567
      },
      {
        "average": 1532.455386,
        "count": 355211315,
        "maximum": 65520,
        "minimum": 16,
        "name": "Intensity",
        "position": 3,
        "stddev": 1487.738875,
        "variance": 2213366.959
      },
      {
        "average": 1.335397593,
        "count": 355211315,
        "maximum": 4,
        "minimum": 1,
        "name": "ReturnNumber",
        "position": 4,
        "stddev": 0.6874446773,
        "variance": 0.4725801844
      },
      {
        "average": 1.670833219,
        "count": 355211315,
        "maximum": 4,
        "minimum": 1,
        "name": "NumberOfReturns",
        "position": 5,
        "stddev": 0.9848752227,
        "variance": 0.9699792043
      },
      {
        "average": 0.4997293287,
        "count": 355211315,
        "maximum": 1,
        "minimum": 0,
        "name": "ScanDirectionFlag",
        "position": 6,
        "stddev": 0.4999999274,
        "variance": 0.2499999274
      },
      {
        "average": 5.729828736e-05,
        "count": 355211315,
        "maximum": 1,
        "minimum": 0,
        "name": "EdgeOfFlightLine",
        "position": 7,
        "stddev": 0.007569346367,
        "variance": 5.729500443e-05
      },
      {
        "average": 2.927439507,
        "count": 355211315,
        "maximum": 17,
        "minimum": 1,
        "name": "Classification",
        "position": 8,
        "stddev": 4.671651239,
        "variance": 21.8243253
      },
      {
        "average": -0.17908474,
        "count": 355211315,
        "maximum": 33,
        "minimum": -40,
        "name": "ScanAngleRank",
        "position": 9,
        "stddev": 13.5471812,
        "variance": 183.5261186
      },
      {
        "average": 2.019363009,
        "count": 355211315,
        "maximum": 3,
        "minimum": 1,
        "name": "UserData",
        "position": 10,
        "stddev": 0.808540034,
        "variance": 0.6537369865
      },
      {
        "average": 574.5223646,
        "count": 355211315,
        "maximum": 1113,
        "minimum": 111,
        "name": "PointSourceId",
        "position": 11,
        "stddev": 307.0998069,
        "values":
        [
          111,
          112,
          113,
          211,
          212,
          213,
          311,
          312,
          313,
          411,
          412,
          413,
          511,
          512,
          513,
          611,
          612,
          613,
          711,
          712,
          713,
          811,
          812,
          813,
          911,
          912,
          913,
          1011,
          1012,
          1013,
          1111,
          1112,
          1113
        ],
        "variance": 94310.29138
      },
      {
        "average": 0,
        "count": 355211315,
        "maximum": 0,
        "minimum": 0,
        "name": "Red",
        "position": 12,
        "stddev": 0,
        "variance": 0
      },
      {
        "average": 0,
        "count": 355211315,
        "maximum": 0,
        "minimum": 0,
        "name": "Green",
        "position": 13,
        "stddev": 0,
        "variance": 0
      },
      {
        "average": 0,
        "count": 355211315,
        "maximum": 0,
        "minimum": 0,
        "name": "Blue",
        "position": 14,
        "stddev": 0,
        "variance": 0
      },
      {
        "average": 407936.2774,
        "count": 355211315,
        "maximum": 408920.504,
        "minimum": 407107.003,
        "name": "GpsTime",
        "position": 15,
        "stddev": 541.7267008,
        "variance": 293467.8184
      },
      {
        "average": 0.00232956138,
        "count": 355211315,
        "maximum": 1,
        "minimum": -0.9999999404,
        "name": "NormalX",
        "position": 16,
        "stddev": 0.2939775889,
        "variance": 0.08642282279
      },
      {
        "average": -0.004616250626,
        "count": 355211315,
        "maximum": 0.9999999404,
        "minimum": -0.9999999404,
        "name": "NormalY",
        "position": 17,
        "stddev": 0.257920775,
        "variance": 0.06652312616
      },
      {
        "average": 0.89238819,
        "count": 355211315,
        "maximum": 1,
        "minimum": 1.852526488e-08,
        "name": "NormalZ",
        "position": 18,
        "stddev": 0.2251013848,
        "variance": 0.05067063343
      }
    ]
  }
}
```

Now we are ready to create flightlines:
```
(pdal) pjhartze@GSE-10:/mnt/f/uh$ mkdir flightlines
(pdal) pjhartze@GSE-10:/mnt/f/uh$ tree -d
.
├── flightlines
├── merged
└── tiles
    ├── ept
    │   ├── ept-data
    │   ├── ept-hierarchy
    │   └── ept-sources
    ├── las
    ├── laz
    └── laz-normal
(pdal) pjhartze@GSE-10:/mnt/f/uh$ lines=111,112,113,211,212,213,311,312,313,411,412,413,511,512,513,611,612,613,711,712,713,811,812,813,911,912,913,1011,1012,1013,1111,1112,1113
(pdal) pjhartze@GSE-10:/mnt/f/uh$ echo $lines | parallel -j+0 pdal translate ./merged/normal.laz ./flighlines/line-{}.laz range '--filters.range.limits="PointSourceId[{}:{}]"' '--writers.las.minor_version=4' '--writers.las.extra_dims="all"'

(pdal) pjhartze@GSE-10:/mnt/f/uh$ parallel -j+0 pdal translate ./merged/normal.laz ./flightlines/line-{}.laz range '--filters.range.limits="PointSourceId[{}:{}]"' '--writers.las.minor_version=4' '--writers.las.extra_dims="all"' ::: $lines

or

(pdal) pjhartze@GSE-10:/mnt/f/uh$ lines=(111 112 113 211 212 213 311 312 313 411 412 413 511 512 513 611 612 613 711 712 713 811 812 813 911 912 913 1011 1012 1013 1111 1112 1113)
(pdal) pjhartze@GSE-10:/mnt/f/uh$ printf '%s\n' "${lines[@]}" | parallel -j+0 pdal translate ./merged/normal.laz ./flightlines2/line-{}.laz range '--filters.range.limits="PointSourceId
[{}:{}]"' '--writers.las.minor_version=4' '--writers.las.extra_dims="all"'
```