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