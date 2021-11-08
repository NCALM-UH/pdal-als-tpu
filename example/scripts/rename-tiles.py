import os
import re

for filename in os.listdir("."):
    if filename.startswith("tpu"):
        x_pattern = "-(.*?)_"
        y_pattern = "_(.*?)\."
        x = re.search(x_pattern, filename).group(1)
        y = re.search(y_pattern, filename).group(1)

        x_new = 271000 + int(x) * 1000
        y_new = 3289000 + int(y) * 1000

        os.rename(filename, f"tpu-{x_new}_{y_new}.laz")