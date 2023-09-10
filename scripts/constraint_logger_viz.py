# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
plt.style.use("seaborn-v0_8")

path = "D:/icra24_viz/local/020/constraints_logger__2023-09-10__14-15-06.txt"

series = {line.split(":")[0]: np.array(line.split(":")[1][:-2].split(", "), np.float64) for line in open(path).read().split("\n") if len(line) > 0}

plt.close()
plt.figure()
plt.title("Constraints")

for k, s in series.items():
    if k not in ["time", "frame"]:
        plt.plot(series["time"], s, lw=0.5, label=k)

plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.legend()
plt.show()
