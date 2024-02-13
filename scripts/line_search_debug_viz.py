import numpy as np
import matplotlib.pyplot as plt
plt.style.use("seaborn-v0_8")

import tkinter as tk
from tkinter import filedialog
def select_file():
    root = tk.Tk()
    root.withdraw()  # Hide the main tkinter window
    file_path = filedialog.askopenfilename()  # Open the dialog and store the selected file path
    return file_path


# path = "D:/builds/stark/output/collision_cloth_parallel_edge_test/line_search.txt"
# path = "D:/builds/stark/output/collision_cloth_edge_edge_tests/line_search.txt"
# path = "D:/builds/stark/output/cloth_friction_slope_test/line_search_cloth_friction_slope_test.txt"
# path = "D:/builds/stark/output/cloth_friction_corner/line_search_cloth_friction_corner_jose.txt"
# path = "D:/builds/stark/output/wrap/line_search.txt"
path = select_file()

start = 0
n = 20
r = [start, start + n]

series = {line.split(":")[0]: np.array(line.split(":")[1][:-2].split(", "), np.float64) for line in open(path).read().split("\n") if len(line) > 0}
x = series["normalized_step_length"]

def f(n):
    if str(n) in series:
        s = series[str(n)]
        plt.close()
        plt.figure()
        plt.title("[%d] E0 = %f" % (n, s[0]))
        plt.plot(x, s[1]*np.ones(len(x)), lw=0.5, c='black')
        plt.plot(x, s[3:])
        plt.xlabel("Step (norm(du): %e)" % s[2])
        plt.ylabel("1.0 - E0/Ebt")
        plt.show()


for i in range(*r):
    f(i)
