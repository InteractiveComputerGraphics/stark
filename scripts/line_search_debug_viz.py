import numpy as np
import matplotlib.pyplot as plt
plt.style.use("seaborn-v0_8")

# path = "D:/builds/stark/output/collision_cloth_parallel_edge_test/line_search.txt"
# path = "D:/builds/stark/output/collision_cloth_edge_edge_tests/line_search.txt"
path = "D:/builds/stark/output/wrap/line_search.txt"
r = [0, 19]

series = {line.split(":")[0]: np.array(line.split(":")[1][:-2].split(", "), np.float64) for line in open(path).read().split("\n") if len(line) > 0}
x = series["normalized_step_length"]

def f(n):
    if str(n) in series:
        s = series[str(n)]
        plt.close()
        plt.figure()
        plt.title("E0 = " + str(s[0]))
        plt.plot(x, s[1]*np.ones(len(x)), lw=0.5, c='black')
        plt.plot(x, s[3:])
        plt.xlabel("Step (norm(du): %e)" % s[2])
        plt.ylabel("1.0 - E0/Ebt")
        plt.show()


for i in range(*r):
    f(i)
