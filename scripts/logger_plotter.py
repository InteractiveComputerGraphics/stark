# -*- coding: utf-8 -*-
import tkinter as tk
from tkinter import filedialog
import os
import numpy as np
import matplotlib.pyplot as plt
plt.style.use('seaborn-v0_8')

# folder = r"D:\builds\stark\output\car"
# # filename = "logger_car_16ms_0.01ra__2024-02-07__16-11-29.txt"
# filename = "logger_car_16ms_1.0ra__2024-02-07__20-56-28.txt"
ylabels = ["sedan_abs_velocity_kmh", "sedan_wheel_3_torque", "sedan_spring_3_dx", "sedan_wheel_3_w_rad_s", "sedan_wheel_3_sliding_v_m_s", "dt"]
xlabel = "sedan_time"

# =============================================================================
def select_file():
    root = tk.Tk()
    root.withdraw()  # Hide the main tkinter window
    file_path = filedialog.askopenfilename()  # Open the dialog and store the selected file path
    return file_path

file_path = select_file()
folder, filename = os.path.split(file_path)
with open(folder + os.sep + filename, "r") as f:
    txt = f.read()
lines = {l.split(": ")[0]: np.array([float(n) for n in l.split(": ")[1].split(", ") if n]) for l in txt.split("\n") if ":" in l}


# Determine the grid size for subplots
n = len(ylabels)
rows = int(np.ceil(n**0.5))  # Calculate number of rows to have a nearly square grid
cols = int(np.ceil(n / rows))  # Calculate number of columns

# Create subplots
fig, axes = plt.subplots(rows, cols, figsize=(5*cols, 5*rows))
fig.tight_layout(pad=5.0)  # Add space between plots for clarity
fig.suptitle(filename)

# Flatten axes array if needed
if n > 1:
    axes = axes.flatten()

for i, ylabel in enumerate(ylabels):
    ax = axes[i] if n > 1 else axes  # Select the current axis
    if xlabel == "":
        ax.plot(lines[ylabel])
    else:
        ax.plot(lines[xlabel], lines[ylabel])
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(ylabel)  # Optionally set a title for each subplot

# Hide any unused axes in the grid
if n > 1:
    for j in range(i + 1, len(axes)):
        axes[j].axis('off')
