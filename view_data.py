#!/usr/bin/env python
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

x = []
y = []
z = []

if len(sys.argv) == 2:
    filename = sys.argv[1]
else:
    filename = "data.csv"

with open(filename, "r") as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='"')
    for row in reader:
        x.append(float(row[0]))
        y.append(float(row[1]))
        z.append(float(row[2]))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(x, y, z, 'ro')

plt.show()
