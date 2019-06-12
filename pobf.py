#!/usr/bin/env python

import numpy as np
from scipy.optimize import leastsq
XYZ = np.array()

if len(sys.argv) == 2:
    filename = sys.argv[1]
else:
    filename = "data.csv"

with open(filename, "rb") as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='"')
    for row in reader:
        XYZ = XYZ.append(np.array([float(row[0]), float(row[1]), float(row[2])]))

print(XYZ)
