# dvrk-calibration-tool

Script written to calibrate the third joint of the dVRK

Currently works for Python 2, untested on Python 3

If you want help with any of the commands, run
```bash
./calibrate.py [subcommand] --help
```

To start, run this command to record all points by palpating:
```bash
./calibrate.py record PSM3
```

To record the points using a Polaris, run:
```bash
./calibrate.py record -p PSM3
```

After running the command, if you want to view the resulting plane from the palpations, run:
```bash
./calibrate.py view data/path_to_folder/plane.csv
```

If you want to view the resulting point cloud from a Polaris, run:
```bash
./calibrate.py view data/path_to_folder/polaris_point_cloud.csv
```

If you want to view all the palpations, run:
```bash
./calibrate.py view data/path_to_folder/
```
If you want to view a single palpation, run:
```bash
./calibrate.py view data/path_to_folder/palpation_i_j.csv
```

After this, to get the offset from the data recorded by palpations, run:
```bash
./calibrate.py analyze data/path_to_folder/plane.csv
```

If you want to get the offset from the data recorded by a Polaris, run:
```bash
./calibrate.py analyze data/path_to_folder/polaris_point_cloud.csv
```

After getting the offset you may want to view the resulting offset vs error graph, by running:
```bash
./calibrate.py view data/path_to_folder/offset_v_error.csv
```
