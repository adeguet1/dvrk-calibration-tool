# dvrk-calibration-tool

Script written to calibrate the third joint of the dVRK

Currently works for Python 2, untested on Python 3

If you want help with any of the commands, run
```bash
./calibrate.py [subcommand] --help
```

To start, run this command to record all points by palpating:
```bash
./calibrate.py record {PSM_NAME} {CONFIG_FILE}
```

To record the points using a tracker, run:
```bash
./calibrate.py record -t {PSM_NAME} {CONFIG_FILE}
```

**Note**: View subcommand needs to be updated to match the updates to PlaneCalibration

This command creates a folder in the format `{ARM_NAME}_{DATE}_{TIME}`, which stores all the values for the calibration: palpation_{row}_{column}.csv and info.txt

After running the command, if you want to view the resulting plane from the palpations, run:
```bash
./calibrate.py view data/{ARM_NAME}_{DATE}_{TIME}/plane.csv
```

If you want to view the resulting point cloud from a Polaris, run:
```bash
./calibrate.py view data/{ARM_NAME}_{DATE}_{TIME}/polaris_point_cloud.csv
```

If you want to view all the palpations, run:
```bash
./calibrate.py view data/{ARM_NAME}_{DATE}_{TIME}/
```
If you want to view a single palpation, run:
```bash
./calibrate.py view data/{ARM_NAME}_{DATE}_{TIME}/palpation_{ROW}_{COLUMN}.csv
```

After this, to get the offset from the data recorded by palpations, run:
```bash
./calibrate.py analyze data/{ARM_NAME}_{DATE}_{TIME}
```

This command outputs a file in the folder called `plane.csv`

If you want to get the offset from the data recorded by a Polaris, run:
```bash
./calibrate.py analyze -t data/{ARM_NAME}_{DATE}_{TIME}
```

After getting the offset you may want to view the resulting offset vs error graph, by running:
```bash
./calibrate.py view data/{ARM_NAME}_{DATE}_{TIME}/offset_v_error.csv
```
