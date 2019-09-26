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

To record the points *n* number of times, run
```bash
./calibrate.py record -n {PSM_NAME} {CONFIG_FILE}
```

This command creates a folder in the format `{ARM_NAME}_{DATE}_{TIME}`, which stores all the values for the calibration: palpation_{row}_{column}.csv and info.txt

After this, to get the offset from the data recorded by palpations, run:
```bash
./calibrate.py analyze data/{ARM_NAME}_{DATE}_{TIME}
```

This command outputs a file in the folder called `plane.csv`

If you want to get the offset from the data recorded by a Polaris, run:
```bash
./calibrate.py analyze -t data/{ARM_NAME}_{DATE}_{TIME}
```

If you would like to view the data while analyzing it, use the options `--view-palpations`, `--view-point-cloud`, and/or `--view-offset-error`. To view all at once, use `--view-all`.
