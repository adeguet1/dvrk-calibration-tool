#!/usr/bin/env python

from __future__ import print_function, division
import sys
import os.path
from copy import copy
import time
import argparse
import xml.etree.ElementTree as ET
import rospy
from analyze import (get_offset_v_error, get_min_value, analyze_palpations,
                     show_tracker_point_cloud, show_palpation_point_cloud)


def parse_info(filename):
    info = {}
    if os.path.exists(filename):
        with open(filename) as info_file:
            info_text = info_file.read()
            for line in info_text.splitlines():
                items = line.split(": ")
                if len(items) != 2:
                    print("File is invalid")
                    sys.exit(1)
                else:
                    key, value = items
                    info[key] = value
        return info
    else:
        if os.path.isdir(filename):
            raise IOError(21, "Is a directory", filename)
        else:
            raise IOError(2, "No such file or directory", filename)


def parse_record(args):
    for i in range(args.n):
        if args.tracker is not None:
            from tracker_recording import TrackerRecording
            recording = TrackerRecording(args.arm, args.tracker)
            joint_set = list(recording.gen_wide_joint_positions())
            print("Starting recording")
            time.sleep(0.5)
            recording.record_joints(joint_set, verbose=args.verbose)
            recording.output_to_csv()
            recording.output_info()
            print("run `./calibrate.py analyze {}`\n"
                "    to analyze the recorded data points."
                .format(recording.folder))
        else:
            from plane_recording import PlaneRecording

            if not args.single_palpation:
                # Full plane palpation
                recording = PlaneRecording(args.arm)
                pts = recording.get_corners()
                goal = copy(pts[2])
                goal.p[2] += 0.10
                recording.arm.move(goal)
                goal = copy(pts[0])
                recording.arm.home()
                goal.p[2] += 0.090
                recording.arm.move(goal)
                goal.p[2] -= 0.085
                recording.arm.move(goal)
                recording.record_points(pts, args.samples, verbose=args.verbose)
                print(("Run `./calibrate.py analyze {}`\n"
                    "to analyze the recorded data points")
                    .format(recording.folder))
                recording.output_info()
            else:
                # Single palpation
                recording = PlaneRecording(args.arm)
                print(("Position the arm at the point you want to palpate at,"
                    "then press enter."),
                    end=' ')
                sys.stdin.readline()
                goal = recording.arm.get_current_position()
                goal.p[2] += 0.05
                recording.arm.move(goal)
                goal.p[2] -= 0.045
                recording.arm.move(goal)
                palp_fn = os.path.join(recording.folder, "single_palpation.csv")
                pos_v_wrench = recording.palpate(palp_fn)
                if not pos_v_wrench:
                    rospy.logerr("Didn't reach surface; closing program")
                    sys.exit(1)
                arm_position_z = recording.analyze_palpation(pos_v_wrench,
                                                            show_graph=True)
                print("Using {}".format(arm_position_z))


def parse_analyze(args):
    # For now only uses one set of data
    folder = os.path.dirname(args.data_folder[0])

    info = parse_info(os.path.join(folder, "info.txt"))

    if info["tracker"] == "True":
        is_tracker = True
    elif info["tracker"] == "False":
        is_tracker = False

    if is_tracker:
        print("Using external tracker calibration...")
        if args.view_point_cloud or args.view_all:
            show_tracker_point_cloud(os.path.join(
                folder,
                "tracker_point_cloud.csv"
            ))
    else:
        print("Using calibration sans external sensors...")
        analyze_palpations(
            folder, show_palpations=args.view_palpations or args.view_all
        )
        if args.view_point_cloud or args.view_all:
            show_palpation_point_cloud(os.path.join(
                folder,
                "plane.csv"
            ))

    offset_v_error_filename = os.path.join(folder, "offset_v_error.csv")

    offset_v_error = get_offset_v_error(
        offset_v_error_filename,
        args.data_folder, is_tracker,
        args.view_offset_error or args.view_all
    )

    # Get offset correction in tenths of millimeter
    # by getting x value of the abs. minimum of the graph
    offset_correction = get_min_value(offset_v_error)[0]

    # Convert correction from tenths of millimeter to milimeter
    offset_correction /= 10

    print("Offset correction: {}mm".format(offset_correction))
    print("Write to config file? (y/N) ", end=' ')
    write_to_file_input = sys.stdin.readline().strip().lower()

    if write_to_file_input == 'y':
        if os.path.exists(info["Config File"]):
            print("Writing offset...")
            tree = ET.parse(info["Config File"])
            root = tree.getroot()
            xpath_search_results = root.findall("./Robot/"
                                                "Actuator[@ActuatorID='2']/"
                                                "AnalogIn/"
                                                "VoltsToPosSI")
            if len(xpath_search_results) == 1:
                VoltsToPosSI = xpath_search_results[0]
            else:
                print("Error: There must be exactly one Actuator 2")
                sys.exit(1)
            current_offset = float(VoltsToPosSI.get("Offset"))
            VoltsToPosSI.set("Offset", str(offset_correction + current_offset))
            tree.write(info["Config File"])
            print(("Wrote offset: {}mm (Current offset) "
                "+ {}mm (Offset correction) "
                "= {}mm (Written offset)")
                .format(current_offset, offset_correction,
                        offset_correction + current_offset))
        else:
            print("Error: File does not exist")
            sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calibrate the dVRK")
    parser.add_argument(
        "-v", "--verbose",
        help="make output verbose", action="store_true"
    )

    subparser = parser.add_subparsers(title="subcommands")

    parser_record = subparser.add_parser(
        "record",
        help="record data for calibration"
    )
    parser_record.add_argument(
        "arm",
        help="arm to record points from"
    )
    parser_record.add_argument(
        "config_file",
        help="config file to write to"
    )
    parser_record.add_argument(
        "-t", "--tracker",
        help="use external tracker",
        nargs='?',
        const="/ndi/fiducials"
    )
    parser_record.add_argument(
        "-n", "--samples",
        help="number of samples per row "
        "(10 is recommended to get higher quality data)",
        default=10,
        type=int,
    )
    parser_record.add_argument(
        "-s", "--single-palpation",
        help="perform single palpation",
        action="store_true",
        default=False
    )
    parser_record.add_argument(
        "-n", "--number",
        help="run n number of times",
        type=int,
    )
    parser_record.set_defaults(func=parse_record)

    parser_analyze = subparser.add_parser(
        "analyze",
        help="analyze outputted data and find offset"
    )
    parser_analyze.add_argument(
        "data_folder",
        help="folder to read from",
        nargs='+'
    )
    parser_analyze.add_argument(
        "--view-palpations",
        help="view graph of palpations",
        default=False,
        action="store_true"
    )
    parser_analyze.add_argument(
        "--view-point-cloud",
        help="view plane/tracker point cloud",
        default=False,
        action="store_true"
    )
    parser_analyze.add_argument(
        "--view-offset-error",
        help="view offset vs error graph",
        default=False,
        action="store_true"
    )
    parser_analyze.add_argument(
        "--view-all",
        help="view all graphs",
        default=False,
        action="store_true"
    )

    parser_analyze.set_defaults(func=parse_analyze)

    args = parser.parse_args()

    args.func(args)
