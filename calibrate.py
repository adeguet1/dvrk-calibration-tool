#!/usr/bin/env python

from __future__ import print_function, division
import sys
import os.path
from copy import copy
import time
from datetime import datetime
import argparse
import xml.etree.ElementTree as ET
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import PyKDL
import rospy
import dvrk
from analyze_data import get_offset_v_error, get_best_fit_plane, get_poly_min, analyze_palpations
from cisstNumericalPython import nmrRegistrationRigid

def plot_data(data_file):
    """Plots the data from the csv file data_file"""

    coords = np.array([])

    tracker_coords = np.array([])

    joint_set = np.array([])

    with open(data_file, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for i, row in enumerate(reader):
            joints = np.array([
                float(row["joint_{}_position".format(joint_num)])
                for joint_num in range(6)
            ])
            joint_set = np.append(joint_set, joints)
            coord = np.array([
                float(row["arm_position_x"]),
                float(row["arm_position_y"]),
                float(row["arm_position_z"])
            ])
            coords = np.append(coords, coord)
            if len(row) == 12:
                tracker = True
                tracker_coord = np.array([
                    float(row["tracker_position_x"]),
                    float(row["tracker_position_y"]),
                    float(row["tracker_position_z"]),
                ])
                tracker_coords = np.append(tracker_coords, tracker_coord)
            else:
                tracker = False

    coords = coords.reshape(-1, 3)

    if tracker:
        tracker_coords = tracker_coords.reshape(-1, 3)

    joint_set = joint_set.reshape(-1, 6)


    if tracker:
        transf, error = nmrRegistrationRigid(coords, tracker_coords)
        rot_matrix = transf.Rotation()
        translation = transf.Translation()
        tracker_coords = (tracker_coords - translation).dot(rot_matrix)
        print("Rigid Registration Error: {}".format(error))

    if not tracker:
        X, Y = np.meshgrid(
            np.arange(
                min(coords[:,0])-0.05,
                max(coords[:,0])+0.05,
                0.05
            ),
            np.arange(
                min(coords[:,1])-0.05,
                max(coords[:,1])+0.05,
                0.05
            )
        )

        (A, B, C), error = get_best_fit_plane(coords)
        Z = A*X + B*Y + C

    # plot points and fitted surface
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    if tracker:
        ax.scatter(tracker_coords[:,0], tracker_coords[:,1], tracker_coords[:,2],
            c='b', s=20, label="Tracker")
        ax.scatter(coords[:,0], coords[:,1], coords[:,2], c='r', s=20, label="Arm")
    else:
        ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2)
        ax.scatter(coords[:,0], coords[:,1], coords[:,2], c='r', s=20)

    plt.xlabel('X')
    plt.ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()


def parse_info(info_filename):
    info = {}
    if os.path.exists(info_filename):
        with open(info_filename) as info_file:
            info_text = info_file.read()
            for line in info_text.splitlines():
                items = line.split(": ")
                if items != 2:
                    print("File is invalid")
                    sys.exit(1)
                else:
                    info[key] = value
        return info
    else:
        print("{} is not a file".format(info_filename))
        sys.exit(1)


def parse_record(args):
    if args.tracker:
        from calibrate_tracker import TrackerRecord
        recording = TrackerRecord(args.arm)
        joint_set = list(recording.gen_wide_joint_positions())
        print("Starting recording")
        time.sleep(0.5)
        recording.record_joints(joint_set, verbose=args.verbose)
        recording.output_to_csv()
        print("Run `./calibrate.py view {}` to view the recorded data points or"
                .format(os.path.join(recording.folder, "tracker_point_cloud.csv")))
        print("run `./calibrate.py analyze {}` to analyze the recorded data points."
                .format(os.path.join(recording.folder, "tracker_point_cloud.csv")))
        recording.output_info()
    else:
        from calibrate_plane import PlaneRecord

        if not args.single_palpation:
            # Full plane palpation
            recording = PlaneRecord(args.arm)
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
            print("Run `./calibrate.py view {}` to view the recorded data points,"
                  .format(os.path.join(recording.folder, "plane.csv")))
            print("run `./calibrate.py analyze {}` to analyze the recorded data points, or"
                  .format(os.path.join(recording.folder, "plane.csv")))
            print("run `./calibrate.py analyze {} -w {}\nto analyze and write the resulting offset"
                  .format(os.path.join(recording.folder, "plane.csv"), args.write))
            recording.output_info()
        else:
            # Single palpation
            recording = PlaneRecord(args.arm)
            print("Position the arm at the point you want to palpate at, then press enter.",
                  end=' ')
            sys.stdin.readline()
            goal = recording.arm.get_current_position()
            goal.p[2] += 0.05
            recording.arm.move(goal)
            goal.p[2] -= 0.045
            recording.arm.move(goal)
            pos_v_wrench = recording.palpate(os.path.join(recording.folder, "single_palpation.csv"))
            if not pos_v_wrench:
                rospy.logerr("Didn't reach surface; closing program")
                sys.exit(1)
            print("Using {}".format(recording.analyze_palpation(pos_v_wrench, show_graph=True)))


def parse_analyze(args):

    folder = os.path.dirname(args.input[0])
    
    info = parse_info(os.path.join(folder, "info.txt"))

    if not args.tracker:
        analyze_palpations(folder, show_graph=args.view)

    offset_v_error_filename = os.path.join(folder, "offset_v_error.csv")

    offset_v_error = get_offset_v_error(offset_v_error_filename, args.input, tracker=args.tracker)
    # min_offset = get_quadratic_min(offset_v_error)
    offset_correction = offset_v_error[(np.where(offset_v_error[:, 1] == np.amin(offset_v_error[:, 1]))[0][0]), 0] / 10
    write_to_file = True if input("Write to config file? (y/N) ").tolower() == "y" else False

    if write_to_file:
        if os.path.exists(info["Config File"]):
            print("Writing offset...")
            tree = ET.parse(info["Config File"])
            root = tree.getroot()
            xpath_search_results = root.findall("./Robot/Actuator[@ActuatorID='2']/AnalogIn/VoltsToPosSI")
            if len(xpath_search_results) == 1:
                VoltsToPosSI = xpath_search_results[0]
            else:
                print("Error: There must be exactly one Actuator with ActuatorID=2")
                sys.exit(1)
            current_offset = float(VoltsToPosSI.get("Offset"))
            VoltsToPosSI.set("Offset", str(offset_correction + current_offset))
            tree.write(info["Config File"])
            print(("Wrote offset: {}mm (Current offset) + {}mm (Offset correction) "
                   "= {}mm (Written offset)").format(current_offset, offset_correction,
                                                   offset_correction + current_offset))
        else:
            print("Error: File does not exist")
            sys.exit(1)
    else:
        print("Offset correction: {}mm".format(offset_correction))


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
        default=False,
        action="store_true"
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
        "-v", "--view",
        help="view graphs of palpations and plane/tracker point cloud",
        default=False,
        action="store_true"
    )
    parser_analyze.set_defaults(func=parse_analyze)

    args = parser.parse_args()
    args.func(args)

