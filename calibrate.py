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
from analyze_data import get_new_offset, get_best_fit_plane, get_quadratic_min
from marker import Marker
from cisstNumericalPython import nmrRegistrationRigid

class Calibration(object):

    ROT_MATRIX = PyKDL.Rotation(
        1,    0,    0,
        0,   -1,    0,
        0,    0,   -1
    )

    def __init__(self, robot_name):
        print("initializing calibration for", robot_name)
        print("have a flat surface below the robot")
        self.data = []
        self.info = {}
        # Add checker for directory
        strdate = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.folder = os.path.join("data", "{}_{}".format(robot_name, strdate))
        os.mkdir(self.folder)
        print("Created folder at {}".format(os.path.abspath(self.folder)))

        self.arm = dvrk.psm(robot_name)
        self.home()

    def home(self):
        "Goes to x = 0, y = 0, extends joint 2 past the cannula, and sets home"
        # make sure the camera is past the cannula and tool vertical
        print("starting home")
        self.arm.home()
        self.arm.close_jaw()

        if self.arm.get_current_joint_position()[2] > 0.12:
            # Already past cannula
            carte_goal = self.arm.get_current_position().p
            carte_goal[2] += 0.04
            self.arm.move(carte_goal)

        goal = np.zeros(6)

        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2') or
            (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            # set in position joint mode
            goal[2] = 0.08
            self.arm.move_joint(goal)
        self.arm.move(self.ROT_MATRIX)

    def output_to_csv(self):
        "Outputs contents of self.data to fpath"
        with open(os.path.join(self.folder, "plane.csv"), 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.data[0].keys())
            writer.writeheader()
            for row in self.data:
                writer.writerow(row)


def choose_filename(fpath):
    """checks if file at fpath already exists.
    If so, it increments the file"""
    if not os.path.exists(fpath):
        new_fname = fpath
    else:
        fname, file_ext = os.path.splitext(fpath)
        i = 1
        new_fname = "{}_{}{}".format(fname, i, file_ext)
        while os.path.exists(new_fname):
            i += 1
            new_fname = "{}_{}{}".format(fname, i, file_ext)
    return new_fname


def plot_data(data_file):
    "Plots the data from the csv file data_file"

    coords = np.array([])

    polaris_coords = np.array([])

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
                polaris = True
                polaris_coord = np.array([
                    float(row["polaris_position_x"]),
                    float(row["polaris_position_y"]),
                    float(row["polaris_position_z"]),
                ])
                polaris_coords = np.append(polaris_coords, polaris_coord)
            else:
                polaris = False

    coords = coords.reshape(-1, 3)

    if polaris:
        polaris_coords = polaris_coords.reshape(-1, 3)

    joint_set = joint_set.reshape(-1, 6)


    if polaris:
        transf, error = nmrRegistrationRigid(coords, polaris_coords)
        rot_matrix = transf.Rotation()
        translation = transf.Translation()
        polaris_coords = (polaris_coords - translation).dot(rot_matrix)
        print("Rigid Registration Error: {}".format(error))

    if not polaris:
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
    if polaris:
        ax.scatter(polaris_coords[:,0], polaris_coords[:,1], polaris_coords[:,2],
            c='b', s=20, label="Polaris")
        ax.scatter(coords[:,0], coords[:,1], coords[:,2], c='r', s=20, label="Arm")
    else:
        ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2)
        ax.scatter(coords[:,0], coords[:,1], coords[:,2], c='r', s=20)

    plt.xlabel('X')
    plt.ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()


def parse_record(args):
    # pts = [
    #     PyKDL.Vector(0.04969137179347108, 0.12200283317260341, -0.19),
    #     PyKDL.Vector(0.09269885200354012, -0.06284151552138104, -0.19),
    #     PyKDL.Vector(-0.06045055029036737, -0.11093816039641696, -0.19)

    # ]
    pts = [
        PyKDL.Vector(0.032352239987663386, 0.09112554778646448, -0.1552123241995624),
        PyKDL.Vector(0.07298771358146197, -0.05720891999824999, -0.16137494511349101),
        PyKDL.Vector(-0.0430777628612891, -0.09345770839964689, -0.15956836746933412),
    ]
    pts = [PyKDL.Frame(Calibration.ROT_MATRIX, pt) for pt in pts]
    if args.polaris:
        from calibrate_polaris import PolarisCalibration
        calibration = PolarisCalibration(args.arm)
        # pts = calibration.get_corners()
        # grid = calibration.gen_grid(pts, args.samples, verbose=args.verbose)
        # calibration.record_points_polaris(grid, verbose=args.verbose)
        joint_set = list(calibration.gen_wide_joint_positions())
        print("Starting calibration")
        time.sleep(0.5)
        calibration.record_joints(joint_set, verbose=args.verbose)
        calibration.output_to_csv()
        print("Run `./calibrate.py view {}` to view the recorded data points,"
                .format(os.path.join(calibration.folder, "plane.csv")))
        print("run `./calibrate.py analyze -p {}` to analyze the recorded data points, or"
                .format(os.path.join(calibration.folder, "plane.csv")))
        print("run `./calibrate.py analyze -p {} -w "
                "~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-daVinci/"
                "sawRobotIO1394-PSM3-28613.xml` to analyze and write the resulting offset"
                .format(os.path.join(calibration.folder, "plane.csv")))
    else:
        from calibrate_plane import PlaneCalibration
        calibration = PlaneCalibration(args.arm)

        if not args.single_palpation:
            pts = calibration.get_corners()
            goal = copy(pts[2])
            goal.p[2] += 0.05
            calibration.arm.move(goal)
            goal = copy(pts[0])
            calibration.arm.home()
            goal.p[2] += 0.090
            calibration.arm.move(goal)
            goal.p[2] -= 0.085
            calibration.arm.move(goal)
            calibration.record_points(pts, args.samples, verbose=args.verbose)
            calibration.output_to_csv()
            print("Run `./calibrate.py view {}` to view the recorded data points,"
                  .format(os.path.join(calibration.folder, "plane.csv")))
            print("run `./calibrate.py analyze {}` to analyze the recorded data points, or"
                  .format(os.path.join(calibration.folder, "plane.csv")))
            print("run `./calibrate.py analyze {} -w "
                  "~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-daVinci/"
                  "sawRobotIO1394-PSM3-28613.xml` to analyze and write the resulting offset"
                  .format(os.path.join(calibration.folder, "plane.csv")))
        else:
            print("Position the arm at the point you want to palpate at, then press enter.",
                  end=' ')
            sys.stdin.readline()
            goal = calibration.arm.get_current_position()
            goal.p[2] += 0.05
            calibration.arm.move(goal)
            goal.p[2] -= 0.045
            calibration.arm.move(goal)
            pos_v_force = calibration.palpate(os.path.join(calibration.folder, "single_palpation.csv"))
            if not pos_v_force:
                rospy.logerr("Didn't reach surface; closing program")
                sys.exit(1)
            print("Using {}".format(calibration.analyze_palpation(pos_v_force, show_graph=True)))



def parse_view(args):
    if os.path.isdir(args.input):
        from calibrate_plane import PlaneCalibration
        files = [
            os.path.join(args.input, filename)
            for filename in os.listdir(args.input)
            if filename.startswith("palpation")
        ]
        for filename in files:
            with open(filename) as csvfile:
                reader = csv.DictReader(csvfile)
                print("Reading {}".format(filename))
                pos_v_force = []
                for row in reader:
                    pos_v_force.append([
                        float(row["x-position"]),
                        float(row["y-position"]),
                        float(row["z-position"]),
                        float(row["wrench"]),
                    ])
                PlaneCalibration.analyze_palpation(pos_v_force, show_graph=True)
    elif os.path.basename(args.input).startswith("palpation"):
        with open(args.input) as csvfile:
            reader = csv.DictReader(csvfile)
            print("Reading {}".format(filename))
            pos_v_force = []
            for row in reader:
                pos_v_force.append([
                    float(row["x-position"]),
                    float(row["y-position"]),
                    float(row["z-position"]),
                    float(row["wrench"]),
                ])
            PlaneCalibration.analyze_palpation(pos_v_force, show_graph=True)
    elif os.path.basename(args.input).startswith("offset_v_error"):
        with open(args.input) as csvfile:
            reader = csv.DictReader(csvfile)
            offsets = []
            errors = []
            for row in reader:
                offsets.append(float(row["offset"]))
                errors.append(float(row["error"]))
            x = np.arange(offsets[0], offsets[-1] + 1, 1)
            polyfit = np.polynomial.Polynomial.fit(offsets, errors, 2)
            equation = polyfit.convert().coef[::-1]
            print("Quadratic fit: {}x^2 + {}x + {}".format(*equation))
            min_x = -equation[1] / (2 * equation[0])
            print("Minimum offset: {}mm".format(min_x / 10))
            min_y = equation[0] * min_x ** 2 + equation[1] * min_x + equation[2]
            y = equation[0] * x ** 2 + equation[1] * x + equation[2]
            # plt.plot(offsets, errors, '-', color="red")
            plt.plot(x, y, '-', color="blue")
            plt.scatter(offsets, errors, s=10, color="green")
            plt.plot(min_x, min_y, 'o', color="purple")
            plt.show()
    else:
        plot_data(args.input)


def parse_analyze(args):
    folder = os.path.dirname(args.input[0])
    offset_v_error_filename = os.path.join(folder, "offset_v_error.csv")
    offset = 1000 * get_new_offset(offset_v_error_filename, args.input, args.polaris)
    if args.write:
        if os.path.exists(args.write):
            print("Writing offset...")
            tree = ET.parse(args.write)
            root = tree.getroot()
            xpath_search_results = root.findall("./Robot/Actuator[@ActuatorID='2']/AnalogIn/VoltsToPosSI")
            if len(xpath_search_results) == 1:
                VoltsToPosSI = xpath_search_results[0]
            else:
                print("Error: There must be exactly one Actuator with ActuatorID=2")
                sys.exit(1)
            current_offset = float(VoltsToPosSI.get("Offset"))
            VoltsToPosSI.set("Offset", str(offset + current_offset))
            tree.write(args.write)
            print(("Wrote offset: {}mm (Current offset) + {}mm (Additional offset) "
                   "= {}mm (Written offset)").format(current_offset, offset,
                                                   offset + current_offset))
        else:
            print("Error: File does not exist")
            sys.exit(1)
    else:
        print("Offset: {}mm".format(offset))


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
        "-o", "--output",
        help="folder to output data",
    )
    parser_record.add_argument(
        "-p", "--polaris",
        help="use polaris",
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

    parser_view = subparser.add_parser("view", help="view outputted data")
    parser_view.add_argument("input", help="data to read from")
    parser_view.set_defaults(func=parse_view)

    parser_analyze = subparser.add_parser(
        "analyze",
        help="analyze outputted data and find offset"
    )
    parser_analyze.add_argument(
        "input",
        help="data to read from",
        nargs='+'
    )
    parser_analyze.add_argument(
        "-o", "--output",
        help="output for the graph of offset versus error "
        "(filename automatically increments)",
    )
    parser_analyze.add_argument(
        "-p", "--polaris",
        help="use polaris",
        default=False,
        action="store_true"
    )
    parser_analyze.add_argument(
        "-n", "--no-output",
        help="do not output graph of offset versus error",
        default=False,
        action="store_true"
    )
    parser_analyze.add_argument(
        "-w", "--write",
        help="write offset to file",
        default=False,
    )
    parser_analyze.set_defaults(func=parse_analyze)

    args = parser.parse_args()
    args.func(args)

