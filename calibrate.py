#!/usr/bin/env python

from __future__ import print_function, division
import sys
import os.path
from copy import copy
import time
import argparse
import csv
import numpy
import scipy
import matplotlib
import PyKDL
import dvrk
import rospy

class Calibration:
    def __init__(self, robot_name, data_file=None, error_fk_outfile=None):
        print(rospy.get_caller_id(), " -> initializing calibration for", robot_name)
        print("have a flat surface below the robot")
        self.arm = dvrk.arm(robot_name)
        self.home()
        self.prepare_cartesian()
        self.data = []
        self.data_file = self.choose_filename(data_file)
        self.error_fk_outfile = choose_filename(error_fk_outfile)
        self.rob_file = "/home/cnookal1/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/deprecated/dvpsm.rob"
        self.rob = crp.robManipulator()
        numpy.set_printoptions(suppress=True)

    def choose_filename(fpath):
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

    def home(self):
        print(rospy.get_caller_id(), ' -> starting home')
        self.arm.home()
        # get current joints just to set size
        goal = numpy.copy(self.arm.get_current_joint_position())
        # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
        goal.fill(0)
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2') or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            goal[2] = 0.12
        self.arm.move_joint(goal, interpolate = True)

    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        goal = numpy.copy(self.arm.get_current_joint_position())
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2') or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            # set in position joint mode
            goal[0] = 0.0
            goal[1] = 0.0
            goal[2] = 0.12
            self.arm.move_joint(goal, interpolate = True)

    def get_corners(self):
        pts = []
        raw_input("Hello. Pick the first corner, then press enter. ")
        pts.append(self.arm.get_current_position())
        raw_input("Pick the second corner, then press enter. ")
        pts.append(self.arm.get_current_position())
        raw_input("Pick the third corner, then press enter. ")
        pts.append(self.arm.get_current_position())
        return pts

    def record_points(self, pts, nsamples):
        if not len(pts) == 3:
            return False

        MOVE_RES = 25
        THRESH = 1.5
        initial = PyKDL.Frame()
        initial.p = copy(pts[2].p)
        initial.M = PyKDL.Rotation(1, 0, 0, 0, -1, 0, 0, 0, -1)
        initial.p[2] += 0.05
        self.arm.move(initial)

        final = PyKDL.Frame()
        final.p = copy(pts[0].p)
        final.M = PyKDL.Rotation(1, 0, 0, 0, -1, 0, 0, 0, -1)
        final.p[2] += 0.05

        self.arm.move(final)

        goal = PyKDL.Frame()

        for i in range(nsamples):
            rightside = pts[1].p + i / (nsamples - 1) * (pts[2].p - pts[1].p)
            leftside = pts[0].p + i / (nsamples - 1) * (pts[2].p - pts[1].p)
            print("moving arm to row ", i)
            for j in range(nsamples):
                print("\tmoving arm to column ", j)
                goal.M = PyKDL.Rotation(1, 0, 0, 0, -1, 0, 0, 0, -1)
                if i % 2 == 0:
                    goal.p = leftside + j / (nsamples - 1) * (rightside - leftside)
                else:
                    goal.p = rightside + j / (nsamples - 1) * (leftside - rightside)
                
                prev_goal = copy(goal)
                goal.p[2] += 0.01
                self.arm.move(goal)

                for k in range(20): # in millimeters
                    goal.p[2] -= 0.001
                    self.arm.move(goal)
                    if abs(self.arm.get_current_wrench_body()[2]) >= THRESH:
                        goal.p[2] += 0.001
                        break
                    elif k == 19:
                        print("Error: Did not reach surface")
                        sys.exit(1)

                for k in range(20): # In tenths of a millimeter
                    goal.p[2] -= 0.0001
                    self.arm.move(goal)
                    if abs(self.arm.get_current_wrench_body()[2]) >= THRESH:
                        dist = (prev_goal.p - self.arm.get_current_position().p)[2]
                        print(prev_goal.p)
                        self.data.append(list(self.arm.get_current_position().p) + list(self.arm.get_current_joint_position()))
                        print("Distance: %fmm" % (dist * 1000))
                        goal.p[2] = prev_goal.p[2]
                        break
                    elif k == 19:
                        print("Error: Did not reach surface when rechecking")
                        sys.exit(1)
                        

                goal.p[2] += 0.01
                self.arm.move(goal)
                # move down until forces acts upon the motor

                
                time.sleep(0.5)
        print(rospy.get_caller_id(), '<- calibration complete')

    def gen_best_fit(self, pts):
        # best-fit linear plane
        A = np.c_[pts[:, 0], pts[:, 1], np.ones(pts.shape[0])]
        C, _, _, _ = scipy.linalg.lstsq(A, pts[:, 2])    # coefficients
        return C

    def gen_best_fit_error(self, pts):
        A, B, C = gen_best_fit(pts)
        errors = np.array([])

        direction = np.array([A, B, -1])
        normal = direction / np.linalg.norm(direction)

        projections = np.array([])

        for pt in pts:
            dist = np.dot(normal, pt - np.array([0, 0, C]))
            projection = pt - dist * normal
            projections = np.append(projections, projection)
            projections = projections.reshape(-1, 3)
            errors = np.append(errors, dist)
            # If this value is close to 0, then the distances are accurate
            # print(A * projection[0] + B * projection[1] + C - projection[2])

        return np.sqrt(sum([error ** 2 for error in errors]) /
                                    len(errors))

    def get_offset(self):
        min = 0
        min_offset = 0

        with open(self.error_fk_outfile) as outfile:
            fk_plot = csv.writer(outfile)
            for num, offset in enumerate(np.arange(-.9, .09, .001)):
                data = joints.copy()
                fk_pts = np.array([])
                for q in data:
                    q[2] += offset
                    fk_pts = np.append(fk_pts, rob.ForwardKinematics(q)[:3, 3])
                fk_pts = fk_pts.reshape((-1, 3))
                error = gen_best_fit_error(fk_pts)
                if num == 0 or error < min:
                    min = error
                    min_offset = offset
                fk_plot.writerow([offset, error])

        for num, offset in enumerate(np.arange(min_offset - 0.02,
                                               min_offset + 0.02,
                                               0.0001)):
            data = joints.copy()
            fk_pts = np.zeros(coords.shape)
            for i, q in enumerate(data):
                q[2] += offset
                fk_pts[i] = rob.ForwardKinematics(q)[:3, 3]
            error = gen_best_fit_error(fk_pts)
            if num == 0 or error < min:
                min = error
                min_offset = offset

        print(min_offset)

    def output_to_csv(self, fpath):
        with open(self.data_file, 'w') as csvfile:
            writer = csv.writer(csvfile, delimiter=',',
                                quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for row in self.data:
                writer.writerow(row)

    def plot_data(self, use_data_file=True):
        coords = np.array([])

        joints = np.array([])

        if use_data_file:
            with open(self.data_file, 'r') as csvfile:
                reader = csv.reader(csvfile)
                for row in reader:
                    joints = np.append(joints,
                                       np.array([float(x) for x in row[3:]]))
                    coords = np.append(coords,
                                       np.array([float(x) for x in row[:3]]))
            coords = coords.reshape(-1, 3)
            joints = joints.reshape(-1, 6)
        else:
            # use self.data
            pass

        X,Y = np.meshgrid(np.arange(min(data[:,0])-0.05, max(data[:,0])+0.05, 0.05),
                          np.arange(min(data[:,1])-0.05, max(data[:,1])+0.05, 0.05))
        A, B, C = self.gen_best_fit(pts)
        Z = C[0]*X + C[1]*Y + C[2]

        # plot points and fitted surface
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2)
        ax.scatter(data[:,0], data[:,1], data[:,2], c='r', s=50)
        plt.xlabel('X')
        plt.ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()


def parse_record(args):
    if args.polaris:
        raise NotImplementedError("Polaris integration is not yet implemented")
    else:
        calibration = Calibration(args.arm)
        pts = calibration.get_corners()
        calibration.record_points(pts, args.samples)
        calibration.output_to_csv(args.output)

def parse_view(args):
    calibration = Calibration(args.arm, data_file=args.input)
    calibration.plot_data()

def parse_analyze(args):
    calibration = Calibration(args.arm, data_file=args.input, error_fk_outfile=args.output)
    offset = calibration.analyze(args.no_output)
    if args.write:
        raise NotImplementedError("Cannot currently write output to xml file")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    subparser = parser.add_subparsers(title="subcommands")

    parser_record = subparser.add_parser("record", help="record data for calibration")
    parser_record.add_argument(
                               "arm",
                               help="arm to record points from"
                              )
    parser_record.add_argument(
                               "-o", "--output",
                               help="output recorded points (filename automatically increments)",
                               default="data/data.csv"
                              )
    parser_record.add_argument(
                               "-p", "--polaris",
                               help="use polaris",
                               default=False,
                               action="store_true"
                              )
    parser_record.add_argument(
                               "-n", "--samples",
                               help="number of samples",
                               default=10,
                               type=int,
                              )
    parser_record.set_defaults(func=parse_record)

    parser_view = subparser.add_parser("view", help="view outputted data")
    parser_view.add_argument("input", help="data to read from", nargs='?', default="data/data.csv")
    parser_view.set_defaults(func=parse_view)

    parser_analyze = subparser.add_parser("analyze", help="analyze outputted data and find offset")
    parser_analyze.add_argument(
                                "input",
                                help="data to read from",
                                nargs='?',
                                default="data/data.csv"
                               )
    parser_analyze.add_argument(
                                "-o", "--output",
                                help="output for the graph of offset versus error (filename automatically increments)",
                                default="data/error_fk.csv"
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
                                action="store_true"
                               )
    parser_analyze.set_defaults(func=parse_analyze)

    args = parser.parse_args()
    args.func(args)

