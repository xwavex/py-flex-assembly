#!/usr/bin/env python3

import sys
import argparse
import math

# import numpy and OpenCV
import numpy as np
import cv2 as cv

# import ros libraries
import roslib
import rospy
import message_filters
from py_flex_assembly.srv import ClampEstimation, ClampEstimationResponse
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion
from py_flex_assembly.msg import Clamp, ClampArray
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R

# NOTE: this import has to come after all ROS imports, else I get the following error:
#   from cv_bridge.boost.cv_bridge_boost import getCvType',
#     'ImportError: /lib/x86_64-linux-gnu/libstdc++.so.6: cannot allocate memory in static TLS block'
from pose_estimator import PoseEstimator

class pose_server:
    def __init__(self, args):
        self.args = args

        self.bridge = CvBridge()

        self.pose_estimator = PoseEstimator(accum_weight=args.accum_weight,
            calibration_frames=args.calibration_frames, threshold=args.threshold,
            area_threshold=args.area_threshold, notch_width=args.notch_width, notch_height=args.notch_height,
            calibration_in=args.calibration_in, calibration_out=args.calibration_out,
            classification_model=args.classification_model, clamp_border=args.clamp_border)

        self.service = rospy.Service('clamp_estimation', ClampEstimation, self.handle_clamp_estimation)
        print("started service")

    def handle_clamp_estimation(self, req):
        if self.args.verbose:
            print("received request")

        depth_img = None
        color_img = None
        # convert the raw image data
        try:
            depth_img = self.bridge.imgmsg_to_cv2(req.depth_img, "passthrough")
            color_img = self.bridge.imgmsg_to_cv2(req.color_img, "passthrough")
            color_img = cv.cvtColor(color_img, cv.COLOR_RGB2BGR)
        except CvBridgeError as e:
            print(e)

        # call the pose estimation
        pose_estimations, classes = self.pose_estimator.process_frame(depth_img, color_img)

        clamp_array = self.construct_clamp_array(pose_estimations, classes)
        if self.args.verbose:
            print("sending reply")
        return ClampEstimationResponse(clamp_array)

    def construct_clamp_array(self, pose_estimations, classes):
        # initialize the clamp array
        clamp_array = ClampArray()
        clamp_array.clamps = []
        header = Header()
        header.stamp = rospy.Time.now()
        clamp_array.header = header
        clamp_array.header.frame_id = "clamp_array"

        # add the clamps to the array
        for i in range(len(pose_estimations)):
            clamp = Clamp()
            pose = Pose()
            pose.position = Point(*pose_estimations[i][0], 0)

            # equivalent to math.acos(np.dot(np.array([1, 0]), pose_estimations[i][2]))
            yaw = math.acos(pose_estimations[i][2][0])
            # check whether the front of the clamp is shown
            roll = 0 if pose_estimations[i][3] else math.pi
            pose.orientation = Quaternion(*R.from_euler('xz', [roll, yaw]).as_quat())
            clamp.clamp_pose = pose

            if classes[i] == "large_thick_gray":
                clamp.type = Clamp.LARGE_THICK_GRAY
            elif classes[i] == "medium_thin_gray":
                clamp.type = Clamp.MEDIUM_THIN_GRAY
            elif classes[i] == "medium_thin_green":
                clamp.type = Clamp.MEDIUM_THIN_GREEN
            elif classes[i] == "small_thin_blue":
                clamp.type = Clamp.SMALL_THIN_BLUE
            else:
                print("Error: unknown clamp class \"" + classes[i] + "\"")
                rospy.signal_shutdown("an error occurred")

            clamp_array.clamps.append(clamp)
        return clamp_array

def main(args):
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--accum_weight', type=float, default=0.2,
                        help='parameter for cv.accumulateWeighted')
    parser.add_argument('-c', '--calibration_frames', type=int, default=100,
                        help='number of initial frames used for calibration (averaging the depth image)')
    parser.add_argument('-t', '--threshold', type=float, default=3.0,
                        help='threshold applied to the difference between depth and average depth image'
                        + '(should be a little smaller than height of detected objects in mm)')
    parser.add_argument('-v', '--verbose', action='store_true', help='displays information about ingoing requests and outgoing replies')
    parser.add_argument('--area_threshold', type=int, default=500,
                        help='threshold used to filter out small found regions (has to be smaller than pixel of object)')
    parser.add_argument('--notch_width', type=int, default=50,
                        help='the width of the notch of the clamp')
    parser.add_argument('--notch_height', type=int, default=8,
                        help='the height of the notch of the clamp')
    # Todo: set standard calibration input path
    parser.add_argument('--calibration_in', type=str, help='the path to an existing calibration', default=None)
    parser.add_argument('--calibration_out', type=str, help='the path where the calculated calibration should be saved', default=None)
    parser.add_argument('--classification_model', type=str, default="resnet18_sides.model",
                        help='the neural network used for classifying clamps')
    parser.add_argument('--clamp_border', type=float, default=0.2,
                        help='the size of the border drawn around a found clamp, when classifying it')
    args = parser.parse_args()
    print(args)

    rospy.init_node('pose_server', anonymous=True)
    ps = pose_server(args)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
