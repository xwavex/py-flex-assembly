#!/usr/bin/env python3

import sys
import argparse
import math
import queue

# import numpy and OpenCV
import numpy as np
import cv2 as cv

# import ros libraries
import roslib
import rospy
import message_filters
from py_flex_assembly.srv import ClampEstimation, ClampEstimationResponse
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion
from py_flex_assembly.msg import Clamp, ClampArray
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R

image_queue = queue.Queue()

class pose_client():
    def __init__(self, args):
        self.args = args

        self.bridge = CvBridge()

        self.sub_depth = message_filters.Subscriber("/camera/global/depth/image_raw", Image)
        self.sub_color = message_filters.Subscriber("/camera/global/color/image_raw", Image)

        ts = message_filters.TimeSynchronizer([self.sub_depth, self.sub_color], 10)
        ts.registerCallback(self.callback)

        rospy.wait_for_service('clamp_estimation')
        self.server = rospy.ServiceProxy('clamp_estimation', ClampEstimation)

    def callback(self, depth, color):
        if args.verbose:
            print("\nreceived images from simulation")

        # noise the images and scale the depth image
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth, "passthrough")

            # the simulation returns the distance in meters while the realsense framework returns the distance in millimeter
            depth_img = depth_img * 1000

            # add noise to the images
            if self.args.noise_input:
                color_img = self.bridge.imgmsg_to_cv2(color, "passthrough")

                depth_img = self.noise_image(depth_img, 0, self.args.noise_depth_std)
                color_img = self.noise_image(color_img, 0, self.args.noise_img_std)

                color_img = self.bridge.cv2_to_imgmsg(color_img, "passthrough")
            else:
                color_img = color

            # debug
            """
            image = cv.convertScaleAbs(depth_img, alpha=0.3)
            image = cv.equalizeHist(image)
            image = cv.applyColorMap(image, cv.COLORMAP_JET)
            cv.namedWindow('depth image', cv.WINDOW_NORMAL & cv.WINDOW_KEEPRATIO)
            cv.imshow('depth image', image)
            cv.waitKey(1)
            """

            depth_img = self.bridge.cv2_to_imgmsg(depth_img, "passthrough")

        except CvBridgeError as e:
            print(e)

        # call the service
        try:
            if args.verbose:
                print("sending images to service")
            resp = self.server(depth_img, color_img)
            if args.verbose:
                print("received response from service")
        except rospy.ServiceException as e:
            print("service call failed: %s"%e)
            return

        # visualization
        # convert the color image
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_img, "passthrough")
            color_img = cv.cvtColor(color_img, cv.COLOR_RGB2BGR)
        except CvBridgeError as e:
            print(e)

        # draw the clamps
        for clamp in resp.clamps.clamps:
            # extract the information about a single clamp from the service response
            roll, _, yaw = R.from_quat([clamp.clamp_pose.orientation.x, clamp.clamp_pose.orientation.y, clamp.clamp_pose.orientation.z, clamp.clamp_pose.orientation.w]).as_euler('xyz')

            orig = np.array([clamp.clamp_pose.position.x, clamp.clamp_pose.position.y])
            vec_2 = np.array([1, 0])
            vec_2 = self.rotate(vec_2, yaw)
            vec_1 = self.rotate(vec_2, -math.pi / 2) if roll == 0 else self.rotate(vec_2, math.pi / 2)

            name = ""
            if clamp.type == Clamp.LARGE_THICK_GRAY:
                name = "LARGE_THICK_GRAY"
            elif clamp.type == Clamp.MEDIUM_THIN_GRAY:
                name = "MEDIUM_THIN_GRAY"
            elif clamp.type == Clamp.MEDIUM_THIN_GREEN:
                name = "MEDIUM_THIN_GREEN"
            elif clamp.type == Clamp.SMALL_THIN_BLUE:
                name = "SMALL_THIN_BLUE"

            # draw a single clamp
            self.draw_pose(color_img, np.int32(orig), vec_1, vec_2, 50, name)

        image_queue.put(color_img)

    # rotates a 2d vector
    def rotate(self, vec, alpha):
        rot_mat = np.empty((2, 2))
        rot_mat[0, 0] = math.cos(alpha)
        rot_mat[0, 1] = -math.sin(alpha)
        rot_mat[1, 0] = math.sin(alpha)
        rot_mat[1, 1] = math.cos(alpha)

        return np.matmul(rot_mat, vec)

    # noise the values of an image pixel wise with gaussian noise
    def noise_image(self, image, mean, std):
        noise = np.random.normal(mean, std, np.prod(image.shape))
        noise = np.reshape(noise, image.shape)

        if image.dtype == np.uint8:
            noise = noise.astype(np.int32)
            result = image.astype(np.int32) + noise
            result = np.minimum(result, np.full(image.shape, 255))
            result = np.maximum(result, np.zeros(image.shape))
            result = result.astype(np.uint8)
        else:
            noise = noise.astype(image.dtype)
            result = image + noise

        return result

    # draw the pose vectors and the clamp name onto the image
    def draw_pose(self, image, corner, vec_1, vec_2, length, name):
        p2 = np.int32(corner + length * vec_1)
        cv.line(image, tuple(corner), tuple(p2), (0, 255, 0), thickness=3, lineType=8, shift=0)

        p2 = np.int32(corner + length * vec_2)
        cv.line(image, tuple(corner), tuple(p2), (0, 0, 255), thickness=3, lineType=8, shift=0)

        cv.putText(image, name, tuple(corner - np.int32(vec_1 * 10) - np.int32(vec_2 * 10)), fontFace=cv.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness=2)
        cv.putText(image, name, tuple(corner - np.int32(vec_1 * 10) - np.int32(vec_2 * 10)), fontFace=cv.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(0, 0, 0), thickness=1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', action='store_true', help='displays information about outgoing requests and ingoing replies')
    parser.add_argument('--noise_input', action='store_false',
                        help='whether simulated noise should be added to the received images')
    parser.add_argument('--noise_img_std', type=float, default=3.37,
                        help='the standard deviation of the gaussian noise applied to the received image')
    parser.add_argument('--noise_depth_std', type=float, default=1.5,
                        help='the standard deviation of the gaussian noise applied to the received depth image')
    args = parser.parse_args()
    print(args)

    pose_subscriber = pose_client(args)
    rospy.init_node('pose_client', anonymous=True)

    cv.namedWindow("estimated clamp poses", cv.WINDOW_NORMAL & cv.WINDOW_KEEPRATIO)
    cv.resizeWindow("estimated clamp poses", 1280, 720)
    while True:
        cv.imshow("estimated clamp poses", image_queue.get())
        key = cv.waitKey(int(1000 / 30))
        if key == ord('q'):
            break
    cv.destroyAllWindows()
