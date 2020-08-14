#!/usr/bin/env python3
import numpy as np
import cv2 as cv
import json

from clamp_classification import ClampClassificator

# normalizes a vector
def normalize(vec):
    if np.linalg.norm(vec) != 0:
        return vec / np.linalg.norm(vec)
    else:
        return vec

# exports a float32 array
def export_calibration(array, path):
    meta_info = {'height': array.shape[0], 'width': array.shape[1]}
    outfile = open(path + '.bytes', 'wb')
    outfile.write(array.tobytes())
    outfile.close()
    with open(path + '.meta', 'w') as f:
        json.dump(meta_info, f)

# import a float32 array
def import_calibration(path):
    f = open(path + '.bytes', 'rb')
    with open(path + '.meta', 'r') as meta_file:
        meta_info = json.load(meta_file)

    img = np.frombuffer(f.read(meta_info['height'] * meta_info['width'] * 4), dtype=np.float32)
    img = np.reshape(img, (meta_info['height'], meta_info['width']))
    return img

class PoseEstimator:
    def __init__(self, accum_weight, calibration_frames, threshold, area_threshold, notch_width,
    notch_height, calibration_in, calibration_out, classification_model, clamp_border):
        self.accum_weight = accum_weight
        self.calibration_frames = calibration_frames
        self.threshold = threshold
        self.area_threshold = area_threshold
        self.notch_width = notch_width
        self.notch_height = notch_height
        self.calibration_in = calibration_in
        self.calibration_out = calibration_out
        self.clamp_border = clamp_border

        self.avg_img = None
        self.accum = None
        self.frame = -1

        self.classifier = ClampClassificator(classification_model)

    # computes a mask containing the area where a notch is supsected to be
    def notch_mask(self, long_side, vec_1, vec_2, width, height):
        # extract the middle of the long side
        middle = long_side[0] + 0.5 * (long_side[1] - long_side[0])

        # create a list containing the corner points of the rectangle
        points = [np.int32(middle + 0.5 * self.notch_width * vec_2)]
        points.append(np.int32(points[-1] + self.notch_height * vec_1))
        points.append(np.int32(points[-1] - self.notch_width * vec_2))
        points.append(np.int32(points[-1] - self.notch_height * vec_1))
        points = np.array([tuple(x) for x in points])

        # create the mask
        mask = np.zeros((width, height), dtype=np.uint8)
        cv.fillConvexPoly(mask, points, 1)
        return mask

    # estimates the pose of a clamp based on the bounding rectangle and the threshed image
    def estimate_pose(self, rect_contour, threshed, front_shown):
        # extract the short sides of the rectangle
        short_side_1 = [rect_contour[0]]
        second_point = np.argmin([np.linalg.norm(rect_contour[0] - x) for x in rect_contour[1:]]) + 1
        short_side_1.append(rect_contour[second_point])

        short_side_2 = [rect_contour[x] for x in range(4) if x not in [0, second_point]]

        # sort the sides s.t. short_side_1 contains the leftmost point of the rectangle
        short_side_1 = np.array(sorted(short_side_1, key=lambda x: x[0]))
        short_side_2 = np.array(sorted(short_side_2, key=lambda x: x[0]))
        if short_side_1[0][0] > short_side_2[0][0]:
            short_side_1, short_side_2 = short_side_2, short_side_1

        # sort the points of the short sides s.t. the first point is the lower one
        short_side_1 = np.array(sorted(short_side_1, key=lambda x: x[1], reverse=True))
        short_side_2 = np.array(sorted(short_side_2, key=lambda x: x[1], reverse=True))

        # extract the long sides of the rectangle
        long_side_1 = [short_side_1[0], short_side_2[0]]
        long_side_2 = [short_side_1[1], short_side_2[1]]

        # extract the initial directions
        vec_1 = short_side_1[1] - short_side_1[0]
        vec_1 = normalize(vec_1)
        vec_2 = short_side_2[0] - short_side_1[0]
        vec_2 = normalize(vec_2)

        # search for the notch of the clamp
        # extract the rectangles in which the notch is suspected to be and compute corresponding masks
        mask_1 = self.notch_mask(long_side_1, vec_1, vec_2, threshed.shape[0], threshed.shape[1])
        mask_2 = self.notch_mask(long_side_2, -vec_1, vec_2, threshed.shape[0], threshed.shape[1])

        # evaluate the masks
        if np.count_nonzero(cv.bitwise_and(threshed, threshed, mask=mask_1)) < np.count_nonzero(cv.bitwise_and(threshed, threshed, mask=mask_2)):
            # the origin lies on long_side_1
            if front_shown:
                # the origin is affected by the side of the clamp which is seen by the camera
                # the side containing the metal parts of the clamp is defined as its front
                origin = long_side_1[0]
            else:
                origin = long_side_1[1]
                vec_2 *= -1
        else:
            # the origin lies on long_side_2
            vec_1 *= -1
            if front_shown:
                origin = long_side_2[1]
                vec_2 *= -1
            else:
                origin = long_side_2[0]

        return [origin, vec_1, vec_2, front_shown]

    def process_frame(self, depth_img, color_img):
        self.frame += 1

        # return arrays
        pose_estimations = []
        classes = []

        # initialization of the calibration and the accumulated depth image
        if self.accum is None:
            self.avg_img = np.zeros(depth_img.shape, np.float32)
            self.accum = np.zeros(depth_img.shape, np.float32)

            if self.calibration_in != None:
                self.avg_img = import_calibration(self.calibration_in)
                if self.avg_img.shape != depth_img.shape:
                    print("the dimensions of the imported calibration don't match the dimensions of the received image")
                    return pose_estimations, classes
                print("imported calibration")
            else:
                print("started calibration")

        # accumulate the depth image to reduce noise
        cv.accumulateWeighted(np.float32(depth_img), self.accum, self.accum_weight)

        # calculate a calibration if no calibration was given
        if self.frame < self.calibration_frames and self.calibration_in == None:
            self.avg_img += depth_img
            return pose_estimations, classes
        elif self.frame == self.calibration_frames and self.calibration_in == None:
            self.avg_img /= self.calibration_frames
            print("finished calibration")
            if self.calibration_out != None:
                export_calibration(self.avg_img, self.calibration_out)
                print("exported calibration")

        # thresholding
        _, threshed = cv.threshold(self.avg_img - self.accum, self.threshold, 255, cv.THRESH_BINARY)
        threshed = np.uint8(threshed)

        # debug
        """
        image = cv.convertScaleAbs(self.avg_img - self.accum, alpha=0.3)
        image = cv.equalizeHist(image)
        image = cv.applyColorMap(image, cv.COLORMAP_JET)
        cv.namedWindow('depth image', cv.WINDOW_NORMAL & cv.WINDOW_KEEPRATIO)
        cv.imshow('depth image', image)
        cv.waitKey(1)
        """

        # extract and process regions
        num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(threshed, connectivity=4)
        if num_labels == 1:
            return pose_estimations, classes
        for label in range(1, num_labels):
            if stats[label, cv.CC_STAT_AREA] < self.area_threshold:
                continue

            left = stats[label, cv.CC_STAT_LEFT]
            right = left + stats[label, cv.CC_STAT_WIDTH]

            top = stats[label, cv.CC_STAT_TOP]
            bottom = top + stats[label, cv.CC_STAT_HEIGHT]

            roi = threshed[top:bottom, left:right]
            roi = cv.copyMakeBorder(roi, 1, 1, 1, 1, cv.BORDER_CONSTANT, 0)

            # findContours behaves differently in OpenCV 3 and 4 and thus the version needs to be checked
            if cv.__version__.startswith("3"):
                contours = cv.findContours(roi, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE, offset=(-1, -1))[1]
            elif cv.__version__.startswith("2") or cv.__version__.startswith("4"):
                contours = cv.findContours(roi, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE, offset=(-1, -1))[0]
            else:
                print("unexpected OpenCV version occurred")
                exit()

            # chose the longest available contour
            contours.sort(key=lambda x: x.shape[0], reverse=True)
            contour = contours[0]

            rotated_rect = cv.minAreaRect(contour)
            rect_contour = np.int32(cv.boxPoints(rotated_rect))
            rect_contour[:, 0] += left
            rect_contour[:, 1] += top

            # debug: draw the bounding onto the threshed image
            #cv.drawContours(threshed, [rect_contour], 0, 255, 3)
            #cv.drawContours(color_img, [rect_contour], 0, (0, 0, 0), 3)

            # classify the clamp
            contour_min_x = min(rect_contour[:, 0])
            contour_max_x = max(rect_contour[:, 0])
            contour_min_y = min(rect_contour[:, 1])
            contour_max_y = max(rect_contour[:, 1])

            contour_len_x = contour_max_x - contour_min_x
            contour_len_y = contour_max_y - contour_min_y

            clamp_min_y = max(0, contour_min_y - int(round(contour_len_y * self.clamp_border)))
            clamp_max_y = min(color_img.shape[0], contour_max_y + int(round(contour_len_y * self.clamp_border)))
            clamp_min_x = max(0, contour_min_x - int(round(contour_len_x * self.clamp_border)))
            clamp_max_x = min(color_img.shape[1], contour_max_x + int(round(contour_len_x * self.clamp_border)))

            clamp = color_img[clamp_min_y : clamp_max_y, clamp_min_x : clamp_max_x, :]

            class_name, front_shown = self.classifier.classify(clamp)
            classes.append(class_name)

            # estimate the pose of the clamp
            pose_estimations.append(self.estimate_pose(rect_contour, threshed, front_shown))

        return pose_estimations, classes
