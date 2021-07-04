#!/usr/bin/env python
from __future__ import print_function
from sensor_msgs import point_cloud2
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import StereoCameraModel
import rospy
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs

class ImageConverter:

    def __init__(self):
        rospy.init_node('image_converter', anonymous=True)
        rospy.Subscriber("/wide_stereo/left/image_rect_color", Image, self.callback)
        rospy.Subscriber("/wide_stereo/disparity", DisparityImage, self.set_disparity)
        rospy.Subscriber('/wide_stereo/points2', PointCloud2, self.get_pointcloud_data)
        self.image_pub = rospy.Publisher("/image_topic_2", Image, queue_size=1)
        self.bridge = CvBridge()
        self.stereo_model = StereoCameraModel()
        self.lower_yellow = np.array([20, 20, 20])
        self.upper_yellow = np.array([40, 255, 255])
        self.disparity = None
        self.cx = None
        self.cy = None
        self.depth = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        left_camera_info = rospy.wait_for_message('/wide_stereo/left/camera_info', CameraInfo)
        right_camera_info = rospy.wait_for_message('/wide_stereo/right/camera_info', CameraInfo)
        self.stereo_model.fromCameraInfo(left_camera_info, right_camera_info)

    def get_pointcloud_data(self, pointcloud_msg):
        gen = point_cloud2.read_points_list(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        self.data = gen
        self.depth = next(point_cloud2.read_points(pointcloud_msg, field_names='z', skip_nans=False, uvs=[(self.cx, self.cy)]))

    def set_disparity(self, disparity_image):
        self.disparity = disparity_image.image

    def callback(self, ros_image):
        '''
        cv_image: image converted to opencv image type from ros image type
        hsv: blurred image converted from bgr color space to hsv color space
        res: resultant image from masking out unwanted colours in the image
        img: convert masked image from hsv to bgr color space
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Apply a blur to the image from camera to reduce noise and then
        # convert the BGR color space of captured image to HSV color space
        hsv = cv2.cvtColor(cv2.GaussianBlur(cv_image,(7,7),1), cv2.COLOR_BGR2HSV)

        # Obtain a mask to 'zero out' the colour that is not of
        # interest, leaving an image with only the desired colours
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        res = cv2.bitwise_and(hsv, hsv, mask=mask)


        try:
            # Find the contour of the shape of interest and
            # then draw in blue the contours that were found
            __, contours, __ = cv2.findContours(mask, 1, 2)
            cv2.drawContours(cv_image, contours, -1, 255, 3)

            # find the biggest countour (c) by the area
            c = max(contours, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)

            # Obtain the centroid of the contour, c, from the moments
            moments = cv2.moments(c)
            self.cx = int(moments['m10']/moments['m00'])
            self.cy = int(moments['m01']/moments['m00'])
            # print(self.cx, self.cy)

            # draw the biggest contour (c) in green
            # cv2.rectangle(res,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)

        except Exception as e:
            print('error in callback function', e)
            

        # Show the original image, mask and masked image
        # cv2.imshow('mask',mask)
        # cv2.imshow('res', res)
        # cv2.imshow("cv_image", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except CvBridgeError as e:
            print('error in publishing the image', e)

    def from_2d_to_3d(self):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.disparity, desired_encoding='passthrough')
            moments_disparity = cv_image[self.cx][self.cy]
            # print(cv_image.shape)
            fruit_centroid = self.stereo_model.projectPixelTo3d((self.cx, self.cy), moments_disparity)
            transform = self.tf_buffer.lookup_transform('base_footprint',
                                       self.stereo_model.tfFrame(),
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second

            fruit_centroid_transformed = PoseStamped()
            fruit_centroid_transformed.header.frame_id = self.stereo_model.tfFrame()
            fruit_centroid_transformed.pose.position.x = fruit_centroid[0]
            fruit_centroid_transformed.pose.position.y = fruit_centroid[1]
            fruit_centroid_transformed.pose.position.z = min(self.depth)
            fruit_centroid_transformed.pose.orientation.x = 0
            fruit_centroid_transformed.pose.orientation.y = 0
            fruit_centroid_transformed.pose.orientation.z = 0
            fruit_centroid_transformed.pose.orientation.w = 1

            fruit_centroid_transformed = tf2_geometry_msgs.do_transform_pose(fruit_centroid_transformed, transform)

            print('transformed 3d point', fruit_centroid_transformed)
            print('untransformed 3d point', fruit_centroid)
            # print('image centroid', self.cx, self.cy)
            # print(cv_image.shape)
            print()
        except Exception as e:
            print('error in 2dto3d', e)
            pass

def main():
    try:
        ic = ImageConverter()
        while ic.cx is None:
            rospy.sleep(0.1)
        while not rospy.is_shutdown():
            ic.from_2d_to_3d()
            rospy.sleep(0.1)
        # rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
