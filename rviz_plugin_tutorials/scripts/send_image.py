#!/usr/bin/env python3

import cv2
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Thread


def send_image():
    rclpy.init()
    node = rclpy.create_node('image_publisher')

    image_topic = '/image_topic'
    image_pub = node.create_publisher(Image, image_topic, 10)
    cv_bridge = CvBridge()

    # Spin in a separate thread
    thread = Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(0.1)

    cap = cv2.VideoCapture('/home/gsk/install/2020-08-03-123458744.mp4')  # Replace 'path_to_your_video.mp4' with your video file path

    while rclpy.ok():
        ret, frame = cap.read()
        if not ret:
            break

        image_msg = cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.frame_id = 'image_frame'
        image_msg.header.stamp = node.get_clock().now().to_msg()
        image_pub.publish(image_msg)

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    send_image()
