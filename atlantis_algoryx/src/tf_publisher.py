#!/usr/bin/env python3

import sys
import os
import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform 
from agxROS2 import PublisherAgxMsgsAny, AnyMessageBuilder
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
sys.path.append(os.getenv("AGX_DIR") + "/data/python/tutorials")
paths = ["/data/python/agxTerrain", "/data/python/"]
for p in paths:
    folder_path = os.getenv("AGX_DIR") + p
    if folder_path not in sys.path:
        sys.path.append(folder_path)

class TFPublisher(Node):

    def __init__(self, ID):
        super().__init__('tf_publisher')
        self.tf_pub = PublisherAgxMsgsAny('robot' + str(ID) + '/bucket_tf')
        self.tf_sub = self.create_subscription(TFMessage, 'robot' + str(ID) + '/tf',self.tf_callback, 1000)

        # Initialize the buffer
        self.tf_buffer = Buffer()

        # Initialize the transform listener without the subscriber
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(5.0, self.timer_callback)

        self.target_frame = 'map'
        self.source_frame = 'base_link'

    def tf_callback(self,msg):
        builder = AnyMessageBuilder()
        transforms = msg.transforms
        # for tf in transforms:
        #     if(tf.child_frame_id == "bucket_link"):
        #         pos = tf.transform.translation
        #         rot = tf.transform.rotation
        #         ##Position
        #         builder.writeFloat32(pos.x)
        #         builder.writeFloat32(pos.y)
        #         builder.writeFloat32(pos.z)
        #         print(pos.x)
        #         print(pos.y)
        #         print(pos.z)
        #         ##Orientation
        #         builder.writeFloat32(rot.x)
        #         builder.writeFloat32(rot.y)
        #         builder.writeFloat32(rot.z)
        #         builder.writeFloat32(rot.w)
        # self.tf_pub.sendMessage(builder.getMessage())
        
    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                "map",  # to frame
                "boom_link",  # from frame
                now
            )
            self.get_logger().info(f"Transform: {transform}")
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TFPublisher(1)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()