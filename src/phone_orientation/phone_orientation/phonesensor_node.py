#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from phonesensors import PhoneSensorsClient
import threading
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from math import pi,sqrt
import time

#rotatin_vector
rot_raw = []
class PhoneSensorsNode(Node):
    def __init__(self):
        super().__init__('phone_orientation')

        #phone IP and port parameters
        self.IPaddress_parameter = self.declare_parameter("phone_IP","192.168.0.75")
        self.port_parameter = self.declare_parameter("IP_port",8080)
        self.topic_parameter = self.declare_parameter("topic","phone_tf")
        self.IP_address = self.get_parameter("phone_IP").value
        self.port = self.get_parameter("IP_port").value
        self.topic = self.get_parameter("topic").value
        # publisher to publish the rotation vector to tf topic
        self.rotation_vector_publisher = self.create_publisher(TFMessage,self.topic,10) #to publish to the joint_states of rviz
        self.number_timer_ = self.create_timer(0.001, self.publish_rot_vec) 
        self.get_logger().info("rotation vector publisher has been started.")

        

    def publish_rot_vec(self):

        """this funtion publisht the data come from the rotation vector
            in the phone to the tf topic to show in rviz"""
        
        #handle IndexError until the Clint of phonesenors connect to the server app on phone
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base'      # parent frame
            t.child_frame_id = 'phone'       # child frame

            # Translation
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

            # Rotation quaternion (normalized)
            t.transform.rotation.x = float(-rot_raw[1])
            t.transform.rotation.y = float(rot_raw[0])
            t.transform.rotation.z = float(rot_raw[2])
            t.transform.rotation.w = float(sqrt(1-(rot_raw[0]**2+rot_raw[1]**2+rot_raw[2]**2)))

            tf_msg = TFMessage(transforms=[t])
            self.rotation_vector_publisher.publish(tf_msg)

        except Exception as e:
            self.get_logger().warn(str(e)+" sensorStream has not yet started!")

    #function to read the sensors from the phone by SensorStreamer App     
    def read_sensors(self):
        while True:
            try:
                with PhoneSensorsClient(self.IP_address,self.port) as client:
                    for data in client:
                        global rot_raw
                        rot_raw = np.array(data.rot.values[0])
                        print(rot_raw)
            except Exception as e:
                time.sleep(1)
                self.get_logger().warn("sensorStream is stopped!")
                

def main(args=None):
   rclpy.init(args=args)
   node = PhoneSensorsNode()

   # create new thread to run the read_sensor TCP client
   sensors_thread = threading.Thread(target=node.read_sensors,daemon=True)
   sensors_thread.start()

   rclpy.spin(node)
   rclpy.shutdown()

if __name__ == '__main__':
   main()
