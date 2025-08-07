#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from phonesensors import PhoneSensorsClient
import threading
import numpy as np
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from math import pi

#rotatin_vector
rot_raw = []
class PhoneSensorsNode(Node):
    def __init__(self):
        super().__init__('my_phone_sensors')
        self.rotation_vector_publisher = self.create_publisher(JointState,'joint_states',10) #to publish to the joint_states of rviz
        self.number_timer_ = self.create_timer(0.001, self.publish_rot_vec) 
        self.get_logger().info("rotation vector publisher has been started.")

        #phone IP and port parameters
        self.IPaddress_parameter = self.declare_parameter("phone_IP","192.168.0.75")
        self.port_parameter = self.declare_parameter("IP_port",8080)
        self.IP_address = self.get_parameter("phone_IP").value
        self.port = self.get_parameter("IP_port").value

    def publish_rot_vec(self):

        """this funtion publisht the data come from the rotation vector
            in the phone to the joint_states topic to show in rviz"""
        
        #handle IndexError until the Clint of phonesenors connect to the server app on phone
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['y', 'r', 'p']
            msg.position = [pi*rot_raw[2], pi*rot_raw[0], -pi*rot_raw[1]]
            self.rotation_vector_publisher.publish(msg)

        except IndexError as ie:
            self.get_logger().error("No data found!!")

    #function to read the sensors from the phone by SensorStreamer App     
    def read_sensors(self):
        try:
            with PhoneSensorsClient(self.IP_address,self.port) as client:
                for data in client:
                    global rot_raw
                    rot_raw = np.array(data.rot.values[0])
        except Exception as e:
            print(e)

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
