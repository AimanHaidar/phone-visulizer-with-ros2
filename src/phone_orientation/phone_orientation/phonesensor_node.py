#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from phonesensors import PhoneSensorsClient
import threading
import numpy as np
from geometry_msgs.msg import Vector3

rot_raw = []
class PhoneSensorsNode(Node):
    def __init__(self):
        super().__init__('my_phone_sensors')
        self.rotation_vector_publisher = self.create_publisher(Vector3,'rot_vec',10)
        self.number_timer_ = self.create_timer(1.0, self.publish_rot_vec)
        self.get_logger().info("rotation vector publisher has been started.")

    def publish_rot_vec(self):

        """this funtion publisht the data come from the rotation vector
            in the phones"""
        
        msg = Vector3()
        msg.x = rot_raw[0]
        msg.y = rot_raw[1]
        msg.z = rot_raw[2]
        self.rotation_vector_publisher.publish(msg)
       

#function to read the sensors from the phone by SensorStreamer App     
def read_sensors():
    with PhoneSensorsClient("192.168.0.75", 8080) as client:
        for data in client:
            global rot_raw
            rot_raw = np.array(data.rot.values[0])

def main(args=None):
   rclpy.init(args=args)
   node = PhoneSensorsNode()

   # create new thread to run the read_sensor TCP client
   sensors_thread = threading.Thread(target=read_sensors,daemon=True)
   sensors_thread.start()

   rclpy.spin(node)
   rclpy.shutdown()

if __name__ == '__main__':
   main()
