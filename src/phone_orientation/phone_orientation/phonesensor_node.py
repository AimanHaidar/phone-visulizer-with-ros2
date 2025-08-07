#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from phonesensors import PhoneSensorsClient
import threading
import numpy as np

class PhoneSensorsNode(Node):
   def __init__(self):
       super().__init__('my_phone_sensors')

#function to read the sensors from the phone by SensorStreamer App     
def read_sensors():
    with PhoneSensorsClient("192.168.0.75", 8080) as client:
        for data in client:
            rot_raw = np.array(data.rot.values[0])
            print("rot: ",rot_raw)

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
