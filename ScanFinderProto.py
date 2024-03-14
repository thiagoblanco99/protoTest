import odm_library
import zmq
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import ros2_numpy as rnp
import math
import matplotlib.pyplot as plt
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import ParameterException
import ThreadWorkerScan
import queue
from threading import Lock

class ScanFinderProto(Node):
    def __init__(self):
        super().__init__('laser_fov_cutter')
        descript_size = ParameterDescriptor(description='This parameter adjusts the side of the window. The default value is 10.')
        self.declare_parameter('size', 20, descript_size)
        descript_min_points= ParameterDescriptor(description='This parameter adjusts the minimum amount of samples to consider a valid average. The default value is 10.')
        self.declare_parameter('min_points', 10, descript_min_points)
        descript_average_size= ParameterDescriptor(description='This parameter adjusts the amount of messages to average. The default value is 5.')
        self.declare_parameter('average_size', 5, descript_average_size)
        descript_param_topic_input= ParameterDescriptor(description='This parameter adjusts the topic from which the laser scan is subscribed. The default value is "fov".')
        self.declare_parameter('topic_input', 'scan', descript_param_topic_input)
        
        self.subscription = self.create_subscription(
            LaserScan,
            self.get_parameter('topic_input').get_parameter_value()._string_value,  # Subscribe to the original laser scan topic
            self.scan_callback,
            10  # QoS profile
        )

        self.msg_average = self.get_parameter('average_size').get_parameter_value()._integer_value 
        self.mutex_cola =Lock()
        self.cola = queue.Queue()
        self.scans=[]
        self.theta=np.nan
        self.average_distance=np.full(self.msg_average, np.nan)
        self.index

        thread_subscriber = ThreadWorkerScan.SolutionSuscriberThread("localhost", 5000 , self.cola, self.mutex_cola)
        thread_publisher = ThreadWorkerScan.SolutionPublisherThread(5000)
        thread_subscriber.start()
        thread_publisher.start()


    def scan_callback(self, msg):

        angle_min=msg.angle_min 
        angle_max=msg.angle_max 
        num_ranges = len(msg.ranges)
        ranges = np.array(msg.ranges)
        angle_increment=msg.angle_increment
        self.scans.append(ranges)
        if len(self.scans) > self.msg_average: # si son mas que que las que quiero usar,  borro el que primero se cargo , osea el mas viejo
            self.scans.pop(0)
        
        if (not self.cola.empty()):
            self.mutex_cola.acquire()
            #####################
            
            self.theta=np.pi/180*self.cola.get() ## Esto se tiene que cambiar por algo que no use queue
            
            #####################            
            self.mutex_cola.release()

            ##### HAY QUE HACER LA CONVERSION DE ANGULO A INDICE PERO HAY QUE CONOCER
            ##### DONDE ES EL ORIGEN DE COORDENADAS. 


            ## Tengo que hacer el promediado
            for i in range (len(self.scans)):
                self.index=int(self.theta-angle_min/angle_increment)#angle min es un numero negativo.
            ## TO DO: Hacer el promediado de los ultimos msg_average mensajes

        print("The range associated with the angle ",self.theta*180/np.pi," is ",ranges[self.index])
        
def main(args=None):
    rclpy.init(args=args)
    your_node = ScanFinderProto()
    rclpy.spin(your_node)
    your_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()