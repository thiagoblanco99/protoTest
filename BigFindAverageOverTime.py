#!/usr/bin/env python3
import odm_library
import zmq
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp
import math
import matplotlib.pyplot as plt
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import ParameterException
import ThreadWorker
import queue
from threading import Lock
import stereo_msgs.msg as DisparityImage
import cv2
##Tengo que chequear que cuando tira un valor del borde se me va de rango.



## voy guardando los ultimos 5 depthmaps en una lista, y promedio los valores de cada uno de ellos cuando es necesario,
#Lo bueno es que puedo cambiar de promedio circular a cuadrado y de popsicion en cualquieer momento y no pasa nada.
# Los resultados siguen siendo validos.


class FindAverage(Node):
    def __init__(self):
        super().__init__('find_average')
        descript_selector = ParameterDescriptor(description='If true, you will be averaging into a circle, if false, you will be averaging into a square. The default value is true.')
        self.declare_parameter('selector', True, descript_selector)
        descript_size = ParameterDescriptor(description='This parameter adjusts the side of the square or radius of the circle in pixels. The default value is 10.')
        self.declare_parameter('size', 20, descript_size)
        descript_topic_input = ParameterDescriptor(description='This parameter adjusts the topic from which the point cloud is subscribed. The default value is "points2".')
        self.declare_parameter('topic_input', 'points2', descript_topic_input)
        descript_min_distance = ParameterDescriptor(description='This parameter adjusts the minimum distance to consider a point valid. The default value is 2 meters.')
        self.declare_parameter('min_distance', 2.0, descript_min_distance)
        descript_max_distance = ParameterDescriptor(description='This parameter adjusts the maximum distance to consider a point valid. The default value is 20 meters.')
        self.declare_parameter('max_distance', 20.0, descript_max_distance)
        descript_min_points= ParameterDescriptor(description='This parameter adjusts the minimum amount of points to consider a valid average. The default value is 10.')
        self.declare_parameter('min_points', 10, descript_min_points)
        descript_average_size= ParameterDescriptor(description='This parameter adjusts the amount of messages to average. The default value is 5.')
        self.declare_parameter('average_size', 5, descript_average_size)


        self.mutex_cola =Lock()
        self.cola = queue.Queue()


        thread_subscriber = ThreadWorker.SolutionSuscriberThread( "10.42.0.1", 5000 , self.cola, self.mutex_cola)
        thread_publisher = ThreadWorker.SolutionPublisherThread(5000)
        thread_subscriber.start()
        thread_publisher.start()
        
        self.row= 240
        self.col= 320
        self.size= self.get_parameter('size').get_parameter_value()._integer_value
        self.subscription = self.create_subscription(
            PointCloud2,  # msg
            self.get_parameter('topic_input').get_parameter_value()._string_value,  # topic to subscribe
            self.callback,
            10  # QoS 
        )
        self.msg_average = self.get_parameter('average_size').get_parameter_value()._integer_value 
        self.depthmaps = [] #lista de depth maps
        self.average_distance = np.full(self.msg_average, np.nan) 
        self.min_dist= self.get_parameter('min_distance').get_parameter_value()._double_value
        self.max_dist= self.get_parameter('max_distance').get_parameter_value()._double_value
    
        self.publisher_ = self.create_publisher(DisparityImage.DisparityImage, 'disparity_dist', 10)



    def callback(self, msg):
        array = rnp.point_cloud2.point_cloud2_to_array(msg)['xyz']
        x, y, z = np.split(array, 3, axis=1)
        #calculate radial distance with x,y,z
        r = np.sqrt(x**2 + y**2 + z**2)


        r = np.where((r > self.max_dist) | (r < self.min_dist), np.nan, r)  
        depth_map_reshape = np.reshape(r, (480, 640))

        self.depthmaps.append(depth_map_reshape) # cargo los depthmaps en la lista
        if len(self.depthmaps) > self.msg_average: # si son mas que que las que quiero usar,  borro el que primero se cargo , osea el mas viejo
            self.depthmaps.pop(0)
            #print(self.distances)
        if (not self.cola.empty()):
            self.mutex_cola.acquire()
            #####################
            
            aux=self.cola.get() ## Esto se tiene que cambiar por algo que no use queue
            
            #####################            
            self.mutex_cola.release()

            x_y_values = aux
            self.row = int(x_y_values[0])
            self.col = int(x_y_values[1])       
            print("Row and column values changed to:", self.row, self.col)

            
        if self.get_parameter('selector').get_parameter_value()._bool_value:
            for i in range(len(self.depthmaps)):
                self.average_distance[i]= odm_library.circle_average(self.depthmaps[i], (self.row, self.col), self.size, min_points=10, plot=False)
        else:
            for i in range(len(self.depthmaps)):
                self.average_distance[i]= odm_library.square_average(self.depthmaps[i], (self.row, self.col), self.size, min_points=10, plot=False)

        res = np.nanmean(self.average_distance)
        dist_string = "{:.2f}".format(res)


        disparity_msg = DisparityImage.DisparityImage()
        disparity_msg.header = msg.header
        disparity_msg.image.header = msg.header
        disparity_msg.image.height = 480
        disparity_msg.image.width = 640
        disparity_msg.image.encoding = "32FC1"#'mono16'
        disparity_msg.image.is_bigendian = 0
        disparity_msg.image.step = 640*4
        disparity_msg.f = 1161.15
        disparity_msg.t = 0.6
        disparity_msg.min_disparity = float(self.min_dist)
        disparity_msg.max_disparity = float(self.max_dist)
        disparity_msg.delta_d = 1.
        disparity_msg.valid_window.x_offset = 0
        disparity_msg.valid_window.y_offset = 0
        disparity_msg.valid_window.width = 640
        disparity_msg.valid_window.height = 480
        disparity_msg.image.data = depth_map_reshape.tobytes()
        # Convert the disparity image data to a numpy array
        image_data = np.frombuffer(disparity_msg.image.data, dtype=np.float32)
        image_data = image_data.reshape((disparity_msg.image.height, disparity_msg.image.width))

        
        # Marco donde se esta haciendo el promediado
        if self.get_parameter('selector').get_parameter_value()._bool_value:
            cv2.circle(image_data, (self.col, self.row), self.size, (255, 0, 0), 2)
            cv2.putText(image_data, "Distancia ="+dist_string, (320,400), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        else:
            dim = int(self.size/2)
            start_row = max(0, int(self.row - dim))
            end_row = min(depth_map_reshape.shape[0]-1, int(self.row + dim))
            start_col = max(0, int(self.col - dim))
            end_col = min(depth_map_reshape.shape[1]-1, int(self.col + dim))
            cv2.rectangle(image_data, (start_col,start_row), (end_col,end_row), (255, 0, 0), 2)
            cv2.putText(image_data, "Distancia ="+dist_string, (320,400), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        
        # Convert the image data back to bytes and assign it to the disparity image message
        disparity_msg.image.data = image_data.tobytes()

        self.publisher_.publish(disparity_msg)
        print("Disparity message published")






def main(args=None):
    rclpy.init(args=args)
    your_node = FindAverage()
    rclpy.spin(your_node)
    your_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()