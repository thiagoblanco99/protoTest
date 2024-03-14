import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import ParameterException
import matplotlib.pyplot as plt

class ScanFinder(Node):
    def __init__(self):
        super().__init__('laser_fov_cutter')

        descript_param_topic_input= ParameterDescriptor(description='This parameter adjusts the topic from which the laser scan is subscribed. The default value is "fov".')
        self.declare_parameter('topic_input', 'fov_scan', descript_param_topic_input)
        self.subscription = self.create_subscription(
            LaserScan,
            self.get_parameter('topic_input').get_parameter_value()._string_value,  # Subscribe to the original laser scan topic
            self.scan_callback,
            10  # QoS profile
        )
    

    def scan_callback(self, msg):
        # read which is the fov of the scan
        angle_min=msg.angle_min
        angle_max=msg.angle_max
        fov_grad=angle_max*180/np.pi-angle_min*180/np.pi
        num_ranges = len(msg.ranges)
        ranges = np.array(msg.ranges)
        angle_increment=msg.angle_increment

        theta= 0 # in degrees
        # I have to find the index of the range that is associated with the angle theta        
        index=int(theta-angle_min/angle_increment)

        print("The range associated with the angle ",theta," is ",ranges[index])
        print("Angle increment: ",angle_increment)
        print("Angle increment in degrees: ",angle_increment*180/np.pi)
        print("Number of ranges: ",num_ranges)
        print("Fov in degrees: ",fov_grad)
        print("Angle min in degrees: ",angle_min*180/np.pi)
        print("Angle max in degrees: ",angle_max*180/np.pi)
        print("Angle min in radians: ",angle_min)
        print("Angle max in radians: ",angle_max)
        # Calculate the angles for each range
        angles = np.arange(angle_min, angle_max, angle_increment)

        # Plot the ranges in a polar plot
        fig = plt.figure()
        ax = fig.add_subplot(111, polar=True)
        ax.scatter(angles, ranges)
        ax.set_xlim(angle_min, angle_max)
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
        ax.set_rlabel_position(0)
        ax.set_xlabel('Angle (degrees)')
        ax.set_ylabel('Range')
        ax.set_title('Laser Scan Ranges')
        ax.grid(True)
        plt.show()
def main(args=None):
    rclpy.init(args=args)

    laser_scan_finder = ScanFinder()

    rclpy.spin_once(laser_scan_finder)

    laser_scan_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
