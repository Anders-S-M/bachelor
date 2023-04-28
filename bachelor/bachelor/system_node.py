# Basic ROS 2 program to do line detection on camera feed
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.action import ActionServer
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import Bool
from bachelor_interfaces.srv import SetDevice
from bachelor_interfaces.action import CondSetDevice
import numpy as np # Python library for math
 
class SystemNode(Node):
    """
    Create an SystemNode class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('System_Node')
        
        # Create the subscriber. This subscriber will receive an boolean
        self.subscription = self.create_subscription(
        Bool, 
        'lines_bool', 
        self.listener_callback, 
        10)
        self.subscription # prevent unused variable warning

        # Action receiving action call to check camera and start motor
        self._action_server = ActionServer(self, CondSetDevice, 'cond_set_device', self.handle_cond_set_device)

        # Service client for call motor to start
        self.cli = self.create_client(SetDevice, 'set_device')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for control_node...')
        self.req = SetDevice.Request()
        
        # Saving latest data from camera node
        self.linestruefalse = True

        

    def listener_callback(self, msg):
        # Display the message on the console
        self.linestruefalse = msg.data
        if self.linestruefalse:
            self.get_logger().info("We see lines :)")
        else:
            self.get_logger().info("We don't see lines :(")



    def handle_cond_set_device(self, goal_handle):
        self.get_logger().info('Set device requested checking camera...')
        response = CondSetDevice.Result()
        if self.linestruefalse:
            self.get_logger().info(str(self.linestruefalse))
            self.get_logger().info('Lines correctly placed, starting motor')
            # Send service call to motor
            self.req.id = goal_handle.request.id
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            success = self.future.result()
            # Log response from motor
            if success:
                self.get_logger().info('Device placed correctly, hopefully...')
            else: 
                self.get_logger().info('Device moved but no cable found...')
        else:
            self.get_logger().info('Lines not correctly placed, try again...')
            success = False
        
        # Since reponse.check = success doesn't work...
        if success:
            response.check = True
        else:
            response.check = False

        goal_handle.succeed()
        return response

    
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  System_Node = SystemNode()
  
  # Spin the node so the callback function is called.
  rclpy.spin(System_Node)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  System_Node.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
