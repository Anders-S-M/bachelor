# Basic ROS 2 program to do line detection on camera feed
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np # Python library for math
 
class CameraNode(Node):
  """
  Create an CameraNode class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('Camera_Node')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    
    # Create the publisher. This publisher will publish an Image
    # to the lines topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'Lines', 10)

    # publisher to send data to system node regard line status
    self.publisher = self.create_publisher(Bool, 'lines_bool', 10)
    self.timer = self.create_timer(0.5, self.timer_callback)


    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    # For capturing status
    self.bool = False

    self.get_logger().info('Camera node is ready...')
   


  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')

    # Convert ROS Image message to OpenCV image
    try:
      img = self.br.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      self.get_logger().info(e)

    # Convert the downsampled image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Binary image
    (_, binary) = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
    
    # erode the black lines
    # Creating kernel
    kernel = np.ones((5, 5), np.uint8)
    
    # Using cv2.erode() method 
    binary = cv2.dilate(binary, kernel, 1) 

    # Apply edge detection method on the image
    edges = cv2.Canny(binary, 50, 150, apertureSize=3)

    # This returns an array of r and theta values
    votes = 100
    voteincr = 20
    maxiterations = 3
    lines = cv2.HoughLines(edges, 1, np.pi/180, votes)

    while(1):        
      if maxiterations == 1:
        break

      if lines is not None:
          l = len(lines)
          if l == 2 or voteincr == 0:
              break
          elif l > 2:
              votes += voteincr
          elif l < 2:
              votes -= voteincr
          voteincr -= 1
      else: 
          votes -= voteincr
      
      lines = cv2.HoughLines(edges,1,np.pi/180, votes)
      
      maxiterations -= 1

        


    linecounter = 0

    # for merging lines
    rhothreshold = 5
    thetathreshold = 0.2

    # for testing
    #img = binary

    # Draw the detected lines on the original frame
    # first ensure there are lines
    if lines is not None:
       sortedlines = sorted(lines, key=lambda x: x[0][0])
       newlines = [sortedlines[0]]
       if len(lines) > 1:
        oldrho = sortedlines[0][0][0]
        oldtheta = sortedlines[0][0][1]
        for line in range(len(sortedlines)-1):
            rho = sortedlines[line + 1][0][0]
            theta = sortedlines[line + 1][0][1]
            # if two lines are ontop not going same directiong or not ontop of each other at all
            if abs(rho - oldrho) > rhothreshold or abs(theta - oldtheta) > thetathreshold:
                newlines.append(sortedlines[line + 1])
                oldrho = rho
                oldtheta = theta

        linecoordinates = []
        for line in newlines:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            cv2.line(img, (x1,y1), (x2,y2), (0,0,255), 1)
            linecounter += 1
            linecoordinates.append((x1,y1) + (x2,y2))

    # add number to image
    cv2.putText(img, str(linecounter), (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Checking whether the line is correctly placed
    if(linecounter == 2):
          linecoordinates = [int(sum(x)/2) for x in zip(linecoordinates[0],linecoordinates[1])]
          goalcoordinates = [421,1037,543,-967]
          goalcoordinates = [abs(x-y) for x, y in zip(linecoordinates,goalcoordinates)]
          accuracy = sum(goalcoordinates)
          if accuracy < 400:
            cv2.line(img, (linecoordinates[0],linecoordinates[1]), (linecoordinates[2],linecoordinates[3]), (0,255,0), 1)
            self.bool = True
          else:
            cv2.line(img, (linecoordinates[0],linecoordinates[1]), (linecoordinates[2],linecoordinates[3]), (0,0,255), 1)
            self.bool = False
    else:
       self.bool = False

    # All the changes made in the input image are finally published
    self.publisher_.publish(self.br.cv2_to_imgmsg(img))

  # Sending message containing the status of the line, used in system_node
  def timer_callback(self):
          msg = Bool()
          msg.data = self.bool  # Status of lines
          self.publisher.publish(msg)
          self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  Camera_Node = CameraNode()
  
  # Spin the node so the callback function is called.
  rclpy.spin(Camera_Node)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  Camera_Node.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
