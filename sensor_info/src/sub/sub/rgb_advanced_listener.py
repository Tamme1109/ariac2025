import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import re
import os
import time
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image as ImageMsg

from ariac_msgs.msg import (
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    BreakBeamStatus as BreakBeamStatusMsg,
)

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.info_subscription = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            "/ariac/sensors/my_other_camera/image",
            self._advanced_logical_camera_cb,
            qos_profile_sensor_data
        )
               
        self.image_RGB_subscription = self.create_subscription(
            ImageMsg,
            "/ariac/sensors/my_camera/rgb_image",
            self._rgb_camera_cb,
            qos_profile_sensor_data,
            # callback group,
        )
        # Subscriber to the breakbeam status topic
        self._breakbeam_sub = self.create_subscription(
            BreakBeamStatusMsg,
            '/ariac/sensors/my_breakbeam/status',
            self._breakbeam_cb,
            qos_profile_sensor_data
        )
        
        self.canSave = True
        # cv_bridge interface
        self._bridge = CvBridge()
        self.cv_image = None
        
        self.msg = ["","",""] #just something to save the message in
        self.numberOfPartsSaved = 0
        
        #for saving the information in json files
        self.script_dir = os.path.dirname(__file__) #bbsolute dir the script is in
        self.rel_path = f"images\{self.get_one_msg(0)}.png"
        self.abs_file_path = os.path.join(self.script_dir,self.rel_path)            
        
    #---Advanced Logical camera---#
    def _advanced_logical_camera_cb(self, msg : AdvancedLogicalCameraImageMsg):
        self.part_poses = msg.part_poses 
        self.sensor_pose = msg.sensor_pose
        ###cleaning up the messages
        #j = 0
        for index in range(len(self.part_poses)): #loop trhoug all the parts poses and add them to the current list self.msg
            self.set_msg(index, {f"part_nr:{self.numberOfPartsSaved}": self.cleanUpPartMsg(self.part_poses[index])}) #clean up the strings so tey are easier to read before saving it in a dictionary
            #j = index
            self.numberOfPartsSaved += 1
        #self.msg[j+1] = self.set_msg((j+1), self.cleanUpString(str(self.sensor_pose))) #add the sensor position at the end   
        
    def set_msg(self,index, msg):#strings are mutable and I don't want trouble
        try:
            self.msg[index] = msg
        except:
            self.msg.append(msg)
            
    def get_msg(self):
        return self.msg
    
    def get_one_msg(self, index):
        msg = self.msg[index]
        return msg
    
    ###The messages contains a lot of unnecessary information, so this will clean up the messages for the parts
    def cleanUpPartMsg(self, stringOfThings):
        values = []
        text = ((str(str(stringOfThings).replace("=", "= ")).replace("'", " ' ")).replace(")", " )")).replace(",", " , ") #make the numbers "stand" alone
        for t in text.split():
            try:
                values.append(float(t))
            except ValueError:
                pass
        vals = [int(values[0]), int(values[1]), [values[2], values[3], values[4]], [values[5], values[6], values[7]]]
        s = ""
        for val in vals:
            s += f"_{val}"
        return s      
        
    #create a new filepath for the next item we want to save
    def updateSaveFilePath(self):
        self.script_dir = os.path.dirname(__file__) #absolute dir the script is in
        self.rel_path = f"images\{self.get_one_msg(0)}.png"
        self.abs_file_path = os.path.join(self.script_dir,self.rel_path)

    #---RGB Camera---#
    def _rgb_camera_cb(self, msg: ImageMsg):
        try:
            self.cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)  
        
        #show the image in the cv2 image window
        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(3)
        #self.samveImage(cv_image)
        
    def savePart(self, rgbImage):
        cv2.imwrite(self.abs_file_path, rgbImage)
        self.updateSaveFilePath()
        self.get_logger().info('saved')
    
    #---Break Beam---#
    def _breakbeam_cb(self, msg):
        data = str(msg)
        #self.get_logger().info(f'detected something: {data[-5:-1]}')
        if data[-5:-1] == 'True' and self.canSave:
            self.get_logger().info('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa')
            self.canSave = False #make sure to only save one picture of each part
            self.savePart(self.cv_image)
        elif data[-6:-1] == 'False':
            self.canSave = True

def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()
    rclpy.spin(data_collector)
    data_collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
