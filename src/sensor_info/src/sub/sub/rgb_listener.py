# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import os
from sensor_msgs.msg import Image as ImageMsg

from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
    BreakBeamStatus as BreakBeamStatusMsg,
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    Part as PartMsg,
    PartPose as PartPoseMsg,
    Order as OrderMsg,
    AssemblyPart as AssemblyPartMsg,
    AGVStatus as AGVStatusMsg,
    AssemblyTask as AssemblyTaskMsg,
    AssemblyState as AssemblyStateMsg,
    CombinedTask as CombinedTaskMsg,
    VacuumGripperState,
)

from cv_bridge import CvBridge, CvBridgeError
import cv2

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_listner')
        self.subscription = self.create_subscription(
            ImageMsg,
            "/ariac/sensors/my_camera/rgb_image",
            self._my_camera_cb,
            qos_profile_sensor_data,
            # callback group,
        )
        self.subscription  # prevent unused variable warning
        
        # cv_bridge interface
        self._bridge = CvBridge()
        
        #save image - file path
        self.numberOfImagesSaved = 0
        self.script_dir = os.path.dirname(__file__) #bbsolute dir the script is in
        self.rel_path = f"images\image_{self.numberOfImagesSaved}.jpg"
        self.abs_file_path = os.path.join(self.script_dir,self.rel_path)
        
        

    def _my_camera_cb(self, msg: ImageMsg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)  
        
        #show the image in the cv2 image window
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        
        self.samveImage(cv_image)
    
    #save the image
    def samveImage(self, image):
        cv2.imwrite(self.abs_file_path, image)
        self.updateSaveFilePath()
   
        
    #create a new filepath for the next item we want to save
    def updateSaveFilePath(self):
        self.numberOfImagesSaved += 1
        self.script_dir = os.path.dirname(__file__) #bbsolute dir the script is in
        self.rel_path = f"images\image_{self.numberOfImagesSaved}.png"
        self.abs_file_path = os.path.join(self.script_dir,self.rel_path)
        
   

def main(args=None):

    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
