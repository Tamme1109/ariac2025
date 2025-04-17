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
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import re
import os

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


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_listner')
        self.subscription = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            "/ariac/sensors/my_other_camera/image",
            self._my_other_camera_cb,
            qos_profile_sensor_data
           
        )
        self.subscription  # prevent unused variable warning
        
        self.msg = ["","",""] #I just want something here
        
        #for saving the information in json files
        self.numberOfPartsSaved = 0
        self.script_dir = os.path.dirname(__file__) #absolute dir the script is in
        self.rel_path = f"savedInfo\savedsaved_part_{self.numberOfPartsSaved}.json"
        self.abs_file_path = os.path.join(self.script_dir,self.rel_path)

        """#for saving al informatio in one file
        self.script_dir = os.path.dirname(__file__) #absolute dir the script is in
        self.rel_path = "savedInfo/saved_parts.json"
        self.abs_file_path = os.path.join(self.script_dir, self.rel_path)
        self.numberOfPartsSaved = 0"""
        
    def _my_other_camera_cb(self, msg : AdvancedLogicalCameraImageMsg):
        self.part_poses = msg.part_poses 
        self.sensor_pose = msg.sensor_pose
        
        ###cleaning up the messages
        j = 0
        for index in range(len(self.part_poses)): #loop trhoug all the parts poses and add them to the current list self.msg
            self.set_msg(index, {f"part_nr:{self.numberOfPartsSaved}": self.cleanUpPartMsg(self.part_poses[index])}) #clean up the strings so tey are easier to read before saving it in a dictionary
            j = index
            self.numberOfPartsSaved += 1
        #self.msg[j+1] = self.set_msg((j+1), self.cleanUpString(str(self.sensor_pose))) #add the sensor position at the end
        
        ###save the messages in a json file
        for item in self.get_msg():
            self.save_msg_to_json(item)
            
        message = self.get_msg()
        self.get_logger().info('part poses: "%s",\n "%s"\n sensor pose: "%s"\n' % (message[0], message[1], message[2])) 
        
        
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
        keys = ["color", "type", "pos", "rotation"]
        data = list(zip(keys, vals))
        return data
    
    ###The message contains a lot of unnecessary information, so we get rid of what's unnecessary for the msg about the sensor position
    def cleanUpString(self, string):
        pattern = r',\s*|\s+|(?<=\d)\s+(?=\d)' ## Split by commas, spaces, or between numbers

        result = str(re.split(pattern, string)).replace("ariac_msgs.msg.", "")
        result = (((result.replace("PartPose(", "")).replace("geometry_msgs.msg.Pose(position=geometry_msgs.msg." , "")).replace("geometry_msgs.msg.Point", "")).replace("geometry_msgs.msg.Quaternion", "")
        result = (result.replace("Part", "")).replace("Point", "")
        result = result[:-4] #get rid of the extra parenteces at the end
        return result
    
    ###For saving the messages
    def save_msg_to_json(self, dictionary):
        with open(self.abs_file_path, 'w', encoding='utf-8') as f:
            json.dump({f"part_nr:{self.numberOfPartsSaved}": dictionary}, f, ensure_ascii=False, indent=4)
        f.close()
        self.updateSaveFilePath()
        
        
    #create a new filepath for the next item we want to save
    def updateSaveFilePath(self):
        self.numberOfPartsSaved += 1
        self.script_dir = os.path.dirname(__file__) #bbsolute dir the script is in
        self.rel_path = f"savedInfo\savedsaved_part_{self.numberOfPartsSaved}.json"
        self.abs_file_path = os.path.join(self.script_dir,self.rel_path)
        
    def save_msg(self, newdata):
        #load the information from the file
        f = open("savedInfo/jsonsaved_parts.")
        data = json.load(f)
        data = data["parts"]
        f.close()
            
        data.update(newdata)
        
        #write new data
        with open(self.abs_file_path, 'w', encoding='utf-8') as f:
            json.dump({"parts": data}, f, ensure_ascii=False, indent=4)
            
        f.close()
    

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
