#import matplotlib.pyplot as plt
import random
import json
from io import StringIO 
import os

import re

class AAAAA:

    def __init__(self):
        self.part1 = "{ariac_msgs.msg.PartPose(part=ariac_msgs.msg.Part(color=1, type=12), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.8550007468284273, y=0.6164772436066459, z=0.13498237281626518), orientation=geometry_msgs.msg.Quaternion(x=0.5000018616682316, y=-0.5000001177088924, z=0.49999993514050056, w=-0.4999980854752264)))" 
        self.part2 = "ariac_msgs.msg.PartPose(part=ariac_msgs.msg.Part(color=3, type=13), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.9250003189085374, y=0.23715737166987902, z=0.029984546719540428), orientation=geometry_msgs.msg.Quaternion(x=0.6123727136688034, y=0.3535447161264618, z=-0.6123713872805299, w=-0.35356339925547253)))"
        self.msg = [self.part1, self.part2]
        
        self.numberOfPartsSaved = 0
        self.script_dir = os.path.dirname(__file__) #absolute dir the script is in
        self.rel_path = f"savedInfo/saved_part_{self.numberOfPartsSaved}.json"
        self.abs_file_path = os.path.join(self.script_dir,self.rel_path)
        
       

        
    def get_msg(self):
        return self.msg
    
    def get_one_msg(self, index):
        msg = self.msg[index]
        return msg
    
    def set_msg(self,index, msg):
        self.msg[index] = msg
    
    def cleanUpString(self, string):
        pattern = r',\s*|\s+|(?<=\d)\s+(?=\d)' ## Split by commas, spaces, or between numbers

        result = str(re.split(pattern, string)).replace("ariac_msgs.msg.", "")
        result = (((result.replace("PartPose(", "")).replace("geometry_msgs.msg.Pose(position=geometry_msgs.msg." , "")).replace("geometry_msgs.msg.Point", "")).replace("geometry_msgs.msg.Quaternion", "")
        result = (result.replace("Part", "")).replace("Point", "")
        result = result[:-4] #get rid of the extra parenteces at the end
        return result
    
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
        data = dict(zip(keys, vals))
        return data
    
    def generate_image_filename(self, part_data: dict, index=None):
        """
        Generates a descriptive image filename using part type and color.
        
        Parameters:
        - part_data (dict): Dictionary with keys like "type" and "color" (parsed from message)
        - index (int, optional): Image number to append (for uniqueness and ordering)
        
        Returns:
        - str: A filename like 'sensor_red_001.png'
        """

        # These lists act as ID → name lookup tables
        types_of_parts = ['pump', 'battery', 'regulator', 'sensor']  # type_id 0–3
        colours = ['blue', 'red', 'green', 'orange', 'purple']       # color_id 0–4

        # Extract type and color IDs
        type_id = part_data["type"]
        color_id = part_data["color"]

        # Convert to readable strings (fallback to ID if unknown)
        try:
            type_name = types_of_parts[type_id]
        except IndexError:
            type_name = f"type{type_id}"

        try:
            color_name = colours[color_id]
        except IndexError:
            color_name = f"color{color_id}"

        # Build the filename with zero-padded index
        if index is not None:
            filename = f"{type_name}_{color_name}_{index:03d}.png"
        else:
            filename = f"{type_name}_{color_name}.png"

        return filename


    
    def cleanUpSensorMsg(self, stringMsg):
        values = []
        text = ((str(str(stringMsg).replace("=", "= ")).replace("'", " ' ")).replace(")", " )")).replace(",", " , ") #make the numbers "stand" alone
        for t in text.split():
            try:
                values.append(float(t))
            except ValueError:
                pass

    def save_msg_to_json(self, dictionary):
        with open(self.abs_file_path, 'w', encoding='utf-8') as f:
            json.dump({"part": {f"part_nr:{self.numberOfPartsSaved}" :dictionary}}, f, ensure_ascii=False, indent=4)
        self.updateSaveFilePath()
        
    def updateSaveFilePath(self):
        self.numberOfPartsSaved += 1
        self.script_dir = os.path.dirname(__file__) #bbsolute dir the script is in
        self.rel_path = f"savedInfo/saved_part_{self.numberOfPartsSaved}.json"
        self.abs_file_path = os.path.join(self.script_dir,self.rel_path)
        
        
    def save_msg(self, newdata):
        #file location
        script_dir = os.path.dirname(__file__) #absolute dir the script is in
        rel_path = "savedInfo/saved_parts.json"
        abs_file_path = os.path.join(script_dir, rel_path)
        
        #get all the old info
        f = open(abs_file_path)
        data = json.load(f)
        data = data["parts"]
        f.close()
            
        data.update(newdata["parts"])
        
        with open(abs_file_path, 'w', encoding='utf-8') as f:
            json.dump({"parts": data}, f, ensure_ascii=False, indent=4)
            
        f.close()
    

def main(args=None):
    a = AAAAA()
    
    """msg1 = a.cleanUpString(a.get_msg(0))
    print(msg1)
    
    msg2 = a.cleanUpString(a.get_msg(1))
    print(msg2)"""
    
    """ for i in range(len(a.msg)):
        a.set_msg(i, a.cleanUpString(a.get_msg(i)))"""
        


#for i, msg in enumerate(a.get_msg()):
 

#   cleaned = a.cleanUpPartMsg(msg)
 #   a.set_msg(i, cleaned)

  #  filename = a.generate_image_filename(cleaned, index=i)
   # print(f"Image would be saved as: {filename}")

    # Should be able to use for testing i guess


""" 
def main(args=None):
    \"\"\"
    Test the part parsing and filename generation without using ROS.
    This just prints filenames based on the example messages inside AAAAA.
    \"\"\"
    a = AAAAA()  # Create an instance of your parser

    # Loop over all the example part messages
    for i, msg in enumerate(a.get_msg()):
        # Parse the string into usable part data
        cleaned = a.cleanUpPartMsg(msg)

        # Save the cleaned data back into the message list (optional)
        a.set_msg(i, cleaned)

        # Generate a smart filename like 'battery_red_001.png'
        filename = a.generate_image_filename(cleaned, index=i)

        # Print out the result for checking
        print(f"Image would be saved as: {filename}")
"""


    i = 0
    for msg in a.get_msg():
        cleaned = a.cleanUpPartMsg(msg)
        a.set_msg(i, cleaned)
        i += 1
        
   
    #for m in a.get_msg():
       # print(m["rotation"])
       
    script_dir = os.path.dirname(__file__)
    rel_path = "savedInfo/saved_part_0.json"
    abs_file_path = os.path.join(script_dir, rel_path)
    f = open(abs_file_path)
    newdata = json.load(f)
    f.close()
    
    a.save_msg(newdata)
    
    script_dir = os.path.dirname(__file__)
    rel_path = "savedInfo/saved_part_1.json"
    abs_file_path = os.path.join(script_dir, rel_path)
    f = open(abs_file_path)
    newdata = json.load(f)
    f.close()
    a.save_msg(newdata)
    
    script_dir = os.path.dirname(__file__)
    rel_path = "savedInfo/saved_part_2.json"
    abs_file_path = os.path.join(script_dir, rel_path)
    f = open(abs_file_path)
    newdata = json.load(f)
    f.close()
    a.save_msg(newdata)
    
   
    """ script_dir = os.path.dirname(__file__) #bbsolute dir the script is in
    rel_path = "savedInfo/saved_part_0.json"
    abs_file_path = os.path.join(script_dir, rel_path)
    f = open(abs_file_path)
    
    data = json.load(f)
    data = data["parts"]
    #print(data)
    f.close()
    script_dir = os.path.dirname(__file__) #bbsolute dir the script is in
    rel_path = "savedInfo/saved_part_1.json"
    abs_file_path = os.path.join(script_dir, rel_path)
    d = open(abs_file_path)
    newdata = json.load(d)
    print(newdata)
    d.close()
    data.update(newdata["parts"])
    
    print(data)

    with open(abs_file_path, 'w', encoding='utf-8') as f:
        json.dump({"parts": data}, f, ensure_ascii=False, indent=4)"""
  

    
   
   
    
   

if __name__ == '__main__':
    main()








 
        
            
