import json
import os


"""script_dir = os.path.dirname(__file__) #absolute dir the script is in
rel_path = "savedInfo/saved_part_0.json"
abs_file_path = os.path.join(script_dir, rel_path)

f = open(abs_file_path)"""

part1 = "{ariac_msgs.msg.PartPose(part=ariac_msgs.msg.Part(color=1, type=12), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.8550007468284273, y=0.6164772436066459, z=0.13498237281626518), orientation=geometry_msgs.msg.Quaternion(x=0.5000018616682316, y=-0.5000001177088924, z=0.49999993514050056, w=-0.4999980854752264)))" 


def cleanUpPartMsg(stringOfThings):
    values = []
    text = ((str(str(stringOfThings).replace("=", "= ")).replace("'", " ' ")).replace(")", " )")).replace(",", " , ") #make the numbers "stand" alone
    for t in text.split():
        try:
            values.append(float(t))
        except ValueError:
            pass

    vals = [int(values[0]),int(values[1]),[values[2],values[3],values[4]],[values[5],values[6],values[7]]]
    """ keys = ["color", "type", "pos", "rotation"]
    data = dict(zip(keys, vals))"""
    return vals

msg = cleanUpPartMsg(part1)

s = ""
for item in msg:
    s += f"_{item}"
    
print(s)