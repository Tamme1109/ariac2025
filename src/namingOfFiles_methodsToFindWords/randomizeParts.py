"""d1 = {"part0" :{"color": 2, "type": 5}}
d2 = {"part1":{"color": 1, "type": 7}}

d1.update(d2)
#print(d1)

d3 = {"parts": d1}
for key in d3:
    print(key)

print(d3["parts"])"""

import random
import math as m
import json
import os

import yaml

typesOfParts = ['pump', 'battery', 'regulator', 'sensor']
colours = ['blue', 'red', 'green', 'orange', 'purple']
flipped = [True, False]

parts = {}
partnr = 0

for part in typesOfParts:
    for colour in colours:
        for i in range(3):
            filppedStatus = random.randint(0,1)
            rotation = random.uniform(0,2)
            offset = random.uniform(-1,1)
            parts[f"part_nr{partnr}"] = {'type': part, 'color': colour, 'offset': offset,
                                        'flipped': flipped[filppedStatus], 'rotation': f"{rotation}pi" }
            partnr +=1
            
            
script_dir = os.path.dirname(__file__) #absolute dir the script is in
rel_path = "randomparts.json"
abs_file_path = os.path.join(script_dir, rel_path)

with open(abs_file_path, 'w', encoding='utf-8') as f:
    json.dump(parts, f, ensure_ascii=False, indent=4)
            
f.close()

script_dir = os.path.dirname(__file__) #absolute dir the script is in
rel_path = "randomparts_yaml.yaml"
abs_file_path = os.path.join(script_dir, rel_path)

with open(abs_file_path, 'w') as f:
    yaml.dump(parts, f)
f.close()
