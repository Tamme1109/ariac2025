import random
import os
from ruamel.yaml import YAML
import math as m

yaml = YAML()
yaml.indent(mapping=4, sequence=4, offset=2)

class updateSpawnParts:
    def __init__(self):
        self.typesOfParts = ['pump', 'battery', 'regulator', 'sensor']
        self.colours = ['blue', 'red', 'green', 'orange', 'purple']
        self.flipped = [True, False]
        self.rotations = ['0', 'pi/2', 'pi/3', 'pi2/3', 'pi/4', 'pi/5', 'pi/6', 'pi/7', 'pi4/7', 'pi/8', 'pi/4', 'pi5/6', 'pi2/5'] 
        
        #for file path
        self.script_dir = os.path.dirname(__file__) #absolute dir the script is in
        self.rel_path = 'tutorial.yaml' #name of the file we want to update with parts
        self.abs_file_path = os.path.join(self.script_dir, self.rel_path)

    def randomizeParts(self):#returns a list with the new parts to spawn
        parts = []
        for part in self.typesOfParts:
            for colour in self.colours:
                for i in range(50):
                    filppedStatus = random.randint(0,1)
                    rotation = random.randint(0, len(self.rotations)-1)
                    offset = random.uniform(-1,1)
                    parts.append({'type': part, 'color': colour, 'number': 1, 'offset': offset,
                                                'flipped': self.flipped[filppedStatus], 'rotation': self.rotations[rotation]})
        return parts
            
    def updateFile(self, newParts):
        f = open(self.abs_file_path)
        data = yaml.load(f)

        new_yaml = {}
        for item in data['parts'].values():#loop trhough the contents of the file
            for i in item:
                if i != 'parts_to_spawn':
                    new_yaml[i] = item[i] #we want to keep everything else in the file
                else:
                    """d = open('randomparts_yaml.yaml')
                    newData = yaml.load(d)
                    new_yaml[i] = newData['parts_to_spawn']
                    d.close()"""
                    new_yaml[i] = newParts 
            
        data['parts'] = {'conveyor_belt': new_yaml}        
        f.close()
        with open(self.abs_file_path, 'w') as f:
            yaml.dump(data, f)
        f.close()   
        
def main(args=None):
    spawner = updateSpawnParts()
    spawner.updateFile(spawner.randomizeParts())


if __name__ == '__main__':
    main()

