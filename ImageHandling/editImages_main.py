from editImages import EditImages
ei = EditImages()
import random
import cv2
import matplotlib.pyplot as plt
import os

class HandleImages:
    def __init__(self):
        self.listofTransformations = [ei.resize, ei.close, ei.dilateImage, ei.erode, ei.flipImage, ei.gaussianBlur, ei.medianBlur, ei.open, ei.rotate, ei.shear, ei.translate]
    
    def _picktransformations(self):#returns a list with 1 to 3 transformations that will be used for an image
        numberofTransformations = random.randint(1,3)
        transformations = []
        while True:
            num = random.randint(0, len(self.listofTransformations)-1)
            if self.listofTransformations[num] not in transformations:
                transformations.append(self.listofTransformations[num])
                
            if len(transformations) >= numberofTransformations:
                break
            
        return transformations
    
    def applyTransformations(self, image):
        transf = self._picktransformations()
        if ei.gaussianBlur in transf: #apply gaussian blur
            transf.remove(ei.gaussianBlur) #remove ei.gaussianblur from list
            image = ei.gaussianBlur(image)
        
        if len(transf) != 0:
            if ei.medianBlur in transf: #apply median blur
                transf.remove(ei.medianBlur)
                image =ei.medianBlur(image)
        else: 
            return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if len(transf) != 0:
            for trans in transf:
                image_rgb = trans(image_rgb)

        return image_rgb
    
    def _displayNewImage(self, image1, image2):
        fig, axs = plt.subplots(1, 2, figsize=(10, 4))
        # Plot the original image
        # Plot the original image
        axs[0].imshow(image1)
        axs[0].set_title('Original Image Shape:'+str(image1.shape))

        # Plot the edited Image
        axs[1].imshow(image2)
        axs[1].set_title('Edited Image Shape:'+str(image2.shape))

        # Remove ticks from the subplots
        for ax in axs:
            ax.set_xticks([])
            ax.set_yticks([])

        # Display the subplots
        plt.tight_layout()
        plt.show()
    
def main(args=None):
    handleImages = HandleImages()
    for filename in os.listdir("testDataSet"):
        if filename.endswith(".png") :
            print(filename)
            image = cv2.imread(f"testDataSet/{filename}") 
            new_image  = handleImages.applyTransformations(image)
            handleImages._displayNewImage(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), new_image)
        else:
            continue
    

if __name__ == '__main__':
    main()
