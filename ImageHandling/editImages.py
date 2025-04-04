import cv2
import numpy as np
import matplotlib.pyplot as plt
import random

class EditImages:
    def __init__(self):
        #these variables are initialized with arbitrary values - these will always be updated before use 
        self.scalefactor = 1/3.0 #scale factor for resizing of images
        self.newangle = 30
        self.shearingX = 0
        self.shearingY = 0
        self.translateX = 0
        self.translateY = 0
        self.gaussianKernelSize = 0
        self.medianBlurParameter = 0
        self.kernel = np.ones((3, 3), np.uint8)
        self.iterationsDilation = 1
        self.eroasionIterations = 1
        self.flippedparameter = 1
    
    def resize(self, image_rgb):#rezise the image
        # Get the original image dimensions
        height, width = image_rgb.shape[:2]
        # Calculate the new image dimensions
        self._updateScaleFactor()
        new_height = int(height * self.scalefactor)
        new_width = int(width * self.scalefactor)
        # Resize image
        new_image = cv2.resize(src =image_rgb, dsize=(new_width, new_height), interpolation=cv2.INTER_CUBIC)
        return new_image
        
    def rotate(self, image_rgb):#rotate the image
        # center of image
        center = (image_rgb.shape[1] // 2, image_rgb.shape[0] // 2)
        self._updateNewAngle()
        rotation_matrix = cv2.getRotationMatrix2D(center, self.newangle, 1)#scale = 1. to change scale use different function
        rotated_image = cv2.warpAffine(image_rgb, rotation_matrix, (image_rgb.shape[1], image_rgb.shape[0]))
        return rotated_image
    
    def shear(self, image_rgb):#shear the image
        # Image shape along X and Y
        width = image_rgb.shape[1]
        height = image_rgb.shape[0]

        self._updateShearingFactors()    
        transformation_matrix = np.array([[1, self.shearingX, 0], 
                                        [0, 1, self.shearingY]], dtype=np.float32)
        sheared_image = cv2.warpAffine(image_rgb, transformation_matrix, (width, height))
        return sheared_image

    def translate(self, image_rgb): #translate the image (X-axis and Y-axis)
        width = image_rgb.shape[1]
        height = image_rgb.shape[0]

        self._updateTranslations()

        translation_matrix = np.array([[1, 0, self.translateX], [0, 1, self.translateY]], dtype=np.float32)
        translated_image = cv2.warpAffine(image_rgb, translation_matrix, (width, height))
        return translated_image
        
    def gaussianBlur(self, image): #apply a gaussian blur to the image (don't use image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) as input!)
        self._updateGaussianKernelSize()
        blurred = cv2.GaussianBlur(image, (self.gaussianKernelSize, self.gaussianKernelSize), 0)
        # Convert blurred image to RGB
        blurred_rgb = cv2.cvtColor(blurred, cv2.COLOR_BGR2RGB)
        return blurred_rgb
    
    def medianBlur(self, image):#apply a median blur on the image (don't use image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) as input!)
        self._updateMedianBlurParameter()
        blurred = cv2.medianBlur(image, self.medianBlurParameter)
        blurred_rgb = cv2.cvtColor(blurred, cv2.COLOR_BGR2RGB)
        return blurred_rgb
    
    def dilateImage(self, image_rgb):
        self._updateDilationIterations()
        dilated = cv2.dilate(image_rgb, self.kernel, iterations=self.iterationsDilation)
        return dilated
        
    def erode(self, image_rgb):
        self._updateErosionIterations()
        eroded = cv2.erode(image_rgb, self.kernel, iterations=self.eroasionIterations)
        return eroded
    
    def open(self, image_rgb): #opening (erosion followed by dilation)
        opening = cv2.morphologyEx(image_rgb, cv2.MORPH_OPEN, self.kernel)
        return opening
    
    def close(self, image_rgb):#closing  (dilation followed by erosion)
        closing = cv2.morphologyEx(image_rgb, cv2.MORPH_CLOSE, self.kernel)
        return closing
    
    def flipImage(self, image_rgb): #flips the image horizontaly, verticaly, or both depending on the randomized variable
        self._updateFlippedParameter()
        flippedImage = cv2.flip(image_rgb, self.flippedparameter)
        return flippedImage
        
    ###Randomize factors/parameters for the image editing
    def _updateScaleFactor(self):
        self.scalefactor = (random.randrange(4,7,1))/10
        
    def _updateNewAngle(self):
        self.newangle = random.randrange(1,360)
        
    def _updateShearingFactors(self):
        self.shearingX = (random.randrange(-3, 3))/10
        self.shearingY = random.randrange(0, 100)
        
    def _updateTranslations(self):
        self.translateX = random.randint(0, 200)
        self.translateY = random.randint(0, 200)
        
    def _updateGaussianKernelSize(self):
        self.gaussianKernelSize = random.randrange(1,27,2)
        
    def _updateMedianBlurParameter(self):
        self.medianBlurParameter =random.randrange(3,7,2)
        
    def _updateDilationIterations(self):
        self.iterationsDilation = random.randint(1,3)
        
    def _updateErosionIterations(self):
        self.eroasionIterations = random.randint(1,3)
        
    def _updateFlippedParameter(self):
        self.flippedparameter = random.randint(-1,1)
        
    ######
    def plot(self, image1, image2):
        # Create subplots
        fig, axs = plt.subplots(1, 2, figsize=(10, 4))

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
    a = EditImages()
    
    """image = cv2.imread('ariac_parts_dataset_1/battery_blue_bin1_1_False_30.png')
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # new_image = a.resize(image_rgb)
    new_image = a.flipImage(image_rgb)
    a.plot(image_rgb, new_image)"""
    
    

if __name__ == '__main__':
    main()
