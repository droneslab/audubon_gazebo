import cv2
import numpy as np

def padImage(img,pxsize):
    height,width,channels = img.shape
    # print(height,width)
    aspect_ratio = float(width/height)
    if aspect_ratio > 2.0 or aspect_ratio < 0.5:
        print("Image is not close to standard aspect ratio\n")
        exit()
    else:
        # Calculate the padding pixel number required for height and width
        temp_width = width
        width_pad = 0
        temp_height = height
        height_pad = 0
        if width%pxsize!=0:
            # print("Padding width")
            while(temp_width%pxsize!=0):
                temp_width+=1
                width_pad+=1
        if height%pxsize !=0:
            # print("Padding height")
            while(temp_height%pxsize!=0):
                temp_height+=1
                height_pad+=1

        # print(height_pad,width_pad)
        final_width = width+width_pad
        final_height = height+height_pad
        print("Original Image Resolution to :"+str(height)+"x"+str(width))
        print("Image Padded to :"+str(final_height)+"x"+str(final_width))
        color = (255,255,255)
        result = np.full((final_height,final_width,channels), color, dtype=np.uint8)

        # Bottom-Right Offset
        result[:height,:width] = img
        # save result
        return result

def padImage2(img,factor):
    height,width,channels = img.shape
    # print(height,width)
    aspect_ratio = float(width/height)
    if aspect_ratio > 2.0 or aspect_ratio < 0.5:
        print("Image is not close to standard aspect ratio\n")
        exit()
    else:
        # Calculate the padding pixel number required for height and width
        temp_width = width
        width_pad = 0
        temp_height = height
        height_pad = 0

        if height%factor !=0:
            # print("Padding height")
            while(temp_height%factor!=0):
                temp_height+=1
                height_pad+=1

        # print(height_pad,width_pad)
        final_width = width+width_pad
        final_height = height+height_pad
        print("Original Image Resolution to :"+str(height)+"x"+str(width))
        print("Image Padded to :"+str(final_height)+"x"+str(final_width))
        color = (255,255,255)
        result = np.full((final_height,final_width,channels), color, dtype=np.uint8)

        # Bottom-Right Offset
        result[:height,:width] = img
        # save result
        return result

img = cv2.imread('ub.png')

w,h,d = img.shape
factor = 5
pixel_size = 50

if(h%(h//factor)!=0):
    img = padImage2(img,(h//factor))

if(w%pixel_size!=0 or h%pixel_size!=0):
    img = padImage(img,pixel_size)



h,w,d = img.shape   
total_spots = (h//pixel_size)*(w//pixel_size)
spots = np.zeros((total_spots,pixel_size,pixel_size,3))
counter = 0
for i in range(0,h,pixel_size):
    for j in range(0,w,pixel_size):
        print(i,i+pixel_size,j,j+pixel_size)
        spots[counter]=(img[i:i+pixel_size,j:j+pixel_size,:])
        counter+=1

spots = np.random.permutation(spots)

new_h = h//factor
new_w = w*factor
new_img = np.zeros((new_h,new_w,3))


counter = 0
for i in range(0,new_h,pixel_size):
    for j in range(0,new_w,pixel_size):
        # print(i,j)
        new_img[i:i+pixel_size,j:j+pixel_size]=spots[counter]
        counter+=1


cv2.imwrite('out.png',new_img)


from matplotlib import pyplot as plt
img = cv2.imread('out.png',0)
# Initiate ORB detector
orb = cv2.ORB_create()
# find the keypoints with ORB
kp = orb.detect(img,None)
# compute the descriptors with ORB
kp, des = orb.compute(img, kp)
# draw only keypoints location,not size and orientation
img2 = cv2.drawKeypoints(img, kp, None, color=(0,255,0), flags=0)
plt.imshow(img2), plt.show()