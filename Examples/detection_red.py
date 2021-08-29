import cv2
import numpy as np
image=cv2.imread("images/red.jpeg")        #read in image
image_size=len(image)*len(image[0]) #get image size
image_dimension=np.array([len(image),len(image[0])])    #get image dimension
filtered_red=cv2.inRange(image,np.array([5,5,200]),np.array([200,200,255])) #filter the image with upper bound and lower bound in bgr format
#show filtered image
cv2.namedWindow("Image")
cv2.imshow("Image",filtered_red)
cv2.waitKey()
#run color connected components to filter the counts and centroid
retval, labels, stats, centroids=cv2.connectedComponentsWithStats(filtered_red) #run CCC on the filtered image
idx=np.where(np.logical_and(stats[:,4]>=0.01*image_size, stats[:,4]<=0.1*image_size))[0]    #threshold the components to find the best one
for i in idx:
    if np.linalg.norm(centroids[i]-image_dimension/2.)<50:  #threshold again, only for ones near the center
        print("red detected")
