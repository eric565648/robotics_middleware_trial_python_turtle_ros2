import cv2
import numpy as np
image=cv2.imread("images/red.jpeg")        #read in image
image_size=len(image)*len(image[0]) #get image size
image_dimension=np.array([len(image),len(image[0])])    #get image dimension

# use hsv
hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# Define the boundaries of "white", "yellow" and "red"
# hsv_white1 = np.array([0,0,150])
# hsv_white2 = np.array([180,100,255])
hsv_blue1 = np.array([100, 150,0])
hsv_blue2 = np.array([140, 255, 255])
hsv_yellow1 = np.array([25,50,50])
hsv_yellow2 = np.array([45,255,255])
# The color "red" needs two set of boundaries cause it pass 255 to 0
hsv_red1 = np.array([0,100,100])
hsv_red2 = np.array([15,255,255])
hsv_red3 = np.array([165,100,100])
hsv_red4 = np.array([180,255,255])

# detect red
# filtered_red=cv2.inRange(cv_image,np.array([5,5,200]),np.array([200,200,255])) #filter the image with upper bound and lower bound in bgr format
filtered_red1 = cv2.inRange(hsv_img, hsv_red1, hsv_red2)
filtered_red2 = cv2.inRange(hsv_img, hsv_red3, hsv_red4)
filtered_red = cv2.bitwise_or(filtered_red1, filtered_red2)
# detect yellow
filtered_yellow = cv2.inRange(hsv_img, hsv_yellow1, hsv_yellow2)
# detect blue
filtered_blue = cv2.inRange(hsv_img, hsv_blue1, hsv_blue2)
    
#show filtered image
cv2.namedWindow("Image Red")
cv2.imshow("Image Red",filtered_red)
cv2.namedWindow("Image Yellow")
cv2.imshow("Image Yellow",filtered_yellow)
cv2.namedWindow("Image Blue")
cv2.imshow("Image Blue",filtered_blue)
cv2.waitKey()

#run color connected components to filter the counts and centroid
retval, labels, stats, centroids=cv2.connectedComponentsWithStats(filtered_red) #run CCC on the filtered image
idx=np.where(np.logical_and(stats[:,4]>=0.01*image_size, stats[:,4]<=0.1*image_size))[0]    #threshold the components to find the best one
for i in idx:
    if np.linalg.norm(centroids[i]-image_dimension/2.)<50:  #threshold again, only for ones near the center
        print("red detected")
