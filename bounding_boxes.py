#GeoTiff Experiments for heterogeneous computing
#This first one uses simple opencv methods to find countours in a binarized geotiff image and returns a png for visualization and the coordinates of the potential objects of interest.
#Author: Tiago A. O. Alves <tiago@ime.uerj.br>

import cv2
import numpy as np
import sys
from osgeo import gdal


cos = np.cos
sin = np.sin

def GetGeoCoordinates(gt, x, y):
    """receives a GeoTransform and a pair of pixel coordinates
    returns a pair of corresponding latitude and longitude"""
    xm, xs, xr, ym, yr, ys = gt
    """Comented out the rotation lines, because our GeoTiff's will probably be all North-up (0 rotation)""" 
    #x_rotated = x*np.array((cos(xr), sin(xr)))
    #y_rotated = y*np.array((-sin(yr), cos(yr)))
    #print("Rotated2 {}".format(x_rotated + y_rotated))
    #x, y = x_rotated + y_rotated
    return (x*xs + xm, y*ys + ym)



fname = sys.argv[1]

gtif = gdal.Open(fname)
gtransform = gtif.GetGeoTransform()

print("GT {}".format(gtransform))
gtif.Close()


img = cv2.imread(fname)

# convert to grayscale
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


#This code converts the grayscale image to a binary one (each pixel has just one bit, black or white) in order to apply simple computer vision algorithms to detect objects
thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)[1]


cv2.imwrite('binary.png', thresh) #saving the binary image just for debugging purposes, this step can be commented out

# get contours
result = img.copy()
contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = contours[0] if len(contours) == 2 else contours[1]
stride = 100
for cntr in contours:
    x,y,w,h = cv2.boundingRect(cntr)
    #w,h = w+stride, h+stride
    cv2.rectangle(result, (x, y), (x+w, y+h), (0, 0, 255), 2)
    ul_x, ul_y = GetGeoCoordinates(gtransform, x-stride, y-stride)
    dw_x, dw_y = GetGeoCoordinates(gtransform, x+stride, y+stride)
 
    print("{} {} {} {}".format(ul_x, ul_y, dw_x, dw_y))
cv2.imwrite('bounding.png',result)      

