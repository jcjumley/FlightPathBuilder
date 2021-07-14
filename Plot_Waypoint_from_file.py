## plot waypoint file (excell)
## only input is waypoin file

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import PIL
from PIL import Image,ImageChops
from PIL.ExifTags import TAGS
import pylab
import tkinter
from tkinter import *
from tkinter import filedialog
import os
import cv2
from DateTime import DateTime
from datetime import datetime
import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.path import Path
import csv
import scipy
from scipy.ndimage.filters import gaussian_filter

STEP_OVER_ALTITUDE  =  200.0 # feet
EPS = 1.0 #1.0E-2

root = tkinter.Tk()

def get_directory_name(caption):
    dirname = tkinter.filedialog.askdirectory(parent=root,initialdir="/",title=caption)
    if len(dirname ) > 0:
        print (' You chose %s' % dirname)
    return dirname

def get_file_name(caption):
    file = tkinter.filedialog.askopenfile(parent=root,mode='rb',title=caption)
    if file != None:
        data = file.read()
    #file.close()
    print (' I got %d bytes from this file.' % len(data))
    return file
# gets the file meta data for an image file
def get_exif(fn):
    ret = {}
    i = PIL.Image.open(fn)
    info = i._getexif()
    for tag, value in info.items():
        decoded = TAGS.get(tag, tag)
        ret[decoded] = value
    return ret

def get_gps_deg(exf):
    GPS = []
    gps = exf['GPSInfo']
    GPS0 = float(gps[2][0][0])/float(gps[2][0][1]) + float(gps[2][1][0])/(60.*float(gps[2][1][1])) + float(gps[2][2][0])/(3600*float(gps[2][2][1]))
    if (gps[1] == u'S'):
        GPS0 = -GPS0
    GPS.append(GPS0)
    GPS1 = float(gps[4][0][0])/float(gps[4][0][1]) + float(gps[4][1][0])/(60.*float(gps[4][1][1])) + float(gps[4][2][0])/(3600*float(gps[4][2][1]))
    if (gps[3] == u'W'):
        GPS1 = -GPS1
    GPS.append(GPS1)
    GPS2 = float(gps[6][0])/float(gps[6][1])
    GPS.append(GPS2)
    return GPS

def plot_data(waypoints,image_points):
    
    plt.ion()
    title_l = input(' Please enter plot title: ')
    title_l = title_l + '\n'
    plt.title(title_l)
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    X = [x[1] for x in waypoints]
    Y = [x[0] for x in waypoints]
    plt.plot(X,Y,'*') # waypoints
    plt.plot(X,Y)     # flight path
    X_image = [x[1] for x in image_points]
    Y_image = [x[0] for x in image_points]
    plt.plot(X_image,Y_image,'+')
    plt.show()
    return


def problem_1():
    ## get waypoint file name
    print ('Please browse for  waypoint file')
    waypoint_file = get_file_name('Please select waypoint file: ')
    wpf_name = waypoint_file.name

    fieldnames_1 = []
    waypoints = []

    z = input('Please enter step over altitude or n to use default: ')
    step_over_altitude = STEP_OVER_ALTITUDE
    if str(z) != 'n' and str(z) != 'N':
        step_over_altitude = float(z)

    with open(wpf_name,'r') as csvfile:
        reader = csv.DictReader(csvfile,dialect='excel')
        fieldnames_1 = reader.fieldnames
        for row in reader:
            altitude = float(row[fieldnames_1[2]])
            if (abs(altitude - step_over_altitude) < EPS):
                # ignore step over points
                continue
            latatude = float(row[fieldnames_1[0]])
            longitude = float(row[fieldnames_1[1]])
            waypoints.append([latatude,longitude,altitude])
            #print('row',row)
    #plot_data(waypoints)
    return waypoints

def problem_2():
    # plot images as data points
    print ('Please browse for  file containing image files')
    ImgDir = get_directory_name('Select Directory with mission files')
    image_points = []
    z = input('Please enter step over altitude or n to use default: ')
    step_over_altitude = STEP_OVER_ALTITUDE
    if str(z) != 'n' and str(z) != 'N':
        step_over_altitude = float(z)
        
    for filenames in os.listdir(ImgDir):
        fn = ImgDir + '/' + filenames
        im = PIL.Image.open(fn)
        #Image_list.append(im)
        exf = get_exif(fn)  # exf is the image metadata
        GPS = get_gps_deg(exf)
        if (abs(GPS[2] - step_over_altitude) < EPS):
            # ignore step over points
            continue
        image_points.append(GPS)
    #plot_data(image_points)
    return image_points
    
def main():
    waypoints = problem_1()
    image_points = problem_2()
    plot_data(waypoints,image_points)
    return
main()

if __name__ == "main":
     # execute only if run as a script
     main()
