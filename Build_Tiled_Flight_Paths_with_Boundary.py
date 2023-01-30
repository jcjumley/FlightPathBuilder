import PIL
from PIL import Image,ImageChops
from PIL.ExifTags import TAGS
import matplotlib.pyplot as plt
import pylab
import tkinter
from tkinter import *
from tkinter import filedialog
import seaborn as sns
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

import collections
from itertools import islice
import linecache

# Mavic Pro camera properties
FOV = 78.8 # Camera field of view 78.8 Deg
CROP_FACTOR = 35/26 # Lense apature
IMAGE_WIDTH = 4000 # pixels
IMAGE_HEIGHT = 3000 # pixels
STEP_OVER_ALTITUDE  = 200.0 # feet

root = tkinter.Tk()
import my_modules as my_modules
from my_modules import get_directory_name as get_directory_name
from my_modules import get_file_name as get_file_name
from my_modules import save_file_name as save_file_name
#from my_modules import get_exif as get_exif
from my_modules import get_gps_deg as get_gps_deg
from my_modules import get_r_of_phi as get_r_of_phi
from my_modules import calculate_distance as calculate_distance
#from my_modules import date_to_nth_day as date_to_nth_day
#from my_modules import calculate_sun_angle as calculate_sun_angle
#from my_modules import date_to_nth_day as date_to_nth_day


# calculate center point of bounding box
def calculate_center_point(bounding_box):
    center_y = (bounding_box[0][0] + bounding_box[1][0])/2.0
    center_x = (bounding_box[0][1] + bounding_box[1][1])/2.0
    return [center_y,center_x]

# given altitude of drone and its camera Field of View calculate pixel size
def calculate_pixel_size(FOV,height):
    pixel_size = abs(4000./(2.0*height*math.tan((math.pi/180.)*FOV/2.0)))
    pixel_size = (35./26.)*pixel_size
    # print 'pixels per foot',pixel_size

def calculate_image_size(FOV,height):
    img_size = 2.0*height*math.tan((math.pi/180.)*FOV/2.0)*(26.0/35.0)
    return [img_size,(3.0/4.0)*img_size]

def serindipity(nodes,xi):
    Xi = [[1,1],[-1,1],[-1,-1],[1,-1]]
    N = np.shape(nodes)[0]
    x = [0.,0.]
    for i in range ( 0 , N):
        #print('xi=',xi)
        #print ('i',Xi[i][0],Xi[i][1])
        x0 = Xi[i][0]*xi[0]
        nu0 = Xi[i][1]*xi[1]
        Ni = (1.+x0)*(1.+nu0)/4.
        #print ('x0 =',x0,'nu0 =',nu0,'Ni = ',Ni)
        x[0] = x[0] + Ni*nodes[i][0]
        x[1] = x[1] + Ni*nodes[i][1]
    return x

def build_corner_nodes(pts): #Bounding box of corner nodes
    min_x = pts[0][0]
    max_x = min_x
    min_y = pts[0][1]
    max_y = min_y
    for i in range (len(pts)):
        if pts[i][0] < min_x:
            min_x = pts[i][0]
        if pts[i][0] > max_x:
            max_x = pts[i][0]
        if pts[i][1] < min_y:
            min_y = pts[i][1]
        if pts[i][1] > max_y:
            max_y = pts[i][1]
    corners = [[float(max_x),float(max_y)],[float(min_x),float(max_y)],[float(min_x),float(min_y)],[float(max_x),float(min_y)]]
    return corners
def row_of_k(k,n_cols):
    row = 0
    last_k_in_row = n_cols[0]
    for row in range (1,len(n_cols)):
        if k <= last_k_in_row:
            return row
        else:
            last_k_in_row += n_cols[row]
    return row
            
def conformal_map_flight_plan(Nodes,n_phi,n_theta,boundary_points,altitude):
    ## Routine that builds waypoint file from boundray by constructing bounding box
    ## and filling the bb with (n_phi by n_theta) grid points conected in a zigzag patern
    ## selecting those grid points within the bb. The routine also creates "step_over"
    ## points if the flight would go outside the boundary
    waypoints = []
    waypoint_string = []
    waypoint_strings = []
    extra_points = []
    n_cols      = []  # number of columns(points)in each row (includes stepover points) 
    n_cols = np.zeros(n_phi,dtype=np.int)
    n_stepovers = [] # number of stovers in each row
    n_stepovers = np.zeros(n_phi,dtype=np.int)
    virtual_ks = [] # array that mapps wapoints to virtual waypoints
    virtual_ls = [] # array that maps stepover points to waypoints
    virtual_ks = np.zeros(n_phi*n_theta,dtype=np.int) # 0 ==> no actual waypoint exist
    virtual_ls = np.zeros(n_phi*n_theta,dtype=np.int) # -1 ==> stepover before waypoint,+1 == after, 0 == no stepover
    virtual_k   = 0
    stepovers = []
    stepover_points = []
    # start in upper left hand coner and proceeded right(east) and down(south)
    delta_phi = 2.0/n_phi
    delta_theta = 2.0/n_theta
    phi = 1. + delta_phi/2
    theta = -(1. - delta_theta/2.0)
    k = 0   # key point counter
    direction = +1.
    x_last = serindipity(Nodes,[0,0]) # Center of bounding box
    row_last_x = 0
    was_in = False
    for i in range (0,n_phi):  # Loop on rows
        phi = phi - delta_phi
        # number of columns(j's) in this row (i)
        for j in range (0,n_theta):  # columns
            last_virtual_k = virtual_k
            virtual_k += 1 #incremet virtual_k nomatter what
            x = serindipity(Nodes,[phi,theta])         
            line_Q = [[x_last[0],x_last[1]],[x[0],x[1]]]  # line segment from last grid point to current grid point
            (is_in,contains) = contains_line_segment(boundary_points,line_Q)
            #print ('i ',i,'j ',j,' k ',k,'is_in ',is_in,' contains ',contains, ' was_in ',was_in)
            if (is_in): # always create a waypoint if current grid point is inside boundary!!
                if ((contains) and (was_in)): # should be redundent except for first time (only waypoint created)
                    n_cols[i] += 1
                    #print('n_cols ',n_cols)
                    waypoints.append([x[0],x[1],altitude])
                    #print ('\n k = ',k,'length of waypoints = ',len(waypoints), waypoints[k],'is_in contains was_in')
                    k += 1
                    virtual_ks[virtual_k] = k
                    x_last = x
                    row_last_x = i
                    was_in = True
                else:
                    if (k != 0): # if not first waypoint create stepover point at last waypoint
                        stepovers.append(k)
                        waypoints.append([x_last[0],x_last[1],STEP_OVER_ALTITUDE])
                        stepover_points.append([x_last[0],x_last[1],STEP_OVER_ALTITUDE])
                        virtual_ks[virtual_k] = k
                        virtual_ls[virtual_k - 1] = 1
                        k += 1
                        virtual_ks[virtual_k] = k
                        virtual_ls[virtual_k] = -1
                        #print ('length of output points',len(waypoints))
                        #print ('\n k = ',k,'length of waypoints = ',len(waypoints),waypoints[k],'sterpover is_in !contains or !was_in k !=0')
                        n_cols[row_last_x] += 1
                        n_stepovers[row_last_x] += 1
                        #print('n_cols ',n_cols)
                        row_last_x = i
                    # always create stepover point at this waypoint (if is_in == True)
                    waypoints.append([x[0],x[1],STEP_OVER_ALTITUDE])
                    k +=1
                    virtual_ks[virtual_k] = k
                    virtual_ls[virtual_k] = -1
                    stepover_points.append([x[0],x[1],STEP_OVER_ALTITUDE])
                    stepovers.append(k)
                    #print ('length of output points',len(waypoints))
                    #print ('\n k = ',k,'length of waypoints = ',len(waypoints),waypoints[k],'sterpover is_in !contains or !was_in')
                    waypoints.append([x[0],x[1],altitude])
                    #print ('length of output points',len(waypoints))
                    #print ('\n k = ',k,'length of waypoints = ',len(waypoints),waypoints[k],'normal is_in !contains or !was_in')
                    # only includes points on flight path des not include steovers            
                    n_cols[i] += 2
                    n_stepovers[i] += 1
                    #print('n_cols ',n_cols)
                    x_last = x
                    was_in = True
            else:
                extra_points.append(x)
             
            theta = theta + direction*delta_theta
        #n_cols.append(n_col)
        direction = -direction           
        theta = theta + direction*delta_theta
    # End loop on rows
    stepover_points = np.asarray(stepover_points)
##    print('shape of stepover_points ', stepover_points.shape)
##    for i in range(len(stepover_points)):
##        print('stepover_points[',i,'] ', stepover_points[i][:])
                       
    waypoints = np.asarray(waypoints)
##    for i in range(len(waypoints)):
##        print('waypoints[',i,'] ', waypoints[i][:])
##    print ('n_cols', n_cols,'\n stepovers',stepovers)
    return [waypoints,stepover_points,extra_points,n_cols,virtual_ks,virtual_ls]   

def build_waypoints(start_point,waypoint,n_cols,virtual_ks):
    ####### Waypoint data items #######
    N = np.shape(waypoint)[0]
    print ('N = ',N)
    latitude = []
    longitude = []
    alt = []
    
    for i in range(0,N):
        latitude.append(waypoint[i][0])
        longitude.append(waypoint[i][1])
        alt.append(waypoint[i][2])
##    print ('alt [0] ',alt[0])
    heading = 0
    curvesize = 0.625
    rotationdir = 0
    gimbalmode = 2
    gimbalpitchangle = 0
    actiontype1 = 5    # rotate gimpal
    actionparam1 = -90 # down 90 degrees
    actiontype2 = 0    # hover
    actionparam2 = 1000 # 1000 mili-seconds
    actiontype3 = 1    # take photo
    actionparam3 = 0
    actiontype4 = -1 # NONE
    actionparam4 = 0
    actiontype5 = -1 # NONE
    actionparam5 = 0
    actiontype6 = -1 # NONE
    actionparam6 = 0
    actiontype7 = -1 # NONE
    actionparam7 = 0
    actiontype8 = -1 # NONE
    actionparam8 = 0
    actiontype9 = -1 # NONE
    actionparam9 = 0
    actiontype10= -1 # NONE
    actionparam10 = 0
    actiontype11 = -1 # NONE
    actionparam11 = 0
    actiontype12 = -1 # NONE
    actionparam12 = 0
    actiontype13 = -1 # NONE
    actionparam13 = 0
    actiontype14 = -1 # NONE
    actionparam14 = 0
    actiontype15 = -1 # NONE
    actionparam15 = 0
    altitudemode = 1
    speed = 0 # meters per sec
    poi_latitude = 0 # point of interest latatude
    poi_longitude = 0 # point of interest longitude
    poi_altitude = 0 # point of interest altitude 
    poi_altitudemode = 0 # point of interest altitude
 
    print ('browse for directory to store waypoint file')
    Waypoint_Dir = get_directory_name('select waypoint directory')
    Waypoint_filename = input(' Enter Way Point file name: ')
    Waypoint_file = Waypoint_Dir + '/' + Waypoint_filename + '.csv'
    all_lines = []
    #this_line = []
    this_line = ['latitude','longitude','altitude(ft)','heading(deg)','curvesize(ft)','rotationdir','gimbalmode','gimbalpitchangle','actiontype1','actionparam1','actiontype2','actionparam2','actiontype3','actionparam3','actiontype4','actionparam4','actiontype5','actionparam5','actiontype6','actionparam6','actiontype7','actionparam7','actiontype8','actionparam8','actiontype9','actionparam9','actiontype10','actionparam10','actiontype11','actionparam11','actiontype12','actionparam12','actiontype13','actionparam13','actiontype14','actionparam14','actiontype15','actionparam15','altitudemode','speed(m/s)','poi_latitude','poi_longitude','poi_altitude(ft)','poi_altitudemode']
    all_lines.append(this_line)
    #x_start = float
    #y_start = float
    x_start = float(start_point[0])
    y_start = float(start_point[1])
    #print ('x_start', x_start,'\n')
    all_lines.append(this_line)
    #print(all_lines)

    with open(Waypoint_file,'w',newline ='\n') as f_csv:
        writer = csv.writer(f_csv,dialect='excel')
        #writer = csv.writer(f_csv)
        waypoint_strings =['latitude','longitude','altitude(ft)','heading(deg)','curvesize(ft)',
                         'rotationdir','gimbalmode','gimbalpitchangle','actiontype1','actionparam1',
                         'actiontype2','actionparam2','actiontype3','actionparam3','actiontype4',
                         'actionparam4','actiontype5','actionparam5','actiontype6','actionparam6',
                         'actiontype7','actionparam7','actiontype8','actionparam8','actiontype9',
                         'actionparam9','actiontype10','actionparam10','actiontype11','actionparam11',
                         'actiontype12','actionparam12','actiontype13','actionparam13','actiontype14',
                         'actionparam14','actiontype15','actionparam15','altitudemode','speed(m/s)',
                         'poi_latitude','poi_longitude','poi_altitude(ft)','poi_altitudemode','\n']
        writer.writerow(waypoint_strings)
        altitude = STEP_OVER_ALTITUDE
        writer.writerow([x_start,y_start,altitude,0,curvesize,rotationdir,gimbalmode,gimbalpitchangle,actiontype1,actionparam1,actiontype2,actionparam2,actiontype3,actionparam3,actiontype4,actionparam4,actiontype5,actionparam5,actiontype6,actionparam6,actiontype7,actionparam7,actiontype8,actionparam8,actiontype9,actionparam9,actiontype10,actionparam10,actiontype11,actionparam11,actiontype12,actionparam12,actiontype13,actionparam13,actiontype14,actionparam14,actiontype15,actionparam15,altitudemode,speed,poi_latitude,poi_longitude,poi_altitude,poi_altitudemode,'\n'])
        row = 0
        n_col = 0
        total_cols = 0
        max_cols_thru_this_row = n_cols[0]
        for k in range (0,N):
            n_col += 1
            #print('row ' ,row)
            #print ('n_cols[ ',row,' ] ', n_cols[row])
            if n_col > max_cols_thru_this_row:
                row += 1
                max_cols_thru_this_row += n_cols[row]
            heading = 90.0
            if (row%2 != 0):   # face west on even numbered rows and east on odd
                heading = -90.0
            x = float(latitude[k])
            y = float(longitude[k])
            z = float(alt[k])
            actiontype3 = 1    # take photo
            actionparam3 = 0
            # dont take photo at stepover points
            if abs(z-STEP_OVER_ALTITUDE) < 0.5:
                actiontype3 = -1
            waypoint_string = [x,y,z,heading,curvesize,rotationdir,gimbalmode,gimbalpitchangle,actiontype1,actionparam1,actiontype2,actionparam2,actiontype3,actionparam3,actiontype4,actionparam4,actiontype5,actionparam5,actiontype6,actionparam6,actiontype7,actionparam7,actiontype8,actionparam8,actiontype9,actionparam9,actiontype10,actionparam10,actiontype11,actionparam11,actiontype12,actionparam12,actiontype13,actionparam13,actiontype14,actionparam14,actiontype15,actionparam15,altitudemode,speed,poi_latitude,poi_longitude,poi_altitude,poi_altitudemode,'\n']
            writer.writerow(waypoint_string)
            waypoint_strings.append([waypoint_string])
##            if k < 10 :
##                #print (waypoint_string)
##                print (waypoint_strings[k][:])
        f_csv.close
        return Waypoint_filename, Waypoint_file # End build_waypoints 

def contains_point(border_points,test_point):
    bpts = np.asarray(border_points)
    myPath = Path(bpts)
    return myPath.contains_point(test_point)

def intersect(line_P,line_Q):
    alpha = [0,0]
    # robust routine to find the intersections between two lines
    intersects = False
    P = np.asarray(line_P)
    Mp = np.asmatrix(P).T
    #print ('Mp ',Mp)
    P0 = Mp[:,0]
    P1 = Mp[:,1]
    #print ('P0 \n',P0,'P1 \n',P1)
    Q = np.asarray(line_Q)
    Mq = np.asmatrix(Q).T
    Q0 = Mq[:,0]
    Q1 = Mq[:,1]
    #print ('Q0 \n',Q0,'\nQ1 \n',Q1) 
    A = (P1 - P0)
    B = (Q0 - Q1)
    C = (Q0 - P0)
    #print ('\n A \n',A,'\n B \n',B,'\n C \n',C)
    M = np.c_[A,B]
    #print ('M \n', M)
    try:
        Iv = np.linalg.inv(M)
        #print ('Iv \n',Iv)
        check_I = Iv*M
        #print ('check \n',check_I)
        alpha = Iv*C
        #print ('alpha \n',alpha)
        if ((alpha[0] >= 0. and alpha[0] <= 1.) and (alpha[1] >= 0. and alpha[1] <= 1.)):
            intersects = True
    except:
        #print('No inverse exist')
        intersects = False
    
    return (intersects,alpha)

def contains_line_segment(border_points,line_Q):
    bpts = np.asarray(border_points)
    is_in = False
    contains = True
    intersect_count = 0
    for i in range (1,len(bpts)):
        x_border_0 = bpts[i-1][0]
        y_border_0 = bpts[i-1][1]
        x_border_1 = bpts[i][0]
        y_border_1 = bpts[i][1]
        line_P = [[x_border_0,y_border_0],[x_border_1,y_border_1]]
        intersects,alpha = intersect(line_P,line_Q)
        if intersects:
            intersect_count += 1
    if intersect_count > 0:
        contains = False
    if intersect_count%2 == 0:
        is_in = True
    return (is_in,contains)

def build_bounding_box(pts):
    min_x = pts[0][0]
    max_x = min_x
    min_y = pts[0][1]
    max_y = min_y
    for i in range (len(pts)):
        if pts[i][0] < min_x:
            min_x = pts[i][0]
        if pts[i][0] > max_x:
            max_x = pts[i][0]
        if pts[i][1] < min_y:
            min_y = pts[i][1]
        if pts[i][1] > max_y:
            max_y = pts[i][1]
    bounding_box = [[float(max_x),float(max_y)],[float(min_x),float(min_y)]]
    return bounding_box

def plot_pathes(Waypoint_filename,corners,waypoints,extra_points,border_points):
    plt.ion()
    title_l = Waypoint_filename + '\n'
    plt.title(title_l)
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    xc = [x[1] for x in corners]
    yc = [x[0] for x in corners]
    plt.plot(xc,yc,'+') # coners
    X = [x[1] for x in waypoints]
    Y = [x[0] for x in waypoints]
    plt.plot(X,Y,'*') # waypoints
    plt.plot(X,Y)     # flight path
    plot_extra_points = input('Do you want to plot extra points? (y/n) ')      
    if (plot_extra_points == 'y' or plot_extra_points == 'Y'):
        xe = [x[1] for x in extra_points]
        ye = [x[0] for x in extra_points]
        plt.plot(xe,ye,'*') # extra points in bounding box not included in flight path
    xb = [x[1] for x in border_points]
    yb = [x[0] for x in border_points]
    plt.plot(xb,yb) # boundary path
    plt.show()
    return
def make_sorties(wpf_name,n_cols):
    MAX_SORTIE_LENGTH = 80 # ONLY FOR THIS TEST CASE
    EPS = 1.0E-8    
    fieldnames_1 = []
    length_base_file_name = len(wpf_name)
    base_file_name = wpf_name[:length_base_file_name-4]
    sortie = 1
    row_number = 0
    N = sum(n_cols)
    n_sorties = math.ceil(N/MAX_SORTIE_LENGTH)
    sortie_length = math.ceil(N/n_sorties)

    with open(wpf_name,'r') as csvfile:
        reader = csv.DictReader(csvfile,dialect='excel')
        fieldnames_1 = reader.fieldnames
        #print ('fieldnames_1[0]',fieldnames_1[0])
        #print ('fieldnames',fieldnames_1)
        for row in reader:
            #this_line = row
            #print('row_number: ',row_number,' this_line[altitude]',this_line[fieldnames_1[2]], '\n')
            #print('this_line ',(this_line.values()))
            if row_number%sortie_length == 0:
                sortie_file_name = base_file_name + '_sortie_' + str(sortie) + '.csv'
                f_csv = open(sortie_file_name,'w',newline ='\n')
                print('row_number ',row_number)
                writer = csv.DictWriter(f_csv,fieldnames_1,dialect='excel')
                writer.writeheader()
                if abs(float(row[fieldnames_1[2]]) - STEP_OVER_ALTITUDE) > EPS:
                    altitude = row[fieldnames_1[2]]
                    stepover_row = row
                    stepover_row[fieldnames_1[2]] = STEP_OVER_ALTITUDE
                    stepover_row[fieldnames_1[12]] = -1
                    writer.writerow(stepover_row)
                    row[fieldnames_1[2]] = altitude
                    row[fieldnames_1[12]] = 1

            writer.writerow(row)
            #writer.writerow(this_line.values())
            if (row_number%sortie_length == sortie_length -1):
                sortie += 1
                if abs(float(row[fieldnames_1[2]]) - STEP_OVER_ALTITUDE) > EPS:
                    stepover_row = row
                    stepover_row[fieldnames_1[2]] = STEP_OVER_ALTITUDE
                    stepover_row[fieldnames_1[12]] = -1
                    writer.writerow(stepover_row)
                f_csv.close()

            row_number += 1
        #print ('keys_1 ',keys_1)
        #print ('fieldnames_1 ',fieldnames_1)
        #print ('fieldnames_1[2] ',fieldnames_1[2])

    return #End of make_sorties
def make_tiled_sorties(wpf_name,n_phi,n_theta,n_cols,virtual_ks,virtual_ls,tile_size,corner_points):
    EPS = 1.0E-8    
    fieldnames_1 = []
    line_buffer =  []
    lines        = []
    length_base_file_name = len(wpf_name)
    base_file_name = wpf_name[:length_base_file_name-4]
    row_number = 0
    #N = sum(n_cols)
    #n_sorties = math.ceil(N/MAX_SORTIE_LENGTH)
    #sortie_length = math.ceil(N/n_sorties)
    n_i_tiles = 2*math.ceil(n_phi/(2*tile_size))
    n_j_tiles = 2*math.ceil(n_theta/(2*tile_size))
    print ('n_i_tiles = ',n_i_tiles,' n_j_tiles =', n_j_tiles)  
    # open general waypoint file & build line buffer
    with open(wpf_name,'r') as csvfile:
        reader = csv.DictReader(csvfile,dialect='excel')
        fieldnames_1 = reader.fieldnames
        for row in reader:
            line_buffer = row
            lines.append(line_buffer)
    #print ('Size of lines = ', len(lines))
    #print ('fieldnames_1[0]',fieldnames_1[0])
    #print ('fieldnames',fieldnames_1)
    #for i_T in range(0,2):  # for testing
    for i_T in range(0,int(n_i_tiles)):
        if(i_T == 0):
            i_start = 0
            i_stop = math.ceil(((n_phi)/2)%tile_size)
        else:
            i_start = i_stop
            i_stop = i_start + tile_size
        if (i_stop > n_phi):
            i_stop = n_phi
        #for j_T in range(0,2): #for testing
        for j_T in range(0,int(n_j_tiles)):
            # Build Tile File name, open csv file and print header:
            is_first = True
            tile_file_name = base_file_name + '_iTile_' + str(i_T)+ '_jTile_' + str(j_T) + '.csv'
            f_csv = open(tile_file_name,'w',newline ='\n')
            ##print('\n ******** i_T = ',i_T, ' j_T = ',j_T,' ******** ')
            writer = csv.DictWriter(f_csv,fieldnames_1,dialect='excel')
            writer.writeheader()
            if (j_T == 0):
                j_first = 0
                j_last = math.ceil(((n_theta)/2)%tile_size)
            else:
                j_first = j_last 
                j_last  = j_first + tile_size
                if (j_last > n_theta):
                    j_last = n_theta
            
##            j_stop = j_first
##            print( 'j_start= ',j_start, 'j_stop = ',j_stop, ' j_first = ',j_first,' j_last = ',j_last)
            delta_phi = 2.0/n_phi
            delta_theta = 2.0/n_theta
##            phi = 1. + delta_phi/2 - i_T*tile_size*delta_phi            
            for i in range(int(i_start),int(i_stop)):
                j_start = j_first
                j_stop = j_last
                j_index = 1
##                phi = phi - delta_phi
                phi = 1.0 - delta_phi*(i + 0.5)
##                theta = -(1. - delta_theta/2.0) + j_T*tile_size*delta_theta
                if (i%2 == 1):
                    j_start = j_last - 1 
                    j_stop = j_first - 1
                    j_index = -1
##                    theta = theta + (tile_size - 1.)*delta_theta
                #print ('i_start = ',int(i_start)," i_stop = ",int(i_stop))
                #print ('j_start = ',int(j_start)," j_stop = ",int(j_stop)," j_index = ",j_index)
                for j in range(int(j_start),int(j_stop),j_index):
                    theta = delta_theta*(j + 0.5) -1.
                    #print ('i = ',i,' phi =', phi,' j = ',j,' theta =', theta)
                    x = serindipity(corner_points,[phi,theta])                  
                    for k in range (0,len(lines)):
                        line_buffer = lines[k]
                        test_x = line_buffer[fieldnames_1[0]]
                        test_y = line_buffer[fieldnames_1[1]]                      
                        if ((abs(float(test_x) - float(x[0])) < EPS) and
                            (abs(float(test_y) - float(x[1])) < EPS)):
                            if (is_first == True):
                                saved_altitude = line_buffer[fieldnames_1[2]]
                                line_buffer[fieldnames_1[2]] = STEP_OVER_ALTITUDE
                                saved_param12 = line_buffer[fieldnames_1[12]]
                                line_buffer[fieldnames_1[12]] = -1
                                writer.writerow(line_buffer)
                                line_buffer[fieldnames_1[2]] = saved_altitude
                                line_buffer[fieldnames_1[12]] = saved_param12
                                is_first = False
                            writer.writerow(lines[k])
##                    theta = theta + j_index*delta_theta            
                    #end loop on j
                #csvfile.close()
                #end loop on i                                         
##            j_start = j_stop                        
            # end loop on j_T
##        i_start = i_stop
        f_csv.close()
        # end loop on i_T
    return #End of make_tiled_sorties

def problem_1():
    # get boundary file
    print (' Please browse for border point file')
    border_points_file = get_file_name(' Please browse for border point file ')
    bpf_name = border_points_file.name
    k = 0
    # read boundary points gps coordinates
    latitude = 0
    longitude = 0
    boundary_points = []
    with open(bpf_name,'r') as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            #x_coord = line[0]
            #y_coord = line[1]
            boundary_point  = [float(line[0]),float(line[1])]
            boundary_points.append(boundary_point)
            #print ('bouundary points: ',boundary_points)
            #print ('boundray point [',k,'] = ',boundary_points[k][0],boundary_points[k][1]) 
            k += 1

    #print ('boundary_points', boundary_points)
    bb = build_bounding_box(boundary_points)
    print ('bounding box:',bb)
    center_point = calculate_center_point(bb)
    print('center point',center_point)
    if contains_point(boundary_points,center_point):
        print (' border contains center point')
    #corner_points = [[bb[0][0],bb[1][1]],[bb[0][0],bb[0][1]],[bb[1][0],bb[0][1]],[bb[1][0],bb[1][1]]]
    corner_points = build_corner_nodes(boundary_points)
    print (' corner_points', corner_points)
    dist_1 = calculate_distance([corner_points[1][0],corner_points[1][1]],[corner_points[3][0],corner_points[3][1]])
    print ('Field dimensions in feet ',dist_1)
    pixel_size = float(input('Please enter pixel size in inches: '))
    iw = IMAGE_WIDTH*(pixel_size/12.0)  # image width in feet
    ih = (IMAGE_HEIGHT/IMAGE_WIDTH)*iw
    print ('image width ',iw ,'ft image height ', ih,'ft')
    altitude = (iw/2.0)*CROP_FACTOR/math.tan((math.pi/180.0)*FOV/2.0)
    # temp fix for 11ft corn
    #altitude += 11.0
    print ('altitude = ',altitude,' feet')
    overlap = input ('Please input overlap percentage: ')
    iw_reduced = iw*(1.0 - float(overlap)/100.0)
    ih_reduced = ih*(1.0 - float(overlap)/100.0)
    #### corrected n_phi /n_theta calculation (changed dist[1] for dis[0]) 
    n_phi   = int(abs(dist_1[0])/iw_reduced)
    n_theta = int(abs(dist_1[1])/ih_reduced)
    print('n_phi = ',n_phi, 'n_theta = ', n_theta)
    (waypoints,stepover_points,extra_points,n_cols,virtual_ks,virtual_ls) = conformal_map_flight_plan (corner_points,n_phi,n_theta,boundary_points,altitude)
    #print ('waypoints ', waypoints)
    #print ('extra_points ', extra_points)
    Waypoint_filename, Waypoint_file = build_waypoints(center_point,waypoints,n_cols,virtual_ks)
    want_sorties = input('Do you want to divide flight path into individual sortie waypoint files? (y/n) ')
    if (want_sorties == 'y' or want_sorties == 'Y'):
        tile_size = int(input('Enter Tile Size: '))
        if (tile_size > 1):
            make_tiled_sorties(Waypoint_file,n_phi,n_theta,n_cols,virtual_ks,virtual_ls,tile_size,corner_points)
        else:
            make_sorties(Waypoint_file,n_cols)      
    plot_results = input('Do you want to plot results? (y/n) ')
    if (plot_results == 'y' or plot_results == 'Y'):
        plot_pathes(Waypoint_filename,corner_points,waypoints,extra_points,boundary_points)
    return

def main():
    problem_1()
    print ('Done at last')
    x = input('Enter any key to exit: ')
    return

main()

if __name__ == "main":
     # execute only if run as a script
     main()
