import piexif
import piexif.helper
import PIL
from PIL import Image,ImageChops
from PIL.ExifTags import TAGS
import os
from DateTime import DateTime
import datetime
import pandas as pd
import numpy
import math
import csv
import sys

#sys.path.insert(0,'C:\F_archive\easystore\Python_Programs')
sys.path.insert(0,'F:\Python_Programs')

import my_modules as my_modules
from my_modules import get_directory_name as get_directory_name
from my_modules import get_file_name as get_file_name
from my_modules import save_file_name as save_file_name
from my_modules import get_exif as get_exif
from my_modules import get_gps_deg as get_gps_deg
from my_modules import get_r_of_phi as get_r_of_phi
from my_modules import calculate_distance as calculate_distance
from my_modules import date_to_nth_day as date_to_nth_day
from my_modules import calculate_sun_angle as calculate_sun_angle
from my_modules import date_to_nth_day as date_to_nth_day




FOV = 78.8 #Mavic Pro camer field of view 78.9 Deg
EarthMeanRadius  =   6371.01	# In km
AstronomicalUnit  =  149597890	# In km


def find_closest_waypoint(GPS,latitude,longitude,altitude):
    MIN_DISTANCE_THRESHOLD = 200.0
    STEPOVER_ALTITUDE = 200.0
    ALTITUDE_TOLERANCE = 5.0 # Arbitrarily set at 5 feet
    min_distance  = 2.0*MIN_DISTANCE_THRESHOLD
    closest_distance = [min_distance,min_distance]
    error_dist = [0.0,0.0]
    closest_waypoint = -1  # Negative return implies error
    for waypoint in range(1,len(latitude)):
        #test for stepover
        if abs(float(altitude[waypoint])-float(STEPOVER_ALTITUDE)) > float(ALTITUDE_TOLERANCE): # skip stepover waypoints
            Waypoint_GPS = [float(latitude[waypoint]),float(longitude[waypoint]),float(altitude[waypoint])]
            error_dist = calculate_distance(Waypoint_GPS,GPS)
            #print ('########## error_dist #########\n',error_dist)
            error_distance = math.sqrt(error_dist[0]*error_dist[0] + error_dist[1]*error_dist[1])
            #print ('******* error_distance ******* ',error_distance)
            if error_distance < min_distance:
                min_distance = error_distance
                closest_waypoint = waypoint
                closest_distance = error_dist
    if min_distance > MIN_DISTANCE_THRESHOLD:
        closest_waypoint = -abs(closest_waypoint)
    return (closest_waypoint,closest_distance)

def log_images():
    # GET Directory containiing image files
    print ('\n Browse for directory containg image files from mission: ')
    ImgDir = get_directory_name('Select Directory with image files')

    # Get Directory to hold reformatted and renamed files
    print ('\n Browse for directory to hold reformatted image files')
    ImgDir_reformatted = get_directory_name('Select directory to hold reformatted filles')

    # GET Waypoint path file
    print ('\n Browse for mission waypoint file: ')
    wpf = get_file_name('Select Waypoint file')
    waypoint_file = wpf.name
    print (' ' + waypoint_file)

    # Get file to hold mission meta data
    print ('\n Browse for directory to write mission meta data:')
    CSVDir = get_directory_name('Select directory to hold CSV file')
    #print (CSVDir)

    # Initialize some stuff
    all_lines = []
    this_line = [ImgDir,waypoint_file,'\n']
    all_lines.append(this_line)
    this_line = ['File','DateTime','Flight Time','Path Length','Weigh Point','Camera GPS', 'Error Distance', 'Error W', 'Error H','Shadow W', 'Shadow H','\n']
    all_lines.append(this_line)

    # get waypoint file data
    longitude = []
    latitude  = []
    altitude  = []
    heading   = []
    
    k = 0
    with open(waypoint_file,'r') as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            latitude.append(line[0])
            longitude.append(line[1])           
            altitude.append(line[2])
            heading.append(line[3])
##            print ('longitude = ',longitude[k], ' latitude = ',latitude[k],
##                   'altitude = ', altitude[k],'heading = ', heading[k])
            k += 1
    k = 0
    csvfile.close()
    
    # Build base file name
    base_file_name = ImgDir.split('/')
    print ('length of base file name',len(base_file_name))
    try:
        sortie = base_file_name[len(base_file_name)-1]
        sortie_number = sortie.split('_')
        sortie_number = int(sortie_number[1])
        print ( 'sortie = ',sortie,'sortie_number', sortie_number)
    except:
        sortie_number = 0
        print ( 'Error asigning sortie number sortie = ',sortie,'sortie_number', sortie_number)
        print(base_file_name)
    base_file = ImgDir.replace(base_file_name[0],"")
    base_file = ImgDir.replace("/","")
    base_file = ImgDir.replace("Images/","")
    base_file = base_file.replace(sortie,"")
    #base_file = base_file_name[2] + '_' + base_file_name[4] +'_' + base_file_name[5]
    base_file = base_file + 'sortie_' + str(sortie_number)
    print ('base_file',base_file)
    CSVfile = CSVDir + '/' + 'base_file' + '.csv'
    print ('CSVfile',CSVfile)
    #F:\Adams_Farm\DT07042019\Images\PS500mils\sortie_1
    for filenames in os.listdir(ImgDir):
        if k >= len(longitude)-1:
            break
        fn = ImgDir + '/' + filenames
        im = PIL.Image.open(fn)
        #Image_list.append(im)
        exf = get_exif(fn)            # exf is the image metadata
        GPS = get_gps_deg(exf)        # GPS coordinat of the camera
        dt = exf['DateTimeDigitized'] # UTC date and time
        t = DateTime(dt)
        print ('********** ', filenames, ' **********')
        if (k == 0):  # initialization stuff
            start_time = t
            old_time = start_time
            start_gps = GPS
            old_gps = start_gps
            seconds = 0
            path_length = 0
            altitude_bias = GPS[2] - float(altitude[1]) # possable positioning erro
   
        if (k == 1):
            altitude_bias = GPS[2] - float(altitude[2]) # possable positioning erro
       
        delta_time = 3600.0*(DateTime.hour(t) - DateTime.hour(old_time)) 
        delta_time += 60.0*(DateTime.minute(t) - DateTime.minute(old_time))
        delta_time += DateTime.second(t) - DateTime.second(old_time)
        seconds += delta_time  # total flite time in seconds
        # print 'Time: ', t, 'Delta Time: ' , delta_time , ' Total Flite Time (seconds): ', seconds
        old_time = t
        distance_s = calculate_distance(GPS,old_gps)
        dist = math.sqrt(distance_s[0]*distance_s[0] + distance_s[1]*distance_s[1])
        path_length += dist
        old_gps = GPS

        # find_closest_waypoint
        (closest_waypoint,error_dist) = find_closest_waypoint(GPS,latitude,longitude,altitude)
        #print ("error_dist \n",error_dist)
        error_distance = math.sqrt(error_dist[0]*error_dist[0] + error_dist[1]*error_dist[1])   

        #starting_day_of_current_year = datetime().date().replace(month=1, day=1)
        solar_angle = calculate_sun_angle(GPS,t)
        height = (GPS[2] - altitude_bias)
        ds_0 = 0.
        ds_1 = 0.
        if ((math.tan(solar_angle[0]) != 0) & (abs(solar_angle[0]) != math.pi/2)):
            ds_0 = height/math.tan(solar_angle[0])

        #print 'GPS[2] = ',GPS[2],' solar_angle[0} ',solar_angle[0],' solar_angle[1} ',solar_angle[1]
        ds_1 = -ds_0*math.sin(solar_angle[1])
        ds_0 = -ds_0*math.cos(solar_angle[1])
        delta_shadow = [ds_0,ds_1]
##        print('****delta shadow *****',delta_shadow)
        pixel_size = 0
        if height != 0:
            pixel_size = abs(4000./(2.0*height*math.tan((math.pi/180.)*FOV/2.0)))

        pixel_size = (35./26.)*pixel_size
        # print 'pixels per foot',pixel_size
        H = 1500  - int(pixel_size*ds_1)
        W = 2000  - int(pixel_size*ds_0)

        # print stuff:
        print("Time: {0:s},Delta Time: {1:5.2f}, Total Flite Time (seconds): {2:5.2f}"
              .format(datetime.datetime.now().time(),delta_time,seconds)) 
        print("Camera GPS    {0:10.6f} {1:10.6f} {2:10.6f}  Distance: {3:5.2f} Path Length:{4:5.2f}"
              .format( GPS[0],GPS[1],GPS[2],dist,path_length)) 
        lat = float(latitude[closest_waypoint])
        long = float(longitude[closest_waypoint])
        alt  = float(altitude[closest_waypoint])
        ht   = alt - float(altitude_bias)
        print("Way Point {0:3d}: {1:10.6f} {2:10.6f} error distance = {3:4.2f} feet height = {4:4.2f} feet"
              .format(closest_waypoint,lat,long,error_distance,alt))   
        print ("pixels per foot {0:5.3f}".format(pixel_size))
        print ("height {0:5.3f} delta shadow {1:8.3f} {2:8.3f}(feet) shadow location (pixels) {3:5d} {4:5d}"
               .format(height,delta_shadow[0],delta_shadow[1],-W,H))
        error_W = int(error_dist[1]*pixel_size)
        error_H = int(error_dist[0]*pixel_size)
        this_line = [filenames,t,seconds, path_length,closest_waypoint,GPS,error_distance,error_W,error_H,W,H,'\n']
        all_lines.append(this_line)
        k += 1

##      Reformat and copy File
        exif_dict = piexif.load(im.info["exif"])
        hdg = float(heading[closest_waypoint])
        if hdg > 0:
           hdg = int(hdg+0.5)
        else:
            hdg = int(hdg-0.5)

        sfn =  base_file + '_' + filenames
        sfn = ImgDir_reformatted + "/sortie_" + str(sortie_number) + "_" + filenames

        ps = 12.0/pixel_size # pixel size in inches
        ps = int(1000*ps)

        a_b = int(1000*altitude_bias)

        comment_string = "Sortie?" + str(sortie_number) + "_Heading?" + str(hdg)
        comment_string = comment_string + "_altitude_bias?(" + str(a_b) + ",1000)"
        comment_string = comment_string + "_WindSpeed?TBD" + "_WindDirection?TBD" 
        comment_string = comment_string + "_UVindex?TBD" + "_PixelSize?(" + str(ps) + ",1000)" 
##        print('comment_string: ',comment_string)
        user_comment = piexif.helper.UserComment.dump(comment_string,encoding='ascii')
        exif_dict["Exif"][piexif.ExifIFD.UserComment] = user_comment       

        exif_bytes = piexif.dump(exif_dict)
        
##      save as output_file
        sfn_jpg = sfn 
        print(sfn_jpg)
        if (abs(hdg) > 10.0):
            print("heading {0:5.2f}".format(hdg))
            im.rotated = im.rotate(-hdg,expand=True)
        im.rotated.save(sfn_jpg,exif=exif_bytes)
        im.rotated.close()
        im.close()
    this_line = ['altitude_bias',altitude_bias]
    all_lines.append(this_line)

##  Write flight log file (CSV)
    CSVfile = CSVDir + "/" + "LogData_" + "sortie_" + str(sortie_number) + '.csv'
    print ("altitude bias = {0:6.2f} feet".format(altitude_bias))    
    with open(CSVfile,'w',newline ='\n') as f_csv:
        writer = csv.writer(f_csv)
        for this_line in all_lines:
            writer.writerow(this_line)
    print (' Done at last !!! \n')
    return

def main():
    log_images()
    return

main()
if __name__ == "main":
    # execute only if run as a script
    main()
        
    
    
    


