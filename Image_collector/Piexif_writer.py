"""
exif writer:
    (1) opens a file:
    (2) read/priont its exif data
    (2) changes it's name
    (3) edit's exif data
    (4) saves file with new name and edited exif data
"""
# import the necessary packages
import piexif
import piexif.helper
import PIL
from PIL import Image,ImageChops

import tkinter
from tkinter import *
from tkinter import filedialog
import os

from DateTime import DateTime
from datetime import datetime
import math

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

def problem_1():

    # Get first file to process
    print ('Please Browse  image ')
    ImageFile = get_file_name('Select Image File')
    im = PIL.Image.open(ImageFile)    

    # Get output file directory and file name
    print ('browse for directory to store output file')
    output_Dir = get_directory_name('select output directory')
    output_filename = input(' Enter output file name: ')
    output_file = output_Dir + '/' + output_filename + '.JPEG'

    # Read exif data
    #exif_dict = piexif.load(ImageFile.name)
    exif_dict = piexif.load(im.info["exif"])

    for ifd_name in exif_dict:
        if ifd_name == "thumbnail":  # Don't print the thumbnail
            break
        print("\n{0} IFD:".format(ifd_name))
        for key in exif_dict[ifd_name]:
            try:
                print(key, exif_dict[ifd_name][key][:10])
            except:
                print(key, exif_dict[ifd_name][key])

    try:
        print('User Comment ' + str(exif_dict["Exif"][piexif.ExifIFD.UserComment]))
    except:
        pass

    # Change GPS altltued
    exif_dict["GPS"][6] = (34142,1000) # Changed to waypoint target

    # Write comment
    comment_string = "Heading_Sortie_WindSpeed_WindDirection_UVindex_PixelSize"
    comment_string = 'Heading?' + '90' + "_Sortie?" + '1' + "_WindSpeed?TBD_WindDirection?TBD_UVindex?TBD_PixelSize?500"
    user_comment = piexif.helper.UserComment.dump(comment_string,encoding="ascii")
    exif_dict["Exif"][piexif.ExifIFD.UserComment] = user_comment

    # Dump exif data
    print ('exif_dict',exif_dict)
    exif_bytes = piexif.dump(exif_dict)

    #save as output_file
    im.save(output_file,exif=exif_bytes)
  
    return
    

def main():
    problem_1()
    #problem_2()

    return

main()
"""
if __name__ == "main":
     # execute only if run as a script
     main()
"""
