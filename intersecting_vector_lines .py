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
def intersect(line_P,line_Q):
    alpha = [0,0]
    # robust routine to find the intersections between two lines
    intersects = False
    P = np.asarray(line_P)
    Mp = np.asmatrix(P).T
    print ('Mp ',Mp)
    P0 = Mp[:,0]
    P1 = Mp[:,1]
    print ('P0 \n',P0,'P1 \n',P1)
    Q = np.asarray(line_Q)
    Mq = np.asmatrix(Q).T
    Q0 = Mq[:,0]
    Q1 = Mq[:,1]
    print ('Q0 \n',Q0,'\nQ1 \n',Q1) 
    A = (P1 - P0)
    B = (Q0 - Q1)
    C = (Q0 - P0)
    print ('\n A \n',A,'\n B \n',B,'\n C \n',C)
    M = np.c_[A,B]
    print ('M \n', M)
    try:
        Iv = np.linalg.inv(M)
        print ('Iv \n',Iv)
        check_I = Iv*M
        print ('check \n',check_I)
        alpha = Iv*C
        print ('alpha \n',alpha)
        if ((alpha[0] >= 0. and alpha[0] <= 1.) and (alpha[1] >= 0. and alpha[1] <= 1.)):
            intersects = True
    except:
        print('No inverse exist')
        intersects = False
    
    return (intersects,alpha)

def problem_1():
    ln_0 = [[39.648143,-82.817382],[39.648426,-82.817382]]
    ln_1 = [[39.6482976315789,-82.8175909891666],[39.6482976315789,-82.8157711025]]
    intersects,alpha = intersect(ln_0,ln_1)
    print ('intersects in range', intersects)
    if intersects:
        x_0 = alpha[0]*ln_0[1][0] + (1-alpha[0])*ln_0[0][0]
        y_0 = alpha[0]*ln_0[1][1] + (1-alpha[0])*ln_0[0][1]
        print ('x_0,y_0',x_0,y_0)

        x_1 = alpha[1]*ln_1[1][0] + (1-alpha[1])*ln_1[0][0]
        y_1 = alpha[1]*ln_1[1][1] + (1-alpha[1])*ln_1[0][1]
        print ('x_1,y_1',x_1,y_1)
        
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
