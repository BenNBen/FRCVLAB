# c 2016 BBarriage
# c 2016 Fordham University
# Purpose is to find the average of each line for a set of 30 HSVD results files
# FRCV LAB FORDHAM UNIVERSITY
# Author: Ben Barriage
# December 2016
import csv
import glob
import os
import sys

print "Please make sure to surround all your input with ' '"
grid = input("Enter the grid number: ")
iMax = input("Enter the iMax number: ")
theta = input("Enter the angle of the trial: ")
date = input("Enter the date of the trial: ")
print "DO NOT surround the following input with ' '"
cores = input("Enter the number of cores used: ") #cores=number of HSVD out files

def combine(job):
    if (job=="resTrows"):
        num=65 #standard number of lines in resTrows
    if(job=="resTDiag"):
        num=25 #standard number of lines in resTDiag
    if(job=="resTDDiag"):
        num=13 #standard number of lines in resTDDiag
    if(job=="resTallSqAng"):
        num=65 #standard number of lines in resTallSqAng
    f = file(job+"_grid"+grid+"_iMax"+iMax+"_angle"+theta+"_"+date+".csv","wr+")
    data = [[0 for x in xrange (cores)] for x in xrange(num)]
    dist = [[0 for x in xrange (cores)] for x in xrange(num)]
    angle = [[0 for x in xrange (cores)] for x in xrange(num)]
    speed = [[0 for x in xrange (cores)] for x in xrange(num)]
    i = 0
    for name in glob.glob(job+iMax+'/HSVDout*.csv'): #standard results files follow format of the job type and the number of images used in testing (iMax)
        with open(name, 'rb') as csvfile:        
            filereader= csv.reader(csvfile, delimiter= ',')
            for row in filereader: 
                my_list = row 
                j = float(my_list[-1]) 
                l = float(my_list[-3]) 
                k = float(my_list[-4]) 
                m = float(my_list[-5]) 
                data[filereader.line_num][i]=j #adds j's value to data, which will represent the % of success
                dist[filereader.line_num][i]=k #adds k's value to dist, which will represent the distance away from the goal location
                angle[filereader.line_num][i]=l #adds l's value to angle, which will represent the offset of the angle from the goal orientation
                speed[filereader.line_num][i]=m #adds m's value to speed, which will represent the number of steps for each run of HSVD
            i = i+1
    for k in range(1,num):
        f.write(str(sum(speed[k])/cores)) #writes average speed (num of Steps) across the number of files to the ouput file
        f.write(", ")
        f.write(str(sum(dist[k])/cores)) #writes average success across the number of files to the output file
        f.write(", ")
        f.write(str(sum(angle[k])/cores))#writes average angle offset across the number of files to the output file
        f.write(", ")
        f.write(str(sum(data[k])/cores))#writes average distance across the number of files to the output file 
        f.write("\n")
        print sum(data[k])/cores
        
#runs through each of the 4 standard scripts used in HSVD testing 
combine("resTrows")
#combine("resTDiag")
#combine("resTallSqAng")
#combine("resTDDiag")
