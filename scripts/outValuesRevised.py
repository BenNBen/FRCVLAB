# FRCV LAB FORDHAM UNIVERSITY
# Author: Ben Barriage
# October 2016
import csv
import glob
import os
import sys

print "Please make sure to surround all your input with ' '"
grid = input("Enter the grid number: ")
iMAX = input("Enter the iMax number: ")
theta = input("Enter the angle of the trial: ")
date = input("Enter the date of the trial: ")
def combine(job):
    if (job=="resTrows"):
        num=65
    if(job=="resTDiag"):
        num=25
    if(job=="resTDDiag"):
        num=49
    if(job=="resTallSqAng"):
        num=65
    f = file(job+"_grid"+grid+"_iMax"+iMax+"_angle"+theta+"_"date+".csv","wr+")
    data = [[0 for x in xrange (30)] for x in xrange(num)]
    dist = [[0 for x in xrange (30)] for x in xrange(num)]
    angle = [[0 for x in xrange (30)] for x in xrange(num)]
    i = 0
    for name in glob.glob(job+iMax+'/HSVDout*.csv'):
        with open(name, 'rb') as csvfile:        
            filereader= csv.reader(csvfile, delimiter= ',')
            for row in filereader: 
                my_list = row 
                j = float(my_list[-1])
                k = float(my_list[-4])
                l = float(my_list[-3])
                data[filereader.line_num][i]=j
                dist[filereader.line_num][i]=k
                angle[filereader.line_num][i]=l
            i = i+1
    for k in range(1,num):
        f.write(str(sum(dist[k])/30))
        f.write(", ")
        f.write(str(sum(angle[k])/30))
        f.write(", ")
        f.write(str(sum(data[k])/30))
        f.write("\n")
        print sum(data[k])/30
combine("resTrows")
#combine("resTDiag")
#combine("resTallSqAng")
#combine("resTDDiag")
