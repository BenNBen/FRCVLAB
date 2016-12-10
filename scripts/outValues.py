# c 2016 BBarriage
# c 2016 Fordham University
# Purpose is to find the average of each line for a set of HSVD result files
# FRCV LAB FORDHAM UNIVERSITY
# Author: Ben Barriage
# October 2016
import csv
import glob
import os
import sys

grid = input("Enter the grid number: ")
status = input("Enter whether your data is clean or unclean: ")
def combine(job):
    if (job=="resTrows"):
        num=21 #standard number of lines in resTrows script
    if(job=="resTDiag"):
        num=13 #standard number of lines in resTDiag script
    if(job=="resTDDiag"):
        num=7 #standard number of lines in resTTDiag script
    if(job=="resTallSqAng"):
        num=65 #standard number of lines in resTallSqAng script
    f = file("GRID"+grid+status+job+".txt","wr+")
    data = [[0 for x in xrange (10)] for x in xrange(num)]
    i = 0
    for name in glob.glob(job+grid+'/HSVDout*.csv'):
        with open(name, 'rb') as csvfile:        
            filereader= csv.reader(csvfile, delimiter= ',')
            for row in filereader: 
                my_list = row 
                j = float(my_list[-1]) #assigns value of last element of a line to variable j
                data[filereader.line_num][i]=j 
            i = i+1
    for k in range(1,num):
        f.write(str(sum(data[k])/10)) #writes line averages to a file
        f.write("\n")
        print sum(data[k])/10
#runs through each of the 4 standard scripts used in HSVD testing
combine("resTrows")
combine("resTDiag")
combine("resTallSqAng")
combine("resTDDiag")
