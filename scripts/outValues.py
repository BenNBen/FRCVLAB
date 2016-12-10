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
        num=21
    if(job=="resTDiag"):
        num=13
    if(job=="resTDDiag"):
        num=7
    if(job=="resTallSqAng"):
        num=65
    f = file("GRID"+grid+status+job+".txt","wr+")
    data = [[0 for x in xrange (10)] for x in xrange(num)]
    i = 0
    for name in glob.glob(job+grid+'/HSVDout*.csv'):
        with open(name, 'rb') as csvfile:        
            filereader= csv.reader(csvfile, delimiter= ',')
            for row in filereader: 
                my_list = row 
                j = float(my_list[-1])
                data[filereader.line_num][i]=j
            i = i+1
    for k in range(1,num):
        f.write(str(sum(data[k])/10))
        f.write("\n")
        print sum(data[k])/10
combine("resTrows")
combine("resTDiag")
combine("resTallSqAng")
combine("resTDDiag")
