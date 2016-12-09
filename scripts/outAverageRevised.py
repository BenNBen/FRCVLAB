# FRCV LAB FORDHAM UNIVERSITY
# Author: Ben Barriage
# October 2016
import csv
import glob
import os 
import sys


print "Please make sure to surround all your input with ' '"
grid = input("Enter the grid number: ")
job = input("Enter the job type: ")
iMax = input("Enter the iMax number: ")
theta = input("Enter the angle value: ")
date = input("Enter the date: ")
f = file(job+"_grid"+grid+"_iMax"+iMax+"_angle"+theta+"_"+date+"Average.csv","wr+")
average = []
dist = []
angle = []
i =0
for name in glob.glob(*"_grid"+*+"_iMax"+*+"_angle"+*+"_"*+".csv"):
    with open(name, 'rb') as csvfile:
        filereader = csv.reader(csvfile, delimiter= ' ')
        for row in filereader:
            my_list=row
            j = float(my_list[-1])
            k = float(my_list[-4])
            l = float(my_list[-3])
            average.append(j)
            dist.append(k)
            angle.append(l)
            i= i+1
            #print(j)
print "Total Average"
print sum(average)/i
print "Dist Average"
print sum(dist)/i
print "Angle Average"
print sum(angle)/i
f.write ("Dist average, Angle average, S Average\n")
f.write(str(sum(dist)/i))
f.write(", ")
f.write(str(sum(angle)/i))
f.write(", ")
f.write(str(sum(average)/i))
