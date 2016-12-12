# c 2016 BBarriage
# c 2016 Fordham University
# Purpose is to find the average of each value in HSVD statistics files
# FRCV LAB FORDHAM UNIVERSITY
# Author: Ben Barriage
# December 2016
import csv
import glob
import os 
import sys

#following lines allow for a user to enter important information regarding the test data for archiving purposes
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
speed = []

i =0
for name in glob.glob("*_grid"+"*_iMax"+"*_angle"+"*_*.csv"): # reads in files across all test files following this format within a directory
    with open(name, 'rb') as csvfile:
        filereader = csv.reader(csvfile, delimiter= ',')
        for row in filereader:
            my_list=row
            j = float(my_list[-1])
            l = float(my_list[-2])
            k = float(my_list[-3])
            m = float(my_list[-4])
            average.append(j) #adds value of j to overall success value
            dist.append(k) #adds value of k to overall distance from goal value
            angle.append(l) #adds value of l to overall angle offset from goal orientation
            speed.append(m) #adds value of m to overall speed (num of steps)
            i= i+1
            
#used for debugging purposes
print "Total Average"
print sum(average)/i
print "Dist Average"
print sum(dist)/i
print "Angle Average"
print sum(angle)/i
print "Speed Average"
print sum(speed)/i

f.write ("Speed Average, Dist average, Angle average, Success Average\n")
f.write(str(sum(speed)/i)) #writes total speed average to the output file
f.write(", ")
f.write(str(sum(dist)/i)) #writes total distance average to the output file
f.write(", ")
f.write(str(sum(angle)/i)) #writes total angle offset average to the output file
f.write(", ")
f.write(str(sum(average)/i)) #writes total success average to the output file 
