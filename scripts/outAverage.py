# c 2016 BBarriage
# c 2016 Fordham University
#purpose is to find the average of each value in HSVD statistics files
# FRCV LAB FORDHAM UNIVERSITY
# Author: Ben Barriage
# October 2016
import csv
import glob
import os 
import sys

#following lines allow for a user to enter important information regarding the test data for archiving purposes
print "Please make sure to surround all your input with ' '"
grid = input("Enter the grid number: ")
status = input("Is your data clean or unclean? ")
f = file("GRID"+grid+"_"+status+"Average.txt","wr+")
average = []
i =0
for name in glob.glob('GRID'+grid+'*.txt'): #reads in files across all test files following this format within a directory
   #data = []
    with open(name, 'rb') as csvfile:
        filereader = csv.reader(csvfile, delimiter= ' ')
        for row in filereader:
            my_list=row
            j = float(my_list[-1]) #finds last value of a line and stores it in variable j
            average.append(j) #adds value of j to overall success value
            i= i+1
            print(j)
print "Total Average"
print sum(average)/i
f.write("Total Average\n")
f.write(str(sum(average)/i)) #writes total success average to the output file
