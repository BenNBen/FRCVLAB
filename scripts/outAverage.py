# FRCV LAB FORDHAM UNIVERSITY
# Author: Ben Barriage
# October 2016
import csv
import glob
import os 
import sys


print "Please make sure to surround all your input with ' '"
grid = input("Enter the grid number: ")
status = input("Is your data clean or unclean? ")
f = file("GRID"+grid+"_"+status+"Average.txt","wr+")
average = []
i =0
for name in glob.glob('GRID'+grid+'*.txt'):
   #data = []
    with open(name, 'rb') as csvfile:
        filereader = csv.reader(csvfile, delimiter= ' ')
        for row in filereader:
            my_list=row
            j = float(my_list[-1])
            average.append(j)
            i= i+1
            print(j)
print "Total Average"
print sum(average)/i
f.write("Total Average\n")
f.write(str(sum(average)/i))
