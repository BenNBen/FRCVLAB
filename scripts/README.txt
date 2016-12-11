outValues.py: initial python script used to average individual lines across HSVDout*.csv files
outValuesRevised.py: revised python script, now averages distance, angle offset, and success rate for individual lines across HSVDout*.csv files

To Run outValues scripts: Ensure HSVDout*.csv files exist in order to grab their information.

outAverage.py: initial python script used to find total average of job files created by outValues.py
outAverageRevised.py: revised python script, now works to average distance, angle offset, and success rate

To Run outAverage scripts: Ensure the corresponding outValue script has been executed in order to obtain files necessary for outAverage script. 

doClean: script used in conjunction with filter pointcloud program found within PCL directory to clean dataDump files for HSVD testing

To Run doClean: Ensure filter executable from PCL folder is in the same folder. doClean takes one argument in its execution, which is the name of the folder to be cleaned. 
   For example, "./doClean GRID11" would produce a directory called "GRID11_clean" which would contain clean versions of the dataDump files contained in Grid11. 
 
