Filter.cpp: used to covnert HSVD dataDump files from a .txt format to a pointcloud data structure, which is then cleaned with statistical filtering. The resulting cloud is then written back to a file in the original format.

This function takes two arguments, the first being the input file and the second being the output file.

CMakeLists.txt: this is used to build an executable version of filter.cpp called filter. 

TO BUILD:
   In order to build the executable, make a new directory called "build" and run "cmake .." within the build directory. Once this is done, simply run make and an executable called filter should be created.

TO RUN:
   ./filter "input file" "output file"

