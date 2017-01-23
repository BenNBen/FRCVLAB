/*
 * c 2017 BBarriage
 * c 2017 Fordham University
 * Author: Ben Barriage
 * Date: 1/23/17
 * Purpose: Take in existing data file and generate a cleaned pointcloud representation of the data, make a vertical slice of the data, and save it as a pointcloud file.
 */

#define PCL_NO_PRECOMPILE
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <stdio.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

struct PointConv //new point data type containing each value from the original data file
{ 
  PCL_ADD_POINT4D;
  int rgba; //represents the union of individual red, blue, and green value for a point
  int r;
  int g;
  int b;
  int u;
  int v;
  int d;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointConv,(float,x,x)(float,y,y)(float,z,z)(int, rgba,rgba)(int, r, r)(int, g, g)(int, b, b)(int, u, u)(int, v, v)(int, d, d))

int rgba(int r, int g, int b) { //function used to convert individual r, g, b values into a unioned value
	return b + 256*g +256*256*r;
}

FILE *in, *out;

/*
  @Purpose: reads in data from the original file and pushes each line in the file as a point in the pointcloud
  @param file: the file which will be read line by line as data is pushed
  @return: program returns an unfiltered point cloud containing original data points from the source file
 */
pcl::PointCloud<PointConv> readRCVfile(FILE *file){
  
  PointConv p;
  pcl::PointCloud<PointConv>::Ptr cloud_unfiltered(new pcl::PointCloud<PointConv>); 
  PointConv point;
  while (fscanf(in,"%f, %f, %f, %d, %d, %d, %d, %d, %d\n", &p.x, &p.y, &p.z, &p.u, &p.v, &p.d, &p.r, &p.g, &p.b)!=EOF) {//scans each line until the end of the file
    point.x =p.x;
    point.y=p.y;
    point.z=p.z;
    point.rgba = rgba(p.r,p.g,p.b);
    point.u=p.u;
    point.v=p.v;
    point.d=p.d;
    point.r=p.r;
    point.g=p.g;
    point.b=p.b;
    cloud_unfiltered->push_back(point); //pushes each point to an unfiltered cloud once values have been assigned to each variable of a point
  }
  cout<< "The cloud size is " <<cloud_unfiltered->size()<<"\n"; //prints cloud size to be compared to the size of the filtered cloud
  return *cloud_unfiltered;
}

/*
  @Purpose: runs statistical filtering on the original data file in order to generate a clean version of the point cloud
  @param cloud: the cloud to be filtered by the statistical filtering program
  @return: program returns a filtered point cloud
 */
pcl::PointCloud<PointConv> doFilter(pcl::PointCloud<PointConv>::Ptr cloud){ 
  cout<<"Slicing\n";
  pcl::PassThrough<PointConv> pass; 
  pcl::PointCloud<PointConv>::Ptr cloud_f(new pcl::PointCloud<PointConv>);
  
  pass.setInputCloud (cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0,1000.0);
  pass.filter (*cloud_f);
  return *cloud_f;
}

/*
  @Purpose: writes the data of the filtered cloud to a pcd file
  @param cloud: the filtered cloud to be written to a file 
  @return: no return value 
 */
void writePCDfile(FILE *file, pcl::PointCloud<PointConv>::Ptr cloud){
  cout <<"Writing!\n";
  PointConv item;
  int k = 0;
  while(k<cloud->size()){//traverses the individual points of the cloud
    item = cloud->points[k];
    fprintf(out,"%lf %lf %lf %d\n", item.x, item.y, item.z, rgba(item.r,item.g, item.b)); 
    //writes a single point's content to a line of the file
    k++;
  }
}

void doHeader(FILE *h, int nP){
  fprintf(h, "# .PCD v.7 - Point Cloud Data file format\n");
  fprintf(h, "FIELDS x y z rgba\n");
  fprintf(h, "SIZE 4 4 4 4\n");
  fprintf(h, "TYPE F F F U\n");
  fprintf(h, "WIDTH %d\n",nP);
  fprintf(h, "HEIGHT 1\n");
  fprintf(h, "POINTS %d\n", nP);
  fprintf(h, "DATA ascii\n");
}

int
main (int argc, char** argv)
{
  pcl::PointCloud<PointConv>::Ptr cloud(new pcl::PointCloud<PointConv>);
  pcl::PointCloud<PointConv>::Ptr cloud_filt(new pcl::PointCloud<PointConv>);
  if(argc!=3){
    printf("Command line arguments infile or outfile missing.\n");
    return 0;
  }
  
  in = fopen(argv[1], "r");
  *cloud = readRCVfile(in);
  fclose(in);

  std::cerr <<"Cloud before filtering: " <<std::endl;
  std::cerr <<*cloud <<std::endl;
  
  *cloud_filt = doFilter(cloud);

  std::cerr<<"Cloud after filtering: "<<std::endl;
  std::cerr<<*cloud_filt <<std::endl;

  long numPoints = cloud_filt->size();

  out =fopen(argv[2],"w");
  doHeader(out, numPoints);
  writePCDfile(out, cloud_filt);
  fclose(out);
  return (0);
}
