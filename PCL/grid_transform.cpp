/*
 * c 2017 BBarriage
 * c 2017 Fordham University
 * Author: Ben Barriage
 * Date: 4/20/17
 * Purpose: Take in grid name as an argument and print out a large pcd file containing pcd files of each individual square in the grid
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <math.h>

// This function displays the help
void
showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " Grid Name" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

int countDir(char *dir);

// This is the main function
int
main (int argc, char** argv)
{
  int OFFSET =400.0;
  //float theta = M_PI/4;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  
  // Show help
  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
  }
  
  char directory[100];
  sprintf(directory,"%s", argv[1]);
  int numDir = countDir(directory);
  printf("The subdirectory count is %d\n", numDir);
  
  int dimension = sqrt(numDir);
  printf("The dimensions of the grid are %d by %d\n", dimension, dimension);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr square_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ( ));
  
  int accY=0;
  for (int x=0;x<dimension;x++){
    int accX=0;
    for(int y=0;y<dimension;y++){
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr xFiltered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr yFiltered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr zFiltered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

      int squareNum=(x+1)+(dimension*y);
      for(int ang=0;ang<360;ang+=36){
	char input[100];
	sprintf(input, "%s/SQUARE%d/%d.pcd",argv[1],squareNum,ang);
	printf("Using %s\n", input);
	pcl::io::loadPCDFile(input,*single_cloud);

	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(single_cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-8000.0,8000.0);
	pass.filter(*xFiltered_cloud);
	pass.setInputCloud(xFiltered_cloud);
	pass.setFilterFieldName("y");
	pass.filter(*yFiltered_cloud);
	pass.setInputCloud(yFiltered_cloud);
	pass.setFilterFieldName("z");
	pass.filter(*zFiltered_cloud);
	
	pcl::PointXYZRGB point;
	for(int x=-50;x<50;x+=10)
	  for(int y=-50;y<50;y+=10)
	    for(int z=-50;z<50;z+=10){
		point.x=x;
		point.y=y;
		point.z=z - 1000;
		point.r=255;
		point.g=0;
		point.b=0;
		zFiltered_cloud->push_back(point);
	    }
	  
	
	
	*source_cloud += *zFiltered_cloud;
      }
      transform.translation() << accX, accY,0.0;
      pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform);
      *square_cloud += *transformed_cloud;
      printf("The x and y OFFSETS are %d and %d\n", accX, accY);
      accX += OFFSET;
    }
    accY -= OFFSET;   //negative for our purposes due to coordinates becoming more negative along the yAxis
  }
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  printf("The combined cloud size was %ul\n", square_cloud->points.size());

 
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statFilt;
  statFilt.setInputCloud(square_cloud);
  statFilt.setMeanK(50);
  statFilt.setStddevMulThresh(0.5);
  statFilt.filter(*cloud_filtered);

  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_filtered);
  //sor.setInputCloud(square_cloud);
  sor.setLeafSize(50.0f, 50.0f, 50.0f);
  sor.filter(*voxel_cloud);


  printf("The statistical filtered grid cloud size is %ul\n", cloud_filtered->points.size());
  printf("The voxel grid filtered cloud size is %ul\n", voxel_cloud->points.size());

  char outFile[100];
  sprintf(outFile, "%s_grid.pcd",argv[1]);

  pcl::io::savePCDFileASCII("test.pcd", *square_cloud);
  pcl::io::savePCDFileASCII("filtered.pcd", *cloud_filtered);
  pcl::io::savePCDFileASCII(outFile, *voxel_cloud);
}


int countDir(char *dir){
  struct dirent *dp;
  DIR *fd;

  int count =0;
  if ((fd = opendir(dir)) == NULL) {
    fprintf(stderr, "listdir: can't open %s\n", dir);
    return 0;
  }
  while ((dp = readdir(fd)) != NULL) {
  if (!strcmp(dp->d_name, ".") || !strcmp(dp->d_name, ".."))
    continue;    /* skip self and parent */
  printf("%s/%s\n", dir, dp->d_name);
  count++;
  }
  closedir(fd);
  return count;
}

int rgba(int r, int g, int b) {
	return b + 256*g +256*256*r;
}
