//#include <windows.h>
#include <string>
#include <iostream>
#include <fstream>

// General -> C++ -> Additional directories: C:\Program Files %28x86%29\PCL 1.6.0\3rdParty\VTK\include\vtk-5.8

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/ascii_io.h>
#include <pcl/io/file_io.h>

const float NULO = -3.37e+38;				// "Invalid Data" code
void LoadRIS( char* inputFileName, int imheight , int  imwidth);
float map( float value, float istart, float istop, float ostart, float ostop );
void LoadASCII(char* inputFileName);


const int imageWidth = 5201;
const int imageHeight = 5201;
float zArray[imageHeight][imageWidth];
float RISmax;
float RISmin;

int  main2 (int argc, char** argv)
{
  //pcl::PointCloud<pcl::PointXYZ> cloud;
  //pcl::MonochromeCloud::ConstPtr cloud (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  
 
  //Load RIS
  char* inputFileName = "130111_Hereford_04.ris";
  LoadRIS(inputFileName, imageHeight, imageWidth);

  // Fill in the cloud data
  cloud->width    = 2500;
  cloud->height   = 2500;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  double size2 = cloud->points.size ();

 /* for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }*/

  /*for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }*/

  int k = 0;
  for (size_t i = 0; i < cloud->height ; i++)
  {
	  for (size_t j = 0; j < cloud->width ; j++)
	  {
		  if( zArray[i][j] != NULO)
		  {
			  cloud->points[k].x = j;
			  cloud->points[k].y = i;
			  uint32_t z = map( zArray[i][j], RISmin, RISmax, -250, 250);
			  cloud->points[k].z = z;
		  }
		  else
		  {
			  cloud->points[k].x = j;
			  cloud->points[k].y = i;
			  cloud->points[k].z = 0;
		  }

		  k++;

	  }
  }

  //pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  //std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;

  //for (size_t i = 0; i < cloud.points.size (); ++i)
   // std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;


   //Load PCD file
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::io::loadPCDFile ("test_pcd.pcd", *cloud2);
 
  
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {
  }
  

  /*
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud(cloud, "sample cloud");
  viewer->addPointCloud((pcl::PointCloud<pcl::PointXYZ>::ConstPtr) cloud);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  //return (viewer);
  */
  return (0);
}


//Map
float map( float value, float istart, float istop, float ostart, float ostop )
{
	return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

// Loads RIS file (not working. Needs to change all zArray[5201][5201] to global variable zArray )
void LoadRIS( char* inputFileName , int imheight , int  imwidth)
{
	// Load input RIS file
	FILE* lRis = fopen ( inputFileName, "rb" );

	// Jump to data position
	for (int i = 0; i < 88; i++){	    
		unsigned char a = getc (lRis);   
	}   

	// Read z array (it should read 5201x5201 positions x 4bytes/pos = 108201604 bytes)
	//Each 'z' is 1 byte in the RIS file
	size_t counter = fread ( zArray , 1 , sizeof(zArray) , lRis );

	/*
	for (int i = 0; i < imheight; i++) 
			for (int j=0; j<imwidth; j++)
				for (int k=0 ; k<4 ; k++)
					size_t counter = fread ( zArray2[i][j] , 1 , 1 , lRis );
	*/		
	
	//Get max value of RIS
	float RISmax = zArray [0][0];
	float RISmin = zArray [0][0];

	for (int i = 0; i < imheight; i++) 
	{
		for (int j=0; j<imwidth; j++)
			{
				zArray[i][j];
				if (zArray[i][j] > RISmax)
				RISmax = zArray [i][j];
				if (zArray[i][j] < RISmin)
				RISmin = zArray [i][j];
			}
	}
	std::cout<<"\nMax value of the RIS file: "<< RISmax << " mm" << "\n";
	std::cout<<"Min value of the RIS file: "<< RISmin << " mm" <<"\n";
	Beep(0,5000);
	

	// Close input file
	fclose (lRis);

	//inputStart = -250;
	//inputStop = 250;
}



//Load ASCII
void LoadASCII(char* inputFileName)
{


	//ifstream filein (inputFileName);
	//string line;
	//getline(filein,line);
	//std::cout << line << endl;

}