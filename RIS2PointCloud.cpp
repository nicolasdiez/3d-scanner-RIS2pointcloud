//Author: Nico Diez (based on Factum´s code RIS file methods and integration Point Cloud Library)
//Description: This file contains all the methods needed to convert and visualize a RIS format image file, into a Point Cloud.

#include <string>
#include <fstream>
#include <algorithm>
#include <math.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <time.h>
#include <limits>
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/point_cloud_conversion.h>

const int imageWidth = 5201;
const int imageHeight = 5201;
const float BAD_POINT = std::numeric_limits<float>::quiet_NaN(); //to flag null points in PCL

void LoadRISBase( char* inputFileName);
void LoadRISTop( char* inputFileName, float zMove);
float map2( float value, float istart, float istop, float ostart, float ostop );
const float NULO = -3.37e+38;				// "Invalid Data" code
void RIS2PointCloudBase(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud , int StartX, int EndX, int StartY, int EndY);
void RIS2PointCloudTop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud , int StartX, int EndX, int StartY, int EndY);

void PointCloud2RIS (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void generateRIS( char* outputFileName);
void DownsampleRISBase(int samplerate);
void DownsampleRISTop (int samplerate);
void RemoveNonCommonPoints();


float zArrayBase[imageHeight][imageWidth];
float zArrayTop[imageHeight][imageWidth];
float zArrayOut[imageHeight][imageWidth];
float RISmax;
float RISmin;

const int downsample = 5;

float zArrayBaseDown[imageHeight/downsample][imageWidth/downsample];
float zArrayTopDown[imageHeight/downsample][imageWidth/downsample];

int	main (int argc, char** argv)
{

	//------------------- Load RIS Files in Global Variables -----------------------
	//char* inputFileName = "data\\Sphere_1.ris";
	//char* inputFileName = "data\\Pyramid\\130611_PrismPrimitive_1_0mm.ris";
	//char* inputFileName = "data\\130701_Wallandsalient_1.ris";
	//char* inputFileName = "data\\130617_RedRelief_4_0mm.ris";
	//char* inputFileName = "data\\130619_PaperTest_2.ris";
	//char* inputFileName = "data\\130111_Hereford_07.ris";
	//char* inputFileName = "data\\130611_LeafTest_3_30mm.ris";
	//char* inputFileName = "data\\130611_ConePrimitive_1_0mm.ris";
	char* inputFileName = "data\\130617_Mechpiece_1_0mm.ris";
	//char* inputFileName = "data\\130618_CylinderTape_3_0mm.ris";
	LoadRISBase(inputFileName);

	//inputFileName = "data\\Sphere_1_15mm.ris";
	//inputFileName = "data\\Pyramid\\130611_PrismPrimitive_1_15mm.ris";
	//inputFileName = "data\\130701_Wallandsalient_1_15mm.ris";
	//inputFileName = "data\\130617_RedRelief_4_15mm.ris";	
	//inputFileName = "data\\130619_PaperTest_2_15mm.ris";
	//inputFileName = "data\\130111_Hereford_07_RS_1_+15mm.ris";   // zMove = - 15 mm !!!
	//inputFileName = "data\\130611_LeafTest_3_45mm.ris";
	//inputFileName = "data\\130611_ConePrimitive_1_RS_15mm.ris";
	inputFileName = "data\\130617_Mechpiece_1_15mm.ris";
	//inputFileName = "data\\130618_CylinderTape_3_15mm.ris";
	float zMove = 15.0f;
	LoadRISTop(inputFileName, zMove);
	
	//------------------- Downsample RIS Files -----------------------
	int samplerate = 1;
	
	DownsampleRISBase ( samplerate );
	DownsampleRISTop ( samplerate );

	//----------------- Remove non common points in the clouds -----------------------

	//RemoveNonCommonPoints();

	//------- Set the ROI on the RIS file to load into PCD (in tenths of mm) ----------
	//The image coordinate origin is located on the left lower corner
	int StartX = 0;// /downsample;
	int EndX = 3000;// /downsample;
	int StartY = 2200;// /downsample;
	int EndY = 5200;// /downsample;
	

	/*//Hereford
	int StartX = 2200;// /downsample;
	int EndX = 5200;// /downsample;
	int StartY = 0;// /downsample;
	int EndY = 3000;// /downsample;
	*/
	//-------- Load RIS Base points into the PCD ----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
	RIS2PointCloudBase(cloud_out, StartX, EndX, StartY, EndY);
	//for (size_t i = 0; i < cloud_out->points.size (); ++i) 
		//std::cout << " Point Cloud Base Point: " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << "  Tenths of mm" << std::endl;
	
	pcl::visualization::CloudViewer viewer2 ("Base Point Cloud");
	viewer2.showCloud (cloud_out);
	while (!viewer2.wasStopped ())
	{
	}

	//-------- Load RIS Top points into the PCD ----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
 	RIS2PointCloudTop(cloud_in , StartX, EndX, StartY, EndY);
	//for (size_t i = 0; i < cloud_in->points.size (); ++i) 
		//std::cout << " Point Cloud Top Point: " << cloud_in->points[i].x << " " << cloud_in->points[i].y << " " << cloud_in->points[i].z << "  Tenths of mm" << std::endl;
	
	pcl::visualization::CloudViewer viewer ("Top Point Cloud");
	viewer.showCloud (cloud_in);
	while (!viewer.wasStopped ())
	{
	}	
	
	//-------------------- REMOVE NaN POINTS FROM CLOUD -------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_2 (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> indices_in;
	std::vector<int> indices_out;
	
	pcl::removeNaNFromPointCloud(*cloud_in,*cloud_in, indices_in);
	pcl::removeNaNFromPointCloud(*cloud_out,*cloud_out, indices_out);

	//---------------------  ITERATIVE CLOSEST POINT  -----------------------
	std::cout << "\n * Aligning with Iterative Closest Point Algorithm... *\n\n";
	clock_t timeICP = clock();
	
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);
	//icp.setEuclideanFitnessEpsilon(-1.797e+5);
	//icp.setMaximumIterations(40);
	//icp.setRANSACIterations(2000);

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	
	std::cout << "Has converged: " << icp.hasConverged() << " Score: " << icp.getFitnessScore() << " Tenths of mm \n" << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	
	timeICP = clock() - timeICP;
	printf ("\nICP consumed %d clicks (%f seconds or %f minutes).\n",timeICP,((float)timeICP)/CLOCKS_PER_SEC, ((float)timeICP)/CLOCKS_PER_SEC/60);

	//Create a pointer to point the Final aligned PointXYZ to use into the CloudViewer class
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final2 (new pcl::PointCloud<pcl::PointXYZ>); 
	*Final2 = Final;
	pcl::visualization::CloudViewer viewer3 ("ICP Merged Point Cloud");
	viewer3.showCloud (Final2);
	while (!viewer3.wasStopped ())
	{
	}
	
	/*
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer3->setBackgroundColor (0, 0, 0);
	viewer3->addPointCloud<pcl::PointXYZ> (Final, "sample cloud");
	viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer3->addCoordinateSystem (1.0);
	viewer3->initCameraParameters ();
	*/


	//for (size_t i = 0; i < cloud_in->points.size (); ++i) 
	//	std::cout << "    " << cloud_in->points[i].x << " " << cloud_in->points[i].y << " " << cloud_in->points[i].z << std::endl;

	/*
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud_in);
	while (!viewer.wasStopped ())
	{
	}
	*/
	
	//------------------ Export Back to RIS --------------------
	std::cout << "\n * Exporting Point Cloud to RIS... *\n";
	for (int i=0; i<5200; i++){
		for (int j=0; j<5200; j++){
			zArrayOut[i][j] = NULO;
		}
	}

	int k = 0;
	cout << "Final size = " << Final.size() << endl;

	for (size_t i = 0; i < Final.points.size (); ++i) 
		std::cout << "   Final Point: " << Final.points[i].x << "("<< floor(Final.points[i].x + 0.5) << ")  " << Final.points[i].y << "  " << Final.points[i].z << std::endl;

	for (size_t i = 0 ; i < Final.size() ; ++i)
	{
		int xRIS = floor(Final.points[k].x + 0.5)  + 1000;
		int yRIS = Final.points[k].y ;
		zArrayOut[xRIS][yRIS] = (float) Final.points[k].z / 10.0f;

		cout << " k = " << k << endl;
		k++;
	}

	/*
	for (int i = 0 ; i < imageWidth ; i++)
	{
		for (int j = 0 ; j < imageHeight ; j++)
		{
			if( Final.points[k].z != 0)
			{		

				zArrayOut[i][j] = Final.points[k].z;
			}
			else
			{
				zArrayOut[i][j] = NULO;
			}

			cout << " k = " << k << endl;
			k++;
			
			if (k == Final.size())
			{
				cout << "Final Reached. k = " << k << endl;
				break;
			}
		}
	}
	*/
	//PointCloud2RIS (Final);
	char* outputFileName = "data\\RISout.ris";
	generateRIS( outputFileName);
	//--------------------------------------
	

	

	/*
	// Fill in the CloudIn data
	cloud_in->width    = 5;
	cloud_in->height   = 5;
	cloud_in->is_dense = false;
	cloud_in->points.resize (cloud_in->width * cloud_in->height);

	for (size_t i = 0; i < cloud_in->points.size (); ++i)
	{
		cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}

	std::cout << "Saved " << cloud_in->points.size () << " data points to input:" << std::endl;

	for (size_t i = 0; i < cloud_in->points.size (); ++i) 
		std::cout << "    " << cloud_in->points[i].x << " " << cloud_in->points[i].y << " " << cloud_in->points[i].z << std::endl;

	*cloud_out = *cloud_in;

	std::cout << "size:" << cloud_out->points.size() << std::endl;

	for (size_t i = 0; i < cloud_in->points.size (); ++i)
		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;

	std::cout << "Transformed " << cloud_in->points.size () << " data points:" << std::endl;

	for (size_t i = 0; i < cloud_out->points.size (); ++i)
		std::cout << "    " << cloud_out->points[i].x << " " <<	cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

	*/

	
	//--------- Downsample BASE Point Clouds -----------------
	/*
	 sensor_msgs::PointCloud2::Ptr cloud_out2_down (new sensor_msgs::PointCloud2 ());
	 sensor_msgs::PointCloud2::Ptr cloud_out2 (new sensor_msgs::PointCloud2 ());

	 sensor_msgs:convertPointCloudToPointCloud2(cloud_out, cloud_out2);

	// Create the filtering object
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (cloud_out2);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_out2_down);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_down (new pcl::PointCloud<pcl::PointXYZ>);
	
	// Fill in the downsampled cloud data
	cloud_out_down->width    = cloud_out->width;
	cloud_out_down->height   = cloud_out->height;
	cloud_out_down->is_dense = false;
	cloud_out_down->points.resize (cloud_out->width * cloud_out->height);

	int samplerate = 20;
	int samplecounter = 20;

	for (int i = 0 ; i < cloud_out_down->size() ; ++i)
	{
		cloud_out_down->points[i].x = NULL;
		cloud_out_down->points[i].y = NULL;
		cloud_out_down->points[i].z = NULL; 

		if (samplecounter == samplerate)
		{
			cloud_out_down->points[i].x = cloud_out->points[i].x;
			cloud_out_down->points[i].y = cloud_out->points[i].y;
			cloud_out_down->points[i].z = cloud_out->points[i].z;
			samplecounter = 0;
		}

		samplecounter++;
	}

	pcl::visualization::CloudViewer viewer4 ("Base Downsampled");
	viewer4.showCloud (cloud_out_down);
	while (!viewer4.wasStopped ())
	{
	}
	*/


	return (0);
}

void main2 ()
{
		
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the CloudIn data
	cloud_in->width    = 25;
	cloud_in->height   = 25;
	cloud_in->is_dense = false;
	cloud_in->points.resize (cloud_in->width * cloud_in->height);

	for (size_t i = 0; i < cloud_in->points.size (); ++i)
	{
		cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}

	/*
	int k = 0;
	for (size_t x = 0; x < cloud_in->width ; ++x)
	{
		for (size_t y = 0; y < cloud_in->height; ++y)
		{
			cloud_in->points[k].x = x;
			cloud_in->points[k].y = y;
			cloud_in->points[k].z = sqrt( 15.0f*15.0f - (float)x*x - (float)y*y );

			k++;
		}
	}
	*/
	std::cout << "Saved " << cloud_in->points.size () << " data points to input:" << std::endl;

	for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " << cloud_in->points[i].x << " " << cloud_in->points[i].y << " " << cloud_in->points[i].z << std::endl;
	
	*cloud_out = *cloud_in;

	std::cout << "size:" << cloud_out->points.size() << std::endl;

	for (size_t i = 0; i < cloud_in->points.size (); ++i)
		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;

	std::cout << "Transformed " << cloud_in->points.size () << " data points:" << std::endl;
	
	for (size_t i = 0; i < cloud_out->points.size (); ++i)
		std::cout << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
	
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	//Visualize
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final2 (new pcl::PointCloud<pcl::PointXYZ>);
	*Final2 = Final;
	pcl::visualization::CloudViewer viewer3 ("Simple Cloud Viewer");
	viewer3.showCloud (Final2);
	while (!viewer3.wasStopped ())
	{
	}
	

	/*
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer4 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer4->setBackgroundColor (0, 0, 0);
	viewer4->addPointCloud<pcl::PointXYZ> (Final2, "sample cloud");
	viewer4->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer4->addCoordinateSystem (1.0);
	viewer4->initCameraParameters ();
	*/

	//return (0);
}

void LoadRISBase( char* inputFileName)
{
	std::cout << "\n * Loading RIS File Base: '" << inputFileName << "' ... *\n";

	// Load input RIS file
	FILE* lRis = fopen ( inputFileName, "rb" );

	// Jump to data position
	for (int i = 0; i < 88; i++){	    
		unsigned char a = getc (lRis);   
	}   

	// Read z array (it should read 5201x5201 positions x 4bytes/pos = 108201604 bytes)
	//Each 'z' is 1 byte in the RIS file
	//!!!! fread coloca el origen abajo a la izquierda como en un sistema tradicional !!!!  
	size_t counter = fread ( zArrayBase , 1 , sizeof(zArrayBase) , lRis );

	/*
	for (int i = 0; i < imheight; i++) 
			for (int j=0; j<imwidth; j++)
				for (int k=0 ; k<4 ; k++)
					size_t counter = fread ( zArray2[i][j] , 1 , 1 , lRis );
	*/		
	
	//Get max value of RIS
	float RISmax = NULO;
	float RISmin = -NULO;
	
	double basenullcount = 0 ;

	for (int i = 0; i < imageWidth; i++) 
	{
		for (int j = 0; j < imageHeight; j++)
			{
				if (/*zArrayBase[i][j] != 0 &&*/ zArrayBase[i][j] != NULO)
				{
					//cout << "zArrayBase[" << i << "][" << j << "] = " << zArrayBase[i][j] << endl;
				
					if (zArrayBase[i][j] > RISmax)
						RISmax = zArrayBase[i][j];
					if (zArrayBase[i][j] < RISmin)
						RISmin = zArrayBase[i][j];
				}

				if (zArrayBase[i][j] == NULO)
				{
					basenullcount++;
				}
			}
	}

	double percetagenull = (basenullcount*100)/(imageWidth*imageHeight);	
	std::cout<<"\nMax value of the RIS file Base: "<< RISmax << " mm" << "\n";
	std::cout<<"Min value of the RIS file Base: "<< RISmin << " mm" <<"\n";
	std::cout<<"Number of NULL Data on RIS Base: "<< basenullcount << " ( "<< percetagenull << " % )" << "\n";
	Beep(0,5000);
	

	// Close input file
	fclose (lRis);

	//inputStart = -250;
	//inputStop = 250;
}

void LoadRISTop( char* inputFileName , float zMove)
{
	std::cout << "\n * Loading RIS File Top: '" << inputFileName << "' (Z axis Move: " << zMove <<" mm) ... *\n";

	// Load input RIS file
	FILE* lRis = fopen ( inputFileName, "rb" );

	// Jump to data position
	for (int i = 0; i < 88; i++){	    
		unsigned char a = getc (lRis);   
	}   

	// Read z array (it should read 5201x5201 positions x 4bytes/pos = 108201604 bytes)
	//Each 'z' is 1 byte in the RIS file
	size_t counter = fread ( zArrayTop , 1 , sizeof(zArrayTop) , lRis );

	/*
	for (int i = 0; i < imheight; i++) 
			for (int j=0; j<imwidth; j++)
				for (int k=0 ; k<4 ; k++)
					size_t counter = fread ( zArray2[i][j] , 1 , 1 , lRis );
	*/		
	
	//Get max value of RIS
	float RISmax = NULO;
	float RISmin = -NULO;
	double topnullcount = 0 ;

	for (int i = 0; i < imageWidth; i++) 
	{
		for (int j=0; j < imageHeight; j++)
			{

				if (/*zArrayTop[i][j] != 0 &&*/ zArrayTop[i][j] != NULO)
				{
					zArrayTop[i][j] = zArrayTop[i][j] + zMove;
					//cout << "zArrayTop[" << i << "][" << j << "] = " << zArrayTop[i][j] << endl;
				
					if (zArrayTop[i][j] > RISmax)
						RISmax = zArrayTop [i][j];
					if (zArrayTop[i][j] < RISmin)
						RISmin = zArrayTop [i][j];
				}

				if (zArrayTop[i][j] == NULO)
				{
					topnullcount++;
				}

			}
	}

	double percetagenull2 = (topnullcount*100)/(imageWidth*imageHeight);	
	std::cout<<"\nMax value of the RIS file Top: "<< RISmax << " mm" << "\n";
	std::cout<<"Min value of the RIS file Top: "<< RISmin << " mm" <<"\n";
	std::cout<<"Number of NULL Data on RIS Top: "<< topnullcount << " ( "<< percetagenull2 << " % )" << "\n";
	Beep(0,5000);
	

	// Close input file
	fclose (lRis);

	//inputStart = -250;
	//inputStop = 250;
}

//Map
float map2( float value, float istart, float istop, float ostart, float ostop )
{
	return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

void RIS2PointCloudBase(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, int StartX, int EndX, int StartY, int EndY)
{
	std::cout << "\n * Converting RIS File Base to Point Cloud Data (ROI x:["<< StartX <<","<< EndX <<"] y:[" << StartY <<","<< EndY << "] Tenths of mm) ... * \n";

// Fill in the cloud data
	cloud_out->width    = EndX - StartX;
	cloud_out->height   = EndY - StartY;
	cloud_out->is_dense = false;
	cloud_out->points.resize (cloud_out->width * cloud_out->height);

	int k = 0;

	for (int i = StartX ; i < cloud_out->width ; i++)
	{
		for (int j = StartY ; j < StartY + cloud_out->height ; j++)
		{
			if( zArrayBase[i][j] != NULO)
			{
				cloud_out->points[k].x = (float) i;
				cloud_out->points[k].y = (float) j;
				//uint32_t z = map2( zArrayBase[i][j], RISmin, RISmax, -250, 250);
				//cloud->points[k].z = z;
				cloud_out->points[k].z = (float) zArrayBase[i][j] * 10;  //Convert to tenths of mm

				//cout << "RIS2PointCloud zArrayBase[" << i << "][" << j << "] = " << zArrayBase[i][j] << endl;
			
			}
			else
			{
				cloud_out->points[k].x = BAD_POINT;
				cloud_out->points[k].y = BAD_POINT;
				cloud_out->points[k].z = BAD_POINT;
			}

			k++;

		}
	}
}

//Generate RIS file (not working. Needs to change all zArray[5201][5201] to global variable zArray )
void generateRIS( char* outputFileName)
{

	cout << "\n\n ** Generating RIS File ** \n\n";

	//--------------------------------------------Read input file-------------------------------------------------
	 size_t a = 0;
	//FILE* in_file = fopen (  "c:\\Lucifer\\raw32_2_ris_in.raw", "rb" );

	// Read data matrix
	//fread (z_input , 1 , sizeof(z_input) , in_file );

	// Close input file
	//fclose (in_file);


	//--------------------------------------------Write output file-------------------------------------------------
	 FILE* output_file;
	 FILE* tmp_file;
	 int b = 0;

	// Abro el archivo binario de salida para escritura
	output_file = fopen ( outputFileName , "wb" );
	
	// Construyo la cabecera
	tmp_file = fopen ( "data\\r.bin" , "rb");
	for (b = 0; b < 88; b++){	    
		a = getc (tmp_file);   
		putc (a , output_file);   
	}   
	fclose (tmp_file);

	// Meto la matriz de datos con las alturas
	fwrite (zArrayOut , 1 , sizeof(zArrayOut) , output_file );

	// Construyo la cola
	tmp_file = fopen ( "data\\r.bin" , "rb" );

	// Avanzo a la posicion de comienzo de la cola
	for (b = 0; b < 88; b++){
		a = getc (tmp_file);
	}

	// Copio los bytes de la cola al archivo de salida
	for (b = 88; b < 124980 + 88; b++){		      
		a = getc (tmp_file);						
		putc (a , output_file);						
	}  
	fclose (tmp_file);


	//Cierro el archivo de salida
	fclose (output_file);

	printf("\nRIS file created: %s", outputFileName );

	Beep(1000, 500);
}


void RIS2PointCloudTop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in , int StartX, int EndX, int StartY, int EndY)
{
	std::cout << "\n * Converting RIS File Top to Point Cloud Data (ROI x:["<< StartX <<","<< EndX <<"] y:[" << StartY <<","<< EndY << "] Tenths of mm) ... * \n";
	// Fill in the cloud data
	cloud_in->width    = EndX - StartX;
	cloud_in->height   = EndY - StartY;
	cloud_in->is_dense = false;
	cloud_in->points.resize (cloud_in->width * cloud_in->height);

	int k = 0;

	for (int i = StartX ; i < cloud_in->width ; i++)
	{
		for (int j = StartY ; j < StartY + cloud_in->height ; j++)
		{
			if( zArrayTop[i][j] != NULO)
			{
				cloud_in->points[k].x = (float) i;
				cloud_in->points[k].y = (float) j;
				//uint32_t z = map2( zArrayBase[i][j], RISmin, RISmax, -250, 250);
				//cloud->points[k].z = z;
				cloud_in->points[k].z = (float) zArrayTop[i][j] * 10; //Convert to tenths of mm

				//cout << "RIS2PointCloud zArrayBase[" << i << "][" << j << "] = " << zArrayBase[i][j] << endl;

			}
			else
			{
				cloud_in->points[k].x = BAD_POINT;
				cloud_in->points[k].y = BAD_POINT;
				cloud_in->points[k].z = BAD_POINT;
			}

			k++;

		}
	}
}


void PointCloud2RIS (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	for (int i=0; i<5200; i++){
		for (int j=0; j<5200; j++){
			zArrayOut[i][j] = NULO;
		}
	}

	int k = 0;

	for (int i = 0 ; i < imageWidth ; i++)
	{
		for (int j = 0 ; j < imageHeight ; j++)
		{
			if( cloud->points[k].z != 0)
			{				
				zArrayOut[i][j] = cloud->points[k].z;
			}
			else
			{
				zArrayOut[i][j] = NULO;
			}

			k++;
		}
	}
}


void DownsampleRISBase (int samplerate)
{
	std::cout << "\n * Downsampling RIS File Base by " << samplerate << " ... * \n";

	if (samplerate =! 0)
	{
		//Downsample by sending to NULL points in the original RIS file
		for (int i = 0 ; i < imageWidth ; i++)
		{
			for (int j = 0 ; j < imageHeight ; j++)
			{
				if ( i%samplerate != 0 || j%samplerate != 0 )
				{
					zArrayBase[i][j] = NULO;
				}
			}
		}
	}
	else
	{
		std::cout << "ERROR: Sample rate must be greater than 0 \n";
	}

	

	/*
	//Downsample coying values to another variable 
	for (int i = 0 ; i < imageWidth ; i = i + downsample )
	{
		for (int j = 0 ; j < imageHeight ; j = j + downsample )
		{
			zArrayBaseDown[i/downsample][j/downsample] = zArrayBase[i][j] ;// /downsample;
			//std::cout << "zArrayBaseDown[" << i/downsample << "," << j/downsample << "]: " << zArrayBaseDown[i/downsample][j/downsample] << std::endl ;
		}
		
	}
	*/
}

void DownsampleRISTop (int samplerate)
{
	std::cout << "\n * Downsampling RIS File Top by " << samplerate << " ... * \n";

	//Downsample by sending to NULL points in the original RIS file
	for (int i = 0 ; i < imageWidth ; i++)
	{
		for (int j = 0 ; j < imageHeight ; j++)
		{
			if ( i%samplerate != 0 || j%samplerate != 0 )
			{
				zArrayTop[i][j] = NULO;
			}
		}
	}
	

	/*
	//Downsample coying values to another variable 
	for (int i = 0 ; i < imageWidth ; i = i + downsample )
	{
		for (int j = 0 ; j < imageHeight ; j = j + downsample )
		{
			zArrayTopDown[i/downsample][j/downsample] = zArrayTop[i][j] ;// /downsample;
			//std::cout << "zArrayBaseDown[" << i/downsample << "," << j/downsample << "]: " << zArrayBaseDown[i/downsample][j/downsample] << std::endl ;
		}

	}
	*/
}

void RemoveNonCommonPoints()
{
	std::cout << "\n * Removing Non Common Points Between Base and Top ... * \n" << endl;

	int removecount = 0;
	//Remove point if it is not present in Base and Top Clouds
	for (int i = 0 ; i < imageWidth ; i++)
	{
		for (int j = 0 ; j < imageHeight ; j++)
		{
			if ( zArrayBase[i][j] == NULO && zArrayTop[i][j] != NULO )
			{
				zArrayBase[i][j] = NULO;
				zArrayTop[i][j] = NULO;
				removecount++;
			}
			else if (zArrayBase[i][j] != NULO && zArrayTop[i][j] == NULO)
			{
				zArrayBase[i][j] = NULO;
				zArrayTop[i][j] = NULO;
				removecount++;
			}
		}
	}

	std::cout << "Removed Points: " << removecount << endl;
}