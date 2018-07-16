/*
 * Copyright (C) 2014 Love Park Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"
#include "o3d3xx_image.h"
#include <ctime>

#include <fstream>


  int camera_id;
  float threshold= 0.99;
  float distance = 40;
  int noChangeCount = 10;
  bool auto_snapshot=true;
//-------------------------------------------------------------
// Quick and dirty viewer application to visualize the various libo3d3xx images
// -- leverages the built-in PCL and OpenCV visualization infrastructure.
//-------------------------------------------------------------

class O3DViewer
{
public:
  O3DViewer(o3d3xx::Camera::Ptr cam, const std::string& descr)
    : cam_(cam),
      description_(descr)
  {}


  void Run()
  {
    int win_w = 800;
    int win_h = 600;

    cv::Mat CurrMat;		//Current Matrix (t=0)
    cv::Mat last1Mat;		//t-1 Matrix
    //cv::Mat last2Mat;		//t-2 Matrix
    cv::Mat refMat;		//Reference Matrix
    cv::Mat SubMat;		//Subtract Matrix = |CurrMat - FolloMat|
    cv::Mat SubRefMat;		//Subtract Temp Matrix = |CurrMat - TempMat|
    cv::Mat XYZMat;
    cv::Mat MeanMat;
    std::string SuMa;
    std::string XyzMa;
    int check = 0;
    int check2 = 0;
    int countloop = 0; 		//Counting 'No Change' loops
    bool change = false;
    bool tempchange = false;	

    int frameCount = 0;
    std::string path = "/home/pi/Desktop/pactrisPro/pcd/";
  


    SuMa = "SuMa.csv";
    XyzMa = "XYZMatrix.csv";

    o3d3xx::FrameGrabber::Ptr fg =
      std::make_shared<o3d3xx::FrameGrabber>(this->cam_);

    o3d3xx::ImageBuffer::Ptr buff =
      std::make_shared<o3d3xx::ImageBuffer>();

    //
    // Setup for point cloud
    //
    std::shared_ptr<pcl::visualization::PCLVisualizer> pclvis_ =
      std::make_shared<pcl::visualization::PCLVisualizer>(this->description_);
    pclvis_->setBackgroundColor(0, 0, 0);
    pclvis_->setSize(win_w, win_h);
    pclvis_->setCameraPosition(-3.0, // x-position
                               0,    // y-position
                               0,    // z-position
                               0,    // x-axis "up" (0 = false)
                               0,    // y-axis "up" (0 = false)
                               1);   // z-axis "up" (1 = true)

    // use "A" and "a" to toggle axes indicators
    pclvis_->registerKeyboardCallback(
     [&](const pcl::visualization::KeyboardEvent& ev)
      {
        if (ev.getKeySym() == "A" && ev.keyDown())
          {
            pclvis_->addCoordinateSystem();
          }
        else if (ev.getKeySym() == "a" && ev.keyDown())
          {
            pclvis_->removeCoordinateSystem();
          }
	else if (ev.getKeySym() == "s" && ev.keyDown())
	  {
	   std::cout << "Manually saving pcd" << std::endl; 
	   this->savePCD(path, camera_id, *(buff->Cloud()));
    	  }
      });


    //
    // Setup for amplitude, depth, and confidence images
    //

    // used for all 2d images
    cv::namedWindow(this->description_, cv::WINDOW_NORMAL|cv::WINDOW_OPENGL);
    cv::resizeWindow(this->description_, win_w, win_h);
    int retval;
    double min, max;
    cv::Rect roi;

    // depth
    cv::Mat display_img;
    cv::Mat depth_colormap_img;

    // confidence
    cv::Mat conf_img;
    cv::Mat conf_colormap_img;

    // amplitude
    cv::Mat amp_colormap_img;
    cv::Mat raw_amp_colormap_img;


    bool is_first = true;

    while (! pclvis_->wasStopped())
      {
	
	
        pclvis_->spinOnce(100);
        if (! fg->WaitForFrame(buff.get(), 500))
          {
            continue;
          }
	
	
	
	// aktuelle matrix in variable speichern

	CurrMat = buff->DepthImage();


        //------------
        // Point cloud
        //------------
        pcl::visualization::PointCloudColorHandlerGenericField<o3d3xx::PointT>
          color_handler(buff->Cloud(), "intensity");

        if (is_first)
          {
            is_first = false;
            pclvis_->addPointCloud(buff->Cloud(), color_handler, "cloud");
	    CurrMat.copyTo(refMat);
	    CurrMat.copyTo(last1Mat);

	    // Initial pcd
	    if(auto_snapshot){
		    this->savePCD(path, camera_id, *(buff->Cloud()));

		std::time_t ts = std::time(0);
		std::stringstream ss;
		ss << "raspistill -o /home/pi/Desktop/pactrisPro/pcd/" <<camera_id <<"/" << ts << ".jpg";
		
		system(ss.str());
				
	    }

          }
	
        else
          {

	// methode aufrufen
	//MeanMat = (CurrMat + last1Mat + last2Mat)/ 3.0f;
	SubMat = CustomDiffMat(CurrMat, last1Mat);
	//SubMat = MeanMat - CurrMat;  //  |MeanMatrix - FollowingMatrix| = SubtractMatrix
	
	
	SubRefMat = CustomDiffMat(refMat,CurrMat); // |RefMatrix - MeanMatrix| = SubtractMatrix

        pclvis_->updatePointCloud(buff->Cloud(), color_handler, "cloud");


	// aktuelle matrix in global var speichern
	
	//last1Mat.copyTo(last2Mat);
	CurrMat.copyTo(last1Mat);

	std::cout<< "Check current and mean matrix" << std::endl;;
	change = ChangeDetection(SubMat);
	
	std::cout<< "Check current and ref matrix" << std::endl;;
	tempchange = ChangeDetection(SubRefMat);

	
	if(change)
	{	
		countloop = 0;
		std::cout<< "-------------------------------" << std::endl << std::endl;;
		std::cout<< "Something is changing !!!" << std::endl << std::endl;
		std::cout<< "-------------------------------" << std::endl;
		
	}
	else
	{	
		countloop++;
		std::cout<< "-------------------------------" << std::endl << std::endl;;
		std::cout<< "NO change -> " << countloop << std::endl << std::endl;
		std::cout<< "-------------------------------" << std::endl;
				
				/*
				if (countloop%10==0){
				take image from pycam
						Py_SetProgramName("snapshot.py");
						Py_Initialize();
						FILE* file = fopen("/home/pi/snapshot.py", "r");
						PyRun_SimpleFile(file, "/home/pi/snapshot.py");
						Py_Finalize();
				system("raspistill -o /home/pi/snapshots/test1.jpg");
				}
				*/
	}

	if(countloop == noChangeCount && tempchange == true) 	// if there is 3 times no changes in front of 3d cam -> snapshot
	{
		std::cout<< "*******************************" << std::endl << std::endl;;
		std::cout<< "Snapshot" << std::endl << std::endl;
		std::cout<< "*******************************" << std::endl;
		CurrMat.copyTo(refMat);
		if (auto_snapshot){
			this->savePCD(path, camera_id, *(buff->Cloud()));
		}
		
	}
	}       

        //------------
        // 2D images
        //------------

        // depth image
        cv::minMaxIdx(buff->DepthImage(), &min, &max);
        cv::convertScaleAbs(buff->DepthImage(),
                            depth_colormap_img, 255 / max);
        cv::applyColorMap(depth_colormap_img, depth_colormap_img,
                          cv::COLORMAP_JET);

        // confidence image: show as binary image of good pixel vs. bad pixel.
        conf_img = buff->ConfidenceImage();
        conf_colormap_img = cv::Mat::ones(conf_img.rows,
                                          conf_img.cols,
                                          CV_8UC1);
        cv::bitwise_and(conf_img, conf_colormap_img,
                        conf_colormap_img);
        cv::convertScaleAbs(conf_colormap_img,
                            conf_colormap_img, 255);
        cv::applyColorMap(conf_colormap_img, conf_colormap_img,
                          cv::COLORMAP_SUMMER);

        // amplitude
        cv::minMaxIdx(buff->AmplitudeImage(), &min, &max);
        cv::convertScaleAbs(buff->AmplitudeImage(),
                            amp_colormap_img, 255 / max);
        cv::applyColorMap(amp_colormap_img, amp_colormap_img,
                          cv::COLORMAP_BONE);

        cv::minMaxIdx(buff->RawAmplitudeImage(), &min, &max);
        cv::convertScaleAbs(buff->RawAmplitudeImage(),
                            raw_amp_colormap_img, 255 / max);
        cv::applyColorMap(raw_amp_colormap_img, raw_amp_colormap_img,
                          cv::COLORMAP_BONE);

        // stich 2d images together and display
        display_img.create(depth_colormap_img.rows*2,
                           depth_colormap_img.cols*2,
                           depth_colormap_img.type());

        roi = cv::Rect(0, 0,
                       depth_colormap_img.cols,
                       depth_colormap_img.rows);
        depth_colormap_img.copyTo(display_img(roi));

        roi = cv::Rect(depth_colormap_img.cols, 0,
                       conf_colormap_img.cols,
                       conf_colormap_img.rows);
        conf_colormap_img.copyTo(display_img(roi));

        roi = cv::Rect(0, depth_colormap_img.rows,
                       amp_colormap_img.cols,
                       amp_colormap_img.rows);
        amp_colormap_img.copyTo(display_img(roi));

        roi = cv::Rect(depth_colormap_img.cols,
                       depth_colormap_img.rows,
                       raw_amp_colormap_img.cols,
                       raw_amp_colormap_img.rows);
        raw_amp_colormap_img.copyTo(display_img(roi));

        cv::imshow(this->description_, display_img);


/*-----Mein Code
		
	XYZMat = buff->XYZImage();

	cv::FileStorage fs2("xyz_file.yml", cv::FileStorage::WRITE);
	fs2 << "yourMatxyz" << XYZMat;

	SaveMatToCsv(XYZMat, XyzMa);

	SaveMatToCsv(SubMat, SuMa);
	
	cv::FileStorage fsdep("Depth_file.yml", cv::FileStorage::WRITE);
	fsdep << "yourMatDe" << buff->DepthImage();

//-----Ende*/


        // `ESC', `q', or `Q' to exit
        retval = cv::waitKey(33);
        if ((retval == 27) || (retval == 113) || (retval == 81))
          {
            break;
          }
      } // end: while (...)
  }





private:
  o3d3xx::Camera::Ptr cam_;
  std::string description_;

    cv::Mat computeMeanMat(cv::Mat mat0, cv::Mat mat1, cv::Mat mat2){

	cv::Mat sumMat = mat0+mat1+mat2;
	return sumMat/3.0f;
    }
	double allpix = 0;
    
int MyCompare(cv::Mat img1, cv::Mat img2)
    {
	int counteq = 0;
	allpix = 0;
	for(int y=0; y != img1.cols; y++) //cols
	  {

		for(int x=0; x != img1.rows; x++) //rows
		  {
			if((img1.at<ushort>(x,y)>0)&&(img2.at<ushort>(x,y)>0)) 
				if(img1.at<ushort>(x,y)==img2.at<ushort>(x,y))
			  	{	
					counteq++;
					allpix++;
			  	}
	  	  }			
	  }			
	std::cout<<"Allpix: "<<allpix<<std::endl;
	return counteq;
    }

cv::Mat CustomDiffMat(cv::Mat CurrMat, cv::Mat last1Mat){
	cv::Mat diffMat = CurrMat.clone();
	int count = 0;
	for(int y=0; y<CurrMat.cols; y++){
		for (int x=0; x<CurrMat.rows; x++){
			bool zeroValue = false;

			if(CurrMat.at<ushort>(x,y)==0){
				zeroValue=true;
			}
			if(last1Mat.at<ushort>(x,y)==0){
				zeroValue=true;
			}
			
			if(!zeroValue){
				if(CurrMat.at<ushort>(x,y) >= last1Mat.at<ushort>(x,y)){
					diffMat.at<ushort>(x,y)= CurrMat.at<ushort>(x,y)-last1Mat.at<ushort>(x,y);
				} else {
					diffMat.at<ushort>(x,y)= last1Mat.at<ushort>(x,y)-CurrMat.at<ushort>(x,y);
				}
				

			} else {
				count++;
				diffMat.at<ushort>(x,y) = 999;
			}
		}
	}
//std::cout << "dif mat done, " << count << " zero values" << std::endl;
	return diffMat;
   }

bool ChangeDetection(cv::Mat img1)
    {

	double counteq = 0;
	double allpix = 0;		//Number of all pixels

	for(int y=0; y != img1.cols; y++)
	  {

		for(int x=0; x != img1.rows; x++) //rows
		  {

			if(img1.at<ushort>(x,y)!=999){
				allpix++;
				if(distance >= img1.at<ushort>(x,y)){
					counteq++;
				}			
			}
	  	  }	
	  }
	
	std::cout<< "Change percentage: " << (counteq/allpix) <<std::endl;
	if((counteq/allpix) > threshold)

	{	
	//	std::cout << "False_Test: " << counteq/allpix * 100 << "%" << std::endl;
		return false;
	}
	else		
	{
	//	std::cout << "True_Test: " << counteq/allpix * 100 << "%" << std::endl;
		return true;
	}	
    }


	void SaveMatToCsv(cv::Mat matrix, std::string filename){
		ofstream outputFile(filename);
		outputFile << format(matrix,"CSV") << std::endl;
		outputFile.close();
	}

	void savePCD(std::string path, int camera_id, pcl::PointCloud<o3d3xx::PointT>& cloud ){
		std::cout << "Saving file to point_cloud.pcd" << std::endl;
		std::time_t ts = std::time(0);
		std::stringstream ss;
		ss << "/home/pi/Desktop/pactrisPro/pcd/" <<camera_id <<"/" << ts << ".pcd";
		pcl::io::savePCDFileASCII(ss.str(), cloud);
	}
}; // end: class O3DViewer

//-------------------------------------------------------------
// Go...
//-------------------------------------------------------------

int main(int argc, const char **argv)
{
  int retval = 0;

  std::string camera_ip;
  uint32_t xmlrpc_port;
  std::string password;
  std::string descr("o3d3xx Viewer");

 // typedef Vec<short, 3> Vec3s;

  try
    {
      //---------------------------------------------------
      // Handle command-line arguments
      //---------------------------------------------------
      o3d3xx::CmdLineOpts opts(descr);
      if (! opts.Parse(argc, argv, &camera_ip, &xmlrpc_port, &password))
        {
          return 0;
        }

camera_id = camera_ip.back() - '0';
std::cout << "CAMERA ID: " << camera_id << std::endl;



// Get Threshold

std::stringstream ss;
ss << "CAM" <<camera_id <<"_TS";
std::string s= ss.str();
char const* tmp = std::getenv(s.c_str());

if ( tmp == NULL ) {
    std::cout << ss.str() << " not given" << std::endl;
} else {
    std::string ts_string( tmp );
    threshold = std::stof(ts_string);
}
std::cout << "Setting threshold for camera " << camera_id << " to " << threshold << std::endl;


// Get distance value
std::stringstream ss1;
ss1 << "CAM" <<camera_id <<"_DIST";
std::string s1= ss1.str();
char const* tmp1 = std::getenv(s1.c_str());

if ( tmp1 == NULL ) {
    std::cout << ss1.str() << " not given" << std::endl;
} else {
    std::string dist_string( tmp1 );
    distance = std::stof(dist_string);
}
std::cout << "Setting distance for camera " << camera_id << " to " << distance << std::endl;

// Get no change count
char const* tmp2 = std::getenv("NCC");

if ( tmp2 == NULL ) {
    std::cout  << "NoChangeCount (NCC) not given" << std::endl;
} else {
    std::string ncc_string( tmp2 );
    noChangeCount = std::stoi(ncc_string);
}
std::cout << "Setting noChangeCount for camera " << camera_id << " to " << noChangeCount << std::endl;


// Get auto snapshot vlaue
char const* tmp3 = std::getenv("AS");

if ( tmp3 == NULL ) {
    std::cout  << "Auto Snapshot (AS) not given" << std::endl;
} else {
    std::string as_string( tmp3 );
    auto_snapshot = std::stoi(as_string);
}
std::cout << "Setting auto snapshot for camera " << camera_id << " to " << auto_snapshot << std::endl;


      //---------------------------------------------------
      // Initialize the camera
      //---------------------------------------------------
      o3d3xx::Camera::Ptr cam =
        std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port, password);

      //---------------------------------------------------
      // Run the viewer
      //---------------------------------------------------
      O3DViewer viewer(cam, descr);
      viewer.Run();
    }
  catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      return 1;
    }

  return retval;
}
