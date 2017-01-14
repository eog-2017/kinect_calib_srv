#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>


#include<signal.h>

#include<kinect_calib_srv/Kinect3D.h>
#include<kinect_calib_srv/srv_calib_test.h>

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define thresh 10
#define kernel_size 3
#define ratio 3
#define lowThreshold 100

//%%%%%%%%%%%%%%%%% erosion & dilation element %%%%%%%%%%%%%%%%%
#define erosion_type 0
#define erosion_size 1

cv_bridge::CvImageConstPtr msg_color;
cv::Mat img_hsv, img_dilated, img_erosion1, img_erosion2,img_live;
cv::Mat red_segmented_lower, red_segmented_upper, red_segmented, green_segmented, blue_segmented;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::visualization::CloudViewer cloud_viewer("cloud");


typedef struct _point_3d_
{
    float real[3];
    int pixel_x;
    int pixel_y;

    bool operator< (const _point_3d_& op) const
    {
        return (pixel_x < op.pixel_x);
    }
}point_3d;


std::vector<point_3d> final_coordinates;

bool points_processed = false;
bool service_called = false;
bool test_service_called = false;
bool mouse_event_processed = false;

void signal_function(int sig)
{
    std::cout << "CAUGHT SIGNAL.....EXITING\n";
    exit(0);
}


void points(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //  std::cout << "in cloud out\n";

    pcl::fromROSMsg( *msg, *ptr_cloud);

    if(!img_live.rows || !img_live.cols)
        img_live.create(ptr_cloud->height,ptr_cloud->width,CV_8UC3);

    for(int i=0;i<img_live.rows;i++)
        for(int j=0;j<img_live.cols;j++)
        {
            unsigned char* pixel = img_live.data + img_live.step[0]*i +img_live.step[1]*j;
            pixel[0] = ptr_cloud->at(j,i).b;
            pixel[1] = ptr_cloud->at(j,i).g;
            pixel[2] = ptr_cloud->at(j,i).r;

        }

    // if(test_service_called)
    cv::imshow("TEST POINT IMAGE", img_live);
    cv::waitKey(1);

    if(service_called)
    {
        //    std::cout << "in cloud in\n";
        // PointCloud to rgb image

        cloud_viewer.showCloud(ptr_cloud);


        cv::Mat img_color(ptr_cloud->height,ptr_cloud->width,CV_8UC3);

        for(int i=0;i<img_color.rows;i++)
            for(int j=0;j<img_color.cols;j++)
            {
                unsigned char* pixel = img_color.data + img_color.step[0]*i +img_color.step[1]*j;
                pixel[0] = ptr_cloud->at(j,i).b;
                pixel[1] = ptr_cloud->at(j,i).g;
                pixel[2] = ptr_cloud->at(j,i).r;

            }

        // cv::imshow("FROM POINT CLOUD",img_color);
        //   }

        //   if(1){ // Red color segmentation

        cv::cvtColor(img_color, img_hsv, CV_BGR2HSV);

        // segmenting red upper
        cv::inRange(img_hsv, cv::Scalar(160, 100, 50), cv::Scalar(179, 255, 255), red_segmented_upper);

        // segmenting red upper
        cv::inRange(img_hsv, cv::Scalar(0, 100, 50), cv::Scalar(10, 255, 255), red_segmented_lower);

        cv::bitwise_or(red_segmented_lower, red_segmented_upper, red_segmented);

        //cv::imshow("color", img_color);
        //cv::imshow("red_segmented", red_segmented);
        //    }

        //    if(1){ // Erosion, Dilation, Erosion

        cv::Mat element = cv::getStructuringElement( erosion_type, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                     cv::Point( erosion_size, erosion_size ) );

        /// Apply the erosion operation
        cv::erode( red_segmented, img_erosion1, element, cv::Point(-1,-1), 4 );
        //imshow( "Erosion 1", img_erosion );

        /// Apply the dilation operation
        cv::dilate( img_erosion1, img_dilated, element, cv::Point(-1,-1), 7 );
        //cv::imshow("dilated", img_dilated);

        cv::erode( img_dilated, img_erosion2, element, cv::Point(-1,-1), 2 );
        //cv::imshow("Erosion 2", img_erosion2);

        //    }

        //    if(1){ // Blob detection & co-ordinate generation

        cv::SimpleBlobDetector::Params params;

        params.filterByColor = true;
        params.blobColor = 255;

        params.filterByArea = false;

        cv::Ptr<cv::SimpleBlobDetector> blob_detector = cv::SimpleBlobDetector::create(params);

        std::vector<cv::KeyPoint> blobs;

        blob_detector->detect( img_erosion2, blobs);

        cv::Mat im_with_blobs;

        img_color.copyTo(im_with_blobs);

        for(int i=0;i<blobs.size();i++){
            cv::circle(im_with_blobs, blobs[i].pt,5, cv::Scalar(0,255,0),-1);
        }

        // Show blobs
        cv::imshow("keypoints", im_with_blobs );
        //  }

        //  if(1){ // vector push from left most


        std::cout<<"*************** UNSORTED ******************"<<std::endl;

        for(int i=0;i<blobs.size();i++){

            point_3d mc;

            mc.pixel_x = int(((cv::KeyPoint)blobs[i]).pt.x);
            mc.pixel_y = int(((cv::KeyPoint)blobs[i]).pt.y);


            mc.real[0] = float(ptr_cloud->at(mc.pixel_x, mc.pixel_y).x);
            mc.real[1] = float(ptr_cloud->at(mc.pixel_x, mc.pixel_y).y);
            mc.real[2] = float(ptr_cloud->at(mc.pixel_x, mc.pixel_y).z);

            final_coordinates.push_back(mc);

            std::cout<< "x= "<< final_coordinates.at(i).pixel_x << " y= " << final_coordinates.at(i).pixel_y
                     << " z " << final_coordinates.at(i).real[2] << std::endl;
        }

        std::sort(final_coordinates.begin(), final_coordinates.end());

        std::cout<<"############### SORTED ######################"<<std::endl;

        for(int i=0;i<final_coordinates.size();i++){

            std::cout<< "x= "<< final_coordinates.at(i).pixel_x << " y= " << final_coordinates.at(i).pixel_y
                     << " z " << final_coordinates.at(i).real[2] << std::endl;
        }

        //  }
        cv::waitKey(1);

        points_processed =true;
    }
    signal(SIGINT,signal_function);

}


bool get3Dpoint_callback(kinect_calib_srv::Kinect3D::Request &req, kinect_calib_srv::Kinect3D::Response &res)
{
    std::cout << "Entered kinect 3D service\n";

    final_coordinates.clear();

    service_called = true;

    while(!points_processed)
        usleep(1000);

    service_called = false;
    points_processed = false;


    std::cout << "SERVICE CALLED AND POINTS PROCESSED\n";

    std::cout << "FILLING RESPONSE\n";

    for(int i=0;i<final_coordinates.size();i++)
    {
        for(int j=0;j<3;j++)
            res.point_3D.data.push_back(((point_3d)final_coordinates[i]).real[j]);

    }

    if(final_coordinates.size())
        return true;
    else
        return false;

    //write your code and push results in res

}



static void mouseAction( int event, int x, int y, int flags, void* data )
{

    if(test_service_called)
    {


        if( x < 0 || x >= img_live.cols || y < 0 || y >= img_live.rows )
        {

            std::cout << "PLEASE CLICK INSIDE THE IMAGE\n";
        }
        else
        {
            point_3d* mouse_click = (point_3d*)data;

            if  ( event == cv::EVENT_LBUTTONDOWN )
            {
                mouse_click->real[0] = float(ptr_cloud->at(x, y).x);
                mouse_click->real[1] = float(ptr_cloud->at(x, y).y);
                mouse_click->real[2] = float(ptr_cloud->at(x, y).z);

                if( !(isnan(mouse_click->real[2])) )
                {
                    std::cout<< "x= " << mouse_click->real[0] <<
                                ", y= " << mouse_click->real[1] <<
                                ", z= " << mouse_click->real[2] << std::endl;

                    mouse_event_processed=true;
                    test_service_called=false;
                }
                else
                    std::cout << "CLICKED ON NAN POINT.. TRY AGAIN\n";
            }
        }
    }

}

bool callback_calib_test(kinect_calib_srv::srv_calib_test::Request &req, kinect_calib_srv::srv_calib_test::Response &res)
{
    std::cout << "IN TEST POINT SERVICE\n";

    test_service_called = true;

    point_3d point;
    cv::setMouseCallback("TEST POINT IMAGE", mouseAction, &point );


    while(!mouse_event_processed)
        usleep(10000);

    mouse_event_processed= false;
    test_service_called = false;

    for(int i=0;i<3;i++)
        res.Point_3D.data.push_back(point.real[i]);


    std::cout << "EXITING TEST POINT SERVICE\n";

    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Komal");

    ros::NodeHandle nh;

    ros::ServiceServer kinectserver= nh.advertiseService("/kinect2/get3Dpoint", get3Dpoint_callback);
    ros::ServiceServer caliberation_test_server = nh.advertiseService("/calib/test_3d_point", callback_calib_test);
    ros::Subscriber point_sub = nh.subscribe("/kinect2/sd/points", 5, points);

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    //ros::spin();,

    return 0;
}

