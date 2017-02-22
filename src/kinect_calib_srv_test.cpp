#include <ros/ros.h>

#include<kinect_calib_srv/srv_calib_test.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "kinect_calib_test");
    ros::NodeHandle nh;

    ros::ServiceClient test_client = nh.serviceClient<kinect_calib_srv::srv_calib_test>("/calib/test_3d_point");

    kinect_calib_srv::srv_calib_test srv;

    if(test_client.call(srv)) {
        ROS_INFO("Service called successfully...");

        ROS_INFO("Point := (%f, %f, %f)", srv.response.Point_3D.data[0], srv.response.Point_3D.data[1], srv.response.Point_3D.data[2]);
        ROS_INFO("Normal := (%f, %f, %f)", srv.response.Point_3D.data[3], srv.response.Point_3D.data[4], srv.response.Point_3D.data[5]);
        ROS_INFO("Major Axis := (%f, %f, %f)", srv.response.Point_3D.data[6], srv.response.Point_3D.data[7], srv.response.Point_3D.data[8]);
    }

    return 0;

}
