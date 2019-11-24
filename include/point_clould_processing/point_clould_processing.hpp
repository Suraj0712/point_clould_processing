// Declaration file
#pragma once
#ifndef POINT_CLOUD_PROCESSING
#define POINT_CLOUD_PROCESSING

#include <ros/ros.h>                // including the ROS header
#include <pcl_ros/point_cloud.h>    // including pcl header
#include <pcl/conversions.h>        // pcl header for conversion
#include <ros/package.h>            // header to get package path for ros library 
#include <string>                   
#include <fstream>                  // file handling related header
#include <pcl/io/pcd_io.h>          // pcl file handling
#include <chrono>                   // Time related library header                  
#include <thread> 


class PointCloudProcessing
{
    public:
        /* Constructor 
            This function will get called Automatically
            We declare subscriber, publisher, environment variables, params 
            and variable initialisation inside the constructor  */
        PointCloudProcessing(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        /* Distructor  
            will get called at the end of program, 
            if you have declared anyhting in the heap memory its recommended 
            to delete them in distructor in order to aviod memory leak  */
        ~PointCloudProcessing();
        /* runOnce function
            Generally this is the only public function in your class apart from constructor and distructor
            so based on the callbacks and availability of required data this function will 
            direct the flow of program  */
        void runOnce();
    private:
        ros::NodeHandle nh_;        //Public node handle
        ros::Subscriber sub_cloud_; //Subscriber for the Cloud data
        ros::Publisher pub_cloud_;  //Publisher for the cloud data

        /* Vectors and point cloud to store the point cloud into the different kinds of data structures */
        std::vector<sensor_msgs::PointCloud2> rs_pt_cloud_vec_;
        std::vector<pcl::PointCloud<pcl::PointXYZ> > pcl_pt_cloud_vec_;
        std::vector<pcl::PCLPointCloud2> pcl_pt_cloud2_vec_;
        sensor_msgs::PointCloud2 rs_pt_cloud_;
        pcl::PointCloud<pcl::PointXYZ> pcl_pt_cloud_;
        pcl::PCLPointCloud2 pcl_pt_cloud2_;

        bool has_point_cloud_; //bool variable to decide the whether or not we have enought data

        // File handling related variables
        std::string pcd_file_path_;
        std::string csv_file_path_;
        std::string pkg_path_;
        std::ofstream myfile_pcd_;
        std::ofstream myfile_csv_;

        int no_of_clouds_;      // variables to store the number of pointclouds based on user input
        int point_cloud_count_; // count the point cloud and rpint the same on the window */

        /* Callback function to handle the messeges received from the topic
            Try to aviod data processing in the callback fuction, the general practise 
            is to store the data from the callback function into some member variable for 
            further processing  */
        void rsCloudCallback(const sensor_msgs::PointCloud2 & msg);
    
        /* this function will access the point cloud from the vector and then
            convert the same to the pcl pointcloud2 and pcl pointcloud and store them in vector*/
        void convertPointCloudtoPCLpc2andPCLpc();

        /* This is the function which we will use for the storing the 
            point clouds in the .pcd and .csv files */
        void savePointCloudinPCDAndCSVfiles();

        /* This function will publish the pointcloud on the topic wait function is 
            implemented by using the chronos and thread, you might ger ERROR while visualisation
            the same in the rviz. just update the frame and then i think you are ggo to go */
        void publishPointCloud();       
};
#endif  