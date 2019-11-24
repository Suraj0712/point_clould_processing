#include "point_clould_processing.hpp"

PointCloudProcessing::PointCloudProcessing(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    std::cout<<"from Constructor \n";

    // storing the values in the member variable
    pnh.getParam("no_of_clouds", no_of_clouds_);
    std::cout<<"Number of point cloud to be processed: "<<no_of_clouds_<<"\n";

    std::cout << typeid(no_of_clouds_).name() << std::endl;

    // Setting up the parameters
    nh_ = nh;
    has_point_cloud_ = false;
    pkg_path_ = ros::package::getPath("point_clould_processing");

    // Defining the publisher and subscribers
    sub_cloud_ = nh_.subscribe("/camera/depth_registered/points", 1, &PointCloudProcessing::rsCloudCallback, this);
    pub_cloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("/point_clould_processing/point_cloud_", 1);  

}

PointCloudProcessing::~PointCloudProcessing()
{
    std::cout<<"from Distructor \n";
}

void PointCloudProcessing::runOnce()
{
    std::cout<<"From Runonce \n";

    // if else loop to check weather we have required number of point clouds to start post processing
    if(has_point_cloud_)
    {
        std::cout<<"Calling the function to publish the point cloud \n";

        // Function call for point cloud processing
        convertPointCloudtoPCLpc2andPCLpc();
        savePointCloudinPCDAndCSVfiles();
        publishPointCloud();
    }
    else
    {
        std::cout<<"Number of point clouds received: "<<point_cloud_count_<<" \n";
    }   
}

void PointCloudProcessing::rsCloudCallback(const sensor_msgs::PointCloud2 & msg)
{
    /* Storing the point clouds in the vector and then once we have required 
    number of point clouds it will change the value in flag variable*/
    if(rs_pt_cloud_vec_.size()<no_of_clouds_)
    {
        rs_pt_cloud_vec_.push_back(msg);
        point_cloud_count_++;
    }
    else
    {
        has_point_cloud_ = true;
    }
}

void PointCloudProcessing::convertPointCloudtoPCLpc2andPCLpc()
{
    // Iterating over the vector to conver and store the point clouds in different formats
    for(int i=0; i<rs_pt_cloud_vec_.size(); i++)
    {
        rs_pt_cloud_ = rs_pt_cloud_vec_[i];
        pcl_conversions::toPCL(rs_pt_cloud_,pcl_pt_cloud2_);
        pcl::fromPCLPointCloud2(pcl_pt_cloud2_, pcl_pt_cloud_);
        
        pcl_pt_cloud_vec_.push_back(pcl_pt_cloud_);
        pcl_pt_cloud2_vec_.push_back(pcl_pt_cloud2_);
    }
}
void PointCloudProcessing::savePointCloudinPCDAndCSVfiles()
{
    // Iterating over point cloud and saving the files in .pcd and .csv files
    for(int i=0; i<pcl_pt_cloud_vec_.size(); i++)
    {
        std::cout<<"Saving the point cloud in the harddrive \n";

        // .pcd file creation
        pcd_file_path_ = pkg_path_ +"/data/"+"PCD_file_for_cloud_"+std::to_string(i)+".pcd";  
        myfile_pcd_.open(pcd_file_path_, std::ios::out | std::ios::app);
        if(!myfile_pcd_)
        {
            std::cout<<".pcd file crearion failed for point cloud \n";
        }
        // Using pcl io for creating the .pcd file
        pcl::io::savePCDFileASCII(pcd_file_path_, pcl_pt_cloud_vec_[i]); 
        myfile_pcd_.close();

        // .csv file creating
        csv_file_path_ = pkg_path_ +"/data/"+"CSV_file_for_cloud_"+std::to_string(i)+".csv";  
        myfile_csv_.open(csv_file_path_, std::ios::out | std::ios::app);
        if(!myfile_csv_)
        {
            std::cout<<".csv file crearion failed for point cloud no \n";
        }
        else
        {
            for (int j=0; j< pcl_pt_cloud_vec_[i].points.size(); j++)
            {   
                myfile_csv_<<pcl_pt_cloud_vec_[i].points[j].x << ","<<pcl_pt_cloud_vec_[i].points[j].y << ","<<pcl_pt_cloud_vec_[i].points[j].z << " \n"; 
            }
        }           
        myfile_csv_.close(); 
    }
    
}
void PointCloudProcessing::publishPointCloud()
{
    std::cout<<"Publishing the point cloud \n";
    while(true)
    {
        for(int i=0; i<pcl_pt_cloud_vec_.size(); i++)
        {
            // invoking the publisher to publish the point cloud
            pub_cloud_.publish(pcl_pt_cloud_vec_[i]);
            // following code introduces the time delay for the publisher
            std::chrono::seconds dura(5);
            std::this_thread::sleep_for(dura);
            std::cout<<"-";
        }
    } 
}