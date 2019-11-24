// Node file to create object and initialising the ROS node
#include "point_clould_processing.hpp" 

int main(int argc, char** argv)
{
    /* initialising the ROS node creating node handle handle 
        for regestring it to the master and then private node handle to
        handle the parameters */
    ros::init(argc, argv, "point_clould_processing"); 
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); 

    PointCloudProcessing node(nh,pnh);  // Creating the object

    ros::Rate rate(1.0); // Defining the looping rate

    /* Looking for any interupt else it will continue looping */
    while (ros::ok())
    {   
        node.runOnce();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}