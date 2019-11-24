The purpose of this ROS package is to give some idea about the ROS publisher, subscribers, params, io handling, PCL library.

for creating this package I have used the structural package that I have already created here
git@github.com:Suraj0712/your_pkg_name.git

Problem statement:
1. So we have to write the subscriber to subscribe to the point cloud topic
2. Store the point clouds in the member variables
3. Convert the point clouds in the PCL point clouds
4. Store the point cloud locally in .csv and .pcd files
5. Publish the point cloud on some topic and visualize the same in the Rviz

I have tried my best to explain the code in its respective files.

if you want to use this package the kidly follow my medium article for the step by step guide to set up the project in the ROS

After a successful build, you can run the code using the following commands

roslaunch point_clould_processing point_clould_processing.launch no_of_clouds:=<# pointclouds you want to process>
