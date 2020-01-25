//#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>
 
typedef pcl::PointCloud<pcl::PointXYZRGB> PCLCloud;
pcl::PCLPointCloud2::Ptr pcl_pc2_temp(new pcl::PCLPointCloud2);
PCLCloud::Ptr temp_cloud(new PCLCloud);

std::ofstream myfile;

std::string pc2_topic_in = "/rplidar/cloud_color";
std::string save_as = "/home/arc/clouds/test/cloud.txt";

uint n_point = 0;   // number of points written

void writeXYZRGB() {
    // Writes xyzrgb data to a text file line by line
    std::string x, y, z, r, g, b;
    auto pt = temp_cloud->points.begin();
    while( pt != temp_cloud->points.end() ) {
        x = std::to_string(pt->x) + " ";
        y = std::to_string(pt->y) + " ";
        z = std::to_string(pt->z) + " ";
        r = std::to_string(pt->r) + " ";
        g = std::to_string(pt->g) + " ";
        b = std::to_string(pt->b) + " ";
        
        if(x != "nan ") {    // Skip any nan values
            myfile.open( save_as , std::ios_base::app);
            myfile << (x + y + z + r + g + b + "\n");
            myfile.close();
            n_point += 1;
        }
        ++pt;
    }
}

void pc2Callback( const sensor_msgs::PointCloud2::ConstPtr& msg ) {
    // Conver the ros msg into a usable form
    pcl_conversions::toPCL(*msg, *pcl_pc2_temp);
    pcl::fromPCLPointCloud2(*pcl_pc2_temp, *temp_cloud);
    writeXYZRGB();
}


int main( int argc, char **argv ) {
    ros::init( argc, argv, "pc2_record" );
    
    ROS_INFO( "pc2_record: Starting Node" );
    ros::NodeHandle nh("~");
    ros::Subscriber sub;
    
    ROS_INFO( "pc2_record: Obtaining Params" );
    nh.getParam("pc2_topic_in", pc2_topic_in );
    nh.getParam("save_as", save_as );
    
    ROS_INFO( "pc2_record: Configuring Node" );
    sub = nh.subscribe( pc2_topic_in, 10, pc2Callback );
    
    ROS_INFO( "pc2_record: Starting Spin" );
    ros::spin();
    
    return 0;
}
