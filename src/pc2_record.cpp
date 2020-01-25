//#include <cstdint>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> //concatenate
 
typedef pcl::PointCloud<pcl::PointXYZRGB> PCLCloud;

pcl::PCLPointCloud2::Ptr pcl_pc2_full(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2::Ptr pcl_pc2_temp(new pcl::PCLPointCloud2);
PCLCloud::Ptr temp_cloud(new PCLCloud);
PCLCloud::Ptr full_cloud(new PCLCloud);
PCLCloud::Ptr cloud_xyzrgb(new PCLCloud);

std::ofstream myfile;

std::string pc2_topic_in = "/rplidar/cloud_color";
std::string save_as = "/home/arc/clouds/test/cloud.txt";

void unpackRGB(){
    cloud_xyzrgb->points.resize(temp_cloud->size());
    cloud_xyzrgb->width = temp_cloud->size();
    cloud_xyzrgb->height = 1;
    
    for( size_t i = 0; i < temp_cloud->size(); i++ ) {
        auto &a = cloud_xyzrgb->points[i];
        auto &b = temp_cloud->points[i];
        a.x = b.x;
        a.y = b.y;
        a.z = b.z;
        a.r = b.r;
        a.g = b.g;
        a.b = b.b;


/*
    auto a = cloud_xyzrgb->points.begin();
    auto b = temp_cloud->points.begin();
    while ( a != cloud_xyzrgb->points.end() ) {
        a->x = b->x;
        a->y = b->y;
        a->z = b->z;
        a->r = b->r;
        a->g = b->g;
        a->b = b->b;
        ++a;
        ++b;
*/
    }
}

void writeXYZRGB(){
    
    std::string x, y, z, r, g, b;
    auto pt = temp_cloud->points.begin();
    while( pt != temp_cloud->points.end() ) {
        x = std::to_string(pt->x) + " ";
        y = std::to_string(pt->y) + " ";
        z = std::to_string(pt->z) + " ";
        r = std::to_string(pt->r) + " ";
        g = std::to_string(pt->g) + " ";
        b = std::to_string(pt->b) + " ";
        
        myfile.open( save_as , std::ios_base::app);
        myfile << (x + y + z + r + g + b + "\n");
        myfile.close();
        ++pt;
    }
    ROS_INFO( "pc2_record: ending write" );
}

void pc2Callback( const sensor_msgs::PointCloud2::ConstPtr& msg ) {
    ROS_INFO( "pc2_record: Cloud callabck" );
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
