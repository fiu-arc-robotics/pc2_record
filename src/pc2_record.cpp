#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Bool.h>
#include <ctime>
#include <string>

 
typedef pcl::PointCloud<pcl::PointXYZRGB> PCLCloud;
pcl::PCLPointCloud2::Ptr pcl_pc2_temp(new pcl::PCLPointCloud2);
PCLCloud::Ptr temp_cloud(new PCLCloud);

std::ofstream myfile;

std::string pc2_topic_in =  "/cloud_color";
std::string filedir = "/home/lattice/"; //  "~/Desktop/";
std::string filename = "cloud";
std::string fileext = ".txt";


uint n_point = 0;   // number of points written
bool is_PauseRecording = false;
bool current_recording_state = true;

std::string makeFilepath() {
    std::string filepath = filedir + filename + fileext;
    return(filepath);
}

std::string makeTime() {
    // current date/time based on current system
   time_t now = time(0);
   tm *ltm = localtime(&now);

   int times[6];
   
   times[0] = 1900 + ltm->tm_year;
   times[1] = 1 + ltm->tm_mon;
   times[2] = ltm->tm_mday;
   times[3] = 1 + ltm->tm_hour;
   times[4] = 1 + ltm->tm_min;
   times[5] = 1 + ltm->tm_sec;
   
   std::string datetime;
   std::string time;
   for(int i = 0; i < 6; i++) {
	   if(i == 0) {
		   datetime = std::to_string(times[i]);
	   } 
	   else {
		   if(times[i] < 10) {
			   time = "0" + std::to_string(times[i]);
		   }
		   else {
			   time = std::to_string(times[i]);
		   }
		   datetime = datetime + "-" + time;
	   }
   }
   return(datetime);
}

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
            myfile.open(makeFilepath(), std::ios_base::app);
            myfile << (x + y + z + r + g + b + "\n");
            myfile.close();
            n_point += 1;
        }
        ++pt;
    }
    ROS_INFO( "pc2_record: Cloud Recorded" );
}

void pc2Callback( const sensor_msgs::PointCloud2::ConstPtr& msg ) {
    if(is_PauseRecording != true) {
        // Conver the ros msg into a usable form
        pcl_conversions::toPCL(*msg, *pcl_pc2_temp);
        pcl::fromPCLPointCloud2(*pcl_pc2_temp, *temp_cloud);
        writeXYZRGB();
    }
}

void boolCallback( const std_msgs::Bool::ConstPtr& msg ) {
    is_PauseRecording = msg->data;
}

void startStopCallback( const std_msgs::Bool::ConstPtr& msg ) {
    bool newstate = msg->data;
    if(newstate != current_recording_state) { 
        if(newstate == true) {
            filename = makeTime();
            is_PauseRecording = false;
        } 
        else {
            is_PauseRecording = true;
        }
    }
}




int main( int argc, char **argv ) {
    ros::init( argc, argv, "pc2_record" );
    
    ROS_INFO( "pc2_record: Starting Node" );
    ros::NodeHandle nh("~");
    ros::Subscriber sub_pc2;
    ros::Subscriber sub_bool_pause;
    ros::Subscriber sub_bool_start;
    
    ROS_INFO( "pc2_record: Obtaining Params" );
    nh.getParam("pc2_topic_in", pc2_topic_in );
    nh.getParam("save_to", filedir );
    filename = makeTime();
    
    ROS_INFO( "pc2_record: Configuring Node" );
    sub_pc2 = nh.subscribe( pc2_topic_in, 10, pc2Callback );
    sub_bool_pause = nh.subscribe( "/pause", 10, boolCallback); 
    sub_bool_start = nh.subscribe( "/start_stop", 10, startStopCallback);
    
    //File opening test
    std::string filepath = makeFilepath();
    myfile.open(filepath, std::ios_base::app);
    myfile << "Hello World!" << std::endl;
    myfile.close();
    
    ROS_INFO( "pc2_record: Starting Spin" );
    std::cout<< makeFilepath() << std::endl;
    ros::spin();
    
    return 0;
}
