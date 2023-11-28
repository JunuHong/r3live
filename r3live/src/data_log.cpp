
#include <math.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <common_lib.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

#define IS_VALID( a ) ( ( abs( a ) > 1e8 ) ? true : false )

int camera_frame_num = 0;
int lidar_frame_num = 0;

std::string data_path = "";
std::string lidar_topic = "";
std::string image_topic = "";

ros::Publisher pub_full;

void image_callback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // save image in .png fromat
    if(cv::imwrite(data_path + "/image/" + std::to_string(camera_frame_num) + ".png", cv_ptr->image))
    {
        std::cout << "Saved image " << camera_frame_num << std::endl;
    }
    else
    {
        std::cerr << "Failed to save image " << lidar_frame_num << std::endl;
    }
    camera_frame_num++;
}

void lidar_callback(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>> pl_buff( 6 );
    pcl::PointCloud<pcl::PointXYZINormal>              pl_full;

    uint plsize = msg->point_num;

    pl_full.resize( plsize );

    for ( int i = 0; i < 6; i++ )
    {
        pl_buff[ i ].reserve( plsize );
    }
    // ANCHOR - remove nearing pts.
    for ( uint i = 1; i < plsize; i++ )
    {
        // clang-format off
        if ( ( msg->points[ i ].line < 6 )
            && ( !IS_VALID( msg->points[ i ].x ) )
            && ( !IS_VALID( msg->points[ i ].y ) )
            && ( !IS_VALID( msg->points[ i ].z ) )
            && msg->points[ i ].x > 0.7 )
        {
            // https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol
            // See [3.4 Tag Information] 
            if ( ( msg->points[ i ].x > 2.0 )
                && ( ( ( msg->points[ i ].tag & 0x03 ) != 0x00 )  ||  ( ( msg->points[ i ].tag & 0x0C ) != 0x00 ) )
                )
            {
                // Remove the bad quality points
                continue;
            }
        // clang-format on
            pl_full[ i ].x = msg->points[ i ].x;
            pl_full[ i ].y = msg->points[ i ].y;
            pl_full[ i ].z = msg->points[ i ].z;
            pl_full[ i ].intensity = msg->points[ i ].reflectivity;

            pl_full[ i ].curvature = msg->points[ i ].offset_time / float( 1000000 ); // use curvature as time of each laser points

            if ( ( std::abs( pl_full[ i ].x - pl_full[ i - 1 ].x ) > 1e-7 ) || ( std::abs( pl_full[ i ].y - pl_full[ i - 1 ].y ) > 1e-7 ) ||
                 ( std::abs( pl_full[ i ].z - pl_full[ i - 1 ].z ) > 1e-7 ) )
            {
                pl_buff[ msg->points[ i ].line ].push_back( pl_full[ i ] );
            }
        }
    }
    if ( pl_buff.size() != 6 )
    {
        return;
    }
    if ( pl_buff[ 0 ].size() <= 7 )
    {
        return;
    }
    ros::Time ct;
    ct.fromNSec( msg->timebase );

    pl_full.height = 1;
    pl_full.width = pl_full.size();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( pl_full, output );
    output.header.frame_id = "livox";
    output.header.stamp = ct;
    pub_full.publish( output );

    // save point cloud in .pcd format
    if(pcl::io::savePCDFileBinary(data_path + "/lidar/" + std::to_string(lidar_frame_num) + ".pcd", pl_full) == 0)
    {
        std::cout << "Saved point cloud " << lidar_frame_num << std::endl;
    }
    else
    {
        std::cerr << "Failed to save point cloud " << lidar_frame_num << std::endl;
    }
    lidar_frame_num++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_log");
    ros::NodeHandle nh;

    nh.param<std::string>("data_path", data_path, "/home/cocel/data/");
    nh.param<std::string>("image_topic", image_topic, "/camera/image_raw/compressed");
    nh.param<std::string>("lidar_topic", lidar_topic, "/livox/lidar");


    if (mkdir((data_path+"/lidar").data(), 0777) == -1)
    {
        if( errno == EEXIST ) {
            std::cout << "Dir already exists" << std::endl;
        } else {
        // something else
            std::cout << "cannot create session name folder error:" << strerror(errno) << std::endl;
            throw std::runtime_error( strerror(errno) );
        }
    }

    if (mkdir((data_path+"/image").data(), 0777) == -1)
    {
        if( errno == EEXIST ) {
            std::cout << "Dir already exists" << std::endl;
        } else {
        // something else
            std::cout << "cannot create session name folder error:" << strerror(errno) << std::endl;
            throw std::runtime_error( strerror(errno) );
        }
    }

    ros::Subscriber sub_image = nh.subscribe(image_topic, 1, image_callback);
    ros::Subscriber sub_lidar = nh.subscribe(lidar_topic, 1, lidar_callback);

    pub_full = nh.advertise< sensor_msgs::PointCloud2 >( "/laser_cloud", 100 );

    ros::spin();
}

