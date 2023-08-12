#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZI PointType;

class PointCloudHandler
{
public:
    PointCloudHandler()
    {
        // 订阅 velodyne 的数据
        sub_ = nh_.subscribe("/velodyne_points", 100, &PointCloudHandler::cloudCallback, this);

        // 分别发布1-8和9-16线号的点云数据
        pub_up = nh_.advertise<sensor_msgs::PointCloud2>("velodyne_cloud_up", 100);
        pub_down = nh_.advertise<sensor_msgs::PointCloud2>("velodyne_cloud_down", 100);
    }

    void cloudCallback(const sensor_msgs::PointCloud2 msg)
    {
        pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
        pcl::fromROSMsg(msg, laserCloudIn);
        std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
        PointType point;

        for(const auto& one_point : laserCloudIn.points)
        {
            point.x = one_point.x;
            point.y = one_point.y;
            point.z = one_point.z;

            double angle = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (N_SCANS == 16)
            {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (N_SCANS - 1) || scanID < 0)
                    continue;

            }

            laserCloudScans[scanID].push_back(point);
        }

        pcl::PointCloud<PointType>::Ptr laserCloudup(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr laserClouddown(new pcl::PointCloud<PointType>());

        for (int i = 0; i < N_SCANS/2; i++)
        { 
            *laserCloudup += laserCloudScans[i];
        }
        for (int i = N_SCANS/2; i < N_SCANS; i++)
        { 
            *laserClouddown += laserCloudScans[i];
        }

        //输出世界参考系的坐标
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(tf::Vector3(0,0,0));
        q.setW(1);
        q.setX(0);
        q.setY(0);
        q.setZ(0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, msg.header.stamp, "map", "map_child"));       

        //pcl转ros输出格式，map是统一坐标系
        sensor_msgs::PointCloud2 laserCloudupOutMsg;
        pcl::toROSMsg(*laserCloudup, laserCloudupOutMsg);
        laserCloudupOutMsg.header.stamp = msg.header.stamp;
        laserCloudupOutMsg.header.frame_id = "map";
        pub_up.publish(laserCloudupOutMsg);
        
        sensor_msgs::PointCloud2 laserClouddownOutMsg;
        pcl::toROSMsg(*laserClouddown, laserClouddownOutMsg);
        laserClouddownOutMsg.header.stamp = msg.header.stamp;
        laserClouddownOutMsg.header.frame_id = "map";
        pub_down.publish(laserClouddownOutMsg);        



        // // 发布点云数据
        // pub1_.publish(cloud_1_8);
        // pub2_.publish(cloud_9_16);

        // // 发布TF数据，建立参考系
        // static tf::TransformBroadcaster br;
        // tf::Transform transform;
        // transform.setOrigin(tf::Vector3(0, 0, 0));
        // tf::Quaternion q;
        // q.setRPY(0, 0, 0);
        // transform.setRotation(q);
        // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "velodyne"));
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_up;
    ros::Publisher pub_down;
    int N_SCANS=16;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loadRosbag");
    PointCloudHandler handler;

    ros::spin();

    return 0;
}
