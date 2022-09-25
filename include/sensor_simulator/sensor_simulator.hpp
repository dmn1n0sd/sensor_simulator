#include <iostream>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
//#include <sensor_msgs/point_cloud_conversion.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>

#include "dmn_msgs/msg/simulation_param.hpp"
#include "dmn_msgs/msg/lidar_sim_param.hpp"

Eigen::Vector3d rotate_rpy( double input_x, double input_y, double input_z, double roll, double pitch, double yaw );
Eigen::Vector3d homogeneous_transform( Eigen::Vector3d src, double trans_x, double trans_y, double trans_z, double roll, double pitch, double yaw );
Eigen::Vector3d inverse_homogeneous_transform( Eigen::Vector3d src, double trans_x, double trans_y, double trans_z, double roll, double pitch, double yaw );

class SimObject
{
    public :
        SimObject();
        ~SimObject();
        void timer_callback();
        void set_param( const dmn_msgs::msg::ObjectSimParam osp );
#if 0
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        //std::shared_ptr<rclcpp::Node> node;
#endif
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        visualization_msgs::msg::Marker marker_;
        int id_;

        Eigen::Vector3d front_surface;
        Eigen::Vector3d rear_surface;
        Eigen::Vector3d left_surface;
        Eigen::Vector3d right_surface;
        Eigen::Vector3d top_surface;
        Eigen::Vector3d bottom_surface;

        Eigen::Vector3d front_normal;
        Eigen::Vector3d rear_normal;
        Eigen::Vector3d left_normal;
        Eigen::Vector3d right_normal;
        Eigen::Vector3d top_normal;
        Eigen::Vector3d bottom_normal;

        Eigen::Vector3d corner_0; //左奥上
        Eigen::Vector3d corner_1; //右奥上
        Eigen::Vector3d corner_2; //左奥下
        Eigen::Vector3d corner_3; //右奥下
        Eigen::Vector3d corner_4; //左手前上
        Eigen::Vector3d corner_5; //右手前上
        Eigen::Vector3d corner_6; //左手前下
        Eigen::Vector3d corner_7; //右手前下

        double roll_rad_;
        double pitch_rad_;
        double yaw_rad_;

    private :
        //void tf_broadcast();

        rclcpp::Clock clock_;
        double length_m_;
        double width_m_;
        double height_m_;
        double pos_x_m_;
        double pos_y_m_;
        double pos_z_m_;
        //double roll_rad_;
        //double pitch_rad_;
        //double yaw_rad_;
        double roll_deg_;
        double pitch_deg_;
        double yaw_deg_;
        std::string topic_name_;
        std::string frame_id_;
        std::string parent_frame_id_;
        int count_;
        bool update_flag_;

};

#if 0
class LidarSimulator : public rclcpp::Node
#else
class LidarSimulator
#endif
{
    public :
        LidarSimulator();
        ~LidarSimulator();
        void timer_callback();
        void set_param( const dmn_msgs::msg::LidarSimParam lsp );
#if 1
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        //std::shared_ptr<rclcpp::Node> node;
#endif
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_debug_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector< std::shared_ptr<SimObject> > objects_;

        int id_;

    private :
        double calc_measuared_distance( double ox, double oy, double oz, double nx, double ny, double nz, double theta, double phi );
        double calc_measuared_distance( Eigen::Vector3d d, Eigen::Vector3d s, Eigen::Vector3d n );
        Eigen::Vector3d calc_transformed_normal (Eigen::Vector3d n);

        void generate_scan_points();
        void update_scan();
        void tf_broadcast();

        rclcpp::Clock clock_;
        rclcpp::Time measure_time_;
        sensor_msgs::msg::PointCloud2 ros2_cloud_;
        pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud_;
        std_msgs::msg::ColorRGBA color_;
        double horizontal_resolution_deg_;
        double vertical_resolution_deg_;
        double horizontal_fov_deg_;
        double vertical_fov_deg_;
        double horizontal_max_deg_;
        double vertical_max_deg_;
        double horizontal_min_deg_;
        double vertical_min_deg_;
        double min_range_m_;
        double max_range_m_;
        double mount_pose_x_m_;
        double mount_pose_y_m_;
        double mount_pose_z_m_;
        double mount_pose_roll_rad_;
        double mount_pose_pitch_rad_;
        double mount_pose_yaw_rad_;
        double mount_pose_roll_deg_;
        double mount_pose_pitch_deg_;
        double mount_pose_yaw_deg_;
        double update_freq_hz_;
        std::string frame_id_;
        std::string parent_frame_id_;
        int count_;
        bool update_flag_;
};

class SensorSimulator : public rclcpp::Node
{
    public :
        SensorSimulator();
        ~SensorSimulator();
    private :
        void simulate_sensors();
        void param_cb( const dmn_msgs::msg::SimulationParam::SharedPtr &msg  );
        void timer_callback();
        void add_lidar( std::string topic_name );
        void add_lidar( const dmn_msgs::msg::LidarSimParam lsp );
        void add_object( const dmn_msgs::msg::ObjectSimParam osp );
        void tf_broadcast();
        std::vector< std::shared_ptr<LidarSimulator> > lidars_;
        std::vector< std::shared_ptr<SimObject> > objects_;
        rclcpp::Clock clock_;
        rclcpp::Subscription<dmn_msgs::msg::SimulationParam>::SharedPtr subscription_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;
        visualization_msgs::msg::MarkerArray objects_marker_array_;
        double update_freq_hz_;
        int count_;
};
