#include "sensor_simulator/sensor_simulator.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


SimObject::SimObject()
#if 0
: node("lidar_simulator")
#endif
{
    clock_ = rclcpp::Clock(RCL_ROS_TIME);
    marker_ = visualization_msgs::msg::Marker();

    length_m_ = 0.0;
    width_m_ = 0.0;
    height_m_ = 0.0;
    pos_x_m_ = 0.0;
    pos_y_m_ = 0.0;
    pos_z_m_ = 0.0;
    roll_rad_ = 0.0;
    pitch_rad_ = 0.0;
    yaw_rad_ = 0.0;
    roll_deg_ = 0.0;
    pitch_deg_ = 0.0;
    yaw_deg_ = 0.0;
    frame_id_ = "";
    parent_frame_id_ = "";
    update_flag_ = true;
    count_ = 0;

#if 0
    //node = rclcpp::Node::make_shared("lidar_simulator");
    RCLCPP_INFO(node->get_logger(), "start");
    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ =
        node->create_publisher<sensor_msgs::msg::PointCloud2>("address_book", 1);
    update_flag_ = false;

    timer_ = node->create_wall_timer(
        1ms, std::bind(&LidarSimulator::timer_callback, this));
    //rclcpp::spin(node);
#endif
}

SimObject::~SimObject()
{
}

#if 0
void SimObject::timer_callback()
{
    //tf_broadcast();
    if (count_ > static_cast<int>(1000.0/update_freq_hz_))
    {
        update_scan();
        count_ = 0;
    }
    count_++;
}
#endif

void SimObject::set_param( const dmn_msgs::msg::ObjectSimParam osp )
{
    id_ = osp.id;
    frame_id_ = osp.name;
    topic_name_ = osp.topic_name;
    parent_frame_id_ = osp.parent_frame_id;

    length_m_ = osp.length;
    width_m_ = osp.width;
    height_m_ = osp.height;
    pos_x_m_ = osp.pos_x;
    pos_y_m_ = osp.pos_y;
    pos_z_m_ = osp.pos_z;
    roll_deg_ = osp.roll;
    pitch_deg_ = osp.pitch;
    yaw_deg_ = osp.yaw;
    roll_rad_ = atan2(sin(roll_deg_*M_PI/180.0),cos((roll_deg_*M_PI/180.0)));
    pitch_rad_ = atan2(sin(pitch_deg_*M_PI/180.0),cos((pitch_deg_*M_PI/180.0)));
    yaw_rad_ = atan2(sin(yaw_deg_*M_PI/180.0),cos((yaw_deg_*M_PI/180.0)));
    if (osp.parent_frame_id.empty())
    {
        parent_frame_id_ = "base_link";
    }
    else
    {
        parent_frame_id_ = osp.parent_frame_id;
    }
    update_flag_ = true;
    count_ = 0;

    marker_.header.frame_id = parent_frame_id_;
    marker_.type = visualization_msgs::msg::Marker::CUBE;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    //geometry_msgs::msg::position
    marker_.pose.position.x = osp.pos_x;
    marker_.pose.position.y = osp.pos_y;
    marker_.pose.position.z = osp.pos_z;

    tf2::Quaternion q;
    q.setRPY(roll_rad_, pitch_rad_, yaw_rad_);
    geometry_msgs::msg::Quaternion quat = tf2::toMsg(q);
    marker_.pose.orientation = quat;

    marker_.scale.x = length_m_;
    marker_.scale.y = width_m_;
    marker_.scale.z = height_m_;

#if 0
    marker_.color.r = 1.0;
    marker_.color.g = 0.0;
    marker_.color.b = 0.0;
    marker_.color.a = 1.0;
#endif
    marker_.color = osp.color;
    publisher_->publish(marker_);

    //std::vector<std::vector<Eigen::Vector3d> > surfaces;
    //std::vector<std::vector<Eigen::Vector3d> > corners;
    Eigen::Vector3d center_pos(pos_x_m_, pos_y_m_, pos_z_m_);

#if 1

    front_normal  = rotate_rpy(  1.0,  0.0,  0.0, roll_rad_, pitch_rad_, yaw_rad_);
    rear_normal   = rotate_rpy( -1.0,  0.0,  0.0, roll_rad_, pitch_rad_, yaw_rad_);
    left_normal   = rotate_rpy(  0.0,  1.0,  0.0, roll_rad_, pitch_rad_, yaw_rad_);
    right_normal  = rotate_rpy(  0.0, -1.0,  0.0, roll_rad_, pitch_rad_, yaw_rad_);
    top_normal    = rotate_rpy(  0.0,  0.0,  1.0, roll_rad_, pitch_rad_, yaw_rad_);
    bottom_normal = rotate_rpy(  0.0,  0.0, -1.0, roll_rad_, pitch_rad_, yaw_rad_);

    front_surface  = center_pos + rotate_rpy( length_m_/2.0, 0.0, 0.0, roll_rad_, pitch_rad_, yaw_rad_);
    rear_surface   = center_pos + rotate_rpy( -length_m_/2.0, 0.0, 0.0, roll_rad_, pitch_rad_, yaw_rad_);
    left_surface   = center_pos + rotate_rpy( 0.0, width_m_/2.0, 0.0, roll_rad_, pitch_rad_, yaw_rad_);
    right_surface  = center_pos + rotate_rpy( 0.0, -width_m_/2.0, 0.0, roll_rad_, pitch_rad_, yaw_rad_);
    top_surface    = center_pos + rotate_rpy( 0.0, 0.0, height_m_/2.0, roll_rad_, pitch_rad_, yaw_rad_);
    bottom_surface = center_pos + rotate_rpy( 0.0, 0.0, -height_m_/2.0, roll_rad_, pitch_rad_, yaw_rad_);

    corner_0 = center_pos + rotate_rpy(  length_m_/2.0,  width_m_/2.0,  height_m_/2.0, roll_rad_, pitch_rad_, yaw_rad_);
    corner_1 = center_pos + rotate_rpy(  length_m_/2.0, -width_m_/2.0,  height_m_/2.0, roll_rad_, pitch_rad_, yaw_rad_);
    corner_2 = center_pos + rotate_rpy(  length_m_/2.0,  width_m_/2.0, -height_m_/2.0, roll_rad_, pitch_rad_, yaw_rad_);
    corner_3 = center_pos + rotate_rpy(  length_m_/2.0, -width_m_/2.0, -height_m_/2.0, roll_rad_, pitch_rad_, yaw_rad_);
    corner_4 = center_pos + rotate_rpy( -length_m_/2.0,  width_m_/2.0,  height_m_/2.0, roll_rad_, pitch_rad_, yaw_rad_);
    corner_5 = center_pos + rotate_rpy( -length_m_/2.0, -width_m_/2.0,  height_m_/2.0, roll_rad_, pitch_rad_, yaw_rad_);
    corner_6 = center_pos + rotate_rpy( -length_m_/2.0,  width_m_/2.0, -height_m_/2.0, roll_rad_, pitch_rad_, yaw_rad_);
    corner_7 = center_pos + rotate_rpy( -length_m_/2.0, -width_m_/2.0, -height_m_/2.0, roll_rad_, pitch_rad_, yaw_rad_);

#endif

    count_ = 0;
}

#if 0
void SimObject::tf_broadcast()
{
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    rclcpp::Time current_time = ros_clock.now();
    tf2::Quaternion q;
    q.setRPY(mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_);
    geometry_msgs::msg::Quaternion quat = tf2::toMsg(q);
    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = current_time;
    trans.header.frame_id = parent_frame_id_.c_str();
    trans.child_frame_id = frame_id_.c_str();
    trans.transform.translation.x = mount_pose_x_m_;
    trans.transform.translation.y = mount_pose_y_m_;
    trans.transform.translation.z = mount_pose_z_m_;
    trans.transform.rotation = quat;
    tf_broadcaster_->sendTransform(trans);
}
#endif

SensorSimulator::SensorSimulator()
: Node("sensor_simulator")
{
    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    count_ = 0;
    update_freq_hz_ = 100.0;

    timer_ = this->create_wall_timer(1ms, std::bind(&SensorSimulator::timer_callback, this));

    //subscription_ = this->create_subscription<dmn_msgs::msg::SimulationParam>("sim_param", 1, std::bind(&SensorSimulator::param_cb, this, _1));
    subscription_ = this->create_subscription<dmn_msgs::msg::SimulationParam>
        ("/simulation_param", 1, [this](dmn_msgs::msg::SimulationParam::SharedPtr msg) { param_cb(msg); } );
}

SensorSimulator::~SensorSimulator()
{
}

void SensorSimulator::add_lidar( std::string topic_name )
{
    std::shared_ptr<LidarSimulator> ls(new LidarSimulator());
    ls->publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name.c_str(), 1);
    ls->timer_ = this->create_wall_timer(
                    1ms, std::bind(&LidarSimulator::timer_callback, ls));
    lidars_.push_back(ls);
}

void SensorSimulator::add_lidar( const dmn_msgs::msg::LidarSimParam lsp )
{
    std::shared_ptr<LidarSimulator> ls(new LidarSimulator());
    ls->set_param( lsp );
    ls->publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lsp.topic_name.c_str(), 1);
    ls->timer_ = this->create_wall_timer(
                    1ms, std::bind(&LidarSimulator::timer_callback, ls));
    ls->tf_broadcaster_ = tf_broadcaster_;
    ls->tf_buffer_ = tf_buffer_;
    ls->tf_listener_ = tf_listener_;

    lidars_.push_back(ls);
}

void SensorSimulator::add_object( const dmn_msgs::msg::ObjectSimParam osp )
{
    std::shared_ptr<SimObject> os(new SimObject());
    os->publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(osp.topic_name.c_str(), 1);
    os->set_param( osp );
#if 0
        os->timer_ = this->create_wall_timer(
                        1ms, std::bind(&LidarSimulator::timer_callback, ls));
        os->tf_broadcaster_ = tf_broadcaster_;
        os->tf_buffer_ = tf_buffer_;
        os->tf_listener_ = tf_listener_;
#endif
    objects_.push_back(os);
}


void SensorSimulator::param_cb( const dmn_msgs::msg::SimulationParam::SharedPtr &msg )
{
    std::vector<dmn_msgs::msg::ObjectSimParam>::iterator iter_osp;
    std::vector<std::shared_ptr<SimObject> >::iterator iter_os;

    objects_marker_array_.markers.clear();
    for (iter_osp=msg->object_params.begin(); iter_osp!=msg->object_params.end(); ++iter_osp)
    {
        bool is_new = true;
        for(iter_os=objects_.begin(); iter_os!=objects_.end(); ++iter_os)
        {
            if ( iter_osp->id == (*iter_os)->id_ )
            {
                (*iter_os)->set_param( *iter_osp );
                is_new = false;
                objects_marker_array_.markers.push_back((*iter_os)->marker_);
                break;
            }
        }
        if (is_new)
        {
            add_object(*iter_osp);
            std::shared_ptr<SimObject> tmp_sm = objects_.back();
            objects_marker_array_.markers.push_back(tmp_sm->marker_);
        }
    }

    std::vector<dmn_msgs::msg::LidarSimParam>::iterator iter_lsp;
    std::vector<std::shared_ptr<LidarSimulator> >::iterator iter_ls;

    for (iter_lsp=msg->lidar_params.begin(); iter_lsp!=msg->lidar_params.end(); ++iter_lsp)
    {
        bool is_new = true;
        for(iter_ls=lidars_.begin(); iter_ls!=lidars_.end(); ++iter_ls)
        {
            if ( iter_lsp->id == (*iter_ls)->id_ )
            {
                (*iter_ls)->set_param( *iter_lsp );
#if 0
                (*iter_ls)->objects_ = objects_marker_array_;
#else
                (*iter_ls)->objects_ = objects_;
#endif
                is_new = false;
                break;
            }
        }
        if (is_new)
        {
            add_lidar(*iter_lsp);
            std::shared_ptr<LidarSimulator> tmp_ls = lidars_.back();
#if 0
            tmp_ls->objects_ = objects_marker_array_;
#else
            tmp_ls->objects_ = objects_;
#endif
        }
    }
}

#if 0
void tf_broad_cast()
{

}
#endif

void SensorSimulator::timer_callback()
{
    if (count_ > static_cast<int>(1000.0/update_freq_hz_))
    {
        //RCLCPP_INFO(this->get_logger(), "test");
        count_ = 0;
        //for ( objects_ );
    }
    count_++;
}