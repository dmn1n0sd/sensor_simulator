#include "sensor_simulator/sensor_simulator.hpp"

using namespace std::chrono_literals;

//#define USE_POLAR_COORDINATE

#ifdef USE_POLAR_COORDINATE
//https://qiita.com/progfay/items/dfeb8c4ff6ab5738b6e7
//http://www.sousakuba.com/Programming/gs_hittest_point_triangle.html
int hittest_point_polygon_2d( Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3, Eigen::Vector3d p ) {

    Eigen::Vector3d vec12 = v2 - v1;
    Eigen::Vector3d vec2p = p - v2;
    Eigen::Vector3d vec23 = v3 - v2;
    Eigen::Vector3d vec3p = p - v3;
    Eigen::Vector3d vec31 = v1 - v3;
    Eigen::Vector3d vec1p = p - v1;

    double op1 = vec12.cross(vec2p).x();
    double op2 = vec23.cross(vec3p).x();
    double op3 = vec31.cross(vec1p).x();

    if( ( op1 > 0 && op2 > 0 && op3 > 0 ) || ( op1 < 0 && op2 < 0 && op3 < 0 ) ) {
        return 0;
    }

    return 1;
}
#endif

Eigen::Vector3d rotate_rpy( double input_x, double input_y, double input_z, double roll, double pitch, double yaw )
{
    Eigen::Vector3d src(input_x, input_y, input_z);
    Eigen::Matrix3d rot_matrix;
    //https://watako-lab.com/2019/01/23/roll_pitch_yaw/
    rot_matrix <<   cos(yaw)*cos(pitch), (cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll)), (cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)),
                    sin(yaw)*cos(pitch), (sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll)), (sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)),
                            -sin(pitch),                               cos(pitch)*sin(roll),                               cos(pitch)*cos(roll);

    return rot_matrix*src;;
}

Eigen::Vector3d homogeneous_transform( Eigen::Vector3d src, double trans_x, double trans_y, double trans_z, double roll, double pitch, double yaw )
{
    Eigen::Vector4d tmp_src(src.x(), src.y(), src.z(), 1.0);

    Eigen::Matrix4d hmgn_matrix;
    hmgn_matrix << cos(yaw)*cos(pitch), (cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll)), (cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)), trans_x,
                    sin(yaw)*cos(pitch), (sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll)), (sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)), trans_y,
                            -sin(pitch),                               cos(pitch)*sin(roll),                               cos(pitch)*cos(roll), trans_z,
                                    0.0,                                                0.0,                                                0.0,     1.0;

    Eigen::Vector4d tmp_vec = hmgn_matrix*tmp_src;

    return  Eigen::Vector3d( tmp_vec.x(), tmp_vec.y(), tmp_vec.z() );
}

Eigen::Vector3d inverse_homogeneous_transform( Eigen::Vector3d src, double trans_x, double trans_y, double trans_z, double roll, double pitch, double yaw )
{
    Eigen::Vector4d tmp_src(src.x(), src.y(), src.z(), 1.0);

    Eigen::Matrix4d hmgn_matrix;
    hmgn_matrix << cos(yaw)*cos(pitch), (cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll)), (cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)), trans_x,
                    sin(yaw)*cos(pitch), (sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll)), (sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)), trans_y,
                            -sin(pitch),                               cos(pitch)*sin(roll),                               cos(pitch)*cos(roll), trans_z,
                                    0.0,                                                0.0,                                                0.0,     1.0;

    Eigen::Vector4d tmp_vec = hmgn_matrix.inverse()*tmp_src;

    return  Eigen::Vector3d( tmp_vec.x(), tmp_vec.y(), tmp_vec.z() );
}

#if 0
Eigen::Vector3d homogeneous_transform( double input_x, double input_y, double input_z, double trans_x, double trans_y, double trans_z, double roll, double pitch, double yaw )
{

    Eigen::Vector4d src(input_x, input_y, input_z, 1.0);
#if 1
    Eigen::Matrix4d hmgn_matrix;
    hmgn_matrix <<   cos(yaw)*cos(pitch), (cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll)), (cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)), trans_x,
                    sin(yaw)*cos(pitch), (sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll)), (cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)), trans_y,
                            -sin(pitch),                               cos(pitch)*sin(roll),                               cos(pitch)*cos(roll), trans_z,
                                    0.0,                                                0.0,                                                0.0,     1.0;

    Eigen::Vector4d tmp_vec = hmgn_matrix*src;

    Eigen::Vector3d  dst( tmp_vec.x(), tmp_vec.y(), tmp_vec.z() );

    return dst;
#else
    return homogeneous_transform( src, trans_x, trans_y, trans_z, roll, pitch, yaw );
#endif
}
#endif

LidarSimulator::LidarSimulator()
#if 0
: node("lidar_simulator")
#endif
{
    clock_ = rclcpp::Clock(RCL_ROS_TIME);
    measure_time_ = clock_.now();

    horizontal_resolution_deg_ = 0.2;
    vertical_resolution_deg_ = 0.2;
    horizontal_fov_deg_ = 120.0;
    vertical_fov_deg_ = 32.0;
    horizontal_max_deg_ = 60.0;
    vertical_max_deg_ = 16.0;
    horizontal_min_deg_ = -60.0;
    vertical_min_deg_ = -16.0;
    min_range_m_ = 0.0;
    max_range_m_ = 25.0;
    mount_pose_x_m_ = 0.0;
    mount_pose_y_m_ = 0.0;
    mount_pose_z_m_ = 0.0;
    mount_pose_roll_deg_ = 0.0;
    mount_pose_pitch_deg_ = 0.0;
    mount_pose_yaw_deg_ = 0.0;
    mount_pose_roll_rad_ = 0.0;
    mount_pose_pitch_rad_ = 0.0;
    mount_pose_yaw_rad_ = 0.0;
    count_ = 0;
    update_freq_hz_ = 10.0;

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

LidarSimulator::~LidarSimulator()
{
    pcl_cloud_.clear();
    pcl::toROSMsg(pcl_cloud_, ros2_cloud_);
    ros2_cloud_.header.stamp = clock_.now();
    ros2_cloud_.header.frame_id = frame_id_.c_str();
    publisher_->publish(ros2_cloud_);
}

#if 1
void LidarSimulator::timer_callback()
{
    tf_broadcast();
    rclcpp::Time now = clock_.now();
    if ( (now - measure_time_).seconds() > 1.0/update_freq_hz_ )
    {
        measure_time_ = now;
        update_scan();
        count_ = 0;
    }
    count_++;
}
#endif

void LidarSimulator::set_param( const sensor_sim_msgs::msg::LidarSimParam lsp )
{
    id_ = lsp.id;
    frame_id_ = lsp.name;
    //topic_name_ = lsp.topic_name;
    parent_frame_id_ = lsp.parent_frame_id;
    horizontal_resolution_deg_ = lsp.horizontal_resolution;
    vertical_resolution_deg_ = lsp.vertical_resolution;
    horizontal_fov_deg_ = lsp.horizontal_fov;
    vertical_fov_deg_ = lsp.vertical_fov;
    horizontal_max_deg_ = horizontal_fov_deg_/2.0;
    vertical_max_deg_ = vertical_fov_deg_/2.0;
    horizontal_min_deg_ = -horizontal_fov_deg_/2.0;
    vertical_min_deg_ = -vertical_fov_deg_/2.0;
    min_range_m_ = lsp.min_detection_range;
    max_range_m_ = lsp.max_detection_range;
    mount_pose_x_m_ = lsp.mouting_positon_x;
    mount_pose_y_m_ = lsp.mouting_positon_y;
    mount_pose_z_m_ = lsp.mouting_positon_z;
    mount_pose_roll_deg_ = lsp.mouting_posture_roll;
    mount_pose_pitch_deg_ = lsp.mouting_posture_pitch;
    mount_pose_yaw_deg_ = lsp.mouting_posture_yaw;
    mount_pose_roll_rad_ = atan2(sin(mount_pose_roll_deg_*M_PI/180.0),cos((mount_pose_roll_deg_*M_PI/180.0)));
    mount_pose_pitch_rad_ = atan2(sin(mount_pose_pitch_deg_*M_PI/180.0),cos((mount_pose_pitch_deg_*M_PI/180.0)));
    mount_pose_yaw_rad_ = atan2(sin(mount_pose_yaw_deg_*M_PI/180.0),cos((mount_pose_yaw_deg_*M_PI/180.0)));
    color_ = lsp.color;

    count_ = 0;
    update_freq_hz_ = lsp.update_frequency;
}

void LidarSimulator::tf_broadcast()
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

void LidarSimulator::update_scan()
{
    sensor_msgs::msg::PointCloud2 tmp_ros2_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA> tmp_pcl_cloud;

    pcl_cloud_.clear();

    double horizontal_fov_rad = atan2(sin(horizontal_fov_deg_*M_PI/180.0),cos((horizontal_fov_deg_*M_PI/180.0)));
    double vertical_fov_rad = atan2(sin(vertical_fov_deg_*M_PI/180.0),cos((vertical_fov_deg_*M_PI/180.0)));
    double horizontal_resolution_rad = atan2(sin(horizontal_resolution_deg_*M_PI/180.0),cos((horizontal_resolution_deg_*M_PI/180.0)));
    double vertical_resolution_rad = atan2(sin(vertical_resolution_deg_*M_PI/180.0),cos((vertical_resolution_deg_*M_PI/180.0)));

    double horizontal_dens = horizontal_fov_rad/horizontal_resolution_rad;
    double vertical_dens = vertical_fov_rad/vertical_resolution_rad;

    std::vector<std::vector<Eigen::Vector3d> > surfaces;
    std::vector<std::vector<Eigen::Vector3d> > corners;
#ifdef USE_POLAR_COORDINATE
    std::vector<std::vector<Eigen::Vector3d> > polars;
#endif
    std::vector<std::vector<Eigen::Vector3d> > normals;

    std::vector< std::shared_ptr<SimObject> >::iterator iter_obj;

    for ( iter_obj = objects_.begin(); iter_obj != objects_.end(); iter_obj++ )
    {
        std::vector<Eigen::Vector3d> tmp_surfaces;
        std::vector<Eigen::Vector3d> tmp_corners;
#ifdef USE_POLAR_COORDINATE
        std::vector<Eigen::Vector3d> tmp_polars;
#endif
        std::vector<Eigen::Vector3d> tmp_normals;

        Eigen::Vector3d transformed_front_surface = inverse_homogeneous_transform( (*iter_obj)->front_surface, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
        Eigen::Vector3d transformed_rear_surface = inverse_homogeneous_transform( (*iter_obj)->rear_surface, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
        Eigen::Vector3d transformed_left_surface = inverse_homogeneous_transform( (*iter_obj)->left_surface, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
        Eigen::Vector3d transformed_right_surface = inverse_homogeneous_transform( (*iter_obj)->right_surface, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
        Eigen::Vector3d transformed_top_surface = inverse_homogeneous_transform( (*iter_obj)->top_surface, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
        Eigen::Vector3d transformed_bottom_surface = inverse_homogeneous_transform( (*iter_obj)->bottom_surface, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );

        tmp_surfaces.push_back( transformed_front_surface );
        tmp_surfaces.push_back( transformed_rear_surface );
        tmp_surfaces.push_back( transformed_left_surface );
        tmp_surfaces.push_back( transformed_right_surface );
        tmp_surfaces.push_back( transformed_top_surface );
        tmp_surfaces.push_back( transformed_bottom_surface );

        surfaces.push_back(tmp_surfaces);

        Eigen::Vector3d tmp_corner;
        Eigen::Vector3d tmp_polar;
        tmp_corner = inverse_homogeneous_transform( (*iter_obj)->corner_0, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
        tmp_corners.push_back(tmp_corner);
#ifdef USE_POLAR_COORDINATE
        tmp_polar.x() = sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() + tmp_corner.z()*tmp_corner.z() );
        tmp_polar.y() = acos( sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() )/tmp_polar.x() )*fabs(tmp_corner.z())/tmp_corner.z();
        tmp_polar.z() = atan2( tmp_corner.y(), tmp_corner.x() );
        tmp_polars.push_back(tmp_polar);
#endif

        tmp_corner = inverse_homogeneous_transform( (*iter_obj)->corner_1, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
        tmp_corners.push_back(tmp_corner);
#ifdef USE_POLAR_COORDINATE
        tmp_polar.x() = sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() + tmp_corner.z()*tmp_corner.z() );
        tmp_polar.y() = acos( sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() )/tmp_polar.x() )*fabs(tmp_corner.z())/tmp_corner.z();
        tmp_polar.z() = atan2( tmp_corner.y(), tmp_corner.x() );
        tmp_polars.push_back(tmp_polar);
#endif

        tmp_corner = inverse_homogeneous_transform( (*iter_obj)->corner_2, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
        tmp_corners.push_back(tmp_corner);
#ifdef USE_POLAR_COORDINATE
        tmp_polar.x() = sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() + tmp_corner.z()*tmp_corner.z() );
        tmp_polar.y() = acos( sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() )/tmp_polar.x() )*fabs(tmp_corner.z())/tmp_corner.z();
        tmp_polar.z() = atan2( tmp_corner.y(), tmp_corner.x() );
        tmp_polars.push_back(tmp_polar);
#endif

        tmp_corner = inverse_homogeneous_transform( (*iter_obj)->corner_3, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
        tmp_corners.push_back(tmp_corner);
#ifdef USE_POLAR_COORDINATE
        tmp_polar.x() = sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() + tmp_corner.z()*tmp_corner.z() );
        tmp_polar.y() = acos( sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() )/tmp_polar.x() )*fabs(tmp_corner.z())/tmp_corner.z();
        tmp_polar.z() = atan2( tmp_corner.y(), tmp_corner.x() );
        tmp_polars.push_back(tmp_polar);
#endif

        tmp_corner = inverse_homogeneous_transform( (*iter_obj)->corner_4, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
        tmp_corners.push_back(tmp_corner);
#ifdef USE_POLAR_COORDINATE
        tmp_polar.x() = sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() + tmp_corner.z()*tmp_corner.z() );
        tmp_polar.y() = acos( sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() )/tmp_polar.x() )*fabs(tmp_corner.z())/tmp_corner.z();
        tmp_polar.z() = atan2( tmp_corner.y(), tmp_corner.x() );
        tmp_polars.push_back(tmp_polar);
#endif

        tmp_corner = inverse_homogeneous_transform( (*iter_obj)->corner_5, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
        tmp_corners.push_back(tmp_corner);
#ifdef USE_POLAR_COORDINATE
        tmp_polar.x() = sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() + tmp_corner.z()*tmp_corner.z() );
        tmp_polar.y() = acos( sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() )/tmp_polar.x() )*fabs(tmp_corner.z())/tmp_corner.z();
        tmp_polar.z() = atan2( tmp_corner.y(), tmp_corner.x() );
        tmp_polars.push_back(tmp_polar);
#endif

        tmp_corner = inverse_homogeneous_transform( (*iter_obj)->corner_6, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
        tmp_corners.push_back(tmp_corner);
#ifdef USE_POLAR_COORDINATE
        tmp_polar.x() = sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() + tmp_corner.z()*tmp_corner.z() );
        tmp_polar.y() = acos( sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() )/tmp_polar.x() )*fabs(tmp_corner.z())/tmp_corner.z();
        tmp_polar.z() = atan2( tmp_corner.y(), tmp_corner.x() );
        tmp_polars.push_back(tmp_polar);
#endif

        tmp_corner = inverse_homogeneous_transform( (*iter_obj)->corner_7, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
        tmp_corners.push_back(tmp_corner);
#ifdef USE_POLAR_COORDINATE
        tmp_polar.x() = sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() + tmp_corner.z()*tmp_corner.z() );
        tmp_polar.y() = acos( sqrt( tmp_corner.x()*tmp_corner.x() + tmp_corner.y()*tmp_corner.y() )/tmp_polar.x() )*fabs(tmp_corner.z())/tmp_corner.z();
        tmp_polar.z() = atan2( tmp_corner.y(), tmp_corner.x() );
        tmp_polars.push_back(tmp_polar);
#endif

        corners.push_back(tmp_corners);
#ifdef USE_POLAR_COORDINATE
        polars.push_back(tmp_polars);
#endif

        tmp_normals.push_back(calc_transformed_normal((*iter_obj)->front_normal));
        tmp_normals.push_back(calc_transformed_normal((*iter_obj)->rear_normal));
        tmp_normals.push_back(calc_transformed_normal((*iter_obj)->left_normal));
        tmp_normals.push_back(calc_transformed_normal((*iter_obj)->right_normal));
        tmp_normals.push_back(calc_transformed_normal((*iter_obj)->top_normal));
        tmp_normals.push_back(calc_transformed_normal((*iter_obj)->bottom_normal));
        normals.push_back(tmp_normals);
    }

#if 1
    for ( int i=0; i<=static_cast<int>(horizontal_dens); i++ )
    {
        double tmp_phi = -horizontal_fov_rad/2.0 + i*horizontal_resolution_rad;
        for ( int j=0; j<=static_cast<int>(vertical_dens); j++ )
        {
            double tmp_theta = -vertical_fov_rad/2.0 + j*vertical_resolution_rad;
            Eigen::Vector3d msm_vec( cos(tmp_theta)*cos(tmp_phi), cos(tmp_theta)*sin(tmp_phi), sin(tmp_theta) );

            pcl::PointXYZRGBA tmp_point;
            double measured_distance = max_range_m_;
            double tmp_measured_distance;

            std::vector< std::shared_ptr<SimObject> >::iterator iter_obj;
            std::vector<std::vector<Eigen::Vector3d> >::iterator iter_suf;
            std::vector<std::vector<Eigen::Vector3d> >::iterator iter_cnr;
            std::vector<std::vector<Eigen::Vector3d> >::iterator iter_plr;
            std::vector<std::vector<Eigen::Vector3d> >::iterator iter_nml;

#ifdef USE_POLAR_COORDINATE
            Eigen::Vector3d v1, v2, v3, v4 ,p;
            int is_outside;
#else
            double dot1, dot2, dot3, a, b;
            Eigen::Vector3d pos, vec;
#endif

#ifdef USE_POLAR_COORDINATE
            for ( iter_obj=objects_.begin(), iter_suf=surfaces.begin(), iter_cnr=corners.begin(), iter_plr=polars.begin(), iter_nml=normals.begin();
                    iter_obj!=objects_.end(), iter_suf!=surfaces.end(), iter_cnr!=corners.end(), iter_plr!=polars.end(), iter_nml!=normals.end();
                        iter_obj++, iter_suf++, iter_cnr++, iter_plr++, iter_nml++ )
#else
            for ( iter_obj=objects_.begin(), iter_suf=surfaces.begin(), iter_cnr=corners.begin(), iter_nml=normals.begin();
                    iter_obj!=objects_.end(), iter_suf!=surfaces.end(), iter_cnr!=corners.end(), iter_nml!=normals.end();
                        iter_obj++, iter_suf++, iter_cnr++, iter_nml++ )
#endif
            {
                //上面
#ifdef USE_POLAR_COORDINATE
                v1 = (*iter_plr)[4];
                v2 = (*iter_plr)[5];
                v3 = (*iter_plr)[1];
                v4 = (*iter_plr)[0];
                p = Eigen::Vector3d(0.0, tmp_theta, tmp_phi);
                is_outside = hittest_point_polygon_2d(v1, v2, v4, p) * hittest_point_polygon_2d(v2, v3, v4, p);
                if ( is_outside == 0 )
                {
                    tmp_measured_distance = calc_measuared_distance( -(*iter_suf)[4], (*iter_nml)[4], tmp_theta, tmp_phi );
                    if ( tmp_measured_distance > min_range_m_ && tmp_measured_distance < measured_distance )
                    {
                        measured_distance = tmp_measured_distance;
                    }
                }
#else
                Eigen::Vector3d tmp_45 = (*iter_cnr)[5] - (*iter_cnr)[4];
                Eigen::Vector3d tmp_40 = (*iter_cnr)[0] - (*iter_cnr)[4];
                tmp_measured_distance = calc_measuared_distance( msm_vec, -(*iter_suf)[4], (*iter_nml)[4] );

                tmp_point.x = tmp_measured_distance*msm_vec.x();
                tmp_point.y = tmp_measured_distance*msm_vec.y();
                tmp_point.z = tmp_measured_distance*msm_vec.z();

                pos = Eigen::Vector3d(tmp_point.x,tmp_point.y,tmp_point.z);
                vec = pos - (*iter_cnr)[4];

                dot1 = tmp_45.dot(tmp_40);
                dot2 = tmp_45.dot(vec);
                dot3 = tmp_40.dot(vec);

                a = dot2/tmp_45.norm();
                b = dot3/tmp_40.norm();

                if ( tmp_measured_distance > min_range_m_ && tmp_measured_distance < measured_distance &&
                        acos(dot2/(tmp_45.norm()*vec.norm())) < acos(dot1/(tmp_45.norm()*vec.norm())) &&
                            a < tmp_45.norm() && b < tmp_40.norm() && b > 0.0 )
                {
                    measured_distance = tmp_measured_distance;
                }
#endif

                //下面
#ifdef USE_POLAR_COORDINATE
                v1 = (*iter_plr)[6];
                v2 = (*iter_plr)[7];
                v3 = (*iter_plr)[3];
                v4 = (*iter_plr)[2];
                p = Eigen::Vector3d(0.0, tmp_theta, tmp_phi);
                is_outside = hittest_point_polygon_2d(v1, v2, v4, p) * hittest_point_polygon_2d(v2, v3, v4, p);
                if ( is_outside == 0 )
                {
                    tmp_measured_distance = calc_measuared_distance( -(*iter_suf)[5], (*iter_nml)[5], tmp_theta, tmp_phi );
                    if ( tmp_measured_distance > min_range_m_ && tmp_measured_distance < measured_distance )
                    {
                        measured_distance = tmp_measured_distance;
                    }
                }
#else
                Eigen::Vector3d tmp_67 = (*iter_cnr)[7] - (*iter_cnr)[6];
                Eigen::Vector3d tmp_62 = (*iter_cnr)[2] - (*iter_cnr)[6];

                tmp_measured_distance = calc_measuared_distance( msm_vec, -(*iter_suf)[5], (*iter_nml)[5] );

                tmp_point.x = tmp_measured_distance*msm_vec.x();
                tmp_point.y = tmp_measured_distance*msm_vec.y();
                tmp_point.z = tmp_measured_distance*msm_vec.z();

                pos = Eigen::Vector3d(tmp_point.x,tmp_point.y,tmp_point.z);
                vec = pos - (*iter_cnr)[6];

                dot1 = tmp_67.dot(tmp_62);
                dot2 = tmp_67.dot(vec);
                dot3 = tmp_62.dot(vec);

                a = dot2/tmp_67.norm();
                b = dot3/tmp_62.norm();

                if ( tmp_measured_distance > min_range_m_ && tmp_measured_distance < measured_distance &&
                        dot2 > dot1 && a < tmp_67.norm() && b < tmp_62.norm() && b > 0.0 )
                {
                    measured_distance = tmp_measured_distance;
                }
#endif

                //前面
#ifdef USE_POLAR_COORDINATE
                v1 = (*iter_plr)[6];
                v2 = (*iter_plr)[7];
                v3 = (*iter_plr)[5];
                v4 = (*iter_plr)[4];
                p = Eigen::Vector3d(0.0, tmp_theta, tmp_phi);
                is_outside = hittest_point_polygon_2d(v1, v2, v4, p) * hittest_point_polygon_2d(v2, v3, v4, p);
                if ( is_outside == 0 )
                {
                    tmp_measured_distance = calc_measuared_distance( -(*iter_suf)[1], (*iter_nml)[1], tmp_theta, tmp_phi );
                    if ( tmp_measured_distance > min_range_m_ && tmp_measured_distance < measured_distance )
                    {
                        measured_distance = tmp_measured_distance;
                    }
                }
#else
                //Eigen::Vector3d tmp_67 = (*iter_cnr)[7] - (*iter_cnr)[6];
                Eigen::Vector3d tmp_64 = (*iter_cnr)[4] - (*iter_cnr)[6];

                tmp_measured_distance = calc_measuared_distance( msm_vec, -(*iter_suf)[1], (*iter_nml)[1] );

                tmp_point.x = tmp_measured_distance*msm_vec.x();
                tmp_point.y = tmp_measured_distance*msm_vec.y();
                tmp_point.z = tmp_measured_distance*msm_vec.z();

                pos = Eigen::Vector3d(tmp_point.x,tmp_point.y,tmp_point.z);
                vec = pos - (*iter_cnr)[6];

                dot1 = tmp_67.dot(tmp_64);
                dot2 = tmp_67.dot(vec);
                dot3 = tmp_64.dot(vec);

                a = dot2/(tmp_67.norm()*vec.norm())*vec.norm();
                b = dot3/(tmp_64.norm()*vec.norm())*vec.norm();
                if ( tmp_measured_distance > min_range_m_ && tmp_measured_distance < measured_distance &&
                        dot2 > dot1 && a < tmp_67.norm() && b < tmp_64.norm() && b > 0.0 )
                {
                    measured_distance = tmp_measured_distance;
                }
#endif

                //後面
#ifdef USE_POLAR_COORDINATE
                v1 = (*iter_plr)[2];
                v2 = (*iter_plr)[3];
                v3 = (*iter_plr)[1];
                v4 = (*iter_plr)[0];
                p = Eigen::Vector3d(0.0, tmp_theta, tmp_phi);
                is_outside = hittest_point_polygon_2d(v1, v2, v4, p) * hittest_point_polygon_2d(v2, v3, v4, p);
                if ( is_outside == 0 )
                {
                    tmp_measured_distance = calc_measuared_distance( -(*iter_suf)[0], (*iter_nml)[0], tmp_theta, tmp_phi );

                    if ( tmp_measured_distance > min_range_m_ && tmp_measured_distance < measured_distance )
                    {
                        measured_distance = tmp_measured_distance;
                    }
                }
#else
                Eigen::Vector3d tmp_23 = (*iter_cnr)[3] - (*iter_cnr)[2];
                Eigen::Vector3d tmp_20 = (*iter_cnr)[0] - (*iter_cnr)[2];

                tmp_measured_distance = calc_measuared_distance( msm_vec, -(*iter_suf)[0], (*iter_nml)[0] );

                tmp_point.x = tmp_measured_distance*msm_vec.x();
                tmp_point.y = tmp_measured_distance*msm_vec.y();
                tmp_point.z = tmp_measured_distance*msm_vec.z();

                pos = Eigen::Vector3d(tmp_point.x,tmp_point.y,tmp_point.z);
                vec = pos - (*iter_cnr)[2];

                dot1 = tmp_23.dot(tmp_20);
                dot2 = tmp_23.dot(vec);
                dot3 = tmp_20.dot(vec);

                a = dot2/(tmp_23.norm()*vec.norm())*vec.norm();
                b = dot3/(tmp_20.norm()*vec.norm())*vec.norm();

                if ( tmp_measured_distance > min_range_m_ && tmp_measured_distance < measured_distance &&
                        dot2 > dot1 && a < tmp_23.norm() && b < tmp_20.norm() && b> 0.0 )
                {
                    measured_distance = tmp_measured_distance;
                }
#endif

                //左側面
#ifdef USE_POLAR_COORDINATE
                v1 = (*iter_plr)[7];
                v2 = (*iter_plr)[3];
                v3 = (*iter_plr)[1];
                v4 = (*iter_plr)[5];
                p = Eigen::Vector3d(0.0, tmp_theta, tmp_phi);
                is_outside = hittest_point_polygon_2d(v1, v2, v4, p) * hittest_point_polygon_2d(v2, v3, v4, p);
                if ( is_outside == 0 )
                {
                    tmp_measured_distance = calc_measuared_distance( -(*iter_suf)[3], (*iter_nml)[3], tmp_theta, tmp_phi );
                    if ( tmp_measured_distance > min_range_m_ && tmp_measured_distance < measured_distance )
                    {
                        measured_distance = tmp_measured_distance;
                    }
                }
#else
                Eigen::Vector3d tmp_73 = (*iter_cnr)[3] - (*iter_cnr)[7];
                Eigen::Vector3d tmp_75 = (*iter_cnr)[5] - (*iter_cnr)[7];

                tmp_measured_distance = calc_measuared_distance( msm_vec, -(*iter_suf)[3], (*iter_nml)[3] );

                tmp_point.x = tmp_measured_distance*msm_vec.x();
                tmp_point.y = tmp_measured_distance*msm_vec.y();
                tmp_point.z = tmp_measured_distance*msm_vec.z();

                pos = Eigen::Vector3d(tmp_point.x,tmp_point.y,tmp_point.z);
                vec = pos - (*iter_cnr)[7];

                dot1 = tmp_73.dot(tmp_75);
                dot2 = tmp_73.dot(vec);
                dot3 = tmp_75.dot(vec);

                a = dot2/(tmp_73.norm()*vec.norm())*vec.norm();
                b = dot3/(tmp_75.norm()*vec.norm())*vec.norm();

                if ( tmp_measured_distance > min_range_m_ && tmp_measured_distance < measured_distance &&
                        dot2 > dot1 && a < tmp_73.norm() && b < tmp_75.norm() && b > 0.0 )
                {
                    measured_distance = tmp_measured_distance;
                }
#endif

                //右側面
#ifdef USE_POLAR_COORDINATE
                v1 = (*iter_plr)[6];
                v2 = (*iter_plr)[2];
                v3 = (*iter_plr)[0];
                v4 = (*iter_plr)[4];
                p = Eigen::Vector3d(0.0, tmp_theta, tmp_phi);
                is_outside = hittest_point_polygon_2d(v1, v2, v4, p) * hittest_point_polygon_2d(v2, v3, v4, p);
                if ( is_outside == 0 )
                {
                    tmp_measured_distance = calc_measuared_distance( -(*iter_suf)[2], (*iter_nml)[2], tmp_theta, tmp_phi );
                    if ( tmp_measured_distance > min_range_m_ && tmp_measured_distance < measured_distance )
                    {
                        measured_distance = tmp_measured_distance;
                    }
                }
#else
                //Eigen::Vector3d tmp_62 = (*iter_cnr)[2] - (*iter_cnr)[6];
                //Eigen::Vector3d tmp_64 = (*iter_cnr)[4] - (*iter_cnr)[6];
                tmp_measured_distance = calc_measuared_distance( msm_vec, -(*iter_suf)[2], (*iter_nml)[2] );

                tmp_point.x = tmp_measured_distance*msm_vec.x();
                tmp_point.y = tmp_measured_distance*msm_vec.y();
                tmp_point.z = tmp_measured_distance*msm_vec.z();

                pos = Eigen::Vector3d(tmp_point.x,tmp_point.y,tmp_point.z);
                vec = pos - (*iter_cnr)[6];

                dot1 = tmp_62.dot(tmp_64);
                dot2 = tmp_62.dot(vec);
                dot3 = tmp_64.dot(vec);

                a = dot2/(tmp_62.norm()*vec.norm())*vec.norm();
                b = dot3/(tmp_64.norm()*vec.norm())*vec.norm();

                if ( tmp_measured_distance > min_range_m_ && tmp_measured_distance < measured_distance &&
                        dot2 > dot1 && a < tmp_62.norm() && b < tmp_64.norm() && b > 0.0 )
                {
                    measured_distance = tmp_measured_distance;
                }
#endif
            }

            if( measured_distance > min_range_m_ && measured_distance < max_range_m_ )
            {
                tmp_point.x = measured_distance*cos(tmp_theta)*cos(tmp_phi);
                tmp_point.y = measured_distance*cos(tmp_theta)*sin(tmp_phi);
                tmp_point.z = measured_distance*sin(tmp_theta);
                tmp_point.r = static_cast<u_int8_t>(color_.r*255.0);
                tmp_point.g = static_cast<u_int8_t>(color_.g*255.0);
                tmp_point.b = static_cast<u_int8_t>(color_.b*255.0);
                tmp_point.a = static_cast<u_int8_t>(color_.a*255.0);
                pcl_cloud_.push_back(tmp_point);
            }
        }
    }
#endif
    pcl::toROSMsg(pcl_cloud_, ros2_cloud_);
    ros2_cloud_.header.stamp = clock_.now();
    ros2_cloud_.header.frame_id = frame_id_.c_str();
    publisher_->publish(ros2_cloud_);
}

//https://knzw.tech/raytracing/?page_id=78
double LidarSimulator::calc_measuared_distance( double ox, double oy, double oz, double nx, double ny, double nz, double theta, double phi )
{
    double sx, sy, sz;

    sx = -ox;
    sy = -oy;
    sz = -oz;

    double dx, dy, dz;
    dx = cos(theta)*cos(phi);
    dy = cos(theta)*sin(phi);
    dz = sin(theta);

#if 0
    double roll = mount_pose_roll_rad_;
    double pitch = mount_pose_pitch_rad_;
    double yaw = mount_pose_yaw_rad_;

    Eigen::Vector3d dst_vec = rotate_rpy(dx,dy,dz,roll,pitch,yaw);
    double tdx, tdy, tdz;
#if 0
    tdx = dst_vec.x();
    tdy = dst_vec.y();
    tdz = dst_vec.z();
#else
    tdx = dx;
    tdy = dy;
    tdz = dz;
#endif
    return - (sx*nx+sy*ny+sz*nz)/(tdx*nx+tdy*ny+tdz*nz);
#else
    double tnx, tny, tnz;
    Eigen::Vector3d src_vec_1(nx,ny,nz);
    Eigen::Vector3d dst_vec_1 = inverse_homogeneous_transform( src_vec_1, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
    Eigen::Vector3d src_vec_2(0.0,0.0,0.0);
    Eigen::Vector3d dst_vec_2 = inverse_homogeneous_transform( src_vec_2, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
    //Eigen::Vector3d dst_vec = rotate_rpy(nx,ny,dz,-roll,-pitch,-yaw);

    tnx = dst_vec_1.x() - dst_vec_2.x();
    tny = dst_vec_1.y() - dst_vec_2.y();
    tnz = dst_vec_1.z() - dst_vec_2.z();
    double norm = sqrt( tnx*tnx + tny*tny + tnz*tnz );
    tnx *= norm;
    tny *= norm;
    tnz *= norm;

    return - (sx*tnx+sy*tny+sz*tnz)/(dx*tnx+dy*tny+dz*tnz);
#endif
}

double LidarSimulator::calc_measuared_distance( Eigen::Vector3d d, Eigen::Vector3d s, Eigen::Vector3d n )
{
    return - (s.x()*n.x()+s.y()*n.y()+s.z()*n.z())/(d.x()*n.x()+d.y()*n.y()+d.z()*n.z());
}

Eigen::Vector3d LidarSimulator::calc_transformed_normal (Eigen::Vector3d n)
{
    Eigen::Vector3d tn;

    Eigen::Vector3d dst_vec_1 = inverse_homogeneous_transform( n, mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );
    Eigen::Vector3d dst_vec_2 = inverse_homogeneous_transform( Eigen::Vector3d::Zero(), mount_pose_x_m_, mount_pose_y_m_, mount_pose_z_m_, mount_pose_roll_rad_, mount_pose_pitch_rad_, mount_pose_yaw_rad_ );

    tn = dst_vec_1 - dst_vec_2;

    return tn.normalized();
    //return tn;
}
