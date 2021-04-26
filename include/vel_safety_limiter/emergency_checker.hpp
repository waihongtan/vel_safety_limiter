#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <mutex>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#ifndef EMERGENCY_CHECKER_H_
#define EMERGENCY_CHECKER_H_

// struct VelLimiter {
//     std::vector<int> direction;
//     std::vector<double> velocity_scaling; 
// };

class EmergencyChecker
{ 
    public:

        EmergencyChecker(ros::NodeHandle &n);
        void lidarCallback(sensor_msgs::LaserScan::Ptr msg);
        double get_distance(double range, double angle);
        std::vector<double> get_vel_limits(); 
        void get_transform();

    private:
        double angle_increment_ = 0.02;
        int partial_range_ = 0;
        double high_distance_threshold_ = 0.6;
        double low_distance_threshold_ = 0.3;
        double shift_angle_ = 0.0;
        int shift_index_ = 0;
        int range_ = 0;
        bool rotate_flag_ = false ;
        std::string laser_frame_ ;
        std::string base_frame_;
        // std::vector<int> emergency_zones_ = {0};
        std::vector<double> velocity_scaling_ = {1.0,1.0,1.0,1.0};
        tf::TransformListener tf_;
        std::mutex mtx_;
        void add_limits_(double scale , int quadrant);
        tf::StampedTransform transform_;



};  

#endif 