#include <vel_safety_limiter/emergency_checker.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <ros/callback_queue.h>

class VelLimit {
    public:
        VelLimit(ros::NodeHandle &n , EmergencyChecker& emergency_checker){
            n.param("max_speed", max_speed_,1.0);
            n.param("max_yaw", max_yaw_, 1.5);
            n.param<std::string>("vel_topic", topic_ , "/cmd_vel/limit");
            checkerptr_ = &emergency_checker;
            cmd_pub_ = n.advertise<geometry_msgs::Twist>(topic_, 1);
            sub_ = n.subscribe("/cmd_vel", 1000, &VelLimit::callback,this);

        }

        void update_vel_limits (std::vector<double>& vel_limits){
            
            vel_limits = checkerptr_->get_vel_limits();
        //     if (vel.direction.size() ==  0){
        //         return;
        //     }
        //     for (int i = 0 ;  i < vel..size() ; i ++){
        //         vel_limits.at(vel.direction[i]) =  vel.velocity_scaling[i]; 
        // }
        }

        void callback(const geometry_msgs::Twist::ConstPtr& msg){ 
            std::vector<double> vel_limits{1.0,1.0,1.0,1.0};            
            VelLimit::update_vel_limits(vel_limits);
            ROS_INFO("Vel Limits %lf , %lf , %lf , %lf",vel_limits[0],vel_limits[1],vel_limits[2],vel_limits[3]);

            geometry_msgs::Twist cmd;
            // cmd.twist = *msg;
            cmd.linear.x = std::max(std::min(msg->linear.x, max_speed_*vel_limits[0]) , -max_speed_*vel_limits[2]);
            cmd.angular.z = std::max(std::min(msg->angular.z, max_yaw_*vel_limits[1]) , -max_yaw_*vel_limits[3]);
            // cmd.header.stamp = ros::Time::now();

            cmd_pub_.publish(cmd);
        }



    private:
        double max_speed_;
        double max_yaw_;
        std::string topic_ ;  
        ros::Publisher cmd_pub_;
        ros::Subscriber sub_;
        EmergencyChecker *checkerptr_ ;


};


int main(int argc, char **argv){

    ros::init(argc, argv, "vel_limiter");
    ros::NodeHandle nh("~");
    ros::NodeHandle n_a("~");
    ros::CallbackQueue callback_queue_a;
    n_a.setCallbackQueue(&callback_queue_a);  
    EmergencyChecker emergency_checker(n_a);
    ros::Subscriber sub = n_a.subscribe("/scan", 1000, &EmergencyChecker::lidarCallback,&emergency_checker);

    std::thread spinner_thread_a([&callback_queue_a]() 
    {
    ros::SingleThreadedSpinner spinner_a;
    spinner_a.spin(&callback_queue_a);
    });
    VelLimit vel_limit (nh, emergency_checker);
    ROS_INFO("Vel Limiter Started");
    ros::spin();
    spinner_thread_a.join();



    return 0;

}