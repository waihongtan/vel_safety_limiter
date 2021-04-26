#include <vel_safety_limiter/emergency_checker.hpp>

    EmergencyChecker::EmergencyChecker(ros::NodeHandle &n)
    {
        n.param("high_distance_threshold", high_distance_threshold_,0.6);
        n.param("low_distance_threshold", low_distance_threshold_,0.3);
        n.param<std::string>("base_frame" , base_frame_ , "base_link");
        // n.param("laser_frame", laser_frame_,"laser");       
        boost::shared_ptr<sensor_msgs::LaserScan const> laserptr;
        sensor_msgs::LaserScan laser;
        laserptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan",n);
        if(laserptr != NULL)
        {
            laser = *laserptr;
        }
        else 
        {
            ROS_ERROR("No scan received");
        }
        laser_frame_ = laser.header.frame_id;
        
        EmergencyChecker::get_transform();
        range_ = round(laser.ranges.size()/4);
        shift_index_ = round(shift_angle_/laser.angle_increment);

    }
    
    void EmergencyChecker::get_transform()
    {
        tf::StampedTransform transform;
        try 
        {
        tf_.waitForTransform(laser_frame_, base_frame_, ros::Time(0), ros::Duration(10.0) );
        tf_.lookupTransform(laser_frame_, base_frame_, ros::Time(0), transform);
        } 
        catch (tf::TransformException ex) 
        {
            ROS_ERROR("%s",ex.what());
        }
        tf::Quaternion q = transform.getRotation();
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        rotate_flag_ = (roll != 0) ? true : false;
        int angle = round((yaw-0.78)*100);
        shift_angle_ = (angle - std::floor(angle / 314 + 0.1) * 314)/100;
        std::cout<< roll<<pitch<<yaw<<shift_angle_<<std::endl;

    }

    double EmergencyChecker::get_distance(double range, double angle){

    double y = range * cos(angle);
    double x = range * sin(angle);
    geometry_msgs::PointStamped old_point;
    geometry_msgs::PointStamped new_point;
    old_point.header.frame_id = laser_frame_ ;
    old_point.point.x = x;
    old_point.point.y = y;
    
    try{
        const std::string base_frame = "base_link";
        tf_.transformPoint(base_frame,old_point, new_point);
        x = new_point.point.x;
        y = new_point.point.y;
        return std::sqrt(std::pow(x,2)+std::pow(y,2));
    }

    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return 0.0;
    }
}



    void EmergencyChecker::add_limits_(double scale , int quadrant){
        mtx_.lock();
        // emergency_zones_[quadrant] = quadrant; // 0 = Front
        // // std::cout<<emergency_zones_[1]<<std::endl;
        velocity_scaling_[quadrant]=std::max(scale,0.0);
        // std::cout<<velocity_scaling_[1]<<std::endl;
        mtx_.unlock();
    }


    void EmergencyChecker::lidarCallback(sensor_msgs::LaserScan::Ptr msg)
    {

        int quadrant = 0; 
        double curr_angle =-0.78;

        // int start_index = round((front_angle_/angle_increment_)-(partial_range__/2));
        // int stop_index  = round(start_index + partial_range__/2);
        float dis = 10000;
        if (rotate_flag_){
            std::reverse(msg->ranges.begin(), msg->ranges.end());

        }

        //Front
        for(int i = quadrant*range_; i < (range_ + quadrant*range_); i++){
            int actuallocation = (i+(msg->ranges.size() - shift_index_))% msg->ranges.size();
            float distance = EmergencyChecker::get_distance(msg->ranges[actuallocation],curr_angle);
            curr_angle += msg->angle_increment;
            if (isnan(distance)){
                    // std::cout << point <<std::endl;
                    continue;
                }
            dis = std::min(dis,distance); 
        }
        
        if (dis <= high_distance_threshold_) {
            double scale = (dis - low_distance_threshold_ )/(high_distance_threshold_-low_distance_threshold_);
            EmergencyChecker::add_limits_(scale,quadrant);
        }
        else{
            double scale = 1.0;
            EmergencyChecker::add_limits_(scale,quadrant);
        }
        // std::cout<<"Front"<<dis<<std::endl;
        // std::cout<<"Scale"<<velocity_scaling_[quadrant]<<std::endl;
        quadrant += 1;
        dis = 10000;
         //Right
        for(int i = quadrant*range_; i < (range_ + quadrant*range_); i++){
            int actuallocation = (i+(msg->ranges.size() - shift_index_))% msg->ranges.size();
            float distance = EmergencyChecker::get_distance(msg->ranges[actuallocation],curr_angle);
            curr_angle += msg->angle_increment;
            if (isnan(distance)){
                    // std::cout << point <<std::endl;
                    continue;
                }
            dis = std::min(dis,distance); 
        }
       
        if (dis <= high_distance_threshold_ ) {
            double scale = (dis - low_distance_threshold_ )/(high_distance_threshold_-low_distance_threshold_);
            // std::cout<<scale<<std::endl;
            // std::cout<<quadrant<<std::endl;
            EmergencyChecker::add_limits_(scale,quadrant);
        }
        else{
            double scale = 1.0;
            EmergencyChecker::add_limits_(scale,quadrant);
        }
        // std::cout<<"Right"<<dis<<std::endl;
        // std::cout<<"Scale"<<velocity_scaling_[quadrant]<<std::endl;
        quadrant += 1;
        dis = 10000;
        

          //Back
        for(int i = quadrant*range_; i < (range_ + quadrant*range_); i++){
            int actuallocation = (i+(msg->ranges.size() - shift_index_))% msg->ranges.size();
            float distance = EmergencyChecker::get_distance(msg->ranges[actuallocation],curr_angle);
            
            curr_angle += msg->angle_increment;
            curr_angle = curr_angle > 3.14 ? curr_angle-3.14*-1 : curr_angle; 
            if (isnan(distance)){
                    // std::cout << point <<std::endl;
                    continue;
                }
            dis = std::min(dis,distance); 
        }

        if (dis <= high_distance_threshold_ +0.25) {
            double scale = (dis - low_distance_threshold_ -0.25)/(high_distance_threshold_-low_distance_threshold_);
            EmergencyChecker::add_limits_(scale,quadrant);
        }
        else{
            double scale = 1.0;
            EmergencyChecker::add_limits_(scale,quadrant);
        }
        // std::cout<<"Back"<<dis<<std::endl;
        // std::cout<<"Scale"<<velocity_scaling_[quadrant]<<std::endl;
        quadrant += 1;
        dis = 10000;
        


         //Left
        for(int i = quadrant*range_; i < msg->ranges.size(); i++){
            int actuallocation = (i+(msg->ranges.size() - shift_index_))% msg->ranges.size();
            float distance = EmergencyChecker::get_distance(msg->ranges[actuallocation],curr_angle);
            
            curr_angle += msg->angle_increment;
            if (isnan(distance)){
                    // std::cout << point <<std::endl;
                    continue;
                }
            dis = std::min(dis,distance); 
        }


        if (dis <= high_distance_threshold_ ) {
            double scale = (dis - low_distance_threshold_ )/(high_distance_threshold_-low_distance_threshold_);
            EmergencyChecker::add_limits_(scale,quadrant);
        }
        else{
            double scale = 1.0;
            EmergencyChecker::add_limits_(scale,quadrant);
        }
        // std::cout<<"Left"<<dis<<std::endl;
        // std::cout<<"Scale"<<velocity_scaling_[quadrant]<<std::endl;
        // mtx_.lock();
        // emergency_zones_.clear(); // 0 = Front
        // velocity_scaling_.clear();
        // mtx_.unlock();

    return; 
    }

    std::vector<double> EmergencyChecker::get_vel_limits(){
        // VelLimiter vel ;
        // // vel.direction = emergency_zones_;
        // vel.velocity_scaling = velocity_scaling_;
        // std::cout<<vel.velocity_scaling[1]<<std::endl;
        // for (int i = 0 ; i < velocity_scaling_.size();i++){
        //     std::cout<<i<<":"<<velocity_scaling_[i]<<std::endl;
        // }
        return velocity_scaling_;
    }      
