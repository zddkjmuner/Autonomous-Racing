#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <numeric>
//include ROS msg type headers and libraries

//initial value
bool Safety_flag = false;
double steer_angle = 0.0;
double velocity = 2;
double min_dis = 0.0;
double dis_threshold = 1.0;
double d_max = 50;
double Gamma = 5;
double L = 2.5;
double GAIN = 0.8;
double w_robo = 0.4;
double car_x, car_y;
double quatx;
double quaty;
double quatz;
double quatw;
double yaw_car;



class PFM {
// The class that hadles wall following
private:
    ros::NodeHandle n;
    double speed;
    //create ROS subscribers and publishers
    ros::Subscriber scan;
    ros::Subscriber odom;
    ros::Publisher pub_drive;
    ros::Publisher pub_bool;
    ros::Publisher pub_brake;
    //ros::Publisher vis_pub;
    std::string WAYPOINT_FILEPATH = "/home/weiyi/catkin_ws/src/pfm/src/wp.csv";
    std::vector<std::vector<float>> waypoints;
    
public:
    PFM() {
        n = ros::NodeHandle();
        speed = 0.0;
        //create ROS subscribers and publishers
	//vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
        pub_drive = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
	pub_brake = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1);
        pub_bool = n.advertise<std_msgs::Bool>("/brake_bool", 1);
	scan = n.subscribe("/scan",100, &PFM::scan_callback,this);
	odom = n.subscribe("/odom",100, &PFM::odom_callback,this);
	waypoints = getWaypoints(WAYPOINT_FILEPATH);
	//reverse points

	double temp1, temp2, temp3;
	for(int i=0 ; i<waypoints.size()/2 ; i++){
		temp1 = waypoints[i][0];
		temp2 = waypoints[i][1];
		temp3 = waypoints[i][2];
		waypoints[i][0] = waypoints[waypoints.size()-1-i][0];
		waypoints[i][1] = waypoints[waypoints.size()-1-i][1];
		waypoints[i][2] = waypoints[waypoints.size()-1-i][2];
		waypoints[waypoints.size()-1-i][0] = temp1;
		waypoints[waypoints.size()-1-i][1] = temp2;
		waypoints[waypoints.size()-1-i][2] = temp3;
	}
    }
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        //get useful odom info
	geometry_msgs::TwistWithCovariance temp = odom_msg->twist;
	geometry_msgs::Twist temp2 = temp.twist;
        speed = temp2.linear.x;
	car_x = odom_msg->pose.pose.position.x;
	car_y = odom_msg->pose.pose.position.y;
	quatx= odom_msg->pose.pose.orientation.x;
	quaty= odom_msg->pose.pose.orientation.y;
	quatz= odom_msg->pose.pose.orientation.z;
	quatw= odom_msg->pose.pose.orientation.w;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
	double min_angle = scan_msg->angle_min;
	double max_angle = scan_msg->angle_max;
	double increment_angle = scan_msg->angle_increment;
	int scan_times = int((max_angle-min_angle)/increment_angle);
	std::vector<float> dis = scan_msg->ranges;
	//ROS_INFO("angle min is [%f]",min_angle);
	//ROS_INFO("scan time is [%d]", scan_times);

	Safety(dis, min_angle, max_angle, increment_angle, scan_times);

        //publish drive/brake message if necessary
	ackermann_msgs::AckermannDriveStamped brk;
	std_msgs::Bool bol_true;
	std_msgs::Bool bol_false;
	brk.drive.speed = 0.0;
	bol_true.data = true;
	bol_false.data = false;
	if(Safety_flag){
	    pub_brake.publish(brk);
	    pub_bool.publish(bol_true);
 	    ROS_INFO("Stop!");
	}
	else{
	    pub_bool.publish(bol_false);	
	}  
	steer_angle = PFM::Get_angle(dis,scan_times,min_angle,max_angle,increment_angle);
	//ROS_INFO("The min is :[%f]",min_dis);
	//ROS_INFO("The steer angle is:[%f]",steer_angle);
	
	ackermann_msgs::AckermannDriveStamped drv;
	drv.header.frame_id = "laser";
	//drv.drive.steering_angle = (1-min_obs*0.7)*steer_angle;
	drv.drive.steering_angle = GAIN*steer_angle;
	drv.drive.speed = velocity;
	pub_drive.publish(drv);
	//usleep(1000);
    }

    double Get_angle(std::vector<float> dis, int scan_time, double min_angle, double max_angle, double increment_angle){
	std::vector<std::vector<float>> info(scan_time, std::vector<float> (4, 0.0));
	std::vector<int> center_angle_idx;
	std::vector<float> half_angle;
	int min_idx;
	int max_idx;
	int num_obs = 0;
	ROS_INFO("1");
	//////////////////////////////////////////////////////idx each obs///////////////////////////////////////////////////
	for(int i=0 ; i<scan_time ; i++){
		if((i==0 && dis.at(i)<dis_threshold) || (i>0 && dis.at(i)<dis_threshold && dis.at(i-1)>=dis_threshold)){
			num_obs += 1;
			min_idx = i;
		}
		if(dis.at(i)<dis_threshold){
			info[i][0] = num_obs;
		}
		if((i+1<scan_time && dis.at(i)<dis_threshold && dis.at(i+1)>=dis_threshold) || (i==scan_time-1 && dis.at(i)<dis_threshold)){
			max_idx = i;
			center_angle_idx.push_back(int((max_idx+min_idx)/2));
			half_angle.push_back((max_idx-min_idx)*increment_angle/2);
		}
	}
	ROS_INFO("2");
	//ROS_INFO("num of obs is [%d]",num_obs);
	//ROS_INFO("size of center_angle_idx is [%d]",center_angle_idx.size());
	//Now we have [111100002222000000333333]
	//////////////////////////////////////////////////////find theta_k/////////////////////////////////////////////////////////////
/*
	for(int i=0 ; i<scan_time ; i++){
		if(info[i][0] != 0.0){
			info[i][1] = center_angle_idx[int(info[i][0])-1]*increment_angle - 0.5*M_PI;
		}
	}
	//////////////////////////////////////////////////////find sig_k////////////////////////////////////////////////////////////////
	for(int i=0 ; i<scan_time ; i++){
		if(info[i][0] != 0.0){
			info[i][3] = half_angle[int(info[i][0])-1];
		}
	}*/
	///////////////////////////////////////////////////////find d_k////////////////////////////////////////////////////////////////
	std::vector<float> mean_dis;
	float sum_dis = 0;
	int count = 1;
	int num = 0;
	for(int i=0 ; i<scan_time ; i++){
		if(i==0 && info[i][0] == 1 && info[i+1][0] != 1){
			mean_dis.push_back(dis.at(i));
			count += 1;
		}
		if(i>0 && info[i-1][0] == count && info[i][0] != count){
			mean_dis.push_back(sum_dis/num);
			num = 0;
			sum_dis = 0;
			count += 1;
		}
		if(info[i][0] == count){
			sum_dis += dis.at(i);
			num += 1;
		}
	}
/*
	for(int i=0 ; i<scan_time ; i++){
		if(info[i][0] != 0.0){
			info[i][2] = mean_dis[info[i][0]-1];
		}
	}*/
	ROS_INFO("3");
	/////////////////////////////////////when consider car width, update sig_k////////////////////////////////////////////////////
	float temp_sig;
	for(int i=0 ; i<num_obs ; i++){
		temp_sig = atan2(mean_dis[i]*tan(half_angle[i])+w_robo*0.5,mean_dis[i]);
		half_angle[i] = temp_sig;
	}
	///////////////////////////////////////////////////////Gaussian part///////////////////////////////////////////////////////////
	std::vector<float> repulsive(scan_time, 0.0);
	std::vector<float> attractive(scan_time,0.0);
	float f_k;
	for(int i=1 ; i<=num_obs ; i++){
		for(int j=0 ; j< scan_time ; j++){
			f_k = (d_max - mean_dis[i])*exp(0.5)*exp(-pow((center_angle_idx[i]*increment_angle-j*increment_angle),2)/(2*pow(half_angle[i],2)));
			repulsive[j] += f_k;
		}
	}
	ROS_INFO("4");
	//////////////////////find goal point///////////////////////
	int min_index;
	float goal_x;
	float goal_y;
	float comp_dis;
	float min_dis = 100000;
	for(int i=0 ; i<waypoints.size() ; i++){
		comp_dis = sqrt(pow(waypoints[i][0]-car_x,2)+pow(waypoints[i][1]-car_y,2));
		if(comp_dis < min_dis){
			min_index = i;
			min_dis = comp_dis;
		}
	}
	bool next_pt_found = false;
	int search = min_index;
	while(!next_pt_found){
		++search;
		
		if(search >= waypoints.size()-2){
			search = 0;
		}

		// ROS_INFO("waypoint[search][0] : %f, wp_size: %d", waypoint[search][0], waypoint.size());

		if(pow(waypoints[search][0]-car_x,2)+pow(waypoints[search][1]-car_y,2) == pow(L,2)){
			goal_x = waypoints[search][0];
			goal_y = waypoints[search][1];
			next_pt_found = true;
		}
		if((pow(waypoints[search][0]-car_x,2)+pow(waypoints[search][1]-car_y,2) <= pow(L,2)) && 
			(pow(waypoints[search+1][0]-car_x,2)+pow(waypoints[search+1][1]-car_y,2) >= pow(L,2))){
			goal_x = (waypoints[search][0] + waypoints[search+1][0])/2;
			goal_y = (waypoints[search][1] + waypoints[search+1][1])/2;
			next_pt_found = true;
		}		
	}
	ROS_INFO("5");
	//ROS_INFO("goal point is [%f],[%f]",goal_x,goal_y);
	//////////////////////find angle of goal/////////////////////
	tf::Quaternion q(quatx, quaty, quatz, quatw);
	tf::Matrix3x3 m(q);
	double roll, pitch;
	m.getRPY(roll, pitch, yaw_car);

	float vehicle_x, vehicle_y;

	Eigen::Matrix2f R;
	Eigen::Vector2f x_c;
	Eigen::Vector2f x_v;
	Eigen::Vector2f x_m;

	R << cos(yaw_car), sin(yaw_car), -sin(yaw_car), cos(yaw_car);

	x_m << goal_x,goal_y;
	x_c << car_x, car_y;

	x_v = R*(x_m - x_c);

	vehicle_x = x_v(0);
	vehicle_y = x_v(1);
	if(vehicle_x==0){
		vehicle_x = 0.001;
	}
	if(vehicle_y==0){
		vehicle_y = 0.001;
	}

	double goal_angle;

	if(vehicle_x>0 && vehicle_y >0){
		goal_angle = atan(vehicle_y/vehicle_x);
	}
	if(vehicle_x>0 && vehicle_y<0){
		goal_angle = atan(vehicle_y/vehicle_x);
	}
	if(vehicle_x<0 && vehicle_y>0){
		goal_angle = atan(-vehicle_x/vehicle_y) + 0.5*M_PI;
	}
	if(vehicle_x<0 && vehicle_y<0){
		goal_angle = -atan(-vehicle_x/-vehicle_y) - 0.5*M_PI;
	}

	double f_at;
	for(int i=0 ; i < scan_time ; i++){
		f_at = Gamma*abs(goal_angle-(i*increment_angle+min_angle));
		attractive[i] = f_at;
	}
	ROS_INFO("6");
	////////////////////find goal idx////////////////////////////
	float goal_value=100000;
	int goal_idx;
	for(int i=0 ; i<scan_time ; i++){
		if(attractive[i]<goal_value){
			goal_value = attractive[i];
			goal_idx = i;
		}
	}
	///////////////generate total f and find min angle////////////
	std::vector<float> total_f(scan_time,0.0);
	for(int i=0 ; i<scan_time ; i++){
		total_f[i] = repulsive[i]+attractive[i];
	}
	double sum_v = std::accumulate(total_f.begin(), total_f.end(), 0.0);
	double mean_v = sum_v / total_f.size();
	double best_angle = 100000;
	int best_idx;
	int dis_idx = 100000;
	int temp_dis = 1000000;
	int count_num = 0;
	for(int i=0; i<scan_time ; i++){
		if(total_f[i]<mean_v*0.01){
			count_num++;
		}
	}
	ROS_INFO("num is [%d]",count_num);

/*	for(int i=0 ; i<scan_time ; i++){
		temp_dis = abs(goal_idx-i);
		if(attractive[i]<best_angle){
			best_angle = total_f[i];
			best_idx = i;
		}
	}*/

	for(int i=0 ; i<scan_time ; i++){
		temp_dis = abs(goal_idx-i);
		if(total_f[i]<mean_v*0.01 && temp_dis<dis_idx){
			best_angle = total_f[i];
			best_idx = i;
		}
	}
	return best_idx*increment_angle+min_angle;
    }
    void Safety(std::vector<float> dis, double min_angle, double max_angle, double increment_angle, int scan_times){
	double speed_vector[scan_times];
	for(int i=0 ; i < scan_times ; i++){
		speed_vector[i] = cos(min_angle+i*increment_angle)*speed;
	}

	double ttc_vector[scan_times];
	for(int i=0 ; i < scan_times ; i++){
		if(speed_vector[i] < 0.00001){
			speed_vector[i] = 0.0;
		}
		ttc_vector[i] = dis.at(i)/speed_vector[i];
	}
	
	float min_ttc = 1.0f / 0.0f;
	for(int i=0 ; i < scan_times ; i++){
		if(ttc_vector[i] < min_ttc){
			min_ttc = ttc_vector[i];
		}
	}
	if(min_ttc < 0.3){
		Safety_flag = true;
	}
    }
    std::vector<std::vector<float>> getWaypoints(std::string filepath){
	std::vector<std::vector<float>> waypoints;

	std::ifstream fin(filepath);
	if (!fin.good()){
		ROS_INFO("Waypoint File Not Found!!");
	}
	std::string line;
	while(getline(fin, line)){
		std::istringstream sin(line);
		std::vector<float> row;
		std::string wp;
		while(getline(sin, wp, ',')){
			row.push_back(atof(wp.c_str()));
		}

		waypoints.push_back(row);

	}
	ROS_INFO("there are total %lu groups of waypoints", waypoints.size());
	ROS_INFO("%f %f %f %f",waypoints[0][0],waypoints[0][1],waypoints[0][2],waypoints[0][3]);
	return waypoints;
   }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "pfm");
    PFM pfm;
    ros::spin();
    return 0;
}
