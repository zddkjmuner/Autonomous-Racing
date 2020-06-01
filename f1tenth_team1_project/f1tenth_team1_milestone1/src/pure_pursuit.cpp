#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>
#include <string>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>


using namespace std;

typedef std::vector<double> vec;
typedef std::vector<vec> mat;
const char* file_name = "<ADD LOCAL ADDRESS HERE>/catkin_ws/src/f1tenth_team1_milestone1/src/wp.csv";

struct state{
	double x;
	double y;
	double yaw;
};


class Pursuit{

private:
	ros::NodeHandle n;
	//ros::Publisher pt_pub; //visualization of waypoints
	ros::Publisher ack_pub; // to steer car, publish to nav topic
	ros::Subscriber pf_sub; // to read current position and heading from particle filter node
	float lookahead_dist = 2.8; //meters
	float p_curve = 0.35;
	mat waypoints;
	

public:
	Pursuit(mat way_pts){
		n = ros::NodeHandle();
		waypoints = way_pts;
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
		pf_sub = n.subscribe("/odom", 100, &Pursuit::localize_car, this);
		ack_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 1);
		//pt_pub = n.advertise<visualization_msgs::Marker>("waypoint_viz", 0);
	}

	~Pursuit(){}

	void localize_car(const nav_msgs::Odometry::ConstPtr& msg){
		state car_pose;
		car_pose.x = msg->pose.pose.position.x;
		car_pose.y = msg->pose.pose.position.y;
		tf::Quaternion q(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w
			);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		car_pose.yaw = yaw;


		//car_pose.yaw = 2*acos(msg->pose.pose.orientation.w);
		//car_pose.yaw = fmod(car_pose.yaw, 2*M_PI);


		//iterate over all saved waypoints to find closest
		float min_localize_dist = 100000;
		float dist;
		int row_num = 0;
		int car_pt_index = 0;
		int row;
		state waypt_values;

		for (row = 0; row < waypoints.size(); row++){
			waypt_values.x = waypoints[row][0];
			waypt_values.y = waypoints[row][1];
			waypt_values.yaw = waypoints[row][2];


			dist = sqrt(pow((waypt_values.x - car_pose.x),2) + 
				pow((waypt_values.y - car_pose.y),2));

			if(dist < min_localize_dist){
				min_localize_dist = dist;
				car_pt_index = row_num;
			}

			++ row_num;

		}


		follow_waypoints(car_pose, car_pt_index);
	}

	void follow_waypoints(state car_pose, int car_pt_index){
		state current_pose, next_pose, waypt_values;
		ackermann_msgs::AckermannDriveStamped driver;
		double car_x, car_y;
		float dist, curvature, angle;
		
		current_pose = {car_pose.x, car_pose.y, car_pose.yaw};

		dist = 0;
		float farthest_within_circle = 0;
		int row = car_pt_index;
		while(dist <= lookahead_dist){
			waypt_values.x = waypoints[row][0];
			waypt_values.y = waypoints[row][1];
			waypt_values.yaw = waypoints[row][2];


			dist = sqrt(pow((waypt_values.x - current_pose.x),2) + 
				pow((waypt_values.y - current_pose.y),2));


			if(dist > farthest_within_circle){
				farthest_within_circle = dist;
				next_pose = {waypt_values.x, waypt_values.y, waypt_values.yaw};

			}

			++row;

			if(row == waypoints.size()){
				row = 0;
			}

		}	

		// ROS_INFO("odom_x: %f, odom_y:%f, now_x: %f, now_y:%f, next_x: %f, next_y: %f",
		// 		car_pose.x, car_pose.y, 
		// 		waypoints[car_pt_index][0], waypoints[car_pt_index][1],
		// 		next_pose.x, next_pose.y);
			
					

		//in car's ref frame

		car_x = cos(current_pose.yaw)*(next_pose.x - current_pose.x) + 
							sin(current_pose.yaw)*(next_pose.y - current_pose.y);
		car_y = -sin(current_pose.yaw)*(next_pose.x - current_pose.x) + 
							cos(current_pose.yaw)*(next_pose.y - current_pose.y);

		curvature = (2*car_y)/(pow(car_x, 2) + pow(car_y, 2));//pow(lookahead_dist,2);


		//ROS_INFO("curv: %f, next_x: %f, next_y:%f", curvature, current_pose.yaw, car_y);
		//ROS_INFO("curve: %f", curvature);

		angle = p_curve*curvature;
		ROS_INFO("steer: %f, yaw: %f", car_y, current_pose.yaw);
		driver.drive.steering_angle = angle;
		double steer_angle_pi = (angle/M_PI)*180;
		if(abs(steer_angle_pi) >= 0 && abs(steer_angle_pi) <= 10){
                	driver.drive.speed = 4.5;
        	}
        	else if(abs(steer_angle_pi) > 10 && abs(steer_angle_pi <= 20)){
                	driver.drive.speed = 4.5;
        	}
        	else{
                	driver.drive.speed = 4.5;
        	}

		ack_pub.publish(driver);
/*
		//visualization code
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		//marker.header.stamp = ros::Time::now();
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		//marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		//marker.pose.position.x = next_pose.x;
		//marker.pose.position.y = next_pose.y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.02;
		marker.scale.y = 0.02;
		marker.scale.z = 0.02;
		marker.color.a = 1;
		marker.color.r = 1;
		marker.color.g = 0;
		marker.color.b = 0;
		
		for(int i=0 ; i<waypoints.size() ; i++){
			marker.id = i;
			marker.pose.position.x = waypoints[i][0];
			marker.pose.position.y = waypoints[i][1];
			pt_pub.publish(marker);
		}*/

	}


};


mat read_csv(){
	string line;
	// char* path = std::filesystem::current_path();
	string file_path(file_name);
	mat m;

	ifstream file;

	file.open(file_path);

	if (file.is_open())
	{
		while(getline(file, line)){
			vec row;
			istringstream iss(line);
			string value;
			while (getline(iss, value, ',')){
				row.push_back(atof(value.c_str()));
			}
			m.push_back(row);
		}
		file.close();
	}
	else 
	{
		ROS_INFO("Incorrect name or unable to open file (Change file address in code!)");
	}

	return m;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pure_pursuit");
	mat waypoints = read_csv();
	Pursuit p(waypoints);
	ros::spin();
	return 0;
}
