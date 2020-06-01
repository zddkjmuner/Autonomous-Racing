#include "rrt/rrt.h"

#include <nav_msgs/Path.h>
std::vector<float> distant;
//std::vector<Node> tree;
double angle;
//double yaw;
int ctn = 0;

bool updated = true;

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
	delete[] &submap;
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) {

    // TO_DO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    std::string pose_topic, scan_topic;
	std::string drive_topic;
    nh_.getParam("/rrt_node/pose_topic", pose_topic);
    nh_.getParam("/rrt_node/scan_topic", scan_topic);
    nh_.getParam("/rrt_node/drive_topic", drive_topic);
    // nh_.getParam("/rrt_node/opp_pose", opp_pose_topic);

    nh_.getParam("/rrt_node/threshold", threshold_);
    nh_.getParam("/rrt_node/near_threshold_", near_threshold_);
    nh_.getParam("/rrt_node/max_iteration", iteration_);
    nh_.getParam("/rrt_node/max_expansion_dist", max_expansion_dist_);
    nh_.getParam("/rrt_node/LOOK_AHEAD", LOOK_AHEAD);
    nh_.getParam("/rrt_node/VELOCITY", velocity);
    
    nh_.getParam("/rrt_node/map_sizex_", map_sizex_);
    nh_.getParam("/rrt_node/map_sizey_", map_sizey_);
    nh_.getParam("/rrt_node/map_resolution_", map_resolution_);
    nh_.getParam("/rrt_node/map_origin_x_", map_origin_x_);
    nh_.getParam("/rrt_node/map_origin_y_", map_origin_y_);

    // ROS publishers
    // TO_DO: create publishers for the the drive topic, and other topics you might need
    rrt_pub = nh_.advertise<sensor_msgs::PointCloud>("rrt_tree", 1);
    path_pub = nh_.advertise<nav_msgs::Path>("rrt_path", 1);
    nav_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
   
    best_point_pub = nh_.advertise<visualization_msgs::Marker>("/best_point", 0);
    rrt_nodes_pub = nh_.advertise<visualization_msgs::Marker>("/rrt_nodes", 0);
    submap_pts_pub = nh_.advertise<visualization_msgs::Marker>("/submap_point", 0);
    tree_pub = nh_.advertise<visualization_msgs::Marker>("/whole_path_point", 0);

    // ROS subscribers
    // TO_DO: create subscribers as you need
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    opp_sub_ = nh_.subscribe("/opp_odom", 10, &RRT::opp_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
	// race_sub = nh_.subscribe("/race_info", 10, &RRT::collision_callback, this);
    //opp_sub_ = nh_.subscribe(opp_odom_topic, 1, &RRT::opp_pos_callback, this);

    //map_sub_ = nh.subscribe("map", 1, boost::bind(&RRT::makemap, this, 1));
    tf::TransformListener listener(ros::Duration(10));
    safety_flag = false;

    //publish visualization
	next_best_point.header.frame_id = "map";
	next_best_point.ns = "waypoint_vis";
	next_best_point.action = visualization_msgs::Marker::ADD;
	next_best_point.pose.position.z = 0;
	next_best_point.pose.orientation.x = 0.0;	
	next_best_point.pose.orientation.y = 0.0;	
	next_best_point.pose.orientation.z = 0.0;	
	next_best_point.pose.orientation.w = 1.0;	
	next_best_point.scale.x = 0.4;	
	next_best_point.scale.y = 0.4;	
	next_best_point.scale.z = 0.4;	
	next_best_point.color.a = 1.0;	
	next_best_point.color.r = 1.0;	
	next_best_point.color.g = 0.0;
	next_best_point.color.b = 0.0;

    //visualization sampled points
	rrt_nodes.header.frame_id = "map";
	rrt_nodes.ns = "tree_node_point_vis";
	rrt_nodes.type = visualization_msgs::Marker::POINTS;
	rrt_nodes.action = visualization_msgs::Marker::ADD;
	rrt_nodes.pose.orientation.w = 1.0;	
	rrt_nodes.scale.x = 0.2;	
	rrt_nodes.scale.y = 0.2;	
	rrt_nodes.scale.z = 0.2;	
	rrt_nodes.color.a = 1.0;	
	rrt_nodes.color.r = 0.0;	
	rrt_nodes.color.g = 0.0;
	rrt_nodes.color.b = 1.0;

     //visualization sampled points
	submap_point.header.frame_id = "map";
	submap_point.ns = "free_point_vis";
	submap_point.type = visualization_msgs::Marker::POINTS;
	submap_point.action = visualization_msgs::Marker::ADD;
	submap_point.pose.orientation.w = 1.0;	
	submap_point.scale.x = 0.1;	
	submap_point.scale.y = 0.1;	
	submap_point.scale.z = 0.1;	
	submap_point.color.a = 1.0;	
	submap_point.color.r = 0.8;	
	submap_point.color.g = 0.8;
	submap_point.color.b = 1.0;

    //visualization sampled points
	tree_path.header.frame_id = "map";
	tree_path.ns = "whole_path_vis";
	tree_path.type = visualization_msgs::Marker::LINE_LIST;
	tree_path.action = visualization_msgs::Marker::ADD;
	tree_path.pose.orientation.w = 1.0;	
	tree_path.scale.x = 0.1;	
	tree_path.color.a = 1.0;	
	tree_path.color.r = 0.0;	
	tree_path.color.g = 1.0;
	tree_path.color.b = 0.0;
    
    freespace_.clear();
    root_.parent = 0;
    root_.children.clear();

    // TO_DO: create a occupancy grid
    std::ifstream fin("/home/shivangimisra/shivangi_ws/src/f1tenth_team1_milestone3/src/skirk.csv");

    std::string line;
    while(getline(fin, line)){
	std::istringstream sin(line);
	std::vector<float> row;
	std::string wp;
	while(getline(sin, wp, ',')){
	    if(atof(wp.c_str()) <= 250){
		row.push_back(0.0);
	    }else{
		row.push_back(1.0);
	    }
	}
	map_global_.push_back(row);
	
    }

    map_copy_ = map_global_;
    //read waypoint
    std::ifstream fin2("/home/shivangimisra/shivangi_ws/src/f1tenth_team1_milestone3/src/new_latest_traj.csv");

    std::string line2;
    while(getline(fin2, line2)){
	std::istringstream sin(line2);
	std::vector<float> row2;
	std::string wp2;
	while(getline(sin, wp2, ',')){
	    row2.push_back(atof(wp2.c_str()));
	}
	waypoint.push_back(row2);
	
    }
 //    double temp1, temp2, temp3;
 //    for(int i=0 ; i<waypoint.size()/2 ; i++){
	// temp1 = waypoint[i][0];
	// temp2 = waypoint[i][1];
	// temp3 = waypoint[i][2];
	// waypoint[i][0] = waypoint[waypoint.size()-1-i][0];
	// waypoint[i][1] = waypoint[waypoint.size()-1-i][1];
	// waypoint[i][2] = waypoint[waypoint.size()-1-i][2];
	// waypoint[waypoint.size()-1-i][0] = temp1;
	// waypoint[waypoint.size()-1-i][1] = temp2;
	// waypoint[waypoint.size()-1-i][2] = temp3;
 //    }
    // ROS_INFO("the size of map is %lu", map_global_.size());
}

void RRT::opp_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
	opp_.x = pose_msg->pose.pose.position.x;
	opp_.y = pose_msg->pose.pose.position.y;

	map_copy_.clear();
	map_copy_ = map_global_;
	int map_opp_x, map_opp_y;
	map_opp_x = (abs(map_origin_x_) + opp_.x-0.1)/map_resolution_;
    map_opp_y = (abs(map_origin_y_) - opp_.y-2.6)/map_resolution_;

	for(int i=map_opp_x-10 ; i<=map_opp_x+10 ; i++){
		for(int j = map_opp_y-8 ; j<= map_opp_y+8 ; j++){
			if(i>0 || i<grid_size-1 || j>0 || j<grid_size-1){
				map_copy_[j][i] = 0;
			}
		}
	}
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // TO_DO: update your occupancy grid
    double min, max;//, angle
    int sample_time;
    min = scan_msg -> angle_min;
    max = scan_msg -> angle_max;
    angle = scan_msg -> angle_increment;
    sample_time = int((max-min)/angle);
    //std::vector<float> distant;
    distant.clear();
    distant = scan_msg -> ranges;

    //transform it to the origin submap coordinate
    Eigen::Matrix2f R;
    Eigen::Vector2f scan_;
    Eigen::Vector2f sub_co;
    // yaw -= 0.7;
    
    double r_fl, p_fl, y_fl, r_fr, p_fr, y_fr, r_bl, p_bl, y_bl;
    // listener_flw2map.lookupTransform("/ego_racecar/base_link", "/ego_racecar/front_left_wheel", ros::Time(0), transform_flw2map);
    // tf::Matrix3x3 m_flw(transform_flw2map.getBasis());
    // m_flw.getRPY(r_fl, p_fl, y_fl);

    // listener_frw2map.lookupTransform("/ego_racecar/base_link", "/ego_racecar/laser", ros::Time(0), transform_frw2map);
    // tf::Matrix3x3 m_frw(transform_frw2map.getBasis());
    // m_frw.getRPY(r_fr, p_fr, y_fr);

    // listener_flw2map.waitForTransform("/map", "/ego_racecar/base_link", ros::Time::now(), ros::Duration(10));
    // try{
    // 	listener_flw2map.lookupTransform("/map", "/ego_racecar/base_link", ros::Time(0), transform_blw2map);
	   //  // map_origin_x_ = -transform_blw2map.getOrigin().x();
	   //  // map_origin_y_ = -transform_blw2map.getOrigin().y();
	   //  tf::Matrix3x3 m_blw(transform_blw2map.getBasis());

	   //  m_blw.getRPY(r_bl, p_bl, y_bl);
	   //  R << cos(-y_bl), -sin(-y_bl), sin(-y_bl), cos(-y_bl);	
    // }
    // catch (tf::TransformException& ex)
    // {
    //     ROS_ERROR("%s",ex.what());
    //     ros::Duration(0.1).sleep();
    //     R << cos(-yaw), -sin(-yaw), sin(-yaw), cos(-yaw);
    // }
    
    R << cos(-yaw), -sin(-yaw), sin(-yaw), cos(-yaw);
    
    float scan_beam_len;
    std::vector<Eigen::Vector2f> new_dist;
    for(int i=0; i<distant.size(); i++){
    	if(cos(i*angle +scan_msg->angle_min)*distant[i]<0)
    	{
    		scan_beam_len = distant[i] + 0.01;
    	}
    	else
    	{
    		scan_beam_len = distant[i] - 0.01;
    	}
    	scan_ << cos(i*angle +scan_msg->angle_min - 0.5*M_PI) * scan_beam_len, sin(i*angle +scan_msg->angle_min - 0.5*M_PI) * scan_beam_len;
		 // scan_ << cos(y) * (distant[i]-0.01), -sin(y) * (distant[i]-0.01);//0.3
        sub_co = R.inverse()*scan_;
		new_dist.push_back(sub_co);

		scan_ << cos(i*angle +scan_msg->angle_min - 0.5*M_PI) * distant[i], sin(i*angle +scan_msg->angle_min - 0.5*M_PI) * distant[i];
		sub_co = R.inverse()*scan_;
		new_dist.push_back(sub_co);

    }

	////////////////if opp in submap, update/////////////////////////////
 //    map_copy_ = map_global_;
	// int map_opp_x, map_opp_y;
	// map_opp_x = (abs(map_origin_x_) + opp_.x-0.1)/map_resolution_;
 //    map_opp_y = (abs(map_origin_y_) - opp_.y-2.6)/map_resolution_;

	// for(int i=map_opp_x-10 ; i<=map_opp_x+10 ; i++){
	// 	for(int j = map_opp_y-8 ; j<= map_opp_y+8 ; j++){
	// 		if(i>0 || i<grid_size-1 || j>0 || j<grid_size-1){
	// 			map_copy_[j][i] = 0;
	// 		}
	// 	}
	// }

  //   int xx =0;
  //   int yy = 0;
  //   float map_x, map_y;
  //   // ROS_INFO("%f, %f", map_origin_x_ , map_origin_y_);
  //   map_x = (abs(map_origin_x_) + curr_.x - 0.1)/map_resolution_;
  //   map_y = (abs(map_origin_y_) - curr_.y-2.6)/map_resolution_; 
  //   submap.clear();
  //   for(int i=0; i<grid_size; i++){
  //       std::vector<float> row3;
		// for(int j=0; j<grid_size; j++){
	 //    	row3.push_back(map_copy_[map_y-grid_size*0.5+j][map_x-grid_size*0.5+i]);
		// }
		// submap.push_back(row3);
  //   }

//////////////////////////////////////////////////////////////////////
////////////////////////change the obstacle map///////////////////////
//////////////////////////////////////////////////////////////////////
/*
    for(int j=0; j< new_dist.size(); j++){
	int row = grid_size*0.5+new_dist[j](1)/map_resolution_;
	int col = grid_size*0.5+new_dist[j](0)/map_resolution_;
	
	//update submap
	if((row <= grid_size-5 && row >= 4) && (col >= 4 && col <= grid_size-5)){
		for(int m=0;m<4;m++){
			for(int n=0;n<8;n++){
				if((grid_size-1-(col-10+m) <= grid_size-1 && grid_size-1-(col-10+m) >= 0) && (grid_size-1-(row-10+n) >= 0 && grid_size-1-(row-10+n) <= grid_size-1)){
				submap[grid_size-1-(row-4+n)][grid_size-1-(col-4+m)] = 0.0;

				// submap[79-(row-4+n)+1][79-(col-4+m)] = 0.0;
				// submap[79-(row-4+n)+1][79-(col-4+m)-1] = 0.0;
				// submap[79-(row-4+n)][79-(col-4+m)-1] = 0.0;
				// submap[79-(row-4+n)-1][79-(col-4+m)-1] = 0.0;
				// submap[79-(row-4+n)-1][79-(col-4+m)] = 0.0;
				// submap[79-(row-4+n)-1][79-(col-4+m)+1] = 0.0;
				// submap[79-(row-4+n)][79-(col-4+m)+1] = 0.0;
				// submap[79-(row-4+n)+1][79-(col-4+m)+1] = 0.0;
				xx = row;
				yy = col;
				}
			}
		}
		
	}
    }*/
	// ROS_INFO("updated the laser scan");

	updated = true;
	Safety(scan_msg);
	// ROS_INFO("%d", safety_flag);
	
}
void RRT::pf_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {

    // tree as std::vector
    std::vector<Node> tree;

    // TO_DO: fill in the RRT main loop
    curr_.x = pose_msg->pose.pose.position.x;
    curr_.y = pose_msg->pose.pose.position.y;
    root_.x = curr_.x;
    root_.y = curr_.y;
    float curr_x = curr_.x;
    float curr_y = curr_.y;
    
    int xx =0;
    int yy = 0;
    int map_x, map_y;
    // ROS_INFO("%f, %f", map_origin_x_ , map_origin_y_);
    map_x = (abs(map_origin_x_) + curr_.x - 0.1)/map_resolution_;
    map_y = (abs(map_origin_y_) - curr_.y-2.6)/map_resolution_; 
    submap.clear();
    for(int i=0; i<grid_size; i++){
        std::vector<float> row3;
		for(int j=0; j<grid_size; j++){
	    	row3.push_back(map_copy_[map_y-grid_size*0.5+j][map_x-grid_size*0.5+i]);
		}
		submap.push_back(row3);
    }

    int min_index;
	float min_dist = 10000000;

	for(int i=0; i<waypoint.size();i++){
		int dist = pow((curr_x-waypoint[i][0]), 2) + pow((curr_y-waypoint[i][1]), 2);
			if(dist < min_dist){
				min_index = i;
				min_dist = dist;
			}
		}

	// ROS_INFO("Found current location");
	float x, y;

	bool cross_zero = false;
	bool next_pt_found = false;
	int search = min_index;
	int k = 0;

	while(!next_pt_found){
		k++;
		++search;
		
		if(search >= waypoint.size()-2){
			search = 0;
		}

		// ROS_INFO("waypoint[search][0] : %f, wp_size: %d", waypoint[search][0], waypoint.size());

		if(pow(waypoint[search][0]-curr_x,2)+pow(waypoint[search][1]-curr_y,2) == pow(LOOK_AHEAD,2)){
			x = waypoint[search][0];
			y = waypoint[search][1];
			next_pt_found = true;
		}
		if((pow(waypoint[search][0]-curr_x,2)+pow(waypoint[search][1]-curr_y,2) <= pow(LOOK_AHEAD,2)) && 
			(pow(waypoint[search+1][0]-curr_x,2)+pow(waypoint[search+1][1]-curr_y,2) >= pow(LOOK_AHEAD,2))){
			x = (waypoint[search][0] + waypoint[search+1][0])/2;
			y = (waypoint[search][1] + waypoint[search+1][1])/2;
			next_pt_found = true;
		}
		
	if(k>=waypoint.size()+1){next_pt_found = true; x = waypoint[min_index][0]; y = waypoint[min_index][1];}	
	}


	
	std::vector<float> best_point;
	best_point.push_back(x);
	best_point.push_back(y);
	next_best_point.id = 1;
	next_best_point.header.stamp = ros::Time::now();
	next_best_point.pose.position.x = x;
	next_best_point.pose.position.y = y;
	best_point_pub.publish(next_best_point);

//////////////////////////////////////////////////////////////////////////////////////////       


    //update goal point
    goal_node_.x = best_point[0];
    goal_node_.y = best_point[1];

    // int map_x, map_y;
    map_x = (abs(map_origin_x_) + curr_.x)/map_resolution_;
    map_y = (abs(map_origin_y_) - curr_.y)/map_resolution_;

    //update submap

if(ctn == 0){
    submap.clear();
    
    for(int i=0; i<grid_size; i++){
        std::vector<float> row3;
	for(int j=0; j<grid_size; j++){
	    row3.push_back(map_copy_[map_y-grid_size*0.5+j][map_x-grid_size*0.5+i]);
	}
	submap.push_back(row3);
    }
	ctn ++;
}
    
    submap_point.points.clear();	
	for(int m=0; m<grid_size; m++){
			for(int n=0; n<grid_size; n++){
			    if(submap[m][n]>0.96){
				submap_point.id = 4;
				submap_point.header.stamp = ros::Time::now();
				geometry_msgs::Point p;
				p.x = curr_.x + (m - grid_size*0.5)*map_resolution_;
				p.y = curr_.y - (n - grid_size*0.5)*map_resolution_;
				p.z = 0;
				submap_point.points.push_back(p);
			    }
			}
	}
	submap_pts_pub.publish(submap_point);


    //goal_node_ in global map
    int goal_map_x = (abs(map_origin_x_) + goal_node_.x)/map_resolution_;
    int goal_map_y = (abs(map_origin_y_) - goal_node_.y)/map_resolution_;
    //goal_node_ in submap
    int goal_submap_x = grid_size*0.5 + goal_map_x - map_x;
    int goal_submap_y = grid_size*0.5 + goal_map_y - map_y;

    // ROS_INFO("goal_node:. (%f, %f)", goal_node_.x, goal_node_.y);
    // ROS_INFO("goal_submap: (%d, %d)", goal_submap_x, goal_submap_y);

/////////////////////////////////////////////////////////////////////////////////////////

    //now we have the submap and the goal point in submap

    std::vector<double> sampled_point;
    int nearest_node = 0;
    Node new_node;
    
    root_.x = grid_size*0.5;
    root_.y = grid_size*0.5;
    root_.cost = 0;
    root_.is_root = true;

    tree.clear();
    tree.push_back(root_);

    //////////////////////////////////////////////////////////
    //rrt main loop

    for(int k=0; k<iteration_; k++){
	sampled_point = sample(goal_submap_x, goal_submap_y);
	// ROS_INFO("sampled point: (%f, %f)", sampled_point[0], sampled_point[1]);
	nearest_node = nearest(tree, sampled_point);
	// ROS_INFO("The nearest node index is: %d", nearest_node);
	new_node = steer(tree[nearest_node], sampled_point);//how far to steer?
	// ROS_INFO("the steered node: (%f, %f)", new_node.x, new_node.y);

	if(!check_collision(tree[nearest_node], new_node)){
		new_node.parent = nearest_node;
		new_node.cost = sqrt(pow(tree[nearest_node].x-new_node.x, 2) + pow(tree[nearest_node].y-new_node.y, 2));
		Node min_node = tree[nearest_node];
		float cmin = cost(tree, tree[nearest_node]) + line_cost(tree[nearest_node], new_node);
		std::vector<int> neighborhood = near(tree, new_node);
		for(int i=0; i< neighborhood.size(); i++){
			if(!check_collision(tree[neighborhood[i]], new_node) && (cost(tree, tree[neighborhood[i]])+line_cost(tree[neighborhood[i]], new_node)<cmin)){
				min_node = tree[neighborhood[i]];	
				cmin = cost(tree, tree[neighborhood[i]])+line_cost(tree[neighborhood[i]], new_node);
				new_node.parent = neighborhood[i];
				new_node.cost = sqrt(pow(tree[neighborhood[i]].x-new_node.x, 2) + pow(tree[neighborhood[i]].y-new_node.y, 2));
			}
		}
		tree.push_back(new_node);
		for(int j=0; j< neighborhood.size(); j++){
			// ROS_INFO("checking the neighborhood %d", j);
			//ROS_INFO("new: newnodex %f newnodey %f  newnode in tree cost %f, line cost %f", new_node.x, new_node.y, cost(tree, new_node), line_cost(tree[neighborhood[j]], new_node));
			//ROS_INFO("old: neighborhoodx %f y %f neigh in tree cost %f", tree[neighborhood[j]].x, tree[neighborhood[j]].y, cost(tree, tree[neighborhood[j]]));
			if(!check_collision(tree[neighborhood[j]], new_node) && (cost(tree, new_node)+line_cost(tree[neighborhood[j]], new_node)< cost(tree, tree[neighborhood[j]]))){
				tree[neighborhood[j]].parent = tree.size()-1;
				// ROS_INFO("change the neighborhood %f, %f parent to new_node", tree[neighborhood[j]].x, tree[neighborhood[j]].y);
			}transform_blw2map.getOrigin().x();
	    // map_origin_y_ = -
		}
		// ROS_INFO("the added new node is: (%f, %f)", new_node.x, new_node.y);
		if(is_goal(new_node, goal_submap_x, goal_submap_y)){
			// ROS_INFO("the current goal is: (%f, %f)", new_node.x, new_node.y);
			break;		
		}
	}
	
    }

	float find_min_dist = 1000000;
	int find_min = 0;
	for(int i=0; i<tree.size(); i++){
		float find_dist = sqrt(pow(tree[i].x-goal_submap_x, 2) + pow(tree[i].y-goal_submap_y, 2));
		if(find_dist < find_min_dist){
			find_min_dist = find_dist;
			find_min = i;
		}
	}
	new_node = tree[find_min];
// ROS_INFO("the final goal is (%f, %f)", new_node.x, new_node.y);
    
/////////////////////////////////////////////////////////////
    std::vector<Node> found_path;
    found_path = find_path(tree, new_node);
    
	std::vector<geometry_msgs::PoseStamped> temp_plan;
	geometry_msgs::PoseStamped pose;

    pose.header.frame_id = "map";
	pose.pose.position.x = curr_.x + (goal_submap_x - grid_size*0.5)*map_resolution_;
	pose.pose.position.y = curr_.y - (goal_submap_y - grid_size*0.5)*map_resolution_;
	pose.pose.orientation.w = 1;
	temp_plan.push_back(pose);

	pose.pose.position.x = curr_.x + (new_node.x - grid_size*0.5)*map_resolution_;
	pose.pose.position.y = curr_.y - (new_node.y - grid_size*0.5)*map_resolution_;
	pose.pose.orientation.w = 1;
	temp_plan.push_back(pose);

    int curr_parent = new_node.parent;
    while(!tree[curr_parent].is_root){

	pose.header.frame_id = "map";
	pose.pose.position.x = curr_.x + (tree[curr_parent].x - grid_size*0.5)*map_resolution_;
	pose.pose.position.y = curr_.y - (tree[curr_parent].y - grid_size*0.5)*map_resolution_;
	pose.pose.orientation.w = 1;
	temp_plan.push_back(pose);
	curr_parent = tree[curr_parent].parent;	

    }
    pose.pose.position.x = curr_.x;
    pose.pose.position.y = curr_.y;
    pose.pose.orientation.w = 1;
    temp_plan.push_back(pose);

    plan_.clear();
    for(int i = temp_plan.size()-1; i>=0; i--){
	plan_.push_back(temp_plan[i]);
    }
    nav_msgs::Path gui_path;
    gui_path.poses.resize(plan_.size());
    if(!plan_.empty()){
	gui_path.header.frame_id = plan_[0].header.frame_id;
	gui_path.header.stamp = plan_[0].header.stamp;
    }
    for(unsigned int i=0; i<plan_.size(); i++){
	gui_path.poses[i] = plan_[i];
    }
    path_pub.publish(gui_path);

////////////////////////////////////////

	tree_path.points.clear();

    for(int i=0; i< tree.size(); i++){
	if(!tree[i].is_root){
		tree_path.id = 5;
		tree_path.header.stamp = ros::Time::now();
		geometry_msgs::Point p;
		p.x = curr_.x + (tree[i].x - grid_size*0.5)*map_resolution_;
		p.y = curr_.y - (tree[i].y - grid_size*0.5)*map_resolution_;
		p.z = 0;
		tree_path.points.push_back(p);
		p.x = curr_.x + (tree[tree[i].parent].x - grid_size*0.5)*map_resolution_;
		p.y = curr_.y - (tree[tree[i].parent].y - grid_size*0.5)*map_resolution_;
		tree_path.points.push_back(p);
	}
    }
    tree_pub.publish(tree_path);
    


/////////////////////////////////////////////////////////////////////////////////////////////
    //we publish the nodes in the tree
	rrt_nodes.points.clear();

    sensor_msgs::PointCloud tree_to_publish;
    tree_to_publish.header.frame_id = "map";
    for(int i=0; i< tree.size(); i++){
	geometry_msgs::Point32 point;
	point.x = curr_.x + (tree[i].x - grid_size*0.5)*map_resolution_;
	point.y = curr_.y - (tree[i].y - grid_size*0.5)*map_resolution_;
	tree_to_publish.points.push_back(point);

	rrt_nodes.id = 4;
	rrt_nodes.header.stamp = ros::Time::now();
	geometry_msgs::Point p;
	p.x = point.x;
	p.y = point.y;
	p.z = 0;
	rrt_nodes.points.push_back(p);

    }
    rrt_nodes_pub.publish(rrt_nodes);
    rrt_pub.publish(tree_to_publish);

//////////////////////////////////////////////////////////////////////////////////////////////
    double roll, pitch, yaw;

    double angle, curvature;


    double quatx= pose_msg->pose.pose.orientation.x;
    double quaty= pose_msg->pose.pose.orientation.y;
    double quatz= pose_msg->pose.pose.orientation.z;
    double quatw= pose_msg->pose.pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
    

    float vehicle_x, vehicle_y;

    Eigen::Matrix2f R;
    Eigen::Vector2f x_c;
    Eigen::Vector2f x_v;
    Eigen::Vector2f x_m;

    R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);

//////////////////////////////////////////////////////////////////////////////////////
	//////////////////////choose which point to chase/////////////////
/////////////////////////////////////////////////////////////////////////////////////

    if(found_path.size()>4){
    x_m << (found_path[found_path.size()-3].x-grid_size*0.5)/20 + curr_x, -(found_path[found_path.size()-3].y-grid_size*0.5)/20 + curr_y;//7?
    }else{
	x_m << (found_path[found_path.size()-2].x-grid_size*0.5)/20 + curr_x, -(found_path[found_path.size()-2].y-grid_size*0.5)/20 + curr_y;
    }
    x_c << curr_x, curr_y;

    //current x, y in the map
    // ROS_INFO("curr_x, curr_y  x %f, y %f", curr_x, curr_y);

    //the goal point in waypoint
    // ROS_INFO("x, y  x %f, y %f", x, y);

    x_v = R.inverse()*(x_m - x_c);

    vehicle_x = x_v(0);
    vehicle_y = x_v(1);

	// ROS_INFO("vehicle_x x %f, y %f", vehicle_x, vehicle_y);


    curvature = 2 * vehicle_y / (pow(vehicle_y, 2) + pow(vehicle_x, 2));
    angle = asin(0.15*curvature);

	double vel;
	vel = velocity;

	if(abs(angle)>0.15 && abs(angle)<0.3){
		vel = velocity;
	}

	if(abs(angle)>0.3){
		vel = velocity;
	}
	
	if(safety_flag)
	{
		vel = 0;
	}
    
    drv.header.frame_id = "pure";
    drv.drive.steering_angle = angle;
    drv.drive.speed = vel;

    // ROS_INFO("the speed is x:%f, angle:%f", vel, angle);
    nav_pub.publish(drv);

}




std::vector<double> RRT::sample(int goal_submap_x, int goal_submap_y) {
    std::vector<double> sampled_point;
    int gen_x, gen_y;
    std::random_device rd;
    std::mt19937 gen(rd());
	int sample_range = 40;
	if(pow(curr_.x - opp_.x, 2) + pow(curr_.y - opp_.y, 2) <= 9){sample_range = 40;}
    std::uniform_int_distribution<int> x_dist(goal_submap_x-sample_range,goal_submap_x+sample_range);//30
    std::uniform_int_distribution<int> y_dist(goal_submap_y-sample_range,goal_submap_y+sample_range);
    
    gen_x = x_dist(gen);
    gen_y = y_dist(gen);
    int cnt = 0;

    while(gen_x >= grid_size || gen_y >= grid_size || gen_x < 0 || gen_y < 0){
	gen_x = x_dist(gen);
    gen_y = y_dist(gen);
    }
    //if it not in free space, then resample it
    while(submap[gen_x][gen_y] < 1.0){
	gen_x = x_dist(gen);
    	gen_y = y_dist(gen);
	while(gen_x >= grid_size || gen_y >= grid_size || gen_x < 0 || gen_y < 0){
	    gen_x = x_dist(gen);
    	    gen_y = y_dist(gen);
        }
    }
    sampled_point.push_back(gen_x);  
    sampled_point.push_back(gen_y);  

    return sampled_point;
}


int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {

    int nearest_node = 0;
    // TO_DO: fill in this method
    double min_dist_ = 10000;
    for(int i=0; i< tree.size(); i++){
	double dist = sqrt(pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2));
	if(dist < min_dist_){
	    nearest_node = i;
	    min_dist_ = dist;
	}
    }
    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {

    Node new_node;
    // TO_DO: fill in this method
    double node_dist = sqrt(pow(sampled_point[0] - nearest_node.x, 2) + pow(sampled_point[1] - nearest_node.y, 2));
    new_node.x = sampled_point[0];
    new_node.y = sampled_point[1];
    // ROS_INFO("node dist %f", node_dist);
    while(node_dist > max_expansion_dist_){
        new_node.x -= (sampled_point[0] - nearest_node.x) * 0.03;
	new_node.y -= (sampled_point[1] - nearest_node.y) * 0.03;
	node_dist = sqrt(pow(new_node.x - nearest_node.x, 2) + pow(new_node.y - nearest_node.y, 2));
    }

    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {

    bool collision = false;
    // TO_DO: fill in this method

    float x1, y1, x2, y2, k, b;
    x1 = nearest_node.x;
    y1 = nearest_node.y;
    x2 = new_node.x;
    y2 = new_node.y;
    k = (y2 - y1)/(x2 - x1);
    b = y1 - k * x1;
    if(isinf(k)){k = 1000;}
    if(isinf(b)){b = 1000;}
 
	for(int i=std::min(x1, x2); i<= x1 + x2 - std::min(x1, x2); i++){
	    int y_map = k*i+b;
	    if(y_map>=grid_size){y_map = grid_size-1;}
	    if(y_map<0){y_map = 0;}
	    if(submap[i][y_map] < 0.96){
		collision = true;
		
		// ROS_INFO("collision x %d, y %d", i, y_map);
		break;
	    }
	}
    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {

    bool close_enough = false;
    // TO_DO: fill in this method

    if(sqrt(pow(latest_added_node.x - goal_x, 2) + pow(latest_added_node.y - goal_y, 2)) < threshold_){
	// ROS_INFO("we found the goal!!");
	// ROS_INFO("x %f, y %f ", latest_added_node.x - goal_x, latest_added_node.y - goal_y);
	close_enough = true;
    }
    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    
    std::vector<Node> found_path;
    // TO_DO: fill in this method

    found_path.push_back(latest_added_node);

    int curr_parent = latest_added_node.parent;
    while(!tree[curr_parent].is_root){
	found_path.push_back(tree[curr_parent]);
	curr_parent = tree[curr_parent].parent;
    }
    found_path.push_back(tree[0]);

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {

    double cost = 0;
    int index = 0;
    // TO_DO: fill in this method
	
	index = node.parent;
	cost += node.cost;
	
	while(!tree[index].is_root){
		cost += tree[index].cost;
		index = tree[index].parent;
	}

    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {

    double cost = 0;
    // TO_DO: fill in this method
    cost = sqrt(pow(n1.x-n2.x, 2)+pow(n1.y-n2.y, 2));

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {

    std::vector<int> neighborhood;
    // TO_DO:: fill in this method

	double distant;
	for(int i=0; i<tree.size(); i++){
		distant = sqrt(pow(tree[i].x-node.x, 2)+pow(tree[i].y-node.y, 2));
		if(distant < near_threshold_){
			neighborhood.push_back(i);
		}
	}

    return neighborhood;
}

void RRT::Safety(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
	float curr_angle, rdot, ttc, ttc_thresh;
	float rdot_thresh = 0.01;
    float min_angle = scan_msg->angle_min;
    float max_angle = scan_msg->angle_max;
    float inc = scan_msg->angle_increment;
    int count = floor((max_angle -min_angle)/inc);
	


    for(int i = 0; i<count; i++){
    	curr_angle = scan_msg->angle_min + i*scan_msg->angle_increment;
    	ttc_thresh = 0.2;
    	if(!isnan(scan_msg->ranges[i]) && !isinf(scan_msg->ranges[i])){
  			rdot = velocity*cos(curr_angle);
  			
  			if(rdot == 0) rdot = rdot_thresh;
  			ttc = scan_msg->ranges[i]/rdot;

		    if(abs(ttc) <= ttc_thresh){
		       safety_flag = true;
		    }
		    else{
		    	safety_flag = false;	
		    }
	     	

    }
    
   }

}
