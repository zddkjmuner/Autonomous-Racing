// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

#include <visualization_msgs/Marker.h>

// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
    std::vector<Node> children;///added
} Node;


class RRT {
public:
    RRT(ros::NodeHandle &nh); 
    virtual ~RRT();
private:
    ros::NodeHandle nh_;

    // ros pub/sub
    // TODO: add the publishers and subscribers you need

    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub_;

    ros::Publisher rrt_pub, path_pub, nav_pub, tree_pub;


    ros::Publisher best_point_pub, rrt_nodes_pub, submap_pts_pub;
    visualization_msgs::Marker next_best_point;
    visualization_msgs::Marker rrt_nodes;
    visualization_msgs::Marker submap_point;
    visualization_msgs::Marker tree_path;

    // tf stuff
    tf::TransformListener listener_flw2map;
    tf::StampedTransform transform_flw2map;
    tf::TransformListener listener_frw2map;
    tf::StampedTransform transform_frw2map;
    tf::TransformListener listener_blw2map;
    tf::StampedTransform transform_blw2map;

    ackermann_msgs::AckermannDriveStamped drv;
    bool safety_flag;
    // TODO: create RRT params
	//map informations
    int map_sizex_;
    int map_sizey_;
    double map_resolution_;
    double map_origin_x_, map_origin_y_;
    //std::vector<int, int> map_info_;
    std::vector<int> map_info_;
    std::string frame_;
    std::map<int, Node*> freespace_;


    std::vector<std::vector<float>> submap;
    std::vector<std::vector<float>> map_global_;
    std::vector<std::vector<float>> waypoint;
    double LOOK_AHEAD;

    std::vector<float> distant;
    double angle;
    double yaw;

    //int goal_submap_x, goal_submap_y;

    std::vector<geometry_msgs::PoseStamped> plan_;

	//rrt elements
    double threshold_, probability_;
	//rrt*
    double near_threshold_;

    Node root_;
    Node goal_node_;
    Node curr_;
    
    double max_expansion_dist_;

    double velocity;
    
	//sampling parameters
    int iteration_;
    int sample_num_;
    double extend_step_;
    std::vector<Node> rrt_nodes_;
    std::vector<Node> current_sampling_;



    // random generator, use this
    std::mt19937 gen;
    //std::uniform_real_distribution<> x_dist;
    //std::uniform_real_distribution<> y_dist;
    

    // callbacks
    // where rrt actually happens
    //void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void pf_callback(const nav_msgs::Odometry::ConstPtr &pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    //tf::TransformListener listener(ros::Duration(10));
    // RRT methods
    std::vector<double> sample(int goal_submap_x, int goal_submap_y);
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);

    //void makemap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    //std::vector<float> getBestPoint(float curr_x, float curr_y, double LOOK_AHEAD, std::vector<std::vector<float>> waypoint, visualization_msgs::Marker chosen_point, ros::Publisher chosen_point_pub);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);
    void Safety(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

};

