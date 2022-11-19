// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) {

    // error for loading parameters 
    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    std::string pose_topic, scan_topic;

    /////////////////////////////////////////////////////pose topic is /odom
    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    
    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1000);

    //layer publisher for rviz
    env_viz_pub = nh_.advertise<visualization_msgs::Marker>("env_viz", 10);
    static_viz_pub = nh_.advertise<visualization_msgs::Marker>("static_viz", 10);
    dynamic_viz_pub = nh_.advertise<visualization_msgs::Marker>("dynamic_viz", 10);
    tree_viz_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker1", 10);
    path_viz_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker2", 10);
    viz_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker3", 10);
    local_viz_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker4", 10);
    line_viz_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker5", 10);



    //layer subscriber for rviz
    env_sub = nh_.subscribe("env_layer", 10, &RRT::env_callback, this);
    static_sub = nh_.subscribe("static_layer", 10, &RRT::static_callback, this);
    dynamic_sub = nh_.subscribe("dynamic_layer", 10, &RRT::dynamic_callback, this);
    map_sub = nh_.subscribe("/map", 10, &RRT::map_callback, this);

    //layer publisher
    env_pub = nh_.advertise<nav_msgs::OccupancyGrid>("env_layer", 10);
    static_pub = nh_.advertise<nav_msgs::OccupancyGrid>("static_layer", 10);
    dynamic_pub = nh_.advertise<nav_msgs::OccupancyGrid>("dynamic_layer", 10);


    // ROS subscribers
    // TODO: create subscribers as you need
    //pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    //scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    //pf_sub_ = nh_.subscribe("/pf/pose/odom", 10, &RRT::pf_callback, this);
    
    // use /odom to test
    pf_sub_ = nh_.subscribe("/odom", 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe("/scan", 10, &RRT::scan_callback, this);
    //exeu_sub = nh_.subscribe("/pf/pose/odom", 10, &RRT::exeu_callback, this);


    // TODO: create a occupancy grid from  @hzheng

    // load map meta data
    boost::shared_ptr<nav_msgs::MapMetaData const> env_metadata_ptr;
    nav_msgs::MapMetaData env_metadata_msg;
    env_metadata_ptr = ros::topic::waitForMessage<nav_msgs::MapMetaData>("/map_metadata");
    if (env_metadata_ptr != NULL) {
        env_metadata_msg = *env_metadata_ptr;
    }
    all_map_metadata = env_metadata_msg;
    map_resolution = env_metadata_msg.resolution;
    //ROS_INFO_STREAM("resolution: " << map_resolution);
    map_width = env_metadata_msg.width;
    map_height = env_metadata_msg.height;
    map_origin = env_metadata_msg.origin;
    geometry_msgs::Point origin = map_origin.position;
    origin_x = origin.x;
    origin_y = origin.y;
    //ROS_INFO_STREAM("map x origin: " << origin_x);
    //ROS_INFO_STREAM("map y origin: " << origin_y);
    //ROS_INFO_STREAM(map_height);
    ROS_INFO("Map Metadata Loaded.");

    // store env layer data
    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
    nav_msgs::OccupancyGrid map_msg;
    // point to the map topic
    map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
    if(map_ptr != NULL) {
        map_msg = *map_ptr;
    }
    std::vector<int8_t> map_data = map_msg.data;
    // convert to int
    std::vector<int> map_data_int(map_data.begin(), map_data.end());
    int* data_start = map_data_int.data();
    // save data to attribute
    // map value 100 if occupied, 0 if free 
    // save map value to eigen matrix for env layer
    Eigen::Map<Eigen::MatrixXi>(data_start, env_layer.rows(), env_layer.cols()) = env_layer;
    ROS_INFO("Map in Eigen.");

    boost::shared_ptr<sensor_msgs::LaserScan const> laser_ptr;
    sensor_msgs::LaserScan laser_msg;
    laser_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan");
    if (laser_ptr != NULL) {
        laser_msg = *laser_ptr;
    }
    SCAN_COUNT = (laser_msg.ranges).size();
    angles_vector.reserve(SCAN_COUNT);
    current_scan.reserve(SCAN_COUNT);
    for (int i=0; i<SCAN_COUNT; i++) {
        angles_vector[i] = laser_msg.angle_min+laser_msg.angle_increment*i;
    }
    ROS_INFO("Laser Params Loaded.");

    // initialize env layer and static layer
    env_layer.resize(map_height, map_width);
    //ROS_INFO_STREAM(map_height);
    //ROS_INFO_STREAM(map_width);
    //ROS_INFO_STREAM(env_layer.size());
    //ROS_INFO_STREAM(env_layer(0,2047));
    env_layer.setZero();

    //ROS_INFO_STREAM(env_layer.rows());
    //ROS_INFO_STREAM(env_layer.cols());
    
    static_layer.resize(map_height, map_width);
    static_layer.setZero();
    //ROS_INFO_STREAM(map_height);
    //ROS_INFO_STREAM(map_width);

    dynamic_layer.resize(map_height, map_width);
    dynamic_layer.setZero();
    //ROS_INFO_STREAM(map_height);
    //ROS_INFO_STREAM(map_width);


    // making sure tf between map and laser is published before running
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/map", "/laser", now, ros::Duration(1.0));
    ROS_INFO("Transform arrived.");

    //ROS_INFO("Gridmap node object init done.");

    ROS_INFO("Created new RRT Object.");

    //load data file
    // read csv file data
        std::ifstream myfile;
        myfile.open("/home/gary/Desktop/data_file.csv",std::ios::in);
        //myfile.open("/home/gary/Desktop/test_data.csv",std::ios::in);
        std::vector<std::string> row;
        std::string line,word,temp;
        if(!myfile.is_open()){
             ROS_INFO_STREAM("cannot open file");
        }
        std::string c;
        double num;
        int count = 0;
        while(std::getline(myfile, line, ',')){
            // get x coordinate
            num  = std::stod(line);
            waypoint_x.push_back(num);
            //ROS_INFO_STREAM(num);
             
            std::getline(myfile,line,',');
            num = std::stod(line);
            waypoint_y.push_back(num);
            //ROS_INFO_STREAM(num);
         
            std::getline(myfile,line,',');
            num = std::stod(line);
            waypoint_angle.push_back(num);
            //ROS_INFO_STREAM(num);

            std::getline(myfile,line,'\n');
            num = std::stod(line);
            waypoint_speed.push_back(num);
            //ROS_INFO_STREAM(num);
            
            //if(count == 1){
            //    break;
            //}
            count++;
        }
    ROS_INFO_STREAM("data file read");

}


//  mapping funtion between pixel coord to xy coord
std::vector<int> RRT::ind_2_rc(int ind) {
    //[row, col]
    std::vector<int> rc;
    int row = floor(ind/map_width);
    //////////////////////////////////////////////////////////////////////////////////// change
    int col = ind%map_width;
    rc.push_back(row);
    rc.push_back(col);
    //ROS_INFO_STREAM("row: " << rc[0]);
    //ROS_INFO_STREAM("col: " << rc[1]);
    return rc;
}

geometry_msgs::Point RRT::cell_2_coord(int ind) {
    std::vector<int> rc = ind_2_rc(ind);
    geometry_msgs::Point coord;
    coord.x = origin_x + rc[1]*map_resolution;
    coord.y = origin_y + rc[0]*map_resolution;
    //if(rc[0] == 0){
        //ROS_INFO_STREAM("coordx in map: " << coord.x);
        //ROS_INFO_STREAM("coordy in map: " << coord.y);
    //}
    
    return coord;
}

// returns rc index
std::vector<int> RRT::coord_2_cell_rc(double x, double y) {
    std::vector<int> rc;
    int col = static_cast<int>((x - origin_x) / map_resolution);
    int row = static_cast<int>((y - origin_y) / map_resolution);
    rc.push_back(row);
    rc.push_back(col);
    return rc;
}
// env rviz
void RRT::env_callback(const nav_msgs::OccupancyGrid::ConstPtr& env_layer_msg){
    std::vector<int8_t> env_layer_raw = env_layer_msg->data;
    std::vector<int> env_layer(env_layer_raw.begin(), env_layer_raw.end());
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.type = marker.CUBE_LIST;
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;

    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 0.0;
    col.g = 0.0;
    col.b = 0.0;

    for (int i=0; i<env_layer.size(); i++){
        if (env_layer[i] != 0) {
            geometry_msgs::Point cube = cell_2_coord(i);
            marker.points.push_back(cube);
            marker.colors.push_back(col);
        }
    }
    env_viz_pub.publish(marker);
     
}

// static rviz
void RRT::static_callback(const nav_msgs::OccupancyGrid::ConstPtr& static_layer_msg){
    std::vector<int8_t> static_layer_raw = static_layer_msg->data;
    std::vector<int> static_layer(static_layer_raw.begin(), static_layer_raw.end());
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.type = marker.CUBE_LIST;
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;

    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 1.0;
    col.g = 0.0;
    col.b = 0.0;

    for (int i=0; i<static_layer.size(); i++){
        // threshold is 50
        if (static_layer[i] >= 50) {
            geometry_msgs::Point cube = cell_2_coord(i);
            marker.points.push_back(cube);
            marker.colors.push_back(col);
        }
    }
    static_viz_pub.publish(marker);
}

void RRT::dynamic_callback(const nav_msgs::OccupancyGrid::ConstPtr& dynamic_layer_msg) {
    std::vector<int8_t> dynamic_layer_raw = dynamic_layer_msg->data;
    std::vector<int> dynamic_layer(dynamic_layer_raw.begin(), dynamic_layer_raw.end());
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.type = marker.CUBE_LIST;
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;

    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 0.0;
    col.g = 0.0;
    col.b = 1.0;

    for (int i=0; i<dynamic_layer.size(); i++){
        if (dynamic_layer[i] != 0) {
            geometry_msgs::Point cube = cell_2_coord(i);
            marker.points.push_back(cube);
            marker.colors.push_back(col);
        }
    }
    dynamic_viz_pub.publish(marker);

}


void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    // TODO: update your occupancy grid

    std::vector<float> ranges = scan_msg->ranges;
    current_scan = ranges;
    // put scan into dynamic layer
    for (int i=0; i<SCAN_COUNT; i++) {
        double range = ranges[i];
        // ROS_INFO("here");
        if (std::isnan(range) || std::isinf(range)) continue;
        // these are in the frame of /laser
        double x = range*cos(angles_vector[i]), y = range*sin(angles_vector[i]);
        // transform into map frame
        geometry_msgs::PointStamped before_tf;
        before_tf.point.x = x;
        before_tf.point.y = y;
        before_tf.header.frame_id = "/laser";
        geometry_msgs::PointStamped after_tf;
        after_tf.header.frame_id = "/map";
        listener.transformPoint("/map", before_tf, after_tf);
        std::vector<int> laser_rc = coord_2_cell_rc(after_tf.point.x, after_tf.point.y);
        int laser_r = laser_rc[0];
        int laser_c = laser_rc[1];
        // check bounds
        if (out_of_bounds(laser_r, laser_c)) continue;
        // add inflation
        for (int i_f=-INFLATION; i_f<=INFLATION; i_f++) {
            for (int j_f=-INFLATION; j_f<=INFLATION; j_f++) {
                int current_r = laser_r - i_f, current_c = laser_c - j_f;
                if (out_of_bounds(current_r, current_c)) continue;
                // assignment value to 
                dynamic_layer(current_r, current_c) = 100;
            }
        }
    }

    int STATIC_THRESH = 50;
    for (size_t i_layer=0, layer_size = env_layer.size(); i_layer<layer_size; i_layer++) {
        //return pointer to the first element
        int *env = env_layer.data();
        int *dyn = dynamic_layer.data();
        int *stat = static_layer.data();
        if (env[i_layer] > 0 && dyn[i_layer] > 0) {
            dyn[i_layer] = 0;
        }
        if (dyn[i_layer] > 0 && stat[i_layer] < 100) {
            stat[i_layer]++;
        }
        if (env[i_layer] == 0 && dyn[i_layer] == 0 && stat[i_layer] > 0) {
            stat[i_layer]--;
        }
        if (dyn[i_layer] > 0 && stat[i_layer] >= STATIC_THRESH) {
            dyn[i_layer] = 0;
        }
        if (stat[i_layer] >= STATIC_THRESH && env[i_layer] > 0) {
            stat[i_layer] = 0;
        }
    }

    pub_layers();
    dynamic_layer.setZero();

}

// publish layers, default: no argument, publishes all current layer via attr
void RRT::pub_layers() {
    // env layer not needed, already done in map callback
    // static layer
    nav_msgs::OccupancyGrid static_layer_msg;
    static_layer_msg.info = all_map_metadata;
    // 
    std::vector<int> static_data(static_layer.data(), static_layer.data()+static_layer.size());
    // convert to int8
    std::vector<int8_t> static_data_int8(static_data.begin(), static_data.end());
    static_layer_msg.data = static_data_int8;

    // dynamic layer
    nav_msgs::OccupancyGrid dynamic_layer_msg;
    dynamic_layer_msg.info = all_map_metadata;
    std::vector<int> dynamic_data(dynamic_layer.data(), dynamic_layer.data()+dynamic_layer.size());
    // convert to int8
    std::vector<int8_t> dynamic_data_int8(dynamic_data.begin(), dynamic_data.end());
    dynamic_layer_msg.data = dynamic_data_int8;

    // publish
    static_pub.publish(static_layer_msg);
    dynamic_pub.publish(dynamic_layer_msg);
    env_pub.publish(env_layer_msg);
}



bool RRT::out_of_bounds(int r, int c){
    return (r < 0 || r >= map_height || c < 0 || c>= map_width);
}
void RRT::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
    // The map callback
    env_layer_msg = *map_msg;
    ROS_INFO("Map rerouted.");
}

///////////////////////////////////////////////////////////////////////////need to change type to
void RRT::pf_callback(const nav_msgs::Odometry::ConstPtr &pose_msg){
//void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    
    // tree as std::vector
    std::vector<Node> tree;
    // pursuit algorithm implementation
    double robot_x = pose_msg->pose.pose.position.x;
    double robot_y = pose_msg->pose.pose.position.y;
    double goal_x_map;
    double goal_y_map;
    double waypoint_x_robot; // the waypoint which satisfies the lookhead distance condition
    double waypoint_y_robot;

    //ROS_INFO_STREAM("robot_x in map: " << robot_x);
    //ROS_INFO_STREAM("robot_y in map: " << robot_y);
    // find goal point
    for(int i = 0; i < waypoint_x.size();i++){
        current_x = waypoint_x[i];
        current_y = waypoint_y[i];
        d = sqrt(pow(current_x-robot_x,2.0)+pow(current_y-robot_y,2.0));
        if(d > lookhead && d < (lookhead + 0.1)){
            // get transformed point in laser frame
            geometry_msgs::PointStamped before_tf;
            before_tf.point.x = current_x;
            before_tf.point.y = current_y;
            before_tf.header.frame_id = "/map";
            geometry_msgs::PointStamped after_tf;
            after_tf.header.frame_id = "/laser";
            listener.transformPoint("/laser",before_tf,after_tf);
            waypoint_x_robot = after_tf.point.x;
            waypoint_y_robot = after_tf.point.y;
            double angle_in_R = atan2(waypoint_y_robot,waypoint_x_robot);

            if(abs(angle_in_R) < (M_PI/2.0)){
                goal_x_map = current_x;
                goal_y_map = current_y;
                break;
            }
        }
    }
    // determine a global goal based on data file
    ROS_INFO_STREAM("goal_x in map: " << goal_x_map);
    ROS_INFO_STREAM("goal_y in map: " << goal_y_map);
    //ROS_INFO_STREAM("robot_x in map: " << robot_x);
    //ROS_INFO_STREAM("robot_y in map: " << robot_y);

    // publish goal point in map frame for testing with global goal
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = goal_x_map;
        marker.pose.position.y = goal_y_map;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;  
        viz_pub.publish(marker);
    
    
    // create root node
    Node root;
    root.x = robot_x;
    root.y = robot_y;
    root.is_root = true;
    root.cost = 0.0;
    root.parent = 0;
    tree.push_back(root);
    std::vector<Node> path;
    int max_iter = 1000;
    int count = 0;

    /*// TODO: fill in the RRT main loop
    for(int i = 0; i < max_iter;i++){
        std::vector<double> free_point = sample(robot_x,robot_y);
        int nearest_node_ind = nearest(tree,free_point);
        //ROS_INFO_STREAM("near node index: " << nearest_node_ind);
        // find nearest node
        Node nearest_node = tree[nearest_node_ind];
        Node new_node1 = steer(nearest_node,free_point,tree);
    
        // obstacle free between nearest and new node
        ////////////////////////////////////problem  here
        if(!check_collision(nearest_node,new_node1)){
            //ROS_INFO_STREAM("check collision process!");
            
            // connect new node and nearest node
            new_node1.parent = nearest_node_ind;
            new_node1.is_root = false;
            new_node1.cost = 0.0;

            // put new node into the tree
            tree.push_back(new_node1);
            //ROS_INFO_STREAM("new node x: " << new_node1.x);
            //ROS_INFO_STREAM("new node y: " << new_node1.y);
            //ROS_INFO_STREAM("goal_x in map: " << goal_x_map);
            //ROS_INFO_STREAM("goal_y in map: " << goal_y_map);
            if(is_goal(new_node1,goal_x_map,goal_y_map)){
                // find path if new node is close to goal
                path = find_path(tree,new_node1);
                ROS_INFO_STREAM("path found!");
                ROS_INFO_STREAM("path size: " << path.size());
                break;
            }
        }
        count++;
    }*/

    // TODO: fill in the RRT* main loop
    for(int i = 0; i < max_iter;i++){
        std::vector<double> free_point2 = sample(robot_x,robot_y);
        int nearest_node_ind = nearest(tree,free_point2);

        // find nearest node and new node
        Node nearest_node2 = tree[nearest_node_ind];
        Node new_node2 = steer(nearest_node2,free_point2,tree);

        // fill in the cost and check obstacle free
        if(!check_collision(nearest_node2,new_node2)){
            new_node2.parent = nearest_node_ind;
            // new node cost = cost(parent) + line cost 
            new_node2.cost = tree[nearest_node_ind].cost + line_cost(tree[nearest_node_ind],new_node2);
            tree.push_back(new_node2);
            // size might be zero based on radius defined !!!!!!!!!!!!!!!!
            std::vector<int> near_subset = near(tree,new_node2,nearest_node_ind);
            // initialize min node, min cost
            Node min_node = nearest_node2;
            double min_cost = nearest_node2.cost + line_cost(nearest_node2,new_node2);
            int min_index = 0;
            //ROS_INFO_STREAM("near_subset size: " << near_subset.size());
            // find each near_node in near_subset
            for(int i = 0; i < near_subset.size();i++){
                int near_node_index = near_subset[i];
                //extract one of near nodes in the tree
                Node near_node = tree[near_node_index];
                double cost = near_node.cost + line_cost(near_node,new_node2);
                if(!check_collision(near_node,new_node2) && (cost < min_cost)){////////////////////////////////////////////////////////////////here!!!!!!!!!!!!!!1
                    // update min node because cost is smaller for near_node
                    min_node = near_node;
                    min_cost = cost;
                    min_index = near_node_index;
                }
            }
            // update new node with its parent with min_node
            new_node2.parent = min_index;
            new_node2.cost = min_cost;
            
            // rewiring process
            for(int i = 0; i < near_subset.size();i++){
                int near_node_index = near_subset[i];
                Node near_node = tree[near_node_index];
                double cost = new_node2.cost + line_cost(new_node2,near_node);
                if(!check_collision(new_node2,near_node) && (cost < near_node.cost)){
                    // delete parent node of near and update with new node corresponding index in tree
                    // since new node is the latest index will be size() - 1
                    near_node.parent = tree.size() - 1;
                    near_node.cost = cost;
                }
            }
            if(is_goal(new_node2,goal_x_map,goal_y_map)){
                path = find_path(tree,new_node2);
                ROS_INFO_STREAM("path found!");
                ROS_INFO_STREAM("path size: " << path.size());
                break;
            }
        }
    }
        ROS_INFO_STREAM("tree size: " << tree.size());

        // visualize tree and path
        visualize_tree(tree);
        if(path.size() > 0) {
            visualize_path(path);
            visualize_line(path,robot_x,robot_y);
        }
        

        // execute the trajectory
        //need testing
        double local_lookhead = 1.0;
        double best_candidate_x; // best path point in robot frame
        double best_candidate_y;
        double angle_diff; // angle between path point and goal in robot frame
        double min_angle_diff = M_PI;
        double goal_angle; // goal angle in robot frame
        double best_x_before_tf;
        double best_y_before_tf;

        // transform global point to robot frame
        geometry_msgs::PointStamped goal_point_before;
        goal_point_before.point.x = goal_x_map;
        goal_point_before.point.y = goal_y_map;
        goal_point_before.header.frame_id = "/map";
        geometry_msgs::PointStamped goal_point_after;
        goal_point_after.header.frame_id = "/laser";
        listener.transformPoint("/laser",goal_point_before,goal_point_after);
        // calculate goal point angle in robot frame in positive form
        goal_angle = abs(atan2(goal_point_after.point.y,goal_point_after.point.x));

        for(int i = 0; i < path.size(); i++){
            Node path_point = path[i];
            double d = sqrt(pow(path_point.x - robot_x,2.0)+pow(path_point.y - robot_y,2.0));
            // transform global point to robot frame
            geometry_msgs::PointStamped path_point_before;
            path_point_before.point.x = path_point.x;
            path_point_before.point.y = path_point.y;
            path_point_before.header.frame_id = "/map";
            geometry_msgs::PointStamped path_point_after;
            path_point_after.header.frame_id = "/laser";
            listener.transformPoint("/laser",path_point_before,path_point_after);

            if(d < (local_lookhead+0.2)){
                // calculate each candidate path point angle in robot frame
                double path_angle = abs(atan2(path_point_after.point.y,path_point_after.point.x));
                angle_diff = abs(path_angle - goal_angle);
                // calculate closest path point angle to the goal point in a certain radius in robot frame
                if(angle_diff < min_angle_diff){
                    min_angle_diff = angle_diff;
                    best_candidate_x = path_point_after.point.x;
                    best_candidate_y = path_point_after.point.y;
                    best_x_before_tf = path_point.x;
                    best_y_before_tf = path_point.y;
                }
            }
        }

        // debug visulization for best path point selection
        /*visualization_msgs::Marker marker2;
        marker2.header.frame_id = "map";
        marker2.header.stamp = ros::Time();
        marker2.id = 0;
        marker2.type = visualization_msgs::Marker::SPHERE;
        marker2.pose.position.x = best_x_before_tf;
        marker2.pose.position.y = best_y_before_tf;
        marker2.pose.position.z = 0.0;
        marker2.pose.orientation.x = 0.0;
        marker2.pose.orientation.y = 0.0;
        marker2.pose.orientation.z = 0.0;
        marker2.pose.orientation.w = 1.0;
        marker2.scale.x = 0.3;
        marker2.scale.y = 0.3;
        marker2.scale.z = 0.3;
        marker2.color.a = 1.0; // Don't forget to set the alpha!
        marker2.color.r = 0.0;
        marker2.color.g = 1.0;
        marker2.color.b = 0.0;  
        local_viz_pub.publish(marker2);*/


        if(abs(best_candidate_y) < 0.05){
            best_candidate_y = 0.0;
        }

        double steering = 2.2*best_candidate_y/(pow(local_lookhead,2.0));

        if(steering <= -0.5189){
            steering = -0.5189;
        }
        if(steering >= 0.5189){
            steering = 0.5189;
        }
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = steering;
        drive_msg.drive.speed = 0.0;
        drive_pub.publish(drive_msg);
    
}
// can be tree or path
void RRT::visualize_line(std::vector<Node> & path,double robot_x, double robot_y){
   visualization_msgs::Marker marker;
   marker.header.frame_id = "/map";
   marker.type = marker.LINE_STRIP;
   marker.scale.x = 0.01;
   marker.scale.y = 0.01;
   marker.scale.z = 0.01;

   std_msgs::ColorRGBA col;
   col.a = 1.0;
   col.r = 0.0;
   col.g = 0.0;
   col.b = 1.0;
    // root node
    geometry_msgs::Point node1;
    node1.x = path[0].x;
    node1.y = path[0].y;
    geometry_msgs::Point node2;
    node2.x = robot_x;
    node2.y = robot_y;
    marker.points.push_back(node1);
    marker.colors.push_back(col);
    marker.points.push_back(node2);
    marker.colors.push_back(col);


   for(int i = 0; i < path.size()-1;i++){
       geometry_msgs::Point node1;
       node1.x = path[i].x;
       node1.y = path[i].y;
       geometry_msgs::Point node2;
       node2.x = path[i+1].x;
       node2.y = path[i+1].y;
       marker.points.push_back(node1);
       marker.colors.push_back(col);
       marker.points.push_back(node2);
       marker.colors.push_back(col);
   }
   line_viz_pub.publish(marker);
}

void RRT::visualize_tree(std::vector<Node> &tree){
   visualization_msgs::Marker marker;
   marker.header.frame_id = "/map";
   marker.type = marker.CUBE_LIST;
   marker.scale.x = 0.08;
   marker.scale.y = 0.08;
   marker.scale.z = 0.08;

   std_msgs::ColorRGBA col;
   col.a = 1.0;
   col.r = 0.0;
   col.g = 1.0;
   col.b = 0.0;
   
   for(int i = 0; i < tree.size();i++){
       geometry_msgs::Point sphere;
       sphere.x = tree[i].x;
       sphere.y = tree[i].y;
       marker.points.push_back(sphere);
       marker.colors.push_back(col);
   }
   tree_viz_pub.publish(marker);
}

void RRT::visualize_path(std::vector<Node> &path){
    visualization_msgs::Marker marker;
   marker.header.frame_id = "/map";
   marker.type = marker.CUBE_LIST;
   marker.scale.x = 0.08;
   marker.scale.y = 0.08;
   marker.scale.z = 0.08;

   std_msgs::ColorRGBA col;
   col.a = 1.0;
   col.r = 1.0;
   col.g = 1.0;
   col.b = 0.0;
   
   for(int i = 0; i < path.size();i++){
       geometry_msgs::Point sphere;
       sphere.x = path[i].x;
       sphere.y = path[i].y;
       marker.points.push_back(sphere);
       marker.colors.push_back(col);
   }
   path_viz_pub.publish(marker);

}


std::vector<double> RRT::sample(double robot_x, double robot_y) {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space
    
    double range_x = 3.0; // 3.0
    double range_y = 3.0;
    bool free_detect = false;
    double sample_x;
    double sample_y;
    bool reach_bounds = false;

    //if(robot_x - range_x < min_coord || robot_x + range_x > max_coord || )

    // take care of outliers !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    std::vector<double> sampled_point;
    // continue sampling until we get a free sample in the space
    while(!free_detect){

        // sampling range in a square range not checking out of bounds !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        std::uniform_real_distribution<double> x_dist(robot_x - range_x, robot_x + range_x);
        std::uniform_real_distribution<double> y_dist(robot_y - range_y, robot_y + range_y);
        // generate random point in map frame
        sample_x = x_dist(gen);
        sample_y = y_dist(gen);

        std::vector<int> sample_rc = coord_2_cell_rc(sample_x,sample_y);
        int sample_r = sample_rc[0];
        int sample_c = sample_rc[1];

        if(out_of_bounds(sample_r,sample_c)) continue;
        // check in layer to see if it is free
        if(env_layer(sample_r,sample_c) == 0 && dynamic_layer(sample_r,sample_c) == 0 && static_layer(sample_r,sample_c) == 0){
            free_detect = true;
        }
    }
    sampled_point.push_back(sample_x);
    sampled_point.push_back(sample_y);
    
    
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

    return sampled_point;
}


int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node;
    // TODO: fill in this method
    int nearest_index;
    double node_distance;
    double min_distance = 10000.0;
    
    // loop through the current RRT tree to find a nearest node
    for(int i = 0; i < tree.size();i++){
        //loop through each node in a tree
        Node object = tree[i];
        //find min distance
        node_distance = sqrt(pow(sampled_point[0] - object.x,2.0) + pow(sampled_point[1] - object.y,2.0));
        if(node_distance < min_distance){
            min_distance = node_distance;
            nearest_index = i;
        }
    }
    // store nearest node
    //nearest_node = tree[nearest_index];
    nearest_node = nearest_index;

    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point, std::vector<Node> &tree) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering

    Node new_node;
    // TODO: fill in this method

    /*// bascially take mid point between sampled point and nearest_node
    double sample_x = sampled_point[0];
    double sample_y = sampled_point[1];

    double mid_x = (sample_x - nearest_node.x)/2.0;
    double mid_y = (sample_y - nearest_node.y)/2.0;

    // fill in new node coord in map frame
    new_node.x = mid_x;
    new_node.y = mid_y;
    */

    // second approach treat sampling point as new node
    double sample_x = sampled_point[0];
    double sample_y = sampled_point[1];
    new_node.x = sample_x;
    new_node.y = sample_y;



    return new_node;
}

bool RRT::check_layers_collision(int r, int c){
    return (static_layer(r,c) > 0 || dynamic_layer(r,c) > 0 || env_layer(r,c) > 0);
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    //ROS_INFO_STREAM("inside check collision function");
    bool collision = false;
    // TODO: fill in this method

    double interpolate_x;
    double interpolate_y;
    // extract xy coord in nodes
    double near_x = nearest_node.x;
    double near_y = nearest_node.y;
    double new_x = new_node.x;
    double new_y = new_node.y;

    std::vector<int> inter_rc;
    int inter_r;
    int inter_c;
    int count = 0;
    double step_size = 0.02;

    // perform interpolation between two nodes
    for(double t = 0; t < 1.0; t+= step_size){
        // check collision for each interpolating point
        //ROS_INFO_STREAM("inside check collision function loop");
        interpolate_x = (1.0 - t)*near_x + t * new_x;
        interpolate_y = (1.0 - t)*near_y + t * new_y;
        inter_rc = coord_2_cell_rc(interpolate_x,interpolate_y);
        inter_r = inter_rc[0];
        inter_c = inter_rc[1];
        //ROS_INFO_STREAM("t: " << t);
        if(check_layers_collision(inter_r,inter_c)) {
            collision = true;
            break;
        }
    }

    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method
    double latest_added_x = latest_added_node.x;
    double latest_added_y = latest_added_node.y;

    if(sqrt(pow(latest_added_x-goal_x,2.0)+pow(latest_added_y-goal_y,2.0)) < goal_threshold)
        close_enough = true;

    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<Node> found_path;
    int parent_index;

    // TODO: fill in this method
    // complete a search in tree based on parents
    // from goal to beginning
   /* Node exam_point = latest_added_node;
    while(!exam_point.is_root){
        found_path.push_back(exam_point);
        // extract parent index of that exam node
        parent_index = exam_point.parent;
        // update to next exam point
        exam_point = tree[parent_index];
    }*/

    //Node exam_point = latest_added_node;
    found_path.push_back(latest_added_node);
    Node exam_point = tree[latest_added_node.parent];

    while(!exam_point.is_root){
        found_path.push_back(exam_point);
        parent_index = exam_point.parent;
        exam_point = tree[parent_index];
    }

    // reverse direction to let root stays on top
    std::reverse(found_path.begin(),found_path.end());
    return found_path;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method
    double parent_cost = 0;
    // cost calculation = cost(parent) + line_cost(n1,n2)
    int parent_index = node.parent;
    // extract parent cost
    parent_cost = tree[parent_index].cost;
    // calculate cost of that node
    cost = parent_cost + line_cost(tree[parent_index],node);
    
    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method
    // take eculidian distance as line cost
    double distance_cost = sqrt(pow(n1.x - n2.x,2.0) + pow(n1.y - n2.y,2.0));
    cost = distance_cost;

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node, int &nearest_node_ind) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    neighborhood.push_back(nearest_node_ind);
    // TODO:: fill in this method

    int near_index;
    double near_radius;
    for(int i = 0; i < tree.size();i++){
        if( i != nearest_node_ind){
            // calculate distance for each node between
            double distance = sqrt(pow(tree[i].x - node.x,2.0) + pow(tree[i].y - node.y,2.0));
            // store near index node
            near_radius = calculate_radius(tree,node,tree[nearest_node_ind]);
            if(distance < near_radius) 
            neighborhood.push_back(i);
        }
    }

    return neighborhood;
}

double RRT::calculate_radius(std::vector<Node> &tree, Node &new_node, Node &nearest_node){
    // This method return radius which determines X near node set
    // Args:
    //  tree (std::vector<Node>): the current tree
    //  new_node (Node): new node obtained from steering function
    //  nearest_node (Node): nearest node obtained from nearest() function
    // Returns:
    //  radius (double): radius for X near subset

    // take mid radius between nearest node and farest node in a tree
    double radius;
    double lower_threshold = sqrt(pow(new_node.x - nearest_node.x,2.0) + pow(new_node.y - nearest_node.y,2.0));
    double upper_threshold;
    double max_index;
    double max_distance = 0.0;
    // determine upper_threshold for radius
    for(int i = 0; i < tree.size();i++){
        double distance = sqrt(pow(tree[i].x - new_node.x,2.0) + pow(tree[i].y - new_node.y,2.0));
        if(distance > max_distance){
            //store max distance index node
            max_index = i;
            max_distance = distance;
        }
    }
    upper_threshold = max_distance;
    radius = (upper_threshold + lower_threshold) / 2.0;

    return radius;

}