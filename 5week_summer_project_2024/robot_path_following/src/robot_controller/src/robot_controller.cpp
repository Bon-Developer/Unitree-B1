#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// Node 구조체 정의
struct Node {
    int id;
    float x;
    float y;
};


//이미지맵 불러오는 함수
// void loadMapFromImage(const std::string& image_path, nav_msgs::OccupancyGrid& map) {
//     // Load image
//     cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
//     if (image.empty()) {
//         ROS_ERROR("Failed to load image: %s", image_path.c_str());
//         return;
//     }

//     // Resize image to desired map size
//     cv::resize(image, image, cv::Size(423, 160)); // Adjust the size as needed 423, 160

//     map.info.resolution = 0.05; // Set resolution (meters per cell)
//     map.info.width = image.cols;
//     map.info.height = image.rows;
//     map.info.origin.position.x = 0;
//     map.info.origin.position.y = 0;
//     map.info.origin.position.z = 0;
//     map.info.origin.orientation.w = 1.0;
    
//     // Convert image to OccupancyGrid data
//     map.data.resize(map.info.width * map.info.height);
//     for (int y = 0; y < map.info.height; y++) {
//         for (int x = 0; x < map.info.width; x++) {
//             int i = x + (map.info.height - y - 1) * map.info.width;
//             if (image.at<uchar>(y, x) <= 128) { // Change to <= 128 for black pixels as walls
//                 map.data[i] = 100; // Occupied
//             } else {
//                 map.data[i] = 0; // Free
//             }
//         }
//     }
// }

// CSV 파일을 읽어 경로를 벡터로 반환하는 함수
std::vector<Node> readPathFromFile(const std::string& filename) {
    std::vector<Node> pathNodes;
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return pathNodes;
    }

    std::string line;
    std::getline(file, line);  // 헤더 무시
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        Node node;
        char comma;
        ss >> node.id >> comma >> node.x >> comma >> node.y;
        pathNodes.push_back(node);
    }

    file.close();

    std::reverse(pathNodes.begin(), pathNodes.end());
    return pathNodes;
}

// 경로를 smoothing하는 함수
std::vector<Node> smoothPath(const std::vector<Node>& pathNodes, int iterations = 5) {
    if (pathNodes.size() <= 2) return pathNodes; // 너무 짧으면 smoothing 불가

    std::vector<Node> smoothedPath = pathNodes;

    for (int it = 0; it < iterations; ++it) {
        std::vector<Node> tempPath = smoothedPath;

        for (size_t i = 1; i < tempPath.size() - 1; ++i) {
            Node prev = tempPath[i - 1];
            Node curr = tempPath[i];
            Node next = tempPath[i + 1];

            // Moving average 필터 적용 (3개의 연속된 점의 평균)
            curr.x = (prev.x + curr.x + next.x) / 3.0f;
            curr.y = (prev.y + curr.y + next.y) / 3.0f;

            smoothedPath[i] = curr;
        }
    }

    return smoothedPath;
}


// smoothed path를 시각화하는 함수
void visualizePath(const std::vector<Node>& pathNodes, ros::Publisher& marker_pub) {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "path";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.1;

    // Set path color and alpha for better visibility
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;

    for (const auto& node : pathNodes) {
        geometry_msgs::Point p;
        p.x = node.x;
        p.y = node.y;
        p.z = 0;

        line_strip.points.push_back(p);
    }

    marker_pub.publish(line_strip);
}

class OdomStorage
{
    public:
        //OdomStorage() : current_odom_idx_(0) {}

        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            current_odom_ = *msg;
            tf::Quaternion quat(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);

            tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_); //quaternion을 이용하여 roll, pitch, yaw 값 추출
        }

        const nav_msgs::Odometry& getOdomData() const
        {
            return current_odom_;
        }

        double getYaw() const
        {
            return yaw_;
        }

    private:
        nav_msgs::Odometry current_odom_;
        size_t current_odom_idx_;
        double roll_, pitch_, yaw_;
};

// 로봇을 움직이는 함수
void moveRobot(const std::vector<Node>& pathNodes, ros::Publisher& twist_pub, ros::Publisher& marker_pub, OdomStorage& odom_storage) 
{
    size_t current_node_index = 0;

    ros::Rate rate(10); //10Hz
    while (ros::ok() && current_node_index < pathNodes.size())
    {
        Node curr = pathNodes[current_node_index];

        float curr_x_odom = odom_storage.getOdomData().pose.pose.position.x; //오도메트리 x 좌표
        float curr_y_odom = odom_storage.getOdomData().pose.pose.position.y; //오도메트리 y 좌표
        float curr_yaw_odom = odom_storage.getOdomData().pose.pose.orientation.z; //오도메트리 yaw 데이터

        //필터링 추가

        //float curr_yaw_odom = odom_storage.getYaw(); //오도메트리 yaw 속도 좌표
        float distance_to_node = sqrt(pow(curr_x_odom - curr.x, 2) + pow(curr_y_odom - curr.y, 2)); //오도메트리 정보와 경로 정보 사이 거리 계산


        if (distance_to_node < 0.2) //목표 지점 도달했을 경우
        {
            ROS_INFO("Reached Node ID: %d, moving to next node", curr.id);
            current_node_index++;

            if (current_node_index >= pathNodes.size())
            {
                ROS_INFO("Reached final destination. Stopping the robot.");
                break;
            }
        }

        else //목표 지점으로 이동 , 목표 위치와 현재 정보의 거리에 따라 dx, dy 변화
        {

            float dx = curr.x - curr_x_odom;
            float dy = curr.y - curr_y_odom;

            float distance = sqrt(dx * dx + dy * dy);
            float desired_yaw = atan2(dy, dx); //목표 지점의 Yaw 값 계산
            float yaw_error = desired_yaw - curr_yaw_odom;

            //Yaw error를 -pi, +pi 범위로 정규화
            while(yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
            while(yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

            // Yaw error가 크면 전진 속도를 줄이기 위함
            float speed = std::min(distance, 0.2f); 

            geometry_msgs::Twist twist_msg;
            twist_msg.linear.x = speed;
            twist_msg.angular.z = yaw_error;

            twist_pub.publish(twist_msg);

            ROS_INFO("Approaching Node ID: %d, position(%f, %f) with distance: %f, speed: %f, desired_yaw: %f, curr_yaw_odom: %f, yaw_error: %f", curr.id, curr.x, curr.y, distance_to_node, speed, desired_yaw, curr_yaw_odom, yaw_error);
        
            // 로봇 위치를 시각화하기 위해 마커를 사용
            // visualization_msgs::Marker rob;
            // rob.type = visualization_msgs::Marker::CUBE;
            // rob.header.frame_id = "map";
            // rob.header.stamp = ros::Time::now();
            // rob.ns = "rob";
            // rob.id = 0;
            // rob.action = visualization_msgs::Marker::ADD;
            // rob.lifetime = ros::Duration();
            // rob.scale.x = 0.5;
            // rob.scale.y = 1;
            // rob.scale.z = 0.25;
            // rob.pose.orientation.w = 1;
            // rob.color.r = 1.0f;
            // rob.color.g = 0.5f;
            // rob.color.b = 0.5f;
            // rob.color.a = 1.0;
            // // rob.pose.position.x = next.x;
            // // rob.pose.position.y = next.y;
            // rob.pose.position.z = 0;

            // marker_pub.publish(rob);
        }
        ros::spinOnce();
        rate.sleep();
    }
    // 마지막 위치에서 멈추기 위해 속도 0으로 설정
    while(ros::ok())
    {
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0;
        stop_msg.angular.z = 0;
        twist_pub.publish(stop_msg);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle n;

    ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    static ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

    OdomStorage odom_storage;
    ros::Subscriber odom_sub = n.subscribe("/b1_controller/odom", 1000, &OdomStorage::odomCallback, &odom_storage);

    //std::string path_filename = "/home/lsy/catkin_ws/src/rrt-ros/src/rrt-planning/src/final_path_0.21.csv"; // file path
    //std::string path_filename = "/home/kriso/ros/5week_project/catkin_ws_ver4/src/robot_controller/final_path_ver3.csv"; // file path
    //std::string path_filename = "/home/kriso/ros/5week_project/catkin_ws_ver8/src/robot_controller/final_path_ver5_preprocess.csv"; // file path
    std::string path_filename = "/home/kriso/ros/5week_project/catkin_ws_ver8/src/robot_controller/final_path_ver6_preprocess.csv";
    std::vector<Node> pathNodes = readPathFromFile(path_filename);



    // PNG 파일에서 OccupancyGrid 맵을 로드
    nav_msgs::OccupancyGrid map;
    //loadMapFromImage("/home/lsy/catkin_ws/src/rrt-ros/src/rrt-planning/src/20240730_3.png", map);
    map_pub.publish(map);

    if (pathNodes.empty()) 
    {
        ROS_ERROR("Path is empty. Exiting...");
        return 1;
    }

    std::vector<Node> smoothedPath = smoothPath(pathNodes);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        visualizePath(smoothedPath, marker_pub);  // Smoothed path 시각화
        moveRobot(smoothedPath, twist_pub, marker_pub, odom_storage);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
