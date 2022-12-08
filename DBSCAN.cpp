#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include <stdlib.h>
#include <stack>
#include<vector>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/LaserScan.h>
#include <cstdlib> // for rand() and srand()
#include <ctime> // for time()
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


const int resolution = 5;                            // resolution for local map
const u_int8_t minPts = 2;
const u_int8_t Eps = 1;                             // 5cm
const u_int8_t CORE = 1;
const u_int8_t NOISE = 2;
const u_int8_t BORDER = 3;
const float THRESHHOLD = 1.42;
bool flag_1 = false;
bool flag_2 = false;


struct odom{
    double x,y,theta;
};

struct Cell{
    int x,y;
    float dist;
};

struct node{
    Cell pos;
    u_int8_t color=0;
    u_int8_t label=0;
    u_int8_t clusterID = 0;
    bool b_visited = false;
};

nav_msgs::OccupancyGrid localmap_;
nav_msgs::OccupancyGrid testmap_;
odom position;
std::vector<Cell> lidar_;
Cell p0;
visualization_msgs::Marker points;
visualization_msgs::MarkerArray list_of_polygons;


void update_localmap(nav_msgs::OccupancyGrid &localmap, std::vector<Cell> &data);
void find_cluster(node* temp, nav_msgs::OccupancyGrid &localmap, std::vector<Cell> num_point, u_int8_t EPS, u_int8_t minPts, int max_width, int max_height, int min_width ,int min_height);
float distance(node *a, node b);
float distance(Cell a, Cell b);
int random(int minN, int maxN);
void convexHull(Cell points[], int n, nav_msgs::OccupancyGrid &localmap);
int compare(const void *vp1, const void *vp2);
int orientation(Cell p, Cell q, Cell r);
int distSq(Cell p1, Cell p2);
Cell nextToTop(std::stack<Cell> &S);
void swap(Cell &p1, Cell &p2);
void polygon_centroid_detection(std::vector<Cell> list, double &a, double &b, double &min_dist);
void draw_marker(nav_msgs::OccupancyGrid localmap, double a, double b, double min_dist);


void get_odom(const nav_msgs::Odometry::ConstPtr& odom){
    position.x = odom->pose.pose.position.x*100 + 500;
    position.y = odom->pose.pose.position.y*100 + 500;
    position.theta = tf::getYaw(odom->pose.pose.orientation);
    flag_2 = true;
}

void get_lidar_data(const sensor_msgs::LaserScan::ConstPtr& scan){
    if(flag_2 == true){
        lidar_.clear();
        int num = int((scan->angle_max - scan->angle_min)/(scan->angle_increment));
        for(int i=0; i < (num-1); i++){
            if(scan->ranges[i] > 0){
                Cell cell;
                double angle = position.theta + scan->angle_max - (i * scan->angle_increment);
                cell.x = int((scan->ranges[i]*cos(angle)*100 + (position.x) + 28.7*cos(position.theta))/resolution);
                cell.y = int((scan->ranges[i]*sin(angle)*100 + (position.y) + 28.7*sin(position.theta))/resolution);
                // i = i + 3;
                lidar_.push_back(cell);
            }
        }
        flag_1 = true;
    }
}

int main(int argc, char** argv){
    clock_t start, end; 
    double time_use; 


    //Define Parameter
    // Inint Localmap param
    localmap_.info.width = 400/resolution;
    localmap_.info.height = 400/resolution;
    localmap_.data.resize(localmap_.info.width*localmap_.info.height);
    localmap_.info.resolution = resolution;
    // 

    testmap_.info.width = 400/resolution;
    testmap_.info.height = 400/resolution;
    testmap_.data.resize(localmap_.info.width*localmap_.info.height);
    testmap_.info.resolution = resolution;



    ros::init(argc, argv, "DBSCAN");
    ros::NodeHandle n;
    ros::Subscriber odom_sub = n.subscribe("odom",10,get_odom);
    ros::Subscriber lidar_sub = n.subscribe("scan1",100,get_lidar_data);
    ros::Publisher localmap_pub = n.advertise<nav_msgs::OccupancyGrid>("localmap", 10);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
    ros::Publisher marker_pub_test = n.advertise<visualization_msgs::Marker>("visualization_marker_test", 10);    
    ros::Rate r(10); //0.2s
    
    // Marker intalization
    //----------------------------------------------------
    points.header.frame_id = "map";
    points.action = visualization_msgs::Marker::ADD;
    points.type = visualization_msgs::Marker::CYLINDER;
    geometry_msgs::Pose pose;
    points.scale.x = 0.5;
    points.scale.y = 0.5;
    points.scale.z = 0.5;
    points.color.g = 1.0f;
    points.color.a = 1.0;
    points.pose.orientation.x = 0;
    points.pose.orientation.y = 0;
    points.pose.orientation.z = 0.0;
    points.pose.orientation.w = 1.0;
    points.scale.z = 0.001;
    points.pose.position.z = 0.0;
    points.lifetime = ros::Duration();
    //-----------------------------------------------------

    while(n.ok()){
        ros::spinOnce();
        points.id = 0;
        if(flag_1 == true){
            list_of_polygons.markers.clear();
            start = clock();
            testmap_ = localmap_;
            update_localmap(testmap_, lidar_);
            end = clock();
            time_use = (double)(end - start) / CLOCKS_PER_SEC;
            ROS_INFO("%f", time_use);
            flag_1 = false;
            localmap_pub.publish(testmap_);
            marker_pub.publish(list_of_polygons);
        }
        // marker_pub_test.publish(points);
        r.sleep();
    }
}

void update_localmap(nav_msgs::OccupancyGrid &localmap, std::vector<Cell> &data){
    std::vector<Cell> num_point;
    node *temp = nullptr;
    temp = new node[localmap.info.width * localmap.info.height];
    localmap.info.origin.position.x = int((position.x)/resolution - (localmap.info.width/2));
    localmap.info.origin.position.y = int((position.y)/resolution - (localmap.info.height/2));
    int max_width = localmap.info.origin.position.x + localmap.info.width;
    int max_height = localmap.info.origin.position.y + localmap.info.height;
    int min_width = localmap.info.origin.position.x;
    int min_height = localmap.info.origin.position.y;
    for(u_int16_t i=0; i<data.size();i++){
        if((data[i].x > localmap.info.origin.position.x) && (data[i].y > localmap.info.origin.position.y) && (data[i].x < max_width) && (data[i].y < max_height)){
            data[i].y = data[i].y - min_height;
            data[i].x = data[i].x - min_width;
            if(temp[data[i].y * localmap.info.width + data[i].x].color != 100){
                num_point.push_back(data[i]);
                temp[data[i].y * localmap.info.width + data[i].x].color = 100;
                temp[data[i].y * localmap.info.width + data[i].x].pos = data[i];
            }        
        }
    }
    // for(int i=0; i<num_point.size();i++){
    //     localmap.data[num_point[i].y *localmap.info.width + num_point[i].x] = 140;
    // }

    find_cluster(temp, localmap, num_point, Eps, minPts, max_width, max_height, min_width, min_height);
}

void find_cluster(node* temp, nav_msgs::OccupancyGrid &localmap, std::vector<Cell> num_point, u_int8_t EPS, u_int8_t minPts, int max_width, int max_height, int min_width ,int min_height){
    srand((int)time(0));
    int count,r, cell;
    u_int8_t ID = 1;
    std::vector<node> near_point, list_of_node;
    std::vector<Cell> cell_segment;
    std::vector<node> cb_point;
    std::vector<std::vector<Cell>> list;
    node *current = nullptr;

    for(u_int16_t i=0; i<num_point.size(); i++){
        count = 0;
        near_point.clear();
        for(int8_t a = -EPS; a <= EPS; a++){
            for(int8_t b = -EPS; b <= EPS; b++){
                Cell temp1;
                temp1.x = num_point[i].x + a;
                temp1.y = num_point[i].y + b;
                int16_t cell = temp1.y * localmap.info.width + temp1.x;
                if((temp1.x >= 0) && (temp1.x <= (max_width - min_width)) && (temp1.y >= 0) && (temp1.y <= (max_height - min_height)) && temp[cell].color == 100){
                        count++;
                        near_point.push_back(temp[cell]);
                }
            }
        }
        cell = num_point[i].y * localmap.info.width + num_point[i].x;
        if(count > minPts){
            temp[cell].label = CORE;
            cb_point.push_back(temp[cell]);
        }
        else if(count <= minPts){
            for(u_int8_t s = 0; s < near_point.size(); s++){
                if(near_point[s].label == CORE){
                    temp[cell].label = BORDER;
                    cb_point.push_back(temp[cell]);
                    break;
                }
            }
        }
    }

    while(cb_point.size() > 0){
        bool alone = false;
        r = random(1,cb_point.size())-1;
        list_of_node.clear();
        if(temp[cb_point[r].pos.y * localmap.info.width + cb_point[r].pos.x].label == CORE){
            current = &temp[cb_point[r].pos.y * localmap.info.width + cb_point[r].pos.x];
            current->clusterID = ID;
            list_of_node.push_back(*current);
            cell_segment.push_back(current->pos);
            
            while(list_of_node.size() != 0){                                  
                for(u_int8_t i=0; i<cb_point.size();i++){
                    int t = cb_point[i].pos.y * localmap.info.width + cb_point[i].pos.x;
                    if(temp[t].b_visited == false){                 // this line has a problem 
                        if((temp[t].clusterID == 0) && (distance(current, temp[t]) <= THRESHHOLD) && (distance(&temp[cb_point[r].pos.y * localmap.info.width + cb_point[r].pos.x],temp[t]) <= 2)){
                            if((temp[t].label == CORE)){
                                temp[t].clusterID = ID;
                                temp[t].b_visited = true;
                                list_of_node.push_back(temp[t]);
                                cell_segment.push_back(temp[t].pos);
                            }
                            else if(temp[t].label == BORDER){
                                temp[t].clusterID = ID;
                                temp[t].b_visited = true;
                                cell_segment.push_back(temp[t].pos);
                            }
                        }
                    alone = true;
                    }
                }
                list_of_node.erase(list_of_node.begin());       // xoa phan tu dau tien ra khoi vector
                if(list_of_node.size() != 0){
                    current = &list_of_node[0];
                }
            }
            if(alone == false){
                cb_point.erase(cb_point.begin()+r);
            }else{
                ID++;
                list.push_back(cell_segment);
                cell_segment.clear();
                bool flag = false;
                while(flag == false){
                    flag = true;
                    for(u_int i=0; i < cb_point.size(); i++){
                        if(temp[cb_point[i].pos.y * localmap.info.width + cb_point[i].pos.x].clusterID != 0){
                            cb_point.erase(cb_point.begin()+i);
                            flag = false;
                            break;
                        }
                    }
                }
            }
        }
        else{
            bool flag5 = false;
            for(u_int l = 0; l < cb_point.size(); l++){
                if(temp[cb_point[l].pos.y * localmap.info.width + cb_point[l].pos.x].label == CORE){
                    flag5 = true;
                }
            }
            if(flag5 == false){
                break;
            }
        }

    }
    // ROS_INFO("%ld",cb_point.size());
    // ROS_INFO("%ld",list.size());

    int color = 0;
    for(u_int8_t i=0; i<list.size();i++){
        color += 50;
        if(list[i].size() >= 3){
            int n = list[i].size();
            Cell points[n];
            for(u_int8_t t=0; t<list[i].size();t++){
                points[t].x = list[i][t].x;
                points[t].y = list[i][t].y;
                localmap.data[list[i][t].y * localmap.info.width + list[i][t].x] = color;
            }
            // ROS_INFO("---");
            convexHull(points,n, localmap);
        }
    }
}

float distance(node *a, node b){
    return sqrtf(pow(a->pos.x - b.pos.x,2) + pow(a->pos.y - b.pos.y,2));
}

float distance_E(Cell a, Cell b){
    return sqrtf(pow(a.x - b.x,2) + pow(a.y - b.y,2));
}

int random(int minN, int maxN){
    return minN + rand() % (maxN + 1 - minN);
}

Cell nextToTop(std::stack<Cell> &S)
{
    Cell p = S.top();
    S.pop();
    Cell res = S.top();
    S.push(p);
    return res;
}

void swap(Cell &p1, Cell &p2)
{
    Cell temp = p1;
    p1 = p2;
    p2 = temp;
}

int distSq(Cell p1, Cell p2)
{
    return (p1.x - p2.x)*(p1.x - p2.x) +
          (p1.y - p2.y)*(p1.y - p2.y);
}


int orientation(Cell p, Cell q, Cell r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
 
    if (val == 0) return 0;  // collinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

int compare(const void *vp1, const void *vp2)
{
   Cell *p1 = (Cell *)vp1;
   Cell *p2 = (Cell *)vp2;
 
   // Find orientation
   int o = orientation(p0, *p1, *p2);
   if (o == 0)
     return (distSq(p0, *p2) >= distSq(p0, *p1))? -1 : 1;
 
   return (o == 2)? -1: 1;
}

void convexHull(Cell points[], int n, nav_msgs::OccupancyGrid &localmap)
{  
        std::vector<Cell> list;
    int ymin = points[0].y, min = 0;
    for (int i = 1; i < n; i++)
    {
        int y = points[i].y;
        if ((y < ymin) || (ymin == y &&
            points[i].x < points[min].x))
            ymin = points[i].y, min = i;
    }
    
    swap(points[0], points[min]);
    p0 = points[0];
    qsort(&points[1], n-1, sizeof(Cell), compare);
    
    int m = 1; // Initialize size of modified array
    for (int i=1; i<n; i++)
    {
        while (i < n-1 && orientation(p0, points[i],
                                        points[i+1]) == 0)
            i++;
    
    
        points[m] = points[i];
        m++;  // Update size of modified array
    }
    if (m < 3){
        std::stack<Cell> S;
        S.push(points[0]);
        S.push(points[1]);
        double a = (points[0].x + points[1].x)/2;
        double b = (points[0].y + points[1].y)/2;
        double min_dist = distance_E(points[0], points[1]);
        if(min_dist<10){
            draw_marker(localmap, a, b, min_dist);
        }
        

    }else{
        std::stack<Cell> S;
        S.push(points[0]);
        S.push(points[1]);
        S.push(points[2]);
        
        for (int i = 3; i < m; i++)
        {
            while (S.size()>1 && orientation(nextToTop(S), S.top(), points[i]) != 2)
                S.pop();
            S.push(points[i]);
        }
        while (!S.empty())
        {
            Cell p = S.top();
            list.push_back(p);
            S.pop();
        }
        double a, b, min_dist;
        polygon_centroid_detection(list,a,b, min_dist);
        if(min_dist<10){
            draw_marker(localmap, a, b, min_dist);
        }
    }



}

void polygon_centroid_detection(std::vector<Cell> list, double &a, double &b, double &min_dist){
    double signedArea = 0;
    double temp_first;
    double temp_second;
    Cell p;
    list.push_back(list[0]);
    for(u_int8_t i=0; i < list.size()-1; i++){
        double x_0 = list[i].x;double x_1 = list[i+1].x;
        double y_0 = list[i].y;double y_1 = list[i+1].y;
        double A = (x_0 * y_1) - (x_1 * y_0);
        signedArea += A;
        temp_first += (x_0 + x_1) * A;
        temp_second += (y_0 + y_1) * A;
    }
    signedArea *= 0.5;
    a = temp_first / (6*signedArea);
    b = temp_second / (6*signedArea);
    p.x = a;
    p.y = b;
    for(u_int8_t i=0; i < list.size()-1; i++){
        float min = distance_E(p,list[i]);
        float temp = distance_E(p,list[i+1]);
        if(temp > min){
            min_dist = temp;
        }
    }
}
                
void draw_marker(nav_msgs::OccupancyGrid localmap, double a, double b, double min_dist){
    points.id = points.id + 1;
    points.pose.position.x = a * resolution + localmap.info.origin.position.x;
    points.pose.position.y = b * resolution + localmap.info.origin.position.y;
    points.scale.x = (min_dist+2)*10;
    points.scale.y = (min_dist+2)*10;
    list_of_polygons.markers.push_back(points);
}

