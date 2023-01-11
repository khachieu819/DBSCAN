#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <stack>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

const float PI = 3.14;
const float resolution = 5;
const u_int16_t L_WIDTH = 250;
const u_int16_t L_HEIGHT = 250;

const float epsilon = 1.4;
const float THRESHHOLD = 1.4;
const u_int8_t minPts = 2;

const u_int8_t CORE = 1;
const u_int8_t BORDER = 2;
const u_int8_t NOISE = 3;



bool isOdomCallback = false;
bool isLidarCallback_1 = false;
bool isLidarCallback_2 = false;
bool isLidarCallback_3 = false;
//------------------------------ STRUCT DATA --------------------------------
//---------------------------------------------------------------------------
struct Cell{
    float x, y;
};

struct Cell_int{
    int x, y;
};

struct Odom{
    Cell position;
    float orientation;
};

struct node{
    Cell pos;
    u_int8_t color=0;
    u_int8_t label=0;
    u_int8_t clusterID = 0;
    bool b_visited = false;
};

struct polygonAttribute{
    std::vector<Cell> edgePoints;
    Cell pointCentroid;
    u_int8_t R;
};

//--------------------------- GLOBAL PARAMS ----------------------------
//----------------------------------------------------------------------
nav_msgs::OccupancyGrid localmap_;
nav_msgs::OccupancyGrid publishmap;
Odom pose, odomLast;
std::vector<Cell> lidar_;
bool firstTime = true;
float delta_x = 0, delta_y = 0;
Cell p0;
visualization_msgs::Marker points;
visualization_msgs::MarkerArray list_of_polygons;
std::vector<polygonAttribute> listPolygon;
//----------------------------------------------------------------------
void setupLocalmap();
void updateLocalMap(std::vector<Cell> data, nav_msgs::OccupancyGrid localmapTemp_);
float distance(node *a, node b);
float distanceCell(Cell a, Cell b);
int random(int minN, int maxN);
void find_cluster(node* temp, nav_msgs::OccupancyGrid &map, std::vector<Cell> points);
void convertData(nav_msgs::OccupancyGrid localmap);
void convexHull(Cell points[], int n, nav_msgs::OccupancyGrid &localmap);
int compare(const void *vp1, const void *vp2);
int orientation(Cell p, Cell q, Cell r);
int distSq(Cell p1, Cell p2);
Cell nextToTop(std::stack<Cell> &S);
void swap(Cell &p1, Cell &p2);
void polygon_centroid_detection(polygonAttribute& polygon);
void draw_marker(polygonAttribute polygonList);
bool compareByLength(visualization_msgs::Marker a, visualization_msgs::Marker b);
void make_line(Cell a, Cell b, nav_msgs::OccupancyGrid &localmap);

void CALLBACK_ODOMETRY_DATA(const nav_msgs::Odometry::ConstPtr& odom){
    pose.position.x = odom->pose.pose.position.x*100;
    pose.position.y = odom->pose.pose.position.y*100;
    pose.orientation = tf::getYaw(odom->pose.pose.orientation);
    isOdomCallback = true;
}

void CALLBACK_LIDARDATA_1(const sensor_msgs::LaserScan::ConstPtr& scan){
    if(isOdomCallback){
        lidar_.clear();
        u_int16_t num = int((scan->angle_max - scan->angle_min)/(scan->angle_increment))-1;
        for(u_int16_t i=0; i < num; i++){
            if(scan->ranges[i] < scan->range_max){
                Cell point;
                double angle = pose.orientation + scan->angle_max - (i * scan->angle_increment);
                point.x = scan->ranges[i]*cos(angle)*100 + (pose.position.x) + 28.7*cos(pose.orientation);
                point.y = scan->ranges[i]*sin(angle)*100 + (pose.position.y) + 28.7*sin(pose.orientation);
                lidar_.push_back(point);
            }
        }
    isLidarCallback_1 = true;
    }
}

void CALLBACK_LIDARDATA_2(const sensor_msgs::LaserScan::ConstPtr& scan){
    if(isOdomCallback){
        lidar_.clear();
        u_int16_t num = int((scan->angle_max - scan->angle_min)/(scan->angle_increment))-1;
        for(u_int16_t i=0; i < num; i++){
            if(scan->ranges[i] < scan->range_max){
                Cell point;
                double angle = 2*PI/3 + pose.orientation + scan->angle_max - (i * scan->angle_increment);
                point.x = scan->ranges[i]*cos(angle)*100 + (pose.position.x) + 28.7*cos(2*PI/3+pose.orientation);
                point.y = scan->ranges[i]*sin(angle)*100 + (pose.position.y) + 28.7*sin(2*PI/3+pose.orientation);
                lidar_.push_back(point);
            }
        }
    isLidarCallback_2 = true;
    }
}

void CALLBACK_LIDARDATA_3(const sensor_msgs::LaserScan::ConstPtr& scan){
    if(isOdomCallback){
        lidar_.clear();
        u_int16_t num = int((scan->angle_max - scan->angle_min)/(scan->angle_increment))-1;
        for(u_int16_t i=0; i < num; i++){
            if(scan->ranges[i] < scan->range_max){
                Cell point;
                double angle = (pose.orientation-2*PI/3) + scan->angle_max - (i * scan->angle_increment);
                point.x = scan->ranges[i]*cos(angle)*100 + (pose.position.x) + 28.7*cos(pose.orientation-2*PI/3);
                point.y = scan->ranges[i]*sin(angle)*100 + (pose.position.y) + 28.7*sin(pose.orientation-2*PI/3);
                lidar_.push_back(point);
            }
        }
    isLidarCallback_3 = true;
    }
}

int main(int argc, char** argv){
    clock_t start, end; 
    double time_use;

    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle n;
    ros::Subscriber lidar_sub1 = n.subscribe("scan1", 5, CALLBACK_LIDARDATA_1);
    ros::Subscriber lidar_sub2 = n.subscribe("scan2", 5, CALLBACK_LIDARDATA_2);
    ros::Subscriber lidar_sub3 = n.subscribe("scan3", 5, CALLBACK_LIDARDATA_3);
    ros::Subscriber odom_sub = n.subscribe("odom", 5, CALLBACK_ODOMETRY_DATA);
    ros::Publisher localmap_pub = n.advertise<nav_msgs::OccupancyGrid>("localmap", 5);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 5);  
    //------SETUP MAP PARAM----------
    //-------------------------------
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
    points.scale.z = 0.1;
    points.pose.position.z = 0.0;
    points.lifetime = ros::Duration(0.05);

    setupLocalmap();
    // Marker intalization
    //----------------------------------------------------
    ros::Rate r(20);
    while(n.ok()){
        ros::spinOnce();
        if(isLidarCallback_1 && isLidarCallback_2 && isLidarCallback_3){
        // if(isLidarCallback_2){                                                  // testing with 1 lidar
            start = clock();
            listPolygon.clear();
            list_of_polygons.markers.clear();
            nav_msgs::OccupancyGrid localmapTemp_;
            localmapTemp_.info = localmap_.info;
            localmapTemp_.data.resize(localmap_.info.width * localmap_.info.height);
            updateLocalMap(lidar_, localmapTemp_);
            publishmap = localmap_;
            convertData(publishmap);
            sort(list_of_polygons.markers.begin(),list_of_polygons.markers.end(),compareByLength);
            if(list_of_polygons.markers.size() > 8){
                list_of_polygons.markers.resize(9);
            }
            localmap_pub.publish(publishmap);
            marker_pub.publish(list_of_polygons);
            end = clock();
            time_use = (double)(end - start) / CLOCKS_PER_SEC;
            ROS_INFO("processing take :%f s", time_use);
            r.sleep();
        }
    }
}

void setupLocalmap(){
    localmap_.info.width = L_WIDTH/resolution+1;
    localmap_.info.height = L_HEIGHT/resolution+1;
    localmap_.info.resolution = resolution;
    localmap_.data.resize(localmap_.info.width * localmap_.info.height);
}

void updateLocalMap(std::vector<Cell> data, nav_msgs::OccupancyGrid localmapTemp_){
    Cell cell, pos;
    float maxWidth, maxHeight, minWidth, minHeight;
    localmap_.info.origin.position.x = pose.position.x - L_WIDTH*0.5;
    localmap_.info.origin.position.y = pose.position.y - L_HEIGHT*0.5;
    maxWidth = localmap_.info.origin.position.x + L_WIDTH;
    maxHeight = localmap_.info.origin.position.y + L_HEIGHT;
    minWidth = localmap_.info.origin.position.x;
    minHeight = localmap_.info.origin.position.y;

    if(firstTime){
        delta_x = 0;
        delta_y = 0;
        firstTime = false;
    }
    else{
        delta_x += (pose.position.x - odomLast.position.x)/resolution;
        delta_y += (pose.position.y - odomLast.position.y)/resolution;
    }
    if(abs(delta_x) >= 1 || abs(delta_y) >= 1){
        for(u_int16_t i=0; i < localmap_.info.width; i++){
            for(u_int16_t j=0; j < localmap_.info.height; j++){
                Cell temp;
                if(localmap_.data[j* localmap_.info.width + i] == 100){
                    Cell cell_new;
                    cell_new.x = i - int(delta_x);
                    cell_new.y = j - int(delta_y);
                    if((cell_new.x >= 0) && (cell_new.y >= 0) && (cell_new.x < localmap_.info.width) && (cell_new.y < localmap_.info.height)){
                        localmapTemp_.data[cell_new.y * localmap_.info.width + cell_new.x] = 100;
                    }
                }
            }
        }
        delta_x -= int(delta_x);
        delta_y -= int(delta_y);
        localmap_.data = localmapTemp_.data;
    }
    pos.x = (pose.position.x - minWidth)/resolution;
    pos.y = (pose.position.y - minHeight)/resolution;
    for(u_int16_t i=0; i < data.size(); i++){
        if((data[i].x > minWidth) && (data[i].y > minHeight) && (data[i].x < maxWidth) && (data[i].y < maxHeight)){
            cell.y = int((data[i].y - minHeight)/resolution);
            cell.x = int((data[i].x - minWidth)/resolution);
            localmap_.data[cell.y * localmap_.info.width + cell.x] = 100;
            // make_line(pos, cell, localmap_);
        }
    }
    odomLast = pose;
}

void convertData(nav_msgs::OccupancyGrid localmap){
    std::vector<Cell> obstaclePoint;
    node *temp = nullptr;
    temp = new node[localmap.info.width * localmap.info.height];
    for(u_int16_t i=0; i < localmap.info.width; i++){
        for(u_int16_t j=0; j < localmap.info.height; j++){
            Cell cell;
            cell.x = i;
            cell.y = j;
            int data = cell.y * localmap.info.width + cell.x;
            if(localmap.data[data] > 60){
                obstaclePoint.push_back(cell);
                temp[data].pos = cell;
            }

        }
    }
    find_cluster(temp, publishmap, obstaclePoint);
}

void find_cluster(node* temp, nav_msgs::OccupancyGrid &localmap, std::vector<Cell> points){
    srand((int)time(0));
    int count,r, cell;
    u_int16_t ID = 1;
    std::vector<node> core_borderPoints, nearPoints, listOfNodes;
    std::vector<Cell> cellSegment;
    std::vector<std::vector<Cell>> list;
    node *current = nullptr;
    for(u_int16_t i=0; i<points.size(); i++){
        count = 0;
        for(u_int16_t j=0; j < points.size(); j++){
            if(distanceCell(points[i],points[j]) < epsilon){
                count++;
            }
        }
        cell = points[i].y * localmap.info.width + points[i].x;
        if(count > minPts){
            temp[cell].label = CORE;
            core_borderPoints.push_back(temp[cell]);
        }
    }

    for(u_int16_t i=0; i<points.size(); i++){
        cell = points[i].y * localmap.info.width + points[i].x;
        if(temp[cell].label != CORE){
            count = 0;
            nearPoints.clear();
            for(u_int16_t j=0; j < points.size(); j++){
                if(distanceCell(points[i],points[j]) < epsilon){
                    count++;
                    nearPoints.push_back(temp[int(points[i].y) * localmap.info.width + int(points[i].x)]);
                }
            }
            if(count <= minPts){
                for(u_int16_t s = 0; s < nearPoints.size(); s++){
                    if(nearPoints[s].label == CORE){
                        temp[cell].label = BORDER;
                        core_borderPoints.push_back(temp[cell]);
                        break;
                    }
                }
            }
        }

    }
    while(core_borderPoints.size() > 0){
        bool alone = false;
        for(u_int16_t i=0; i < core_borderPoints.size(); i++){
            if(temp[int(core_borderPoints[i].pos.y * localmap.info.width + core_borderPoints[i].pos.x)].label == CORE){
                r = i;

                break;
            }
        }
        listOfNodes.clear();
        if(temp[int(core_borderPoints[r].pos.y) * localmap.info.width + int(core_borderPoints[r].pos.x)].label == CORE){
            current = &temp[int(core_borderPoints[r].pos.y) * localmap.info.width + int(core_borderPoints[r].pos.x)];
            current->clusterID = ID;
            listOfNodes.push_back(*current);
            cellSegment.push_back(current->pos);
            count = 0;  
            while(listOfNodes.size() != 0){                          
                for(u_int16_t i=0; i<core_borderPoints.size();i++){
                    int t = int(core_borderPoints[i].pos.y) * localmap.info.width + int(core_borderPoints[i].pos.x);
                    if(temp[t].b_visited == false){            
                        if((temp[t].clusterID == 0) && (distance(current, temp[t]) <= THRESHHOLD)){
                            if((temp[t].label == CORE)){
                                temp[t].clusterID = ID;
                                temp[t].b_visited = true;
                                listOfNodes.push_back(temp[t]);
                                cellSegment.push_back(temp[t].pos);
                                count++;
                                
                            }
                            else if(temp[t].label == BORDER){
                                temp[t].clusterID = ID;
                                temp[t].b_visited = true;
                                cellSegment.push_back(temp[t].pos);
                                count++;
                            }
                        }
                    alone = true;
                    }
                    if(count >= 15){
                        break;
                    }
                }
                listOfNodes.erase(listOfNodes.begin());       // xoa phan tu dau tien ra khoi vector
                if(listOfNodes.size() != 0){
                    current = &listOfNodes[0];
                }
            }
            if(alone == false){
                core_borderPoints.erase(core_borderPoints.begin()+r);
            }else{
                ID++;
                list.push_back(cellSegment);
                cellSegment.clear();
                bool flag = false;
                while(flag == false){
                    flag = true;
                    for(u_int i=0; i < core_borderPoints.size(); i++){
                        if(temp[int(core_borderPoints[i].pos.y) * localmap.info.width + int(core_borderPoints[i].pos.x)].clusterID != 0){
                            core_borderPoints.erase(core_borderPoints.begin()+i);
                            flag = false;
                            break;
                        }
                    }
                } 
            }
        }
        else{
            bool flag5 = false;
            for(u_int l = 0; l < core_borderPoints.size(); l++){
                if(temp[int(core_borderPoints[l].pos.y) * localmap.info.width + int(core_borderPoints[l].pos.x)].label == CORE){
                    flag5 = true;
                }
            }
            if(flag5 == false){
                break;
            }
        }

    }
    // ROS_INFO("%ld",list.size());
    for(u_int8_t i=0; i<list.size();i++){
        if(list[i].size() >= 3){
            int n = list[i].size();
            Cell points[n];
            for(u_int16_t t=0; t<list[i].size();t++){
                points[t].x = list[i][t].x;
                points[t].y = list[i][t].y;
            }
            convexHull(points,n, localmap);
        }
    }
}
float distanceCell(Cell a, Cell b){
    return sqrtf(pow(a.x - b.x,2) + pow(a.y - b.y,2));
}

float distance1(float a_x, float a_y, float b_x, float b_y){
    return sqrtf(pow(a_x - b_x,2) + pow(a_y - b_y,2));
}

float distance(node *a, node b){
    return sqrtf(pow(a->pos.x - b.pos.x,2) + pow(a->pos.y - b.pos.y,2));
}

int random(int minN, int maxN){
    return minN + rand() % (maxN + 1 - minN);
}

bool compareByLength(visualization_msgs::Marker a, visualization_msgs::Marker b)
{
    return a.pose.orientation.z < b.pose.orientation.z;
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
{   polygonAttribute polygon;
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
        m++;
    }
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
            polygon.edgePoints.push_back(p);
            S.pop();
        }
        polygon_centroid_detection(polygon);
}

void polygon_centroid_detection(polygonAttribute& polygon){
    double signedArea = 0;
    double temp_first;
    double temp_second;
    Cell p;
    polygon.edgePoints.push_back(polygon.edgePoints[0]);
    for(u_int8_t i=0; i < polygon.edgePoints.size()-1; i++){
        double x_0 = polygon.edgePoints[i].x;double x_1 = polygon.edgePoints[i+1].x;
        double y_0 = polygon.edgePoints[i].y;double y_1 = polygon.edgePoints[i+1].y;
        double A = (x_0 * y_1) - (x_1 * y_0);
        signedArea += A;
        temp_first += (x_0 + x_1) * A;
        temp_second += (y_0 + y_1) * A;
    }
    signedArea *= 0.5;
    polygon.pointCentroid.x = (temp_first / (6*signedArea));
    polygon.pointCentroid.y = (temp_second / (6*signedArea));
    if(polygon.pointCentroid.x > -999){
        double R = 0, max;
        for(u_int8_t i = 0; i < polygon.edgePoints.size(); i++){
            max = distanceCell(polygon.pointCentroid, polygon.edgePoints[i]);
            if(max >= R){
                R = max;
            }
        }
    polygon.R = R;
    listPolygon.push_back(polygon);
    draw_marker(polygon);
    }

}
                
void draw_marker(polygonAttribute polygonList){
    Cell temp;
    points.points.clear();
    points.id = points.id + 1;
    points.pose.position.x = ((polygonList.pointCentroid.x+0.5) * resolution + localmap_.info.origin.position.x)*0.01+0.01;
    points.pose.position.y = ((polygonList.pointCentroid.y+0.5) * resolution + localmap_.info.origin.position.y)*0.01+0.01;
    points.pose.orientation.z = distance1(points.pose.position.x, points.pose.position.y , pose.position.x, pose.position.y);
    points.scale.x = (polygonList.R+0.5)*2*resolution*0.01+0.01;
    points.scale.y = (polygonList.R+0.5)*2*resolution*0.01+0.01;
    if(points.pose.position.x > -1000 && points.pose.position.x < 1000){
        list_of_polygons.markers.push_back(points);
    }
    
}

// void make_line(Cell a, Cell b, nav_msgs::OccupancyGrid &localmap){
//     a.x = int(a.x);
//     b.x = int(b.x);
//     a.y = int(a.y);
//     b.y = int(b.y);
//     localmap.data[int(b.y) * localmap.info.width + int(b.x)] += 10;
//     if(localmap.data[int(b.y) * localmap.info.width + int(b.x)] > 100){
//         localmap.data[int(b.y) * localmap.info.width + int(b.x)] = 100;
//     }
// 	Cell cell;
// 	if (b.y>a.y){
// 		cell = a;
// 		a = b;
// 		b = cell;
// 	}
// 	if ((b.y != a.y)&&(a.x>b.x)) for(int i = b.y;i < a.y;i++){
//  		for (int j = (a.x + ((b.x-a.x)*(a.y-i)/(a.y-b.y)));j <= (a.x + ((b.x-a.x)*(a.y-(i+1))/(a.y-b.y)));j++){
//  			cell.y = i;
//  			cell.x = j;
//             if(cell.y != b.y && cell.x != b.x){
//                 localmap.data[int(cell.y) * localmap.info.width + int(cell.x)] -= 3;
//                 if(localmap.data[int(cell.y) * localmap.info.width + int(cell.x)] < 0){
//                     localmap.data[int(cell.y) * localmap.info.width + int(cell.x)] = 0;
//                 }
//             }

//  		}
//  	}

//  	else if ((b.y != a.y)&&(a.x<=b.x)) for(int i=b.y;i<a.y;i++){
//  		for (int j = (a.x + ((b.x-a.x)*(a.y-i-1)/(a.y-b.y)));j <= (a.x + ((b.x-a.x)*(a.y-i)/(a.y-b.y)));j++){
//  			cell.y = i;
//  			cell.x = j;
//             if(cell.y != b.y && cell.x != b.x){
//                 localmap.data[int(cell.y) * localmap.info.width + int(cell.x)] -= 3;
//                 if(localmap.data[int(cell.y) * localmap.info.width + int(cell.x)] < 0){
//                     localmap.data[int(cell.y) * localmap.info.width + int(cell.x)] = 0;
//                 }
//             }
//  		}
//  	}

//  	else {
//  		if(a.x<b.x){
//  			for(int i =a.x;i<=b.x;i++){
//  				cell.y = a.y;
//  				cell.x = i;
//                 if(cell.y != b.y && cell.x != b.x){
//                     localmap.data[int(cell.y) * localmap.info.width + int(cell.x)] -= 3;
//                     if(localmap.data[int(cell.y) * localmap.info.width + int(cell.x)] < 0){
//                         localmap.data[int(cell.y) * localmap.info.width + int(cell.x)] = 0;
//                     }
//                 }
//  			}
//  		}
//  			else if(a.x>=b.x){
//  				for(int i = b.x;i<=a.x;i++){
//  					cell.y = a.y;
//  					cell.x = i;
//                 if(cell.y != b.y && cell.x != b.x){
//                     localmap.data[int(cell.y) * localmap.info.width + int(cell.x)] -= 3;
//                     if(localmap.data[int(cell.y) * localmap.info.width + int(cell.x)] < 0){
//                         localmap.data[int(cell.y) * localmap.info.width + int(cell.x)] = 0;
//                     }
//                 }
//  				}

//  			}
//  	}
// }

