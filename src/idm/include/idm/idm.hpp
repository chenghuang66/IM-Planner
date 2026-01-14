#pragma once

#include "map.hpp"
#include "line.hpp"
#include "bpq.hpp"
#include "time.hpp"
#include "point.hpp"
#include <omp.h>
#include <queue>
#include <vector>
#include <memory>
#include <fstream>
#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/time_synchronizer.h>

#include<message_filters/synchronizer.h>

#include<message_filters/subscriber.h>

#include<message_filters/sync_policies/approximate_time.h>


#include <idm/DmMsg.h>
// #define LOG
#define VIS
#define SAVE
#define _OMP_H

#ifdef _OMP_H
static const int ThreadNum = 32;
#endif

static const int unknown_vis = -1;
static const int obstacle_vis = 100;
static const int dangerous_vis = 99;
static const int safeenough_vis = 0;

typedef Map<GridAccumulator, Hpatch<GridAccumulator>> DM;

class IDM
{
public:
    IDM(ros::NodeHandle &);
    // 转换为文件记录
    void toFile();
    // DM utils
    bool valid() const { return gotFirstData; } // whther idm is initialized
    std::shared_ptr<DM> &getDM() { return mapPtr; }
    double getRes() const { return mapResolution; }
    int getMapSizeX() const { return mapPtr->getMapSizeX(); };
    int getMapSizeY() const { return mapPtr->getMapSizeY(); };
    Point getOrigin() const { return mapPtr->map2world(IntPoint(0, 0)); }
    IntPoint worldToMap(const Point &corrd) const { return mapPtr->world2map(corrd); }
    Point mapToWorld(const IntPoint &corrd) const { return mapPtr->map2world(corrd); }
    bool isInsideMap(const IntPoint &corrd) const { return mapPtr->isInside(corrd); }
    double distToObs(const IntPoint &corrd) const { return mapPtr->cell(corrd).dis; }
    Point nearestObs(const IntPoint &corrd) const { return mapPtr->cell(corrd).obs; }
    bool isInsideMap(const Point &corrd) const { return mapPtr->isInside(corrd); }
    double distToObs(const Point &corrd) const { return mapPtr->cell(corrd).dis; }
    Point nearestObs(const Point &corrd) const { return mapPtr->cell(corrd).obs; }

private: // functions
    // 转换为占用栅格可视化
    void distance_map_Vis();
    void patchesVis();
    void callback(const nav_msgs::Odometry::ConstPtr &odom,
                  const sensor_msgs::LaserScanConstPtr &scan);
    void update();

private: // parameters
    double maxDectDist;
    double maxObsDist;
    double minObsDist;
    double deltaObsDist;
    double mapResolution;
    double mapPadding;
    double occThreshold;
    double patchMagnitude;
    std::string laserFrame;
    std::string scanTopic;
    std::string robotFrame;
    std::string odomTopic;
    std::string globalFrame;
    std::string nodeName;
    std::string savePath;
    // Todo
    // std::string readPath;

private: // utils
    bool gotFirstData;
    // 保存2d栅格地图中8联通邻居栅格的偏移
    const std::array<IntPoint, 8> nbrShift = {
        IntPoint(-1, -1), IntPoint(-1, 0), IntPoint(-1, 1),
        IntPoint(0, -1), IntPoint(0, 1),
        IntPoint(1, -1), IntPoint(1, 0), IntPoint(1, 1)};
    ros::NodeHandle nh;
    ros::NodeHandle localNh;
    ros::Publisher pubDM;
    ros::Publisher pubDmVis;
    ros::Publisher pubPatches;
    Point preMin, preMax, curMin, curMax;
    ros::Publisher pubPreCoverArea;
    ros::Publisher pubCurCoverArea;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    message_filters::Subscriber<nav_msgs::Odometry> *odomSub;
    message_filters::Subscriber<sensor_msgs::LaserScan> *scanSub;
    message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::LaserScan> *sync;
    // 保存每一帧的边缘栅格
    std::set<IntPoint, pointcomparator<int>> edgeSet;
    // 保存移除的障碍物曾占据的栅格
    std::set<IntPoint, pointcomparator<int>> addSet;
    // 保存增加的障碍物要占据的栅格
    std::set<IntPoint, pointcomparator<int>> delSet;
    // 优先队列
    BucketPrioQueue<IntPoint> updateQue;
    // DM Pointer
    std::shared_ptr<DM> mapPtr;
    // 计时器
    Timer clock;
    // 评价参数
    size_t count = 0;
    double totalTime = 0.0;
    double meanTimePerFrame = 0.0;
    double minTimePerFrame = std::numeric_limits<double>::max();
    double maxTimePerFrame = 0.0;
    size_t totalCells = 0;
    size_t meanCellsPerUpdate = 0;
    size_t minCellsPerUpdate = std::numeric_limits<size_t>::max();
    size_t maxCellsPerUpdate = 0;
    nav_msgs::Odometry current_odom_; 
    sensor_msgs::LaserScan current_scan;
    OrientedPoint curPose_;
    double minX;
    double minY;
    double maxX;
    double maxY;
    std::vector<std::vector<IntPoint>> beams_;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void timercallback(const ros::TimerEvent& e);
    ros::Subscriber odomsub;
    ros::Subscriber scansub;
    ros::Timer timer; 
};

IDM::IDM(ros::NodeHandle &nh_) : nh(nh_), localNh("~")
{
    ROS_INFO("-----------------------------------------");
    ROS_INFO("-----------IDM Object Init!!!-----------");
    gotFirstData = false;
    // parameters
    localNh.param("/idm_gen_node/MaxDetDist", maxDectDist, 10.0);
    localNh.param("/idm_gen_node/MaxObsDist", maxObsDist, 5.0);
    localNh.param("/idm_gen_node/MinObsDist", minObsDist, 0.5);
    localNh.param("/idm_gen_node/MapResolution", mapResolution, 0.1);
    localNh.param("/idm_gen_node/OccThreshold", occThreshold, 0.5);
    localNh.param("/idm_gen_node/PatchMagnitude", patchMagnitude, 5.0);
    localNh.param<std::string>("/idm_gen_node/RobotFrame", robotFrame, "/robot");
    localNh.param<std::string>("/idm_gen_node/LaserFrame", laserFrame, "/laser");
    localNh.param<std::string>("/idm_gen_node/OdomTopic", odomTopic, "/odom");
    localNh.param<std::string>("/idm_gen_node/ScanTopic", scanTopic, "/scan");
    localNh.param<std::string>("/idm_gen_node/GlobalFrame", globalFrame, "/map");
    localNh.param<std::string>("/savePath", savePath, "/home/shaw/dm.csv");
    // Todo
    // nh.param<std::string>("/iedf/readPath", readPath);
    deltaObsDist = maxObsDist - minObsDist;
    // patchMagnitude最小为1.0
    patchMagnitude = std::max(1.0, patchMagnitude);
    // mapPadding为一个Patch大小
    mapPadding = mapResolution * (1 << static_cast<int>(patchMagnitude));
    // 构建订阅器与同步器
    ROS_INFO("odomTopic: %s", odomTopic.c_str());
    ROS_INFO("scanTopic: %s", scanTopic.c_str());
    ROS_INFO("robotFrame: %s", robotFrame.c_str());
    ROS_INFO("laserFrame: %s", laserFrame.c_str());
    odomSub = new message_filters::Subscriber<nav_msgs::Odometry>(nh, odomTopic.c_str(), 1);
    scanSub = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, scanTopic.c_str(), 1);
    sync = new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::LaserScan>(*odomSub, *scanSub, 1000);
    (*sync).registerCallback(boost::bind(&IDM::callback, this, _1, _2));
    // odomsub = nh.subscribe(odomTopic.c_str(), 10, &IDM::odomCallback,this); 
    // scansub = nh.subscribe(scanTopic.c_str(), 10, &IDM::scanCallback,this); 
    // timer = nh.createTimer(ros::Duration(0.1), &IDM::timercallback, this);

    // odomSub->registerCallback(odomCallback);
    // scanSub->registerCallback(scanCallback);

    // typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), *odomSub, *scanSub);
    // // sync.setInterMessageLowerBound(ros::Duration(3));  // 设置时间容忍度
    // sync.registerCallback(boost::bind(&IDM::callback, this, _1, _2));
    
    // typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> mysync;
    // message_filters::Synchronizer<mysync> sync(mysync(10),*odomSub,*scanSub);
    // sync.registerCallback(boost::bind(&IDM::callback,this,_1,_2));

    
    // Publisher
    pubDM = nh.advertise<idm::DmMsg>("/DM", 10, true);
    pubDmVis = nh.advertise<nav_msgs::OccupancyGrid>("/dmaVis", 10, true);
    pubPatches = nh.advertise<visualization_msgs::MarkerArray>("/patches", 10, true);
    pubPreCoverArea = nh.advertise<geometry_msgs::PolygonStamped>("/preCoverArea", 10, true);
    pubCurCoverArea = nh.advertise<geometry_msgs::PolygonStamped>("/curCoverArea", 10, true);
    // get transform
    try
    {
        ros::Time now = ros::Time::now();
        listener.waitForTransform(robotFrame.c_str(), laserFrame.c_str(), now, ros::Duration(1.0));
        listener.lookupTransform(robotFrame.c_str(), laserFrame.c_str(), now, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        transform.setOrigin(tf::Vector3(0, 0, 0));
    }
    double saved_rx = transform.getOrigin().x();
    double saved_ry = transform.getOrigin().y();
    ROS_INFO("Translation is %f %f", saved_rx, saved_ry);
    ROS_INFO("-----------------------------------------");
}

void IDM::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    double dx = transform.getOrigin().x();
    double dy = transform.getOrigin().y();
    double robot_ori_x = odom->pose.pose.position.x;
    double robot_ori_y = odom->pose.pose.position.y;
    double robot_ori_theta = tf::getYaw(odom->pose.pose.orientation);
    double laser_ori_x = robot_ori_x + dx * cos(robot_ori_theta) - dy * sin(robot_ori_theta);
    double laser_ori_y = robot_ori_y + dy * cos(robot_ori_theta) + dx * sin(robot_ori_theta);
    double laser_ori_theta = robot_ori_theta;
    // 初始化位姿信息
    curPose_.x = laser_ori_x;
    curPose_.y = laser_ori_y;
    curPose_.theta = laser_ori_theta;
    // OrientedPoint curPose_(laser_ori_x, laser_ori_y, laser_ori_theta);
    // TransPoint trans(curPose);
    // std::cout<<"curPose_.x "<<curPose_.x<<std::endl;
}

void IDM::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    TransPoint trans(curPose_);
    beams_.clear();
    ROS_INFO("---scanCallback-");
     minX = curPose_.x - mapPadding;
     minY = curPose_.y - mapPadding;
     maxX = curPose_.x + mapPadding;
     maxY = curPose_.y + mapPadding;
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        double dist = scan->ranges[i];
        // 去除无效观测的点
        if (isnan(dist) || isinf(dist))
            continue;
        // 去除观测范围外的点
        if (dist <= 0.0 || dist >= maxDectDist)
            continue;
        double angle = scan->angle_min +
                       scan->angle_increment * i;
        Point hit(dist * std::cos(angle),
                  dist * std::sin(angle));
        trans(hit);
        std::vector<IntPoint> beam;
        lineTraversal(curPose_, hit, mapResolution, beam);
        beams_.push_back(beam);
        minX = std::min(minX, hit.x);
        minY = std::min(minY, hit.y);
        maxX = std::max(maxX, hit.x);
        maxY = std::max(maxY, hit.y);
    }
}



void IDM::distance_map_Vis()
{
    if (mapPtr == nullptr)
    {
        ROS_INFO("nullptr---mapPtr");
        return;
    }
        
    nav_msgs::OccupancyGrid dmVis;
    dmVis.header.frame_id = globalFrame;
    dmVis.header.stamp = ros::Time::now();
    dmVis.info.origin.position.x =
        mapPtr->map2world(IntPoint(0, 0)).x;
    dmVis.info.origin.position.y =
        mapPtr->map2world(IntPoint(0, 0)).y;
    dmVis.info.origin.orientation.w = 1.0;
    dmVis.info.resolution = mapPtr->getDelta();
    int width = mapPtr->getMapSizeX();
    int height = mapPtr->getMapSizeY();
    dmVis.info.width = width;
    dmVis.info.height = height;
    dmVis.data.resize(mapPtr->getMapSizeX() * mapPtr->getMapSizeY());
    idm::DmMsg dm;
    dm.header = dmVis.header;
    dm.info = dmVis.info;
    dm.dist.resize(mapPtr->getMapSizeX() * mapPtr->getMapSizeY());
    dm.obst.resize(mapPtr->getMapSizeX() * mapPtr->getMapSizeY());
    auto index = [width](int x, int y) -> int
    { return x + y * width; };
#ifdef _OMP_H
#pragma omp parallel for schedule(dynamic, ThreadNum) num_threads(ThreadNum)
#endif
    for (int y = 0; y < mapPtr->getMapSizeY(); ++y)
    {
        for (int x = 0; x < mapPtr->getMapSizeX(); ++x)
        {
            double dis = mapPtr->cell(x, y).dis;
            if (dis == std::numeric_limits<double>::max())
                // dmVis.data[index(x, y)] = unknown_vis;
                dmVis.data[index(x, y)] = safeenough_vis;
            else if (dis == 0)
                dmVis.data[index(x, y)] = obstacle_vis;
            else if (dis <= minObsDist)
                dmVis.data[index(x, y)] = dangerous_vis;
            else if (dis >= maxObsDist)
                dmVis.data[index(x, y)] = safeenough_vis;
            else
                dmVis.data[index(x, y)] = 100 * (maxObsDist - dis) / deltaObsDist;
            dm.dist[index(x, y)] = dis;
            dm.obst[index(x, y)].x = mapPtr->cell(x, y).obs.x;
            dm.obst[index(x, y)].y = mapPtr->cell(x, y).obs.y;
        }
    }
    // ROS_INFO("publish dmVis");
    pubDmVis.publish(dmVis);
    pubDM.publish(dm);
}

void IDM::patchesVis()
{
    // vis patches
    visualization_msgs::MarkerArray patches;
    // vis centers
    visualization_msgs::Marker centers;
    centers.header.frame_id = globalFrame;
    centers.header.stamp = ros::Time::now();
    centers.ns = pubPatches.getTopic();
    centers.type = visualization_msgs::Marker::SPHERE_LIST;
    centers.action = visualization_msgs::Marker::ADD;
    centers.id = 1;
    centers.scale.x = 0.5;
    centers.scale.y = 0.5;
    centers.scale.z = 0.5;
    centers.color.a = 0.5;
    centers.color.r = 1.0;
    centers.color.g = 0.0;
    centers.color.b = 0.0;
    centers.pose.orientation.w = 1.0;
    // vis corners
    visualization_msgs::Marker corners;
    corners.header.frame_id = globalFrame;
    corners.header.stamp = ros::Time::now();
    corners.ns = pubPatches.getTopic();
    corners.type = visualization_msgs::Marker::SPHERE_LIST;
    corners.action = visualization_msgs::Marker::ADD;
    corners.id = 2;
    corners.scale.x = 0.3;
    corners.scale.y = 0.3;
    corners.scale.z = 0.3;
    corners.color.a = 0.5;
    corners.color.r = 0.0;
    corners.color.g = 1.0;
    corners.color.b = 0.0;
    corners.pose.orientation.w = 1.0;
    // vis middles
    visualization_msgs::Marker middles;
    middles.header.frame_id = globalFrame;
    middles.header.stamp = ros::Time::now();
    middles.ns = pubPatches.getTopic();
    middles.type = visualization_msgs::Marker::SPHERE_LIST;
    middles.action = visualization_msgs::Marker::ADD;
    middles.id = 3;
    middles.scale.x = 0.2;
    middles.scale.y = 0.2;
    middles.scale.z = 0.2;
    middles.color.a = 0.5;
    middles.color.r = 0.0;
    middles.color.g = 0.0;
    middles.color.b = 1.0;
    middles.pose.orientation.w = 1.0;
    // vis contour
    visualization_msgs::Marker contour;
    contour.header.frame_id = globalFrame;
    contour.header.stamp = ros::Time::now();
    contour.ns = pubPatches.getTopic();
    contour.type = visualization_msgs::Marker::LINE_LIST;
    contour.action = visualization_msgs::Marker::ADD;
    contour.id = 4;
    contour.scale.x = 0.1;
    contour.color.a = 0.5;
    contour.color.r = 1.0;
    contour.color.g = 0.0;
    contour.color.b = 1.0;
    contour.pose.orientation.w = 1.0;

    double patch_len = (1 << int(patchMagnitude)) * mapResolution;
    uint rows = mapPtr->storage().getXSize();
    uint cols = mapPtr->storage().getYSize();
    Point origin(mapPtr->map2world(IntPoint(0, 0)).x,
                 mapPtr->map2world(IntPoint(0, 0)).y);
    centers.points.resize(rows * cols);
#ifdef _OMP_H
#pragma omp parallel for schedule(dynamic, ThreadNum) num_threads(ThreadNum)
#endif
    for (uint row = 0; row < rows; ++row)
    {
        for (uint col = 0; col < cols; ++col)
        {
            geometry_msgs::Point vertex;
            vertex.x = origin.x + (row + 0.5) * patch_len;
            vertex.y = origin.y + (col + 0.5) * patch_len;
            centers.points[row * cols + col] = vertex;
        }
    }
    corners.points.resize((rows + 1) * (cols + 1));
#ifdef _OMP_H
#pragma omp parallel for schedule(dynamic, ThreadNum) num_threads(ThreadNum)
#endif
    for (uint row = 0; row < rows + 1; ++row)
    {
        for (uint col = 0; col < cols + 1; ++col)
        {
            geometry_msgs::Point vertex;
            vertex.x = origin.x + row * patch_len;
            vertex.y = origin.y + col * patch_len;
            corners.points[row * (cols + 1) + col] = vertex;
        }
    }
    std::vector<geometry_msgs::Point> row_middles((rows + 1) * cols);
#ifdef _OMP_H
#pragma omp parallel for schedule(dynamic, ThreadNum) num_threads(ThreadNum)
#endif
    for (uint row = 0; row < rows + 1; ++row)
    {
        for (uint col = 0; col < cols; ++col)
        {
            geometry_msgs::Point vertex;
            vertex.x = origin.x + row * patch_len;
            vertex.y = origin.y + (col + 0.5) * patch_len;
            row_middles[row * cols + col] = vertex;
        }
    }
    middles.points.insert(middles.points.end(), row_middles.begin(), row_middles.end());
    std::vector<geometry_msgs::Point> col_middles((cols + 1) * rows);
#ifdef _OMP_H
#pragma omp parallel for schedule(dynamic, ThreadNum) num_threads(ThreadNum)
#endif
    for (uint row = 0; row < rows; ++row)
    {
        for (uint col = 0; col < cols + 1; ++col)
        {
            geometry_msgs::Point vertex;
            vertex.x = origin.x + (row + 0.5) * patch_len;
            vertex.y = origin.y + col * patch_len;
            col_middles[col * rows + row] = vertex;
        }
    }
    middles.points.insert(middles.points.end(), col_middles.begin(), col_middles.end());
    std::vector<geometry_msgs::Point> row_contour((rows + 1) * cols * 2);
#ifdef _OMP_H
#pragma omp parallel for schedule(dynamic, ThreadNum) num_threads(ThreadNum)
#endif
    for (uint row = 0; row < rows + 1; ++row)
    {
        for (uint col = 0; col < cols; ++col)
        {
            geometry_msgs::Point vertex;
            vertex.x = origin.x + row * patch_len;
            vertex.y = origin.y + col * patch_len;
            // contour.points.push_back(vertex);
            row_contour[(row * cols + col) * 2] = vertex;
            vertex.y = origin.y + (col + 1) * patch_len;
            // contour.points.push_back(vertex);
            row_contour[(row * cols + col) * 2 + 1] = vertex;
        }
    }
    contour.points.insert(contour.points.end(), row_contour.begin(), row_contour.end());
    std::vector<geometry_msgs::Point> col_contour(rows * (cols + 1) * 2);
#ifdef _OMP_H
#pragma omp parallel for schedule(dynamic, ThreadNum) num_threads(ThreadNum)
#endif
    for (uint row = 0; row < rows; ++row)
    {
        for (uint col = 0; col < cols + 1; ++col)
        {
            geometry_msgs::Point vertex;
            vertex.x = origin.x + row * patch_len;
            vertex.y = origin.y + col * patch_len;
            col_contour[(col * rows + row) * 2] = vertex;
            vertex.x = origin.x + (row + 1) * patch_len;
            col_contour[(col * rows + row) * 2 + 1] = vertex;
        }
    }
    contour.points.insert(contour.points.end(), col_contour.begin(), col_contour.end());
    patches.markers.push_back(centers);
    patches.markers.push_back(corners);
    patches.markers.push_back(middles);
    patches.markers.push_back(contour);
    pubPatches.publish(patches);
}

void IDM::toFile()
{
    if (mapPtr == nullptr)
        return;
    std::fstream mapWriter;
    mapWriter.open(savePath.c_str(), std::ios::out);
    mapWriter.setf(std::ios::fixed);
    mapWriter.precision(6);
    if (!mapWriter.is_open())
        exit(0);
    for (int y = 0; y < mapPtr->getMapSizeY(); ++y)
    {
        for (int x = 0; x < mapPtr->getMapSizeX(); ++x)
        {
            Grid &g = mapPtr->cell(x, y);
            if (g.sta == Grid::GridState::free || g.sta == Grid::GridState::occ)
            {
                mapWriter << "("
                          << g.pos << " " << g.obs << " " << g.dis
                          << ")"
                          << ",";
            }
            else
            {
                mapWriter << "("
                          << g.pos << " "
                          << "(#,#)"
                          << " "
                          << "max"
                          << ")"
                          << ",";
            }
        }
        mapWriter << std::endl;
    }
    mapWriter.close();
}

void IDM::timercallback(const ros::TimerEvent& e)
{
#ifdef LOG
    ROS_INFO("------------------------------");
    ROS_INFO("-----------callback-----------");
#endif
    // std::cout<<"count "<<count<<std::endl;
    count++;
    clock.begin();
    // vis preCoverArea & curCoverArea
    {
        if (!gotFirstData)
        {
            preMin = Point(curPose_.x - mapPadding,
                           curPose_.y - mapPadding);
            preMax = Point(curPose_.x + mapPadding,
                           curPose_.y + mapPadding);
        }
        else
        {
            preMin = curMin;
            preMax = curMax;
        }
        curMin = Point(minX, minY);
        curMax = Point(maxX, maxY);
        geometry_msgs::PolygonStamped curCoverArea;
        curCoverArea.header.frame_id = globalFrame;
        geometry_msgs::PolygonStamped preCoverArea;
        preCoverArea.header.frame_id = globalFrame;
        geometry_msgs::Point32 lb;
        lb.x = curMin.x;
        lb.y = curMin.y;
        curCoverArea.polygon.points.push_back(lb);
        lb.x = preMin.x;
        lb.y = preMin.y;
        preCoverArea.polygon.points.push_back(lb);
        geometry_msgs::Point32 lu;
        lu.x = curMin.x;
        lu.y = curMax.y;
        curCoverArea.polygon.points.push_back(lu);
        lu.x = preMin.x;
        lu.y = preMax.y;
        preCoverArea.polygon.points.push_back(lu);
        geometry_msgs::Point32 ru;
        ru.x = curMax.x;
        ru.y = curMax.y;
        curCoverArea.polygon.points.push_back(ru);
        ru.x = preMax.x;
        ru.y = preMax.y;
        preCoverArea.polygon.points.push_back(ru);
        geometry_msgs::Point32 rb;
        rb.x = curMax.x;
        rb.y = curMin.y;
        curCoverArea.polygon.points.push_back(rb);
        rb.x = preMax.x;
        rb.y = preMin.y;
        preCoverArea.polygon.points.push_back(rb);
        curCoverArea.header.stamp = ros::Time::now();
        pubCurCoverArea.publish(curCoverArea);
        preCoverArea.header.stamp = ros::Time::now();
        pubPreCoverArea.publish(preCoverArea);
    }
    // 保留mapPadding的余量
    Point localMin(minX - mapPadding, minY - mapPadding);
    Point localMax(maxX + mapPadding, maxY + mapPadding);
    if (!gotFirstData) // 获取到第一帧点云
    {
        ROS_INFO("IDM Initialize!!!");



        mapPtr = std::make_shared<DM>(
            Point((localMin.x + localMax.x) / 2.0, (localMin.y + localMax.y) / 2.0),
            localMin.x, localMin.y, localMax.x, localMax.y, mapResolution, static_cast<int>(patchMagnitude));
        // 根据地图信息生成activeArea(Todo: 有冗余计算，有待优化!)
        // Hpatch<GridAccumulator>::PointSet activeArea;
        // IntPoint mapLocalMin = mapPtr->world2map(minX, minY);
        // IntPoint mapLocalMax = mapPtr->world2map(maxX, maxY);
        // for (int x = mapLocalMin.x; x < mapLocalMax.x; ++x)
        // {
        //     for (int y = mapLocalMin.y; y < mapLocalMax.y; ++y)
        //     {
        //         // 得到所处栅格p1坐落在patch（栅格补丁）的栅格坐标p2
        //         IntPoint p = mapPtr->storage().patchIndexes(x, y);
        //         // 将patch的栅格坐标p2插入activeArea
        //         activeArea.insert(p);
        //     }
        // }
        // // 为activeArea里面的没有分配内存的区域分配内存
        // mapPtr->storage().setActiveArea(activeArea, true);
        // mapPtr->storage().allocActiveArea();
        gotFirstData = true;
    }
    else if (!mapPtr->isInside(minX, minY) ||
             !mapPtr->isInside(maxX, maxY))
    {
#ifdef LOG
        ROS_INFO("IDM Resize!!!");
#endif
        int preSizeX = mapPtr->getMapSizeX();
        int preSizeY = mapPtr->getMapSizeY();
        Point predMin = mapPtr->map2world(IntPoint(0, 0));
        Point predMax = mapPtr->map2world(IntPoint(
            preSizeX, preSizeY));
        Point curMin(std::min(localMin.x, predMin.x),
                     std::min(localMin.y, predMin.y));
        Point curMax(std::max(localMax.x, predMax.x),
                     std::max(localMax.y, predMax.y));
        mapPtr->resize(curMin.x, curMin.y, curMax.x, curMax.y);
    }
    // 根据SCAN信息更新DM
    // 计算travel Cell以及hit Cell
    std::map<IntPoint, int, pointcomparator<int>> travelCells;
    std::map<IntPoint, int, pointcomparator<int>> hitCells;
    // std::cout << "beams size: " << beams.size() << std::endl;
    for (auto beam : beams_)
    {
        // std::cout << "beam size: " << beam.size() << std::endl;
        for (size_t i = 0; i < beam.size() - 1; ++i)
        {
            Point pWorld((beam[i].x + 0.5) * mapResolution,
                         (beam[i].y + 0.5) * mapResolution);
            IntPoint pMap = mapPtr->world2map(pWorld);
            auto iter = travelCells.find(pMap);
            if (iter == travelCells.end())
                travelCells.insert({pMap, 1});
            else
                iter->second += 1;
        }
        // beam的最后一点为hitPoint，反映在地图中就是障碍物
        Point pWorld((beam.rbegin()->x + 0.5) * mapResolution,
                     (beam.rbegin()->y + 0.5) * mapResolution);
        IntPoint pMap = mapPtr->world2map(pWorld);
        auto iter = hitCells.find(pMap);
        if (iter == hitCells.end())
            hitCells.insert({pMap, 1});
        else
            iter->second += 1;
    }
#ifdef LOG
    std::cout << "hitSize: " << hitCells.size() << std::endl;
    std::cout << "travelSize: " << travelCells.size() << std::endl;
#endif
    // 遍历当前帧点云覆盖的栅格
    IntPoint min = mapPtr->world2map(Point(minX, minY));
    IntPoint max = mapPtr->world2map(Point(maxX, maxY));
    for (int x = min.x; x <= max.x; ++x)
    {
        for (int y = min.y; y <= max.y; ++y)
        {
            IntPoint id(x, y);
            GridAccumulator &cur = mapPtr->cell(id);
            // 更新栅格状态
            auto iter1 = hitCells.find(id);
            if (iter1 != hitCells.end())
            {
                cur.update(iter1->second, iter1->second);
                hitCells.erase(iter1);
            }
            auto iter2 = travelCells.find(id);
            if (iter2 != travelCells.end())
            {
                cur.update(iter2->second, 0);
                travelCells.erase(iter2);
            }
            // 更新AddSet, DelSet
            if (cur.getSta() == Sta::origin) // 如果栅格未初始化
            {
                if (cur > occThreshold) // 如果为占用栅格
                {
                    cur.setOcc(mapPtr->map2world(id));
                    addSet.insert(id);
                }
                else // 如果为空闲栅格, 其val初始化为最大值
                {
                    cur.setIdeal(mapPtr->map2world(id));
                }
            }
            // 如果全局地图不为障碍物而局部地图为障碍物
            else if (cur.getSta() == Sta::free)
            {
                if (cur > occThreshold)
                {
                    cur.setOcc(mapPtr->map2world(id));
                    addSet.insert(id);
                }
            }
            // 如果全局地图为障碍物而局部地图不为障碍物
            else if (cur.getSta() == Sta::occ)
            {
                if (cur <= occThreshold)
                {
                    cur.setIdeal(mapPtr->map2world(id));
                    delSet.insert(id);
                }
            }
        }
    }
    // std::cout << "hitSize: " << hitCells.size() << std::endl;
    // std::cout << "travelSize: " << travelCells.size() << std::endl;
    if (addSet.empty())
    {
        // 如果addSet为空，就将上次更新边缘栅格添加到addSet中，
        // 防止因为新添加观测帧因观测不到障碍物而无法更新地图。
        for (int x = min.x; x <= max.x; ++x)
            addSet.insert(IntPoint(x, min.y));
        for (int x = min.x; x <= max.x; ++x)
            addSet.insert(IntPoint(x, max.y));
        for (int y = min.y + 1; y < max.y; ++y)
            addSet.insert(IntPoint(min.x, y));
        for (int y = min.y + 1; y < max.y; ++y)
            addSet.insert(IntPoint(max.x, y));
    }
    update();
    double t = clock.end(MILLISEC);
    totalTime += t;
    meanTimePerFrame = totalTime / (double)count;
    minTimePerFrame = std::min(minTimePerFrame, t);
    maxTimePerFrame = std::max(maxTimePerFrame, t);
#ifdef LOG
    ROS_INFO("Using time: %f ms", t);
    ROS_INFO("meanTimePerFrame %f, minTimePerFrame: %f, maxTimePerFrame: %f", meanTimePerFrame, minTimePerFrame, maxTimePerFrame);
    ROS_INFO("meanCellsPerUpdate: %ld, minCellsPerUpdate: %ld, maxCellsPerUpdate: %ld", meanCellsPerUpdate, minCellsPerUpdate, maxCellsPerUpdate);
    ROS_INFO("------------------------------");
#endif

    distance_map_Vis();
    patchesVis();

    // std::cout<<"count "<<count<<std::endl;
    // // toFile();
    // std::cout<<"count "<<count<<std::endl;
}

void IDM::callback(const nav_msgs::Odometry::ConstPtr &odom,
                   const sensor_msgs::LaserScanConstPtr &scan)
{
#ifdef LOG
    ROS_INFO("------------------------------");
    ROS_INFO("-----------callback-----------");
#endif

    count++;
    clock.begin();
    double dx = transform.getOrigin().x();
    double dy = transform.getOrigin().y();
    double robot_ori_x = odom->pose.pose.position.x;
    double robot_ori_y = odom->pose.pose.position.y;
    
    double robot_ori_theta = tf::getYaw(odom->pose.pose.orientation);

    // std::cout<<"x "<<robot_ori_x <<"x "<<robot_ori_y <<"x "<<robot_ori_theta<<std::endl;
    double laser_ori_x = robot_ori_x + dx * cos(robot_ori_theta) - dy * sin(robot_ori_theta);
    double laser_ori_y = robot_ori_y + dy * cos(robot_ori_theta) + dx * sin(robot_ori_theta);
    double laser_ori_theta = robot_ori_theta;
    // 初始化位姿信息
    OrientedPoint curPose(laser_ori_x, laser_ori_y, laser_ori_theta);
    TransPoint trans(curPose);
    // 初始化观测信息
    std::vector<std::vector<IntPoint>> beams;
    std::vector<bool> hits;
    double minX = curPose.x - mapPadding;
    double minY = curPose.y - mapPadding;
    double maxX = curPose.x + mapPadding;
    double maxY = curPose.y + mapPadding;
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        double dist = scan->ranges[i];
        // 去除无效观测的点
        if (isnan(dist) || isinf(dist))
            continue;
        // 去除观测范围外的点
        if (dist <= 0.0 || dist >= maxDectDist)
            continue;
        double angle = scan->angle_min +
                       scan->angle_increment * i;
        Point hit(dist * std::cos(angle),
                  dist * std::sin(angle));
        trans(hit);
        std::vector<IntPoint> beam;
        lineTraversal(curPose, hit, mapResolution, beam);
        beams.push_back(beam);
        minX = std::min(minX, hit.x);
        minY = std::min(minY, hit.y);
        maxX = std::max(maxX, hit.x);
        maxY = std::max(maxY, hit.y);
    }
    // vis preCoverArea & curCoverArea
    {
        if (!gotFirstData)
        {
            preMin = Point(curPose.x - mapPadding,
                           curPose.y - mapPadding);
            preMax = Point(curPose.x + mapPadding,
                           curPose.y + mapPadding);
        }
        else
        {
            preMin = curMin;
            preMax = curMax;
        }
        curMin = Point(minX, minY);
        curMax = Point(maxX, maxY);
        geometry_msgs::PolygonStamped curCoverArea;
        curCoverArea.header.frame_id = globalFrame;
        geometry_msgs::PolygonStamped preCoverArea;
        preCoverArea.header.frame_id = globalFrame;
        geometry_msgs::Point32 lb;
        lb.x = curMin.x;
        lb.y = curMin.y;
        curCoverArea.polygon.points.push_back(lb);
        lb.x = preMin.x;
        lb.y = preMin.y;
        preCoverArea.polygon.points.push_back(lb);
        geometry_msgs::Point32 lu;
        lu.x = curMin.x;
        lu.y = curMax.y;
        curCoverArea.polygon.points.push_back(lu);
        lu.x = preMin.x;
        lu.y = preMax.y;
        preCoverArea.polygon.points.push_back(lu);
        geometry_msgs::Point32 ru;
        ru.x = curMax.x;
        ru.y = curMax.y;
        curCoverArea.polygon.points.push_back(ru);
        ru.x = preMax.x;
        ru.y = preMax.y;
        preCoverArea.polygon.points.push_back(ru);
        geometry_msgs::Point32 rb;
        rb.x = curMax.x;
        rb.y = curMin.y;
        curCoverArea.polygon.points.push_back(rb);
        rb.x = preMax.x;
        rb.y = preMin.y;
        preCoverArea.polygon.points.push_back(rb);
        curCoverArea.header.stamp = ros::Time::now();
        pubCurCoverArea.publish(curCoverArea);
        preCoverArea.header.stamp = ros::Time::now();
        pubPreCoverArea.publish(preCoverArea);
    }
    // 保留mapPadding的余量
    Point localMin(minX - mapPadding, minY - mapPadding);
    Point localMax(maxX + mapPadding, maxY + mapPadding);
    if (!gotFirstData) // 获取到第一帧点云
    {
#ifdef LOG
        ROS_INFO("IDM Initialize!!!");
#endif
        mapPtr = std::make_shared<DM>(
            Point((localMin.x + localMax.x) / 2.0,
                  (localMin.y + localMax.y) / 2.0),
            localMin.x, localMin.y, localMax.x, localMax.y, mapResolution, static_cast<int>(patchMagnitude));
        // 根据地图信息生成activeArea(Todo: 有冗余计算，有待优化!)
        // Hpatch<GridAccumulator>::PointSet activeArea;
        // IntPoint mapLocalMin = mapPtr->world2map(minX, minY);
        // IntPoint mapLocalMax = mapPtr->world2map(maxX, maxY);
        // for (int x = mapLocalMin.x; x < mapLocalMax.x; ++x)
        // {
        //     for (int y = mapLocalMin.y; y < mapLocalMax.y; ++y)
        //     {
        //         // 得到所处栅格p1坐落在patch（栅格补丁）的栅格坐标p2
        //         IntPoint p = mapPtr->storage().patchIndexes(x, y);
        //         // 将patch的栅格坐标p2插入activeArea
        //         activeArea.insert(p);
        //     }
        // }
        // // 为activeArea里面的没有分配内存的区域分配内存
        // mapPtr->storage().setActiveArea(activeArea, true);
        // mapPtr->storage().allocActiveArea();
        gotFirstData = true;
    }
    else if (!mapPtr->isInside(minX, minY) ||
             !mapPtr->isInside(maxX, maxY))
    {
#ifdef LOG
        ROS_INFO("IDM Resize!!!");
#endif
        int preSizeX = mapPtr->getMapSizeX();
        int preSizeY = mapPtr->getMapSizeY();
        Point predMin = mapPtr->map2world(IntPoint(0, 0));
        Point predMax = mapPtr->map2world(IntPoint(
            preSizeX, preSizeY));
        Point curMin(std::min(localMin.x, predMin.x),
                     std::min(localMin.y, predMin.y));
        Point curMax(std::max(localMax.x, predMax.x),
                     std::max(localMax.y, predMax.y));
        mapPtr->resize(curMin.x, curMin.y, curMax.x, curMax.y);
    }
    // 根据SCAN信息更新DM
    // 计算travel Cell以及hit Cell
    std::map<IntPoint, int, pointcomparator<int>> travelCells;
    std::map<IntPoint, int, pointcomparator<int>> hitCells;
    // std::cout << "beams size: " << beams.size() << std::endl;
    for (auto beam : beams)
    {
        // std::cout << "beam size: " << beam.size() << std::endl;
        for (size_t i = 0; i < beam.size() - 1; ++i)
        {
            Point pWorld((beam[i].x + 0.5) * mapResolution,
                         (beam[i].y + 0.5) * mapResolution);
            IntPoint pMap = mapPtr->world2map(pWorld);
            auto iter = travelCells.find(pMap);
            if (iter == travelCells.end())
                travelCells.insert({pMap, 1});
            else
                iter->second += 1;
        }
        // beam的最后一点为hitPoint，反映在地图中就是障碍物
        Point pWorld((beam.rbegin()->x + 0.5) * mapResolution,
                     (beam.rbegin()->y + 0.5) * mapResolution);
        IntPoint pMap = mapPtr->world2map(pWorld);
        auto iter = hitCells.find(pMap);
        if (iter == hitCells.end())
            hitCells.insert({pMap, 1});
        else
            iter->second += 1;
    }
    // std::cout << "hitSize: " << hitCells.size() << std::endl;
    // std::cout << "travelSize: " << travelCells.size() << std::endl;
    // 遍历当前帧点云覆盖的栅格
    IntPoint min = mapPtr->world2map(Point(minX, minY));
    IntPoint max = mapPtr->world2map(Point(maxX, maxY));
    for (int x = min.x; x <= max.x; ++x)
    {
        for (int y = min.y; y <= max.y; ++y)
        {
            IntPoint id(x, y);
            GridAccumulator &cur = mapPtr->cell(id);
            // 更新栅格状态
            auto iter1 = hitCells.find(id);
            if (iter1 != hitCells.end())
            {
                cur.update(iter1->second, iter1->second);
                hitCells.erase(iter1);
            }
            auto iter2 = travelCells.find(id);
            if (iter2 != travelCells.end())
            {
                cur.update(iter2->second, 0);
                travelCells.erase(iter2);
            }
            // 更新AddSet, DelSet
            if (cur.getSta() == Sta::origin) // 如果栅格未初始化
            {
                if (cur > occThreshold) // 如果为占用栅格
                {
                    cur.setOcc(mapPtr->map2world(id));
                    addSet.insert(id);
                }
                else // 如果为空闲栅格, 其val初始化为最大值
                {
                    cur.setIdeal(mapPtr->map2world(id));
                }
            }
            // 如果全局地图不为障碍物而局部地图为障碍物
            else if (cur.getSta() == Sta::free)
            {
                if (cur > occThreshold)
                {
                    cur.setOcc(mapPtr->map2world(id));
                    addSet.insert(id);
                }
            }
            // 如果全局地图为障碍物而局部地图不为障碍物
            else if (cur.getSta() == Sta::occ)
            {
                if (cur <= occThreshold)
                {
                    cur.setIdeal(mapPtr->map2world(id));
                    delSet.insert(id);
                }
            }
        }
    }
    // std::cout << "hitSize: " << hitCells.size() << std::endl;
    // std::cout << "travelSize: " << travelCells.size() << std::endl;
    if (addSet.empty())
    {
        // 如果addSet为空，就将上次更新边缘栅格添加到addSet中，
        // 防止因为新添加观测帧因观测不到障碍物而无法更新地图。
        for (int x = min.x; x <= max.x; ++x)
            addSet.insert(IntPoint(x, min.y));
        for (int x = min.x; x <= max.x; ++x)
            addSet.insert(IntPoint(x, max.y));
        for (int y = min.y + 1; y < max.y; ++y)
            addSet.insert(IntPoint(min.x, y));
        for (int y = min.y + 1; y < max.y; ++y)
            addSet.insert(IntPoint(max.x, y));
    }
    update();
    double t = clock.end(MILLISEC);
    totalTime += t;
    meanTimePerFrame = totalTime / (double)count;
    minTimePerFrame = std::min(minTimePerFrame, t);
    maxTimePerFrame = std::max(maxTimePerFrame, t);
#ifdef LOG
    ROS_INFO("Using time: %f ms", t);
    ROS_INFO("meanTimePerFrame %f, minTimePerFrame: %f, maxTimePerFrame: %f", meanTimePerFrame, minTimePerFrame, maxTimePerFrame);
    ROS_INFO("meanCellsPerUpdate: %ld, minCellsPerUpdate: %ld, maxCellsPerUpdate: %ld", meanCellsPerUpdate, minCellsPerUpdate, maxCellsPerUpdate);
    ROS_INFO("------------------------------");
#endif

    distance_map_Vis();
    patchesVis();
    // std::cout<<"count "<<count<<std::endl;
    // toFile();
    // std::cout<<"count "<<count<<std::endl;

}

void IDM::update()
{
#ifdef LOG
    ROS_INFO("ADD %d Obstacles, DEL %d Obstacles.",
             static_cast<int>(addSet.size()),
             static_cast<int>(delSet.size()));
#endif
    for (auto addIdx : addSet)
    {
        updateQue.push(0, addIdx);
    }
    std::set<IntPoint, pointcomparator<int>> delArea;
    std::queue<IntPoint> Q;
    for (auto delIdx : delSet)
    {
        Grid &del = mapPtr->cell(delIdx);
        Q.push(delIdx);
        while (!Q.empty())
        {
            IntPoint curIdx = Q.front();
            Q.pop();
            for (size_t i = 0; i < nbrShift.size(); ++i)
            {
                IntPoint nbrIdx = curIdx + nbrShift[i];
                if (!mapPtr->isInside(nbrIdx))
                    continue;
                Grid &nbr = mapPtr->cell(nbrIdx);
                if ((nbr.getSta() == Sta::free) &&
                    (nbr.obs.x == del.pos.x) &&
                    (nbr.obs.y == del.pos.y))
                {
                    nbr.setIdeal();
                    Q.push(nbrIdx);
                    delArea.insert(nbrIdx);
                }
            }
        }
    }
    addSet.clear();
    delSet.clear();
    for (auto curIdx : delArea)
    {
        Grid &cur = mapPtr->cell(curIdx);
        bool flag = false;
        int square;
        for (size_t i = 0; i < nbrShift.size(); ++i)
        {
            IntPoint nbrIdx = curIdx + nbrShift[i];
            if (!mapPtr->isInside(nbrIdx))
                continue;
            Grid &nbr = mapPtr->cell(nbrIdx);
            if ((nbr.getSta() == Sta::origin) || (nbr.getSta() == Sta::ideal))
                continue;
            square = std::pow((cur.pos.x - nbr.obs.x) / mapResolution, 2) +
                     std::pow((cur.pos.y - nbr.obs.y) / mapResolution, 2);
            double dis = std::sqrt(square) * mapResolution;
            if (dis < cur.dis)
            {
                cur.setFree(cur.pos, nbr.obs);
                flag = true;
                square = square;
            }
        }
        if (flag)
            updateQue.push(square, curIdx);
    }
#ifdef LOG
    ROS_INFO("Update %d Grids.", updateQue.size());
#endif
    size_t cnt = 0;
    while (!updateQue.empty())
    {
        cnt++;
        IntPoint curIdx = updateQue.pop();
        Grid &cur = mapPtr->cell(curIdx);
        bool flag = false;
        int square;
        for (int i = 0; i < 8; ++i)
        {
            IntPoint nbrIdx = curIdx + nbrShift[i];
            if (!mapPtr->isInside(nbrIdx))
                continue;
            Grid &nbr = mapPtr->cell(nbrIdx);
            if (nbr.getSta() == Sta::origin)
                continue;
            square = (std::pow((cur.pos.x - nbr.obs.x) / mapResolution, 2) +
                      std::pow((cur.pos.y - nbr.obs.y) / mapResolution, 2));
            double dis = std::sqrt(square) * mapResolution;
            if (dis < cur.dis)
            {
                cur.setObs(nbr.obs);
                cur.setDis(dis);
                flag = true;
            }
        }
        if (flag)
        {
            updateQue.push(square, curIdx);
            continue;
        }
        for (int i = 0; i < 8; ++i)
        {
            IntPoint nbrIdx = curIdx + nbrShift[i];
            if (!mapPtr->isInside(nbrIdx))
                continue;
            Grid &nbr = mapPtr->cell(nbrIdx);
            if (nbr.getSta() == Sta::origin || nbr.getSta() == Sta::occ)
                continue;
            square = std::pow((nbr.pos.x - cur.obs.x) / mapResolution, 2) +
                     std::pow((nbr.pos.y - cur.obs.y) / mapResolution, 2);
            double dis = std::sqrt(square) * mapResolution;
            if (dis < nbr.dis)
            {
                if (nbr.getSta() == Sta::ideal)
                    nbr.setFree(nbr.pos, cur.obs);
                else
                {
                    nbr.setObs(cur.obs);
                    nbr.setDis(dis);
                }
                updateQue.push(square, nbrIdx);
            }
        }
    }
    totalCells += cnt;
    meanCellsPerUpdate = totalCells / count;
    minCellsPerUpdate = std::min(minCellsPerUpdate, cnt);
    maxCellsPerUpdate = std::max(maxCellsPerUpdate, cnt);
}