#pragma once

#include <algorithm>
#include <queue>
#include <memory>
#include <unordered_map>
#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <idm/DmMsg.h>
#include <Eigen/Eigen>
#include "params.hpp"
#include "node3d.hpp"
#include "rsspace.hpp"

class IHA
{
    typedef std::shared_ptr<Node3d> nptr;

public:
    IHA(ros::NodeHandle &nh) : nh_(nh)
    {
        subStart_ = nh.subscribe("/carla/ego_vehicle/odometry", 1, &IHA::startCallback, this);
        subGoal_ = nh.subscribe("/move_base_simple/goal", 1, &IHA::goalCallback, this);
        subDM_ = nh.subscribe("/DM", 1, &IHA::distCallback, this);
        // for visualization
        pubF_ = nh.advertise<geometry_msgs::PoseArray>("/Forward", 1, true);
        pubR_ = nh.advertise<geometry_msgs::PoseArray>("/Reverse", 1, true);
        pubPath_ = nh.advertise<nav_msgs::Path>("/ref_path", 1, true);
        planTimer_ = nh.createTimer(ros::Duration(1), &IHA::planLoop, this);
        rspGenerator_ = std::make_shared<RSSpace>(FLAGS_MinTurningRadius, FLAGS_MinStepSize);
        frontCenter_ = (3.0 * (FLAGS_Wheelbase + FLAGS_FrontHang) - FLAGS_RearHang) / 4.0;
        rearCenter_ = ((FLAGS_Wheelbase + FLAGS_FrontHang) - 3 * FLAGS_RearHang) / 4.0;
        safeDistance_ = sqrt(pow(FLAGS_VehicleWidth, 2) +
                             pow((FLAGS_Wheelbase + FLAGS_FrontHang + FLAGS_RearHang) / 4.0, 2));
    }
    ~IHA() = default;

private:
    // ros utils
    ros::NodeHandle nh_;
    ros::Subscriber subStart_;
    ros::Subscriber subGoal_;
    ros::Subscriber subDM_;
    ros::Publisher pubF_, pubR_;
    ros::Publisher pubPath_;
    ros::Timer planTimer_;
    void startCallback(const nav_msgs::OdometryConstPtr &);
    void goalCallback(const geometry_msgs::PoseStampedConstPtr &);
    void distCallback(const idm::DmMsgConstPtr &);
    // task utils
    Eigen::Vector3d start_;
    Eigen::Vector3d goal_;
    Eigen::Vector3d origin_;
    Eigen::MatrixXd dist_;
    double resolution_;
    bool valid_start_ = false;
    bool valid_goal_ = false;
    bool valid_dist_ = false;
    nptr snp_;
    nptr gnp_;
    // hybrid astar utils
    template <typename T1, typename T2>
    struct Comparator
    {
        bool operator()(
            const std::pair<T1, T2> &l,
            const std::pair<T1, T2> &r) const
        {
            return l.first >= r.first;
        }
    };
    typedef std::pair<double, std::string> Elem;
    typedef std::vector<std::pair<double, std::string>> ElemVec;
    typedef std::priority_queue<Elem, ElemVec, Comparator<double, std::string>> PrioQue;
    typedef std::unordered_map<std::string, std::shared_ptr<Node3d>> HashSet;
    struct Path
    {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> phi;
        
    };
    std::shared_ptr<RSSpace> rspGenerator_;
    // collision check uitls
    double frontCenter_;  // 车辆后轴至前1/4点
    double rearCenter_;   // 车辆后轴至后1/4点
    double safeDistance_; // 无碰撞距离
    enum CollisionCheckResult
    {
        Collision, // 碰撞
        FreeSpace, // 安全
        OutsideMap // 未知
    };
    typedef CollisionCheckResult CCR;
    /**
     * @brief 碰撞检测函数：
     * @param x: 地图坐标系下x坐标
     * @param y: 地图坐标系下y坐标
     * @param phi: 地图坐标系下phi坐标
     * @param DM: 距离地图
     */
    CCR collisionCheck(const double &x,
                       const double &y,
                       const double &phi,
                       const Eigen::MatrixXd &DM);
    // functions
    /**
     * @brief 规划主逻辑函数
     */
    void planLoop(const ros::TimerEvent &);
    /**
     * @brief 路径规划函数
     * @param start:地图坐标系下起始位姿
     * @param end: 地图坐标系下终点位姿
     * @param origin: 地图相对世界坐标系位姿
     * @param DM: 距离地图
     */
    std::shared_ptr<Path> pathSearch(const Eigen::Vector3d &start,
                                     const Eigen::Vector3d &end,
                                     const Eigen::Vector3d &origin,
                                     const Eigen::MatrixXd &DM);
    /**
     * @brief A*启发式计算函数
     * @param x: 地图x轴栅格坐标
     * @param y: 地图y轴栅格坐标
     * @param DM: 距离地图
     * @param AstarLookup: A*启发式表
     */
    void updateAstarLookup(const int x,
                           const int y,
                           const Eigen::MatrixXd &DM,
                           Eigen::MatrixXi &AstarLookup);

    /**
     * @brief Reeds Shepp Conenction函数
     * @param curS: 当前起始
     * @param curG: 当前终点
     * @param DM: 距离地图
     * @param result: 结果路径
     */
    bool rspConnection(nptr &curS,
                       nptr &curG,
                       Eigen::MatrixXd &DM,
                       std::shared_ptr<Path> &result);
    /**
     * @brief 节点拓展函数
     * @param curr: 当前节点指针
     * @param from: 拓展方向
     * @param id: 拓展id
     * @param DM: 距离地图
     */
    nptr calcNextNode(const nptr &curr,
                      const bool &from,
                      const int &id,
                      const Eigen::MatrixXd &DM);
    /**
     * @brief 节点拓展Cost计算函数
     * @param curr: 父亲节点
     * @param next: 儿子节点
     */
    double wptsCost(const nptr &curr, const nptr &next);

    /**
     * @brief AstarCost函数
     * @param x: 地图x轴栅格坐标
     * @param y: 地图y轴栅格坐标
     * @param astarLookup: AstarLookup表
     */
    double astarCost(const int &x, const int &y, const Eigen::MatrixXi &astarLookup);

    /**
     * @brief ObstacleCost函数
     * @param x: 地图坐标系x轴坐标
     * @param y: 地图坐标系y轴坐标
     * @param DM: 距离地图
     */
    double obstCost(const double &x, const double &y, const Eigen::MatrixXd &DM);
};

void IHA::startCallback(const nav_msgs::OdometryConstPtr &msg)
{
    start_ << msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        tf::getYaw(msg->pose.pose.orientation);
    // std::cout << "Start: " << goal_.transpose() << std::endl;
    valid_start_ = true;
}

#include <geometry_msgs/Quaternion.h>
void IHA::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    goal_ << msg->pose.position.x,
        msg->pose.position.y,
        tf::getYaw(msg->pose.orientation);

    std::cout << "Goal: " << goal_.transpose() << std::endl;
    valid_goal_ = true;
}

void IHA::distCallback(const idm::DmMsgConstPtr &msg)
{
    origin_ << msg->info.origin.position.x,
        msg->info.origin.position.y,
        tf::getYaw(msg->info.origin.orientation);
    dist_.resize(msg->info.width, msg->info.height);
    for (int x = 0; x < msg->info.width; ++x)
    {
        for (int y = 0; y < msg->info.height; ++y)
        {
            dist_(x, y) = msg->dist.at(x + y * msg->info.width);
        }
    }
    resolution_ = msg->info.resolution;
    valid_dist_ = true;
}

#include <ctime>
#include <iostream>
#include <fstream>

void IHA::planLoop(const ros::TimerEvent &event)
{

    // geometry_msgs::Quaternion quat_msg;
    // quat_msg.x = 0;
    // quat_msg.y = 0;
    // quat_msg.z = -0.9999874978841718;
    // quat_msg.w = 0.005000407518734412;

    // goal_ <<-12.8, 21.5, tf::getYaw(quat_msg);

    // std::cout << "Goal: " << goal_.transpose() << std::endl;
    // valid_goal_ = true;
    // std::cout<<"valid_start_ " << valid_start_ << " valid_dist_ "<<valid_dist_<<std::endl;

    if (valid_start_ && valid_goal_ && valid_dist_)
    {
        
        // 将起点和终点转到地图坐标系
        Eigen::Vector3d s(
            start_(0) - origin_(0),
            start_(1) - origin_(1),
            start_(2));
        Eigen::Vector3d g(
            goal_(0) - origin_(0),
            goal_(1) - origin_(1),
            goal_(2));

        // 规划安全路径

        std::clock_t start = std::clock();
        std::shared_ptr<Path> result = pathSearch(s, g, origin_, dist_);
    
        std::clock_t end = std::clock();
        double elapsed = static_cast<double>(end - start) / CLOCKS_PER_SEC;
        std::cout << "pathSearch time: " << elapsed << " seconds" << std::endl;        


            // 打开一个文件以写入
        std::ofstream outfile("/home/chen/ws_ros/src/output.txt");
        if (!outfile) {
            std::cerr << "无法打开文件" << std::endl;
            // return 1;
        }

        nav_msgs::Path msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        for (size_t i = 0; i < result->x.size(); ++i)
        {
            geometry_msgs::PoseStamped pos;
            pos.pose.position.x = result->x.at(i) + origin_(0);
            pos.pose.position.y = result->y.at(i) + origin_(1);
            pos.pose.position.z = result->y.at(i) + origin_(1);
            pos.pose.orientation = tf::createQuaternionMsgFromYaw(result->phi.at(i));
            msg.poses.push_back(pos);


            outfile << pos.pose.position.x << "," 
                    << pos.pose.position.y << "," 
                    << result->phi.at(i) << std::endl;

        }

        // 关闭文件
        outfile.close();

        std::cout << "路径已成功写入文件" << std::endl;

        pubPath_.publish(msg);
    }
    if (!valid_start_)
    {
        ROS_ERROR("Need a start!!!");
    }
    if (!valid_goal_)
    {
        ROS_INFO("Need a goal!");
    }
    if (!valid_dist_)
    {
        ROS_ERROR("Need a distance map!!!");
    }
}

std::shared_ptr<IHA::Path> IHA::pathSearch(const Eigen::Vector3d &start,
                                           const Eigen::Vector3d &goal,
                                           const Eigen::Vector3d &origin,
                                           const Eigen::MatrixXd &dist)
{

    std::cout << "start in map: " << start.transpose()
                << ", goal in map: " << goal.transpose() 
                << ", origin in map: " << origin.transpose() 
                << std::endl;
    Eigen::Vector3d ori = origin;
    Eigen::MatrixXd dm = dist;                // 距离地图
    // std::cout << "dm test!!!!!!!!!!!! " << dm << std::endl; 
    int w = dm.rows();                        // 栅格地图宽度
    int h = dm.cols();                        // 栅格地图高度
    auto inMap = [w, h](int x, int y) -> bool // 检测某一栅格是否在地图内
    { return ((0 <= x && x < w) && (0 <= y && y < h)); };

    snp_ = std::make_shared<Node3d>(start(0), start(1), start(2)); // 起点节点智能指针
    Eigen::MatrixXi astarLookup;
    updateAstarLookup(snp_->getIdx(), snp_->getIdy(), dm, astarLookup);

    gnp_ = std::make_shared<Node3d>(goal(0), goal(1), goal(2)); // 终点节点智能指针
    bool goalInMap = inMap(gnp_->getIdx(), gnp_->getIdy());
    
    PrioQue sPrioque;
    HashSet sOpenSet, sCloseSet;
    sPrioque.emplace(snp_->getCost(), snp_->getIndex());
    // if(snp_->getIndex() == "97-146--26")
    // {
    //     std::cout<<" zzzzzzzzzzzzz"<<std::endl;
    // }

    sOpenSet.emplace(snp_->getIndex(), snp_);

    PrioQue gPrioQue;
    HashSet gOpenSet, gCloseSet;
    gPrioQue.emplace(gnp_->getCost(), gnp_->getIndex());
    gOpenSet.emplace(gnp_->getIndex(), gnp_);
    
    nptr curS = snp_;
    nptr curG = gnp_;
    geometry_msgs::PoseArray Forwards, Reverses;
    Forwards.header.frame_id = "map";
    Reverses.header.frame_id = "map";
    std::shared_ptr<Path> result = std::make_shared<Path>();
    int idd =0;
    while (!sPrioque.empty() && !gPrioQue.empty())
    {
        pubF_.publish(Forwards);
        pubR_.publish(Reverses);
        std::string curId;

        curId = sPrioque.top().second;
        curS = sOpenSet[curId];
        sPrioque.pop();
        if (curS == nullptr)
        {
            // std::cout<<" zzzzzzzzzzdddddd"<<std::endl;
            continue;
        }
        
        // std::cout<<" i="<<idd << " ,curId="<<curId<<std::endl;
        // bug
        idd++;
        // std::cout<<" sssssss"<<std::endl;
        // std::cout<<" i="<<idd << " ,curId="<<curS->getIndex()<<std::endl;
        sOpenSet.erase(curS->getIndex());

        sCloseSet.emplace(curS->getIndex(), curS);

        curId = gPrioQue.top().second;
        curG = gOpenSet[curId];
        gPrioQue.pop();

        if (curG == nullptr)
        {
            // std::cout<<" fffffffffffffffffff"<<std::endl;
            continue;
        }
        // std::cout<<" tttttttttttttt"<<std::endl;
        // std::cout<<" i="<<idd << " ,curId="<<curG->getIndex()<<std::endl;

        gOpenSet.erase(curG->getIndex());
        gCloseSet.emplace(curG->getIndex(), curG);

        // 如果满足条件则使用reedsSheppCurveConnection
        if (rspConnection(curS, curG, dm, result))
        {
            return result;
        }

        // 正向节点拓展
        for (int i = 0; i < FLAGS_SteerClassNumber * 2; ++i)
        {
            nptr cur_next = calcNextNode(curS, true, i, dist);
            if (cur_next == nullptr)
            {
                continue;
            }
            if (sCloseSet.find(cur_next->getIndex()) != sCloseSet.end())
            {
                continue;
            }
            else
            {
                cur_next->setG(wptsCost(curS, cur_next));
                cur_next->setH(astarCost(cur_next->getIdx(), cur_next->getIdy(), astarLookup));
                auto iter = sOpenSet.find(cur_next->getIndex());
                if (iter != sOpenSet.end())
                {
                    if (iter->second->getCost() > cur_next->getCost())
                    {
                        iter->second = cur_next;
                        sPrioque.emplace(cur_next->getCost(), cur_next->getIndex());
                        // std::cout<<" 111111111111111111"<<std::endl;
                        // std::cout<<" i="<<idd << " ,cur_next="<<cur_next->getIndex()<<std::endl;

                    }
                }
                else
                {
                    sOpenSet.emplace(cur_next->getIndex(), cur_next);
                    sPrioque.emplace(cur_next->getCost(), cur_next->getIndex());
                    // std::cout<<" 222222222222222"<<std::endl;
                    // std::cout<<" i="<<idd << " ,cur_next="<<cur_next->getIndex()<<std::endl;

                    geometry_msgs::Pose pos;
                    pos.position.x = cur_next->getX() + ori(0);
                    pos.position.y = cur_next->getY() + ori(1);
                    pos.orientation = tf::createQuaternionMsgFromYaw(cur_next->getPhi());
                    Forwards.poses.push_back(pos);
                }
            }
        }

        // 逆向节点扩展
        for (int i = 0; i < FLAGS_SteerClassNumber * 2; ++i)
        {
            nptr cur_next = calcNextNode(curG, false, i, dist);
            if (cur_next == nullptr)
            {
                continue;
            }
            if (gCloseSet.find(cur_next->getIndex()) != sCloseSet.end())
            {
                continue;
            }
            else
            {
                cur_next->setG(wptsCost(curG, cur_next));
                auto iter = gOpenSet.find(cur_next->getIndex());
                if (iter != gOpenSet.end())
                {
                    if (iter->second->getCost() > cur_next->getCost())
                    {
                        iter->second = cur_next;
                        gPrioQue.emplace(cur_next->getCost(), cur_next->getIndex());
                    }
                }
                else
                {
                    gOpenSet.emplace(cur_next->getIndex(), cur_next);
                    gPrioQue.emplace(cur_next->getCost(), cur_next->getIndex());
                    geometry_msgs::Pose pos;
                    pos.position.x = cur_next->getX() + ori(0);
                    pos.position.y = cur_next->getY() + ori(1);
                    pos.orientation = tf::createQuaternionMsgFromYaw(cur_next->getPhi());
                    Reverses.poses.push_back(pos);
                }
            }
        }
    }
    return nullptr;
}

void IHA::updateAstarLookup(const int x, const int y, const Eigen::MatrixXd &dist, Eigen::MatrixXi &astarLookup)
{
    int map_width_ = dist.rows();
    int map_height_ = dist.cols();
    astarLookup.resize(map_width_, map_height_);
    int *cor = new int[2 * (map_width_ + map_height_) * 2 * 2];
    short *status = new short[map_width_ * map_height_];
    memset(status, 0x00, sizeof(short) * map_width_ * map_height_);
    int heapIndex = 0;
    int len[2] = {0, 0};             // 内层和外层节点个数
    astarLookup(x, y) = 0;           // 终点到自身的距离为0
    status[x * map_height_ + y] = 1; // 终点状态由0变为1，代表已经探索过
    cor[2 * heapIndex + 0] = x;
    cor[2 * heapIndex + 1] = y;
    len[heapIndex]++; // len[0]变为1
    // 双层循环，内循环里面的一次循环是将上次拓展的其中一个节点进行拓展
    do
    {
        while (len[heapIndex] > 0)
        {
            int l = len[heapIndex] - 1;
            int i = cor[l * 2 * 2 + heapIndex * 2 + 0];
            int j = cor[l * 2 * 2 + heapIndex * 2 + 1];
            int iup = std::min(i + 1, map_width_ - 1);
            int idown = std::max(i - 1, 0);
            int jup = std::min(j + 1, map_height_ - 1);
            int jdown = std::max(j - 1, 0);
            if (status[iup * map_height_ + j] == 0 && dist(iup, j) != 0)
            {
                astarLookup(iup, j) = astarLookup(i, j) + 1;
                status[iup * map_height_ + j] = 1;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 0] = iup;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 1] = j;
                len[1 - heapIndex]++;
            }
            if (status[idown * map_height_ + j] == 0 && dist(idown, j) != 0)
            {
                astarLookup(idown, j) = astarLookup(i, j) + 1;
                status[idown * map_height_ + j] = 1;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 0] = idown;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 1] = j;
                len[1 - heapIndex]++;
            }
            if (status[i * map_height_ + jup] == 0 && dist(i, jup) != 0)
            {
                astarLookup(i, jup) = astarLookup(i, j) + 1;
                status[i * map_height_ + jup] = 1;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 0] = i;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 1] = jup;
                len[1 - heapIndex]++;
            }
            if (status[i * map_height_ + jdown] == 0 && dist(i, jdown) != 0)
            {
                astarLookup(i, jdown) = astarLookup(i, j) + 1;
                status[i * map_height_ + jdown] = 1;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 0] = i;
                cor[len[1 - heapIndex] * 2 * 2 + (1 - heapIndex) * 2 + 1] = jdown;
                len[1 - heapIndex]++;
            }
            len[heapIndex]--;
        }
        heapIndex = 1 - heapIndex;
    } while (len[heapIndex] > 0);
    delete[] cor;
    delete[] status;
    cor = nullptr;
    status = nullptr;
}

bool IHA::rspConnection(nptr &begin, nptr &final, Eigen::MatrixXd &dist, std::shared_ptr<Path> &result)
{
    std::shared_ptr<RSSpace::RSPath> rsp = std::make_shared<RSSpace::RSPath>();
    if (!rspGenerator_->shortestRSP(begin->getX(), begin->getY(), begin->getPhi(),
                                    final->getX(), final->getY(), final->getPhi(),
                                    rsp))
    {
        // ROS_INFO("Rsp connection failed!");
        return false;
    }
    for (size_t i = 0; i < rsp->x.size(); ++i)
    {
        if (collisionCheck(rsp->x.at(i), rsp->y.at(i), rsp->phi.at(i), dist) == Collision)
        {
            // ROS_INFO("Rsp connection collide!");
            return false;
        }
    }
    // ROS_INFO("Rsp connection succeed!");
    std::vector<double> result_xs;
    std::vector<double> result_ys;
    std::vector<double> result_phis;
    // ROS_INFO("backtracking forward path!");
    nptr temp_begin = begin;
    while (temp_begin->getPred() != nullptr)
    {
        std::vector<double> xs = temp_begin->getXs();
        std::vector<double> ys = temp_begin->getYs();
        std::vector<double> phis = temp_begin->getPhis();
        result_xs.insert(result_xs.begin(), xs.begin(), xs.end());
        result_ys.insert(result_ys.begin(), ys.begin(), ys.end());
        result_phis.insert(result_phis.begin(), phis.begin(), phis.end());
        temp_begin = temp_begin->getPred();
    }
    result_xs.emplace(result_xs.begin(), snp_->getX());
    result_ys.emplace(result_ys.begin(), snp_->getY());
    result_phis.emplace(result_phis.begin(), snp_->getPhi());

    // ROS_INFO("backtracking rs path!");
    result_xs.insert(result_xs.end(), rsp->x.begin() + 1, rsp->x.end() - 1);
    result_ys.insert(result_ys.end(), rsp->y.begin() + 1, rsp->y.end() - 1);
    result_phis.insert(result_phis.end(), rsp->phi.begin() + 1, rsp->phi.end() - 1);
    // ROS_INFO("backtracking reverse path!");
    
    nptr temp_final = final;
    while (temp_final->getPred() != nullptr)
    {
        std::vector<double> xs = temp_final->getXs();
        std::vector<double> ys = temp_final->getYs();
        std::vector<double> phis = temp_final->getPhis();
        std::reverse(xs.begin(), xs.end());
        std::reverse(ys.begin(), ys.end());
        std::reverse(phis.begin(), phis.end());
        result_xs.insert(result_xs.end(), xs.begin(), xs.end());
        result_ys.insert(result_ys.end(), ys.begin(), ys.end());
        result_phis.insert(result_phis.end(), phis.begin(), phis.end());
        temp_final = temp_final->getPred();
    }
    result_xs.emplace(result_xs.end(), gnp_->getX());
    result_ys.emplace(result_ys.end(), gnp_->getY());
    result_phis.emplace(result_phis.end(), gnp_->getPhi());
    result->x = result_xs;
    result->y = result_ys;
    result->phi = result_phis;
    return true;
}

IHA::nptr IHA::calcNextNode(const nptr &cur, const bool &from, const int &idx, const Eigen::MatrixXd &dist)
{
    std::vector<double> xs, ys, phis;
    int8_t wpt_num = 0;
    double steering = 0.0;
    double traveled_distance = 0.0;
    double steer_delta = 2 * FLAGS_MaxSteer / (FLAGS_SteerClassNumber - 1);
    if (idx < FLAGS_SteerClassNumber)
    {
        steering = -FLAGS_MaxSteer + steer_delta * idx;
        traveled_distance = FLAGS_MinStepSize;
    }
    else
    {
        steering = -FLAGS_MaxSteer + steer_delta * (idx - FLAGS_SteerClassNumber);
        traveled_distance = -FLAGS_MinStepSize;
    }
    double obs_cost = 0.0;
    double last_x = cur->getX();
    double last_y = cur->getY();
    double last_phi = cur->getPhi();
    for (int8_t i = 1; i <= FLAGS_StepClassNumber; ++i)
    {
        double x = last_x + traveled_distance * cos(last_phi);
        double y = last_y + traveled_distance * sin(last_phi);
        double phi = last_phi + traveled_distance / FLAGS_Wheelbase * tan(steering);
        CCR sta = collisionCheck(x, y, phi, dist);
        if (sta == Collision)
        {
            break;
        }
        else
        {
            wpt_num++;
            xs.push_back(x);
            ys.push_back(y);
            phis.push_back(phi);
            obs_cost += obstCost(x, y, dist);
        }
        last_x = x;
        last_y = y;
        last_phi = phi;
    }
    if (wpt_num == 0)
    {
        return nullptr;
    }
    nptr next = std::make_shared<Node3d>(xs, ys, phis);
    next->setSteer(steering);
    if (from)
        next->setGear(traveled_distance > 0.0);
    else
        next->setGear(traveled_distance < 0.0);
    if (cur->getGear() == next->getGear())
    {
        next->setKeep(wpt_num + cur->getWptNum());
    }
    else
    {
        next->setKeep(wpt_num);
    }
    next->setPred(cur);
    next->setO(obs_cost / wpt_num);
    return next;
}

IHA::CCR IHA::collisionCheck(const double &x, const double &y, const double &phi, const Eigen::MatrixXd &dist)
{
    // std::cout << x << ", " << y << ", " << phi << std::endl;
    int width_ = dist.rows();
    int height_ = dist.cols();
    int x1 = std::floor((x + frontCenter_ * cos(phi)) / FLAGS_MapResolution);
    int y1 = std::floor((y + frontCenter_ * sin(phi)) / FLAGS_MapResolution);
    int x2 = std::floor((x + rearCenter_ * cos(phi)) / FLAGS_MapResolution);
    int y2 = std::floor((y + rearCenter_ * sin(phi)) / FLAGS_MapResolution);
    // std::cout << x1 << ", " << y1 << ", " << x2 << ", " << y2 << std::endl;
    if (x1 < 0 || x1 >= width_ || y1 < 0 || y1 >= height_ ||
        x2 < 0 || x2 >= width_ || y2 < 0 || y2 >= height_) // 需要在地图内
    {
        return OutsideMap;
    }
    // std::cout << safeDistance_ << ", " << dist(x1, y1) << ", " << dist(x2, y2) << std::endl;
    if (isinf(dist(x1, y1)) || isinf(dist(x2, y2))) // 需要在已经观测到的地图内
    {
        return OutsideMap;
    }
    if (dist(x1, y1) < safeDistance_ ||
        dist(x2, y2) < safeDistance_) // distance map collision check
    {
        return Collision;
    }
    return FreeSpace;
}

double IHA::wptsCost(const nptr &curr, const nptr &next)
{
    double piecewise_cost = (next->getWptNum() - 1) * FLAGS_MinStepSize;
    if (!next->getGear()) // 后退
    {
        piecewise_cost *= (1 + FLAGS_ReversePenalty);
    }
    if (next->getKeep() < FLAGS_MinKeepStep) // 档位切换
    {
        piecewise_cost *= (1 + (FLAGS_MinKeepStep - next->getKeep()) * FLAGS_KeepPenalty);
    }
    piecewise_cost *= (1 + abs(curr->getSteer() - next->getSteer()) * FLAGS_SteerPenalty);
    return piecewise_cost + curr->getG();
}

double IHA::astarCost(const int &x, const int &y, const Eigen::MatrixXi &astarLookup)
{
    int w = astarLookup.rows();                               // 查找表宽度
    int h = astarLookup.cols();                               // 查找表高度
    auto inTable = [w, h](const int &x, const int &y) -> bool // 检测某一栅格是否在表内
    { return ((0 <= x && x < w) && (0 <= y && y < h)); };
    if (inTable(x, y))
    {
        return astarLookup(x, y) * FLAGS_MapResolution;
    }
    else
    {
        return astarLookup.maxCoeff();
    }
}

double IHA::obstCost(const double &x, const double &y, const Eigen::MatrixXd &DM)
{
    int w = DM.rows();                                      // 栅格地图宽度
    int h = DM.cols();                                      // 栅格地图高度
    auto inMap = [w, h](const int &x, const int &y) -> bool // 检测某一栅格是否在地图内
    { return ((0 <= x && x < w) && (0 <= y && y < h)); };
    int x_ = std::floor(x / FLAGS_MapResolution);
    int y_ = std::floor(y / FLAGS_MapResolution);
    if (inMap(x_, y_) && (!isinf(DM(x_, y_))))
    {
        return 1.0 / DM(x_, y_);
    }
    else
    {
        return 1.0 / safeDistance_;
    }
}
