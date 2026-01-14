#pragma once

#include <cmath>
#include <string>
#include <memory>
#include <vector>

#include "params.hpp"

class Node3d
{
    typedef std::shared_ptr<Node3d> nptr;

public:
    // construct
    Node3d() = default;
    Node3d(double x, double y, double phi) : x_(x), y_(y), phi_(phi)
    {
        traversed_x_.push_back(x);
        traversed_y_.push_back(y);
        traversed_phi_.push_back(phi);
        idx_ = static_cast<int>(x_ / FLAGS_MapResolution);
        idy_ = static_cast<int>(y_ / FLAGS_MapResolution);
        idphi_ = static_cast<int>(phi_ / FLAGS_HeadingResolution);
        index_ = std::to_string(idx_) + "-" + std::to_string(idy_) + "-" +
                 std::to_string(idphi_);
    }
    Node3d(std::vector<double> &xs, std::vector<double> &ys, std::vector<double> &phis)
    {
        x_ = xs.back();
        y_ = ys.back();
        phi_ = phis.back();
        traversed_x_ = xs;
        traversed_y_ = ys;
        traversed_phi_ = phis;
        wpt_num = traversed_x_.size();
        idx_ = static_cast<int>(x_ / FLAGS_MapResolution);
        idy_ = static_cast<int>(y_ / FLAGS_MapResolution);
        idphi_ = static_cast<int>(phi_ / FLAGS_HeadingResolution);
        index_ = std::to_string(idx_) + "-" + std::to_string(idy_) + "-" +
                 std::to_string(idphi_);
    }
    virtual ~Node3d() = default;
    // get
    inline double getX() const { return x_; }
    inline double getY() const { return y_; }
    inline double getPhi() const { return phi_; }
    inline std::vector<double> getXs() const { return traversed_x_; }
    inline std::vector<double> getYs() const { return traversed_y_; }
    inline std::vector<double> getPhis() const { return traversed_phi_; }
    inline uint getWptNum() const { return wpt_num; }
    inline int getIdx() const { return idx_; }
    inline int getIdy() const { return idy_; }
    inline int getIdphi() const { return idphi_; }
    inline double getSteer() const { return steer_; }
    inline bool getGear() const { return gear_; }
    inline nptr getPred() const { return pred_; }
    inline double getG() const { return g_; }
    inline double getH() const { return h_; }
    inline double getO() const { return o_; }
    inline int getKeep() const { return keep_; }
    inline double getCost() const { return FLAGS_WeightCostToCome * g_ +
                                           FLAGS_WeightCostToGoal * h_ +
                                           FLAGS_WeightCostToObstacle * o_; }
    inline std::string getIndex() const { return index_; }
    // set
    inline void setSteer(double steer) { steer_ = steer; }
    inline void setGear(bool gear) { gear_ = gear; }
    inline void setPred(nptr pred) { pred_ = pred; }
    inline void setG(double g) { g_ = g; }
    inline void setH(double h) { h_ = h; }
    inline void setO(double i) { o_ = i; }
    inline void setKeep(double keep) { keep_ = keep; }

private:
    double x_ = 0.;   // corrdinate x in occupancy map
    double y_ = 0.;   // corrdinate y in occupancy map
    double phi_ = 0.; // phi angle in occupancy map
    std::vector<double> traversed_x_;
    std::vector<double> traversed_y_;
    std::vector<double> traversed_phi_;
    uint wpt_num = 1;     // waypoint number
    int idx_ = 0;         // index x in node map
    int idy_ = 0;         // index y in node map
    int idphi_ = 0;       // index yaw in node map
    double steer_ = 0.;   // steer angle
    bool gear_ = true;    // true for forward and false for reverse;
    nptr pred_ = nullptr; // pointer to pred node
    double g_ = 0.;       // cost to come
    double h_ = 0.;       // cost to goal
    double o_ = 0.;       // cost to obstacle
    int keep_ = 0;        // keep in one direction
    std::string index_;   // index of node
};