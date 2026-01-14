#pragma once

#include <cmath>
#include <vector>
#include <limits>
#include <memory>
#include <utility>
#include <iostream>

class RSSpace
{
public:
    struct RSPath
    {
        double total_len = 0.;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> phi;
        std::vector<double> seg_len;
        std::vector<char> seg_type;
        std::vector<bool> seg_gear;
    };
    struct RSPParam
    {
        bool flag = false;
        double t = 0.;
        double u = 0.;
        double v = 0.;
    };
    RSSpace(double rho, double stepsz) : rho_(rho), stepsz_(stepsz) {}
    // normalize radius to [-pi, pi)
    double mod2pi(double rad)
    {
        double a = fmod(rad + M_PI, 2.0 * M_PI);
        if (a < 0.0)
        {
            a += (2.0 * M_PI);
        }
        return a - M_PI;
    }
    // transform to polar coordinate
    std::pair<double, double> polar(double x, double y)
    {
        double r = sqrt(x * x + y * y);
        double theta = atan2(y, x);
        return std::pair<double, double>(r, theta);
    }
    std::pair<double, double> calcTauOmega(const double u, const double v, const double xi,
                                           const double eta, const double phi);
    double distance(const double x0, const double y0, const double phi0,
                    const double x1, const double y1, const double phi1);
    bool shortestRSP(const double x0, const double y0, const double phi0,
                     const double x1, const double y1, const double phi1,
                     std::shared_ptr<RSPath> optimal_path);
    bool generateRSP(const double x0, const double y0, const double phi0,
                     const double x1, const double y1, const double phi1,
                     std::vector<RSPath> *possible_paths);
    bool setRSP(const int size, const double *lengths, const std::string &types,
                std::vector<RSPath> *possible_paths, const int idx);
    void SLS(const double x, const double y, const double phi, RSPParam *param);
    void LSL(const double x, const double y, const double phi, RSPParam *param);
    void LSR(const double x, const double y, const double phi, RSPParam *param);
    void LRL(const double x, const double y, const double phi, RSPParam *param);
    void LRLRn(const double x, const double y, const double phi, RSPParam *param);
    void LRLRp(const double x, const double y, const double phi, RSPParam *param);
    void LRSR(const double x, const double y, const double phi, RSPParam *param);
    void LRSL(const double x, const double y, const double phi, RSPParam *param);
    void LRSLR(const double x, const double y, const double phi, RSPParam *param);
    bool generateConfig(const double x0, const double y0, const double phi0,
                        const double x1, const double y1, const double phi1,
                        RSPath *optimal_path);
    void interpolation(const int index, const double pd, const char m,
                       const double ox, const double oy, const double ophi,
                       std::vector<double> *px, std::vector<double> *py,
                       std::vector<double> *pphi, std::vector<bool> *pgear);

private:
    double rho_;    // TURNNING RADIUS
    double stepsz_; // STEP SIZE
};

std::pair<double, double> RSSpace::calcTauOmega(const double u, const double v, const double xi,
                                                const double eta, const double phi)

{
    double delta = mod2pi(u - v);
    double A = std::sin(u) - std::sin(delta);
    double B = std::cos(u) - std::cos(delta) - 1.0;
    double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
    double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
    double tau = (t2 < 0) ? mod2pi(t1 + M_PI) : mod2pi(t1);
    double omega = mod2pi(tau - u + v - phi);
    return std::pair<double, double>(tau, omega);
}

double RSSpace::distance(const double x0, const double y0, const double phi0,
                         const double x1, const double y1, const double phi1)
{
    double d = std::numeric_limits<double>::infinity();
    std::vector<RSPath> possible_paths;
    if (generateRSP(x0, y0, phi0, x1, y1, phi1, &possible_paths))
    {
        for (size_t i = 0; i < possible_paths.size(); ++i)
        {
            if (possible_paths.at(i).total_len > 0 &&
                possible_paths.at(i).total_len < d)
            {
                d = possible_paths.at(i).total_len;
            }
        }
    }
    return d;
}

bool RSSpace::shortestRSP(const double x0, const double y0, const double phi0,
                          const double x1, const double y1, const double phi1,
                          std::shared_ptr<RSPath> optimal_path)
{
    std::vector<RSPath> possible_paths;
    if (!generateRSP(x0, y0, phi0, x1, y1, phi1, &possible_paths))
    {
        std::cout << "Fail to generate different combination of Reed Shepp Paths" << std::endl;
        return false;
    }
    size_t optimal_path_index = 0;
    double optimal_path_length = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < possible_paths.size(); ++i)
    {
        if (possible_paths.at(i).total_len > 0 &&
            possible_paths.at(i).total_len < optimal_path_length)
        {
            optimal_path_index = i;
            optimal_path_length = possible_paths.at(i).total_len;
        }
    }
    if (!generateConfig(x0, y0, phi0, x1, y1, phi1, &(possible_paths.at(optimal_path_index))))
    {
        return false;
    }
    if (std::abs(possible_paths[optimal_path_index].x.back() -
                 x1) > 1e-3 ||
        std::abs(possible_paths[optimal_path_index].y.back() -
                 y1) > 1e-3 ||
        std::abs(possible_paths[optimal_path_index].phi.back() -
                 mod2pi(phi1)) > 1e-3)
    {
        std::cout << "RSP end position not right" << std::endl;
        return false;
    }
    (*optimal_path).x = possible_paths[optimal_path_index].x;
    (*optimal_path).y = possible_paths[optimal_path_index].y;
    (*optimal_path).phi = possible_paths[optimal_path_index].phi;
    (*optimal_path).total_len = possible_paths[optimal_path_index].total_len;
    (*optimal_path).seg_gear = possible_paths[optimal_path_index].seg_gear;
    (*optimal_path).seg_type = possible_paths[optimal_path_index].seg_type;
    (*optimal_path).seg_len = possible_paths[optimal_path_index].seg_len;
    return true;
}

bool RSSpace::generateRSP(const double x0, const double y0, const double phi0,
                          const double x1, const double y1, const double phi1,
                          std::vector<RSPath> *possible_paths)
{
    double dx = x1 - x0, dy = y1 - y0, dphi = phi1 - phi0;
    double c = cos(phi0), s = sin(phi0);
    double x = (c * dx + s * dy) / rho_, y = (-s * dx + c * dy) / rho_;
    double xb = x * cos(dphi) + y * sin(dphi);
    double yb = x * sin(dphi) - y * cos(dphi);
    int RSP_num = 46;
    possible_paths->resize(RSP_num);
    bool succ = true;
#pragma omp parallel for schedule(dynamic, 2) num_threads(8)
    for (int i = 0; i < RSP_num; ++i)
    {
        RSPParam RSP_param;
        int tmp_length = 0;
        double RSP_lengths[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
        double x_param = 1.0;
        double y_param = 1.0;
        std::string rsp_type;
        if (i > 2 && i % 2)
        {
            x_param = -1.0;
        }
        if (i > 2 && i % 4 < 2)
        {
            y_param = -1.0;
        }
        if (i < 2)
        { // SCS case
            if (i == 1)
            {
                y_param = -1.0;
                rsp_type = "SRS";
            }
            else
            {
                rsp_type = "SLS";
            }
            SLS(x, y_param * y, y_param * dphi, &RSP_param);
            tmp_length = 3;
            RSP_lengths[0] = RSP_param.t;
            RSP_lengths[1] = RSP_param.u;
            RSP_lengths[2] = RSP_param.v;
        }
        else if (i < 6)
        { // CSC, LSL case
            LSL(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
            if (y_param > 0)
            {
                rsp_type = "LSL";
            }
            else
            {
                rsp_type = "RSR";
            }
            tmp_length = 3;
            RSP_lengths[0] = x_param * RSP_param.t;
            RSP_lengths[1] = x_param * RSP_param.u;
            RSP_lengths[2] = x_param * RSP_param.v;
        }
        else if (i < 10)
        { // CSC, LSR case
            LSR(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
            if (y_param > 0)
            {
                rsp_type = "LSR";
            }
            else
            {
                rsp_type = "RSL";
            }
            tmp_length = 3;
            RSP_lengths[0] = x_param * RSP_param.t;
            RSP_lengths[1] = x_param * RSP_param.u;
            RSP_lengths[2] = x_param * RSP_param.v;
        }
        else if (i < 14)
        { // CCC, LRL case
            LRL(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
            if (y_param > 0)
            {
                rsp_type = "LRL";
            }
            else
            {
                rsp_type = "RLR";
            }
            tmp_length = 3;
            RSP_lengths[0] = x_param * RSP_param.t;
            RSP_lengths[1] = x_param * RSP_param.u;
            RSP_lengths[2] = x_param * RSP_param.v;
        }
        else if (i < 18)
        { // CCC, LRL case, backward
            LRL(x_param * xb, y_param * yb, x_param * y_param * dphi, &RSP_param);
            if (y_param > 0)
            {
                rsp_type = "LRL";
            }
            else
            {
                rsp_type = "RLR";
            }
            tmp_length = 3;
            RSP_lengths[0] = x_param * RSP_param.v;
            RSP_lengths[1] = x_param * RSP_param.u;
            RSP_lengths[2] = x_param * RSP_param.t;
        }
        else if (i < 22)
        { // CCCC, LRLRn
            LRLRn(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
            if (y_param > 0.0)
            {
                rsp_type = "LRLR";
            }
            else
            {
                rsp_type = "RLRL";
            }
            tmp_length = 4;
            RSP_lengths[0] = x_param * RSP_param.t;
            RSP_lengths[1] = x_param * RSP_param.u;
            RSP_lengths[2] = -x_param * RSP_param.u;
            RSP_lengths[3] = x_param * RSP_param.v;
        }
        else if (i < 26)
        { // CCCC, LRLRp
            LRLRp(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
            if (y_param > 0.0)
            {
                rsp_type = "LRLR";
            }
            else
            {
                rsp_type = "RLRL";
            }
            tmp_length = 4;
            RSP_lengths[0] = x_param * RSP_param.t;
            RSP_lengths[1] = x_param * RSP_param.u;
            RSP_lengths[2] = -x_param * RSP_param.u;
            RSP_lengths[3] = x_param * RSP_param.v;
        }
        else if (i < 30)
        { // CCSC, LRLRn
            tmp_length = 4;
            LRLRn(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
            if (y_param > 0.0)
            {
                rsp_type = "LRSL";
            }
            else
            {
                rsp_type = "RLSR";
            }
            RSP_lengths[0] = x_param * RSP_param.t;
            if (x_param < 0 && y_param > 0)
            {
                RSP_lengths[1] = 0.5 * M_PI;
            }
            else
            {
                RSP_lengths[1] = -0.5 * M_PI;
            }
            if (x_param > 0 && y_param < 0)
            {
                RSP_lengths[2] = RSP_param.u;
            }
            else
            {
                RSP_lengths[2] = -RSP_param.u;
            }
            RSP_lengths[3] = x_param * RSP_param.v;
        }
        else if (i < 34)
        { // CCSC, LRLRp
            tmp_length = 4;
            LRLRp(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
            if (y_param)
            {
                rsp_type = "LRSR";
            }
            else
            {
                rsp_type = "RLSL";
            }
            RSP_lengths[0] = x_param * RSP_param.t;
            if (x_param < 0 && y_param > 0)
            {
                RSP_lengths[1] = 0.5 * M_PI;
            }
            else
            {
                RSP_lengths[1] = -0.5 * M_PI;
            }
            RSP_lengths[2] = x_param * RSP_param.u;
            RSP_lengths[3] = x_param * RSP_param.v;
        }
        else if (i < 38)
        { // CCSC, LRLRn, backward
            tmp_length = 4;
            LRLRn(x_param * xb, y_param * yb, x_param * y_param * dphi, &RSP_param);
            if (y_param > 0)
            {
                rsp_type = "LSRL";
            }
            else
            {
                rsp_type = "RSLR";
            }
            RSP_lengths[0] = x_param * RSP_param.v;
            RSP_lengths[1] = x_param * RSP_param.u;
            RSP_lengths[2] = -x_param * 0.5 * M_PI;
            RSP_lengths[3] = x_param * RSP_param.t;
        }
        else if (i < 42)
        { // CCSC, LRLRp, backward
            tmp_length = 4;
            LRLRp(x_param * xb, y_param * yb, x_param * y_param * dphi, &RSP_param);
            if (y_param > 0)
            {
                rsp_type = "RSRL";
            }
            else
            {
                rsp_type = "LSLR";
            }
            RSP_lengths[0] = x_param * RSP_param.v;
            RSP_lengths[1] = x_param * RSP_param.u;
            RSP_lengths[2] = -x_param * M_PI * 0.5;
            RSP_lengths[3] = x_param * RSP_param.t;
        }
        else
        { // CCSCC, LRSLR
            tmp_length = 5;
            LRSLR(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
            if (y_param > 0.0)
            {
                rsp_type = "LRSLR";
            }
            else
            {
                rsp_type = "RLSRL";
            }
            RSP_lengths[0] = x_param * RSP_param.t;
            RSP_lengths[1] = -x_param * 0.5 * M_PI;
            RSP_lengths[2] = x_param * RSP_param.u;
            RSP_lengths[3] = -x_param * 0.5 * M_PI;
            RSP_lengths[4] = x_param * RSP_param.v;
        }
        if (tmp_length > 0)
        {
            if (RSP_param.flag && !setRSP(tmp_length, RSP_lengths, rsp_type, possible_paths, i))
            {
                succ = false;
            }
        }
    }
    if (!succ)
    {
        return false;
    }
    if (possible_paths->size() == 0)
    {
        return false;
    }
    return true;
}

void RSSpace::SLS(const double x, const double y, const double phi, RSPParam *param)
{
    double phi_mod = mod2pi(phi);
    double xd = 0.0;
    double u = 0.0;
    double t = 0.0;
    double v = 0.0;
    double epsilon = 1e-1;
    if (y > 0.0 && phi_mod > epsilon && phi_mod < M_PI)
    {
        xd = -y / tan(phi_mod) + x;
        t = xd - tan(phi_mod / 2.0);
        u = phi_mod;
        v = sqrt((x - xd) * (x - xd) + y * y) - tan(phi_mod / 2.0);
        param->flag = true;
        param->u = u;
        param->t = t;
        param->v = v;
    }
    else if (y < 0.0 && phi_mod > epsilon && phi_mod < M_PI)
    {
        xd = -y / tan(phi_mod) + x;
        t = xd - tan(phi_mod / 2.0);
        u = phi_mod;
        v = -sqrt((x - xd) * (x - xd) + y * y) - tan(phi_mod / 2.0);
        param->flag = true;
        param->u = u;
        param->t = t;
        param->v = v;
    }
}

void RSSpace::LSL(const double x, const double y, const double phi, RSPParam *param)
{
    std::pair<double, double> p = polar(x - std::sin(phi), y - 1.0 + std::cos(phi));
    double u = p.first;
    double t = p.second;
    double v = 0.0;
    if (t >= 0.0)
    {
        v = mod2pi(phi - t);
        if (v >= 0.0)
        {
            param->flag = true;
            param->u = u;
            param->t = t;
            param->v = v;
        }
    }
}

void RSSpace::LSR(const double x, const double y, const double phi, RSPParam *param)
{
    std::pair<double, double> p = polar(x + std::sin(phi), y - 1.0 - std::cos(phi));
    double u1 = p.first * p.first;
    double t1 = p.second;
    double u = 0.0;
    double theta = 0.0;
    double t = 0.0;
    double v = 0.0;
    if (u1 >= 4.0)
    {
        u = std::sqrt(u1 - 4.0);
        theta = std::atan2(2.0, u);
        t = mod2pi(t1 + theta);
        v = mod2pi(t - phi);
        if (t >= 0.0 && v >= 0.0)
        {
            param->flag = true;
            param->u = u;
            param->t = t;
            param->v = v;
        }
    }
}

void RSSpace::LRL(const double x, const double y, const double phi, RSPParam *param)
{
    std::pair<double, double> p = polar(x - std::sin(phi), y - 1.0 + std::cos(phi));
    double u1 = p.first;
    double t1 = p.second;
    double u = 0.0;
    double t = 0.0;
    double v = 0.0;
    if (u1 <= 4.0)
    {
        u = -2.0 * std::asin(0.25 * u1);
        t = mod2pi(t1 + 0.5 * u + M_PI);
        v = mod2pi(phi - t + u);
        if (t >= 0.0 && u <= 0.0)
        {
            param->flag = true;
            param->u = u;
            param->t = t;
            param->v = v;
        }
    }
}

void RSSpace::LRLRn(const double x, const double y, const double phi,
                    RSPParam *param)
{
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
    double u = 0.0;
    if (rho <= 1.0 && rho >= 0.0)
    {
        u = std::acos(rho);
        if (u >= 0 && u <= 0.5 * M_PI)
        {
            std::pair<double, double> tau_omega = calcTauOmega(u, -u, xi, eta, phi);
            if (tau_omega.first >= 0.0 && tau_omega.second <= 0.0)
            {
                param->flag = true;
                param->u = u;
                param->t = tau_omega.first;
                param->v = tau_omega.second;
            }
        }
    }
}

void RSSpace::LRLRp(const double x, const double y, const double phi,
                    RSPParam *param)
{
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho = (20.0 - xi * xi - eta * eta) / 16.0;
    double u = 0.0;
    if (rho <= 1.0 && rho >= 0.0)
    {
        u = -std::acos(rho);
        if (u >= 0 && u <= 0.5 * M_PI)
        {
            std::pair<double, double> tau_omega = calcTauOmega(u, u, xi, eta, phi);
            if (tau_omega.first >= 0.0 && tau_omega.second >= 0.0)
            {
                param->flag = true;
                param->u = u;
                param->t = tau_omega.first;
                param->v = tau_omega.second;
            }
        }
    }
}

void RSSpace::LRSR(const double x, const double y, const double phi,
                   RSPParam *param)
{
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    std::pair<double, double> p = polar(-eta, xi);
    double rho = p.first;
    double theta = p.second;
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;
    if (rho >= 2.0)
    {
        t = theta;
        u = 2.0 - rho;
        v = mod2pi(t + 0.5 * M_PI - phi);
        if (t >= 0.0 && u <= 0.0 && v <= 0.0)
        {
            param->flag = true;
            param->u = u;
            param->t = t;
            param->v = v;
        }
    }
}

void RSSpace::LRSL(const double x, const double y, const double phi,
                   RSPParam *param)
{
    double xi = x - std::sin(phi);
    double eta = y - 1.0 + std::cos(phi);
    std::pair<double, double> p = polar(xi, eta);
    double rho = p.first;
    double theta = p.second;
    double r = 0.0;
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;

    if (rho >= 2.0)
    {
        r = std::sqrt(rho * rho - 4.0);
        u = 2.0 - r;
        t = mod2pi(theta + std::atan2(r, -2.0));
        v = mod2pi(phi - 0.5 * M_PI - t);
        if (t >= 0.0 && u <= 0.0 && v <= 0.0)
        {
            param->flag = true;
            param->u = u;
            param->t = t;
            param->v = v;
        }
    }
}

void RSSpace::LRSLR(const double x, const double y, const double phi,
                    RSPParam *param)
{
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    std::pair<double, double> p = polar(xi, eta);
    double rho = p.first;
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;
    if (rho >= 2.0)
    {
        u = 4.0 - std::sqrt(rho * rho - 4.0);
        if (u <= 0.0)
        {
            t = mod2pi(
                atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
            v = mod2pi(t - phi);

            if (t >= 0.0 && v >= 0.0)
            {
                param->flag = true;
                param->u = u;
                param->t = t;
                param->v = v;
            }
        }
    }
}

bool RSSpace::setRSP(const int size, const double *lengths, const std::string &types,
                     std::vector<RSPath> *possible_paths, const int idx)
{
    RSPath path;
    std::vector<double> length_vec(lengths, lengths + size);
    std::vector<char> type_vec(types.begin(), types.begin() + size);
    path.seg_len = length_vec;
    path.seg_type = type_vec;
    double sum = 0.0;
    for (int i = 0; i < size; ++i)
    {
        sum += std::abs(lengths[i]);
    }
    path.total_len = sum;
    if (path.total_len <= 0.0)
    {
        return false;
    }
    possible_paths->at(idx) = path;
    return true;
}

bool RSSpace::generateConfig(const double x0, const double y0, const double phi0,
                             const double x1, const double y1, const double phi1,
                             RSPath *optimal_path)
{
    double stepsz_scaled = stepsz_ / rho_;
    size_t wpt_num = static_cast<size_t>(std::floor(optimal_path->total_len / stepsz_scaled +
                                                    static_cast<double>(optimal_path->seg_len.size()) + 4));
    std::vector<double> px(wpt_num, 0.0);
    std::vector<double> py(wpt_num, 0.0);
    std::vector<double> pphi(wpt_num, 0.0);
    std::vector<bool> pgear(wpt_num, true);
    int index = 1;
    double d = 0.0;
    double pd = 0.0;
    double ll = 0.0;
    if (optimal_path->seg_len.at(0) > 0.0)
    {
        pgear.at(0) = true;
        d = stepsz_scaled;
    }
    else
    {
        pgear.at(0) = false;
        d = -stepsz_scaled;
    }
    pd = d;
    for (size_t i = 0; i < optimal_path->seg_type.size(); ++i)
    {
        char m = optimal_path->seg_type.at(i);
        double l = optimal_path->seg_len.at(i);
        if (l > 0.0)
        {
            d = stepsz_scaled;
        }
        else
        {
            d = -stepsz_scaled;
        }
        double ox = px.at(index);
        double oy = py.at(index);
        double ophi = pphi.at(index);
        index--;
        if (i >= 1 && optimal_path->seg_len.at(i - 1) * optimal_path->seg_len.at(i) > 0)
        {
            pd = -d - ll;
        }
        else
        {
            pd = d - ll;
        }
        while (std::abs(pd) <= std::abs(l))
        {
            index++;
            interpolation(index, pd, m, ox, oy, ophi, &px, &py, &pphi, &pgear);
            pd += d;
        }
        ll = l - pd - d;
        index++;
        interpolation(index, l, m, ox, oy, ophi, &px, &py, &pphi, &pgear);
    }
    double epsilon = 1e-15;
    while (std::fabs(px.back()) < epsilon && std::fabs(py.back()) < epsilon &&
           std::fabs(pphi.back()) < epsilon && pgear.back())
    {
        px.pop_back();
        py.pop_back();
        pphi.pop_back();
        pgear.pop_back();
    }
    for (size_t i = 0; i < px.size(); ++i)
    {
        optimal_path->x.push_back(std::cos(-phi0) * px.at(i) +
                                  std::sin(-phi0) * py.at(i) +
                                  x0);
        optimal_path->y.push_back(-std::sin(-phi0) * px.at(i) +
                                  std::cos(-phi0) * py.at(i) +
                                  y0);
        optimal_path->phi.push_back(mod2pi(pphi.at(i) + phi0));
    }
    optimal_path->seg_gear = pgear;
    for (size_t i = 0; i < optimal_path->seg_len.size(); ++i)
    {
        optimal_path->seg_len.at(i) = optimal_path->seg_len.at(i) * rho_;
    }
    optimal_path->total_len = optimal_path->total_len * rho_;
    return true;
}

void RSSpace::interpolation(const int index, const double pd, const char m,
                            const double ox, const double oy, const double ophi,
                            std::vector<double> *px, std::vector<double> *py,
                            std::vector<double> *pphi, std::vector<bool> *pgear)
{
    double ldx = 0.0;
    double ldy = 0.0;
    double gdx = 0.0;
    double gdy = 0.0;
    if (m == 'S')
    {
        px->at(index) = ox + pd * rho_ * std::cos(ophi);
        py->at(index) = oy + pd * rho_ * std::sin(ophi);
        pphi->at(index) = ophi;
    }
    else
    {
        ldx = std::sin(pd) * rho_;
        if (m == 'L')
        {
            ldy = (1.0 - std::cos(pd)) * rho_;
        }
        else if (m == 'R')
        {
            ldy = (1.0 - std::cos(pd)) * (-rho_);
        }
        gdx = std::cos(-ophi) * ldx + std::sin(-ophi) * ldy;
        gdy = -std::sin(-ophi) * ldx + std::cos(-ophi) * ldy;
        px->at(index) = ox + gdx;
        py->at(index) = oy + gdy;
    }

    if (pd > 0.0)
    {
        pgear->at(index) = true;
    }
    else
    {
        pgear->at(index) = false;
    }

    if (m == 'L')
    {
        pphi->at(index) = ophi + pd;
    }
    else if (m == 'R')
    {
        pphi->at(index) = ophi - pd;
    }
}