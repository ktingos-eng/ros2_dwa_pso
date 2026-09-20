#include "ros2_dwa_pso/dwa_pso_planner.hpp"
#include "ros2_dwa_pso/planner_utils.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>

#include <random>
#include <vector>
#include <algorithm>
#include <limits>

double DwaPsoPlanner::eval_cost(const double v, const double w, const size_t k)
{
    // Predict pose
    planner_types::Trajectory t = eval_trajectory(this->odom, v, w);

    const uint q = compute_collision_violation(t);
    this->tcurr.info.COLLISION = q > 0 ? true : false;

    const double penalty = alpha_(k) * std::pow(beta_(q),gamma_(q));

    std::vector<double> costs;
    costs.push_back(velocity_cost(t.vel.v));
    costs.push_back(heading_cost(t));
    costs.push_back(clearence_cost(t));
    costs.push_back(progress_cost(
        t.predicted_pose.x_hat,
        t.predicted_pose.y_hat)
    );
    costs.push_back(oscillation_cost(t.vel.w));

    std::vector<double> weights;
    weights.push_back(this->w_vel);
    weights.push_back(this->w_head);
    weights.push_back(this->w_clear);
    weights.push_back(this->w_prog);
    weights.push_back(this->w_osc);

    const double sum_w = std::accumulate(weights.begin(), weights.end(), 0);

    this->tcurr.info.scores.vel = costs[0] / sum_w;
    this->tcurr.info.scores.head = costs[1] / sum_w;
    this->tcurr.info.scores.clearence = costs[2] / sum_w;
    this->tcurr.info.scores.progress = costs[3] / sum_w;
    this->tcurr.info.scores.oscillation = costs[4] / sum_w;
    this->tcurr.info.scores.collision = penalty;

    double total_cost = 0.0;
    for(double& c : costs){
        total_cost += c;
    }

    total_cost = total_cost / sum_w;
    
    // Return objective function cost value
    return (penalty + total_cost);
}

planner_types::Trajectory DwaPsoPlanner::eval_trajectory(
    const nav_msgs::msg::Odometry& odom,
    const double v,
    const double w
) {
    constexpr double EPS_W = 1e-4;

    planner_types::Trajectory t;

    t.vel.v = v;
    t.vel.w = w;

    tf2::Quaternion q{
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    };
    tf2::Matrix3x3 m{q};

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    const double x0 = odom.pose.pose.position.x;
    const double y0 = odom.pose.pose.position.y;
    const double phi0 = yaw;
    // const double dt = this->dt_ms * 1e-3;
    const double dt = this->predict_time;
    const double res = this->costmap.info.resolution;

    double phi_hat = wrap_angle(phi0 + w * dt);

    geometry_msgs::msg::PoseStamped point;

    point.pose.position.x = x0;
    point.pose.position.y = y0;

    t.path.poses.push_back(point);

    double x_hat, y_hat;
    if (std::fabs(w) > EPS_W) {
        t.info.IS_LINEAR = false;
        const double s0 = std::sin(phi0);
        const double c0 = std::cos(phi0);
        const double s1 = std::sin(phi0 + w * dt);
        const double c1 = std::cos(phi0 + w * dt);

        const double R = v / w;

        x_hat = x0 + R * (s1 - s0);
        y_hat = y0 - R * (c1 - c0);

        const double Mx = -R * std::sin(phi0);
        const double My = +R * std::cos(phi0);
        const double xc = x0 + Mx;
        const double yc = y0 + My;

        const double theta0 = std::atan2(y0 - yc, x0 - xc);
        const double theta_hat = std::atan2(y_hat - yc, x_hat - xc);

        // Signed angular displacement following the motion direction
        double dtheta = wrap_angle(theta_hat - theta0);

        if (t.vel.w > 0.0) {
            if (dtheta < 0.0) {
                dtheta += 2.0 * M_PI;
            }
        } else {
            if (dtheta > 0.0) {
                dtheta -= 2.0 * M_PI;
            }
        }

        const double arc_len = std::abs(R * dtheta);
        const size_t pnum = static_cast<size_t>(std::ceil(arc_len / res));

        const double step_theta = dtheta / static_cast<double>(pnum);

        double x = x0;
        double y = y0;
        for (size_t i = 0; i <= pnum; i++) {
            const double theta = theta0 + static_cast<double>(i) * step_theta;
            x = xc + std::fabs(R) * std::cos(theta);
            y = yc + std::fabs(R) * std::sin(theta);

            point.pose.position.x = x;
            point.pose.position.y = y;
            t.path.poses.push_back(point);
        }

        t.center.xc = xc;
        t.center.yc = yc;
        t.radius = std::fabs(R);
    } else {
        t.info.IS_LINEAR = true;
        t.radius = 0.0;
        t.center.xc = 0.0;
        t.center.yc = 0.0;

        x_hat = x0 + v * dt * std::cos(phi0);
        y_hat = y0 + v * dt * std::sin(phi0);

        const double dx = x_hat - x0;
        const double dy = y_hat - y0;
        const double dist = std::hypot(dx, dy);

        const size_t pnum = static_cast<size_t>(std::ceil(dist / res));

        double x = x0;
        double y = y0;
        for (size_t i = 0; i <= pnum; i++) {
            const double s = std::min(static_cast<double>(i) * res, dist);
            const double u = (dist > 1e-9) ? s / dist : 0.0;
            x = x0 + u * dx;
            y = y0 + u * dy;

            point.pose.position.x = x;
            point.pose.position.y = y;
            t.path.poses.push_back(point);
        }
    }

    t.origin.x0 = x0;
    t.origin.y0 = y0;
    t.origin.phi0 = phi0;

    t.predicted_pose.x_hat = x_hat;
    t.predicted_pose.y_hat = y_hat;
    t.predicted_pose.phi_hat = phi_hat;

    this->tcurr = t;

    return t;
}

int DwaPsoPlanner::compute_collision_violation(const planner_types::Trajectory& t) {
    int c_max = 0;
    for(const auto& p : t.path.poses){
        const double x = p.pose.position.x;
        const double y = p.pose.position.y;

        const int c = get_cell_val(x,y);

        if(c < 0 && this->REJECT_OOB){
            return 10000;
        }

        c_max = c > c_max ? c : c_max;
    }

    if(c_max > this->thr_cost){
        return std::abs(c_max - this->thr_cost) / this->occ_norm;
    }

    return 0.0;
}

double DwaPsoPlanner::velocity_cost(const double v){
    const double v_max = this->wnd_curr.v_max;
    const double v_min = this->wnd_curr.v_min;

    if(std::fabs(v_max - v_min) < 1e-9) {
        return 0.0;
    }

    return this->w_vel * (v_max - v) / (v_max - v_min);
}

double DwaPsoPlanner::heading_cost(const planner_types::Trajectory& t){
    const double x_hat = t.predicted_pose.x_hat;
    const double y_hat = t.predicted_pose.y_hat;
    const double phi_hat = t.predicted_pose.phi_hat;

    const double dx = this->goal.x - x_hat;
    const double dy = this->goal.y - y_hat;

    const double phi_g = std::atan2(dy, dx);
    
    const double dphi = wrap_angle(phi_g - phi_hat);

    return this->w_head * std::pow(std::sin(dphi / 2),2);
}

double DwaPsoPlanner::clearence_cost(const planner_types::Trajectory& t){
    constexpr double MAX_OCC_COST = 100.0;

    // Min cost --> 0
    int max_cost = -1;
    for(const auto& p : t.path.poses){
        const double x = p.pose.position.x;
        const double y = p.pose.position.y;

        const int c = get_cell_val(x,y);

        max_cost = c > max_cost ? c : max_cost;
    }

    return this->w_clear * (static_cast<double>(max_cost) / MAX_OCC_COST);
}

// Projected progression
// double DwaPsoPlanner::progress_cost(const double x_hat, const double y_hat){
//     const double x0 = this->odom.pose.pose.position.x;
//     const double y0 = this->odom.pose.pose.position.y;
//     const double xg = this->goal.x;
//     const double yg = this->goal.y;

//     const double dg = std::hypot(xg - x0, yg - y0);
//     if (dg < 1e-9) {
//         return 0.0;
//     }

//     const double gx = (xg - x0) / dg;
//     const double gy = (yg - y0) / dg;

//     const double dx = x_hat - x0;
//     const double dy = y_hat - y0;
//     const double ds = dx * gx + dy * gy;

//     if(ds < 0.0){
//         return 500.0;
//     }

//     return this->w_prog * (1 - ds / dg);
// }

// Terminal eucledian distance
double DwaPsoPlanner::progress_cost(const double x_hat, const double y_hat){
    const double x0 = this->odom.pose.pose.position.x;
    const double y0 = this->odom.pose.pose.position.y;
    const double xg = this->goal.x;
    const double yg = this->goal.y;

    const double d0 = std::hypot(xg - x0, yg - y0);
    if (d0 < 1e-9) {
        return 0.0;
    }

    const double d_hat = std::hypot(xg - x_hat, yg - y_hat);

    return this->w_prog * (d_hat / d0);
}

double DwaPsoPlanner::oscillation_cost(const double w){
    const double w_curr = this->odom.twist.twist.angular.z;
    const double aa_max = this->limits.max_acc.angular;
    const double dt = this->dt_ms * 1e-3;

    const double osc_cost = this->w_osc * std::abs(w - w_curr) / (aa_max * dt);
    
    if(this->COND_OSC_COST) {
        
        if(!this->osc_initialized){
            this->reset_osc_state();
        }

        if(this->check_osc_reset()){
            this->reset_osc_state();
        }

        const int w_sign = signed_dir(w);

        if(w_sign !=0 && last_w_sign !=0 && w_sign != last_w_sign){
            return osc_cost;
        }

    } else {
        return osc_cost;
    }

    return 0.0;
}

bool DwaPsoPlanner::check_osc_reset() const {
    const double x = this->odom.pose.pose.position.x;
    const double y = this->odom.pose.pose.position.y;

    tf2::Quaternion q{
        this->odom.pose.pose.orientation.x,
        this->odom.pose.pose.orientation.y,
        this->odom.pose.pose.orientation.z,
        this->odom.pose.pose.orientation.w
    };

    tf2::Matrix3x3 m(q);

    double roll, pitch, phi;
    m.getRPY(roll, pitch, phi);

    const double ds = std::hypot(x - this->x_reset, y - this->y_reset);
    const double dphi = std::fabs(wrap_angle(phi - this->phi_reset));

    return (ds > this->osc_reset_dist) || (dphi > this->osc_reset_angle);
}

void DwaPsoPlanner::reset_osc_state(){
    const double x = this->odom.pose.pose.position.x;
    const double y = this->odom.pose.pose.position.y;

    tf2::Quaternion q(
        this->odom.pose.pose.orientation.x,
        this->odom.pose.pose.orientation.y,
        this->odom.pose.pose.orientation.z,
        this->odom.pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);

    double roll, pitch, phi;
    m.getRPY(roll, pitch, phi);

    this->x_reset = x;
    this->y_reset = y;
    this->phi_reset = phi;

    this->last_v_sign = 0;
    this->last_w_sign = 0;

    this->osc_initialized = true;
}

int DwaPsoPlanner::get_cell_val(double x, double y){
    const uint w = this->costmap.info.width;
    const uint h = this->costmap.info.height;
    const double lamda = this->costmap.info.resolution;
    const double x0 = this->costmap.info.origin.position.x;
    const double y0 = this->costmap.info.origin.position.y;

    int i = static_cast<int>(std::floor((x - x0) / lamda));
    int j = static_cast<int>(std::floor((y - y0) / lamda));

    if (i < 0 || j < 0 || i >= static_cast<int>(w) || j >= static_cast<int>(h)){
        return -1;
    }

    return this->costmap.data[j * w + i];
}

void DwaPsoPlanner::update_osc_memory(const double v, const double w){
    const int v_sign = signed_dir(v);
    const int w_sign = signed_dir(w);

    if(v_sign != 0){
        last_v_sign = v_sign;
    }

    if(w_sign != 0){
        last_w_sign = w_sign;
    }
}

void DwaPsoPlanner::pub_path(){
    this->tbest.path.header.frame_id = "odom";
    this->tbest.path.header.stamp = this->now();
    path_pub_->publish(this->tbest.path);
}

void DwaPsoPlanner::pub_metrics(){
    planner_interfaces::msg::Metrics msg;
    msg.robot_clearence = this->robot_clearence;
    msg.comp_time = this->comp_time;
    metrics_pub_->publish(msg);
}