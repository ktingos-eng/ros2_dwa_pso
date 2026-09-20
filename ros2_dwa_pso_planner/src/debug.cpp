#include "ros2_dwa_pso/dwa_pso_planner.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>

#include <random>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

static double beta_(const uint q){
    if(q > 0){
        return 100.0;
    } else {
        return 0.0;
    }
}

void DwaPsoPlanner::debug_bruteforce_line_scan(const nav_msgs::msg::Odometry& odom_local)
{
    constexpr int N = 21;

    const double v_min = this->wnd_curr.v_min;
    const double v_max = this->wnd_curr.v_max;

    if (v_max - v_min < 1e-9) {
        RCLCPP_INFO(this->get_logger(), "[BF] skipped: degenerate velocity window");
        return;
    }

    double best_v = v_min;
    double best_J = std::numeric_limits<double>::infinity();

    RCLCPP_INFO(this->get_logger(), "========== BRUTE FORCE LINE SCAN (w = 0) ==========");

    for (int i = 0; i < N; ++i) {
        const double a = static_cast<double>(i) / static_cast<double>(N - 1);
        const double v = v_min + a * (v_max - v_min);

        trajectory t = eval_trajectory(odom_local, v, 0.0);

        const double J_head  = this->heading_cost(t);
        const double J_vel   = this->velocity_cost(v);
        const double J_prog  = this->progress_cost(
            t.predicted_pose.x_hat,
            t.predicted_pose.y_hat
        );
        const double J_clear = this->clearence_cost(t);

        const double J_total = J_head + J_vel + J_prog + J_clear;

        if (J_total < best_J) {
            best_J = J_total;
            best_v = v;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "[BF] i=%02d  v=%.6f  J=%.6f | head=%.6f vel=%.6f prog=%.6f clear=%.6f",
            i, v, J_total, J_head, J_vel, J_prog, J_clear
        );
    }

    RCLCPP_INFO(
        this->get_logger(),
        "[BF] BEST straight candidate -> v=%.6f  J=%.6f",
        best_v, best_J
    );

    RCLCPP_INFO(this->get_logger(), "==================================================");
}


void DwaPsoPlanner::debug_bruteforce_grid_scan(const nav_msgs::msg::Odometry& odom_local)
{
    constexpr int NV = 21;   // samples on linear velocity axis
    constexpr int NW = 21;   // samples on angular velocity axis

    const double v_min = this->wnd_curr.v_min;
    const double v_max = this->wnd_curr.v_max;
    const double w_min = this->wnd_curr.w_min;
    const double w_max = this->wnd_curr.w_max;

    if ((v_max - v_min) < 1e-9 || (w_max - w_min) < 1e-9) {
        RCLCPP_INFO(this->get_logger(), "[GRID] skipped: degenerate dynamic window");
        return;
    }

    double best_v = v_min;
    double best_w = w_min;
    double best_J = std::numeric_limits<double>::infinity();

    trajectory best_t{};
    bool best_collision = false;

    RCLCPP_INFO(this->get_logger(), "========== BRUTE FORCE GRID SCAN ==========");
    RCLCPP_INFO(
        this->get_logger(),
        "[GRID] window: v=[%.6f, %.6f], w=[%.6f, %.6f], NV=%d, NW=%d",
        v_min, v_max, w_min, w_max, NV, NW
    );

    for (int iw = 0; iw < NW; ++iw) {
        const double aw = static_cast<double>(iw) / static_cast<double>(NW - 1);
        const double w = w_min + aw * (w_max - w_min);

        for (int iv = 0; iv < NV; ++iv) {
            const double av = static_cast<double>(iv) / static_cast<double>(NV - 1);
            const double v = v_min + av * (v_max - v_min);

            trajectory t = this->eval_trajectory(odom_local, v, w);

            const uint q = compute_collision_violation(t);
            bool traj_collision = q > 0 ? true : false;
            this->tcurr.info.COLLISION = traj_collision;
            const double penalty = 100.0 * std::pow(beta_(q), 2);

            const double J_vel   = this->velocity_cost(v);
            const double J_head  = this->heading_cost(t);
            const double J_clear = this->clearence_cost(t);
            const double J_prog  = this->progress_cost(
                t.predicted_pose.x_hat,
                t.predicted_pose.y_hat
            );
            const double J_osc   = this->oscillation_cost(w);

            const double J_total = penalty + J_vel + J_head + J_clear + J_prog + J_osc;

            if (J_total < best_J) {
                best_J = J_total;
                best_v = v;
                best_w = w;
                best_t = t;
                best_collision = traj_collision;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "[GRID] iv=%02d iw=%02d | v=%.6f w=%.6f | J=%.6f | vel=%.6f head=%.6f prog=%.6f clear=%.6f osc=%.6f pen=%.6f coll=%d",
                iv, iw, v, w, J_total,
                J_vel, J_head, J_prog, J_clear, J_osc, penalty,
                static_cast<int>(traj_collision)
            );
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "[GRID] BEST candidate -> v=%.6f w=%.6f J=%.6f coll=%d",
        best_v, best_w, best_J, static_cast<int>(best_collision)
    );

    RCLCPP_INFO(
        this->get_logger(),
        "[GRID] BEST scores -> vel=%.6f head=%.6f prog=%.6f clear=%.6f osc=%.6f",
        this->velocity_cost(best_v),
        this->heading_cost(best_t),
        this->progress_cost(best_t.predicted_pose.x_hat, best_t.predicted_pose.y_hat),
        this->clearence_cost(best_t),
        this->oscillation_cost(best_w)
    );

    RCLCPP_INFO(this->get_logger(), "===========================================");
}