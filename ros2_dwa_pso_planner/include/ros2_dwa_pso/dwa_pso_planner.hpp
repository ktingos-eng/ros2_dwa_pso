#ifndef DWA_PSO_PLANNER_HPP
#define DWA_PSO_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <planner_interfaces/msg/metrics.hpp>
#include <mutex>
#include <atomic>

class DwaPsoPlanner : public rclcpp::Node {

    public:
        DwaPsoPlanner();

        struct DynamicLimits {
            struct vel {
                double linear;
                double angular;
            };
            struct acc {
                double linear;
                double angular;
            };
            vel max_vel;
            acc max_acc;
        };

        struct window {
            double v_min, v_max;
            double w_min, w_max;
        };

        struct trajectory {
            struct info {
                bool IS_LINEAR;
                bool COLLISION;
                struct scores {
                    double head;
                    double vel;
                    double clearence;
                    double oscillation;
                    double collision;
                    double progress;
                } scores;
            } info;
            struct origin {
                double x0;
                double y0;
                double phi0;
            } origin;
            struct predicted_pose {
                double x_hat;
                double y_hat;
                double phi_hat;
            } predicted_pose;
            struct center {
                double xc;
                double yc;
            } center;
            struct vel {
                double v;
                double w;
                double v0;
                double w0;
            } vel;
            double radius;
            nav_msgs::msg::Path path;
        };

        struct Particle {
            double v{0.0}, w{0.0};
            double vv{0.0}, vw{0.0};
            double pbest_v{0.0}, pbest_w{0.0};
            double pbest_cost{std::numeric_limits<double>::infinity()};
            double cost{std::numeric_limits<double>::infinity()};
        };
    
    private:
        // ROS callback functions
        void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg);
        void costmapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        // Timer Callback - Main controller loop function
        void plannerCB();

        window compute_dynamic_window(const nav_msgs::msg::Odometry& odom);

        // Optimization functions - PSO and grid-scan methods
        geometry_msgs::msg::Twist pso_optimize_cmd(const window& wnd);
        geometry_msgs::msg::Twist grid_optimize_cmd(const nav_msgs::msg::Odometry& odom_local);

        void init_swarm(std::vector<Particle>& swarm, window wnd);

        double eval_cost(const double v, const double w, const size_t k);
        trajectory eval_trajectory(const nav_msgs::msg::Odometry& odom,
            const double v, const double w
        );

        // Cost-calculation functions
        double velocity_cost(const double v);
        double heading_cost(const trajectory& t);
        double clearence_cost(const trajectory& t);
        double progress_cost(const double x_hat, const double y_hat);
        double oscillation_cost(const double w);

        int compute_collision_violation(const trajectory& t);
        int get_cell_val(double x, double y);

        void get_params();

        void pub_path();
        void pub_metrics();

        bool check_osc_reset() const;
        void reset_osc_state();
        void update_osc_memory(const double v, const double w);

        // Debug functions
        void debug_bruteforce_line_scan(const nav_msgs::msg::Odometry& odom_local);
        void debug_bruteforce_grid_scan(const nav_msgs::msg::Odometry& odom_local);

        rclcpp::CallbackGroup::SharedPtr sub_group_;
        rclcpp::CallbackGroup::SharedPtr planner_group_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<planner_interfaces::msg::Metrics>::SharedPtr metrics_pub_;
        rclcpp::TimerBase::SharedPtr planner_timer_;

        std::mutex odom_mtx;
        nav_msgs::msg::Odometry last_odom, odom;
        std::atomic<bool> have_odom{false};

        std::mutex costmap_mtx;
        nav_msgs::msg::OccupancyGrid last_costmap, costmap;
        std::atomic<bool> have_costmap{false};

        trajectory tcurr, tbest;
        window wnd_curr;

        double robot_clearence{0.0};
        double comp_time{0.0};

        // Oscillation memory
        int last_v_sign{0};   // -1, 0, +1
        int last_w_sign{0};   // -1, 0, +1

        // Pose at last oscillation reset
        double x_reset{0.0};
        double y_reset{0.0};
        double phi_reset{0.0};

        // Init flag
        bool osc_initialized{false};


        /*
        -------------- ROS params --------------
        */
        geometry_msgs::msg::Point goal;

        // DWA                                    
        double dt_ms{100.0};
        double predict_time{1.0};
        double eps_goal{1e-2};
        DynamicLimits limits{{10.0, 5.0}, {2.0, 5.0}};
        double w_head{1.0};
        double w_vel{1.0};
        double w_prog{1.0};
        double w_clear{1.0};
        double w_osc{1.0};
        bool REJECT_OOB{false};
        bool COND_OSC_COST{true};

        // PSO
        size_t imax{30}; // max iterations
        int n_par{30}; // particle number

        double eps_head{1e-2};
        double eps_cost{1e-4};
        int patience{5};
        bool RANDOM_INIT{false};

        double acc_cog{2.0};
        double acc_soc{2.0};
        double iner_start{0.9};
        double iner_end{0.4};
        int occ_norm{10};

        // TBD PARAMS
        int thr_cost{80};
        double osc_reset_dist{0.1}, osc_reset_angle{0.1};
        /*
        -------------- ROS params --------------
        */
};

#endif