#include "ros2_dwa_pso/dwa_pso_planner.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>

#include <random>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

// #define DEBUG

DwaPsoPlanner::DwaPsoPlanner()
: Node("dwa_pso_planner") {
    goal.x = 2.0;
    goal.y = 1.0;
    goal.z = 0.0;

    this->get_params();

    sub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    planner_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = sub_group_;

    // Reliable quality of service for odom sub
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg){
            this->odomCB(msg);
        },
        opts
    );

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap/costmap",
        qos,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            this->costmapCB(msg);
        },
        opts
    );

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::QoS(10)
    );

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/path",
        rclcpp::QoS(10)
    );

    metrics_pub_ = this->create_publisher<planner_interfaces::msg::Metrics>(
        "/planner_metrics",
        rclcpp::QoS(10)
    );

    planner_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)dt_ms),
        [this](){
            this->plannerCB();
        },
        planner_group_
    );
}

void DwaPsoPlanner::plannerCB()
{

    if(this->have_odom.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lk(this->odom_mtx);
        this->odom = this->last_odom;
    } else {
        return;
    }

    if(this->have_costmap.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lk(this->costmap_mtx);
        this->costmap = this->last_costmap;
    } else {
        return;
    }

    this->wnd_curr = this->compute_dynamic_window(this->odom);

    geometry_msgs::msg::Twist cmd_vel;

    // // Init zero cmd before computation
    // cmd_vel.linear.x = 0.0;
    // cmd_vel.linear.y = 0.0;
    // cmd_vel.linear.z = 0.0;

    // cmd_vel.angular.x = 0.0;
    // cmd_vel.angular.y = 0.0;
    // cmd_vel.angular.z = 0.0;

    auto t0 = this->now();
    cmd_vel = this->pso_optimize_cmd(this->wnd_curr);
    // cmd_vel = this->grid_optimize_cmd(this->odom);
    this->comp_time = (this->now() - t0).seconds();

    double x = this->odom.pose.pose.position.x;
    double y = this->odom.pose.pose.position.y;

    this->robot_clearence = this->get_cell_val(x,y);

    this->pub_path();
    this->pub_metrics();
    
    RCLCPP_INFO(this->get_logger(),"%s", "##################################");
    RCLCPP_INFO(this->get_logger(),"%s", "----------------------------------");
    RCLCPP_INFO(this->get_logger(),"ODOM: (%f)", odom.twist.twist.linear.x);
    RCLCPP_INFO(this->get_logger(),"WINDOW: (%f, %f)", wnd_curr.v_max, wnd_curr.w_max);
    RCLCPP_INFO(this->get_logger(), "LINEAR: (%f)   ANGULAR: (%f)", cmd_vel.linear.x, cmd_vel.angular.z);

    RCLCPP_INFO(this->get_logger(),"%s", "----------------------------------");
    RCLCPP_INFO(this->get_logger(),"HEAD: (%f)", this->tbest.info.scores.head);
    RCLCPP_INFO(this->get_logger(),"VEL: (%f)", this->tbest.info.scores.vel);
    RCLCPP_INFO(this->get_logger(),"PROG: (%f)", this->tbest.info.scores.progress);
    RCLCPP_INFO(this->get_logger(),"CLEAR: (%f)", this->tbest.info.scores.clearence);
    RCLCPP_INFO(this->get_logger(),"OSC: (%f)", this->tbest.info.scores.oscillation);
    RCLCPP_INFO(this->get_logger(),"COLL: (%f)", this->tbest.info.scores.collision);


    // Optional safety: keep command inside window bounds
    cmd_vel.linear.x  = std::clamp(cmd_vel.linear.x,  wnd_curr.v_min, wnd_curr.v_max);
    cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, wnd_curr.w_min, wnd_curr.w_max);

    // Terminate cmd when goal reached
    double dx = goal.x - odom.pose.pose.position.x;
    double dy = goal.y - odom.pose.pose.position.y;
    double goal_err = std::hypot(dx,dy);

    if(goal_err < this->eps_goal) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;

        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;
    }

    this->update_osc_memory(cmd_vel.linear.x, cmd_vel.angular.z);

    // No movement during debug
    #ifdef DEBUG
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;

        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;
    #endif

    cmd_pub_->publish(cmd_vel);

}

void DwaPsoPlanner::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(odom_mtx);
    last_odom = *msg;
    have_odom.store(true, std::memory_order_release);
}

void DwaPsoPlanner::costmapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(costmap_mtx);
    last_costmap = *msg;
    have_costmap.store(true, std::memory_order_release);
}

DwaPsoPlanner::window DwaPsoPlanner::compute_dynamic_window(const nav_msgs::msg::Odometry& odom)
{
    const double dt = this->dt_ms * 1e-3;

    const double v_curr = odom.twist.twist.linear.x;
    const double w_curr = odom.twist.twist.angular.z;
    const double al_max = this->limits.max_acc.linear;
    const double aa_max = this->limits.max_acc.angular;

    const double dv = al_max * dt;
    const double dw = aa_max * dt;

    window wnd;
    wnd.v_min = std::clamp(v_curr - dv, -this->limits.max_vel.linear,  this->limits.max_vel.linear);
    wnd.v_max = std::clamp(v_curr + dv, -this->limits.max_vel.linear,  this->limits.max_vel.linear);

    wnd.w_min = std::clamp(w_curr - dw, -this->limits.max_vel.angular, this->limits.max_vel.angular);
    wnd.w_max = std::clamp(w_curr + dw, -this->limits.max_vel.angular, this->limits.max_vel.angular);

    // Normalize to forward motion only
    wnd.v_min = std::max(wnd.v_min, 0.0);

    // Normalize to braking constaint
    const double x0 = odom.pose.pose.position.x;
    const double y0 = odom.pose.pose.position.y;
    const double dg = std::hypot(x0 - this->goal.x, y0 - this->goal.y);

    wnd.v_max = std::min(wnd.v_max, std::sqrt(2 * al_max * dg));
    wnd.v_max = std::max(wnd.v_max, wnd.v_min);
    
    return wnd;
}

geometry_msgs::msg::Twist DwaPsoPlanner::pso_optimize_cmd(const window& wnd)
{
    const double c1 = this->acc_cog;
    const double c2 = this->acc_soc;
    const double w_in  = this->iner_start;
    const double w_end = this->iner_end;

    const double v_min = wnd.v_min;
    const double v_max = wnd.v_max;
    const double w_min = wnd.w_min;
    const double w_max = wnd.w_max;

    if (v_max <= v_min || w_max <= w_min || this->n_par <= 0) {
        geometry_msgs::msg::Twist out;
        out.linear.x = 0.0;
        out.angular.z = 0.0;
        return out;
    }

    const double v_range = std::max(1e-9, v_max - v_min);
    const double w_range = std::max(1e-9, w_max - w_min);
    const double pv_max  = 0.5 * v_range;
    const double pw_max  = 0.5 * w_range;

    std::vector<Particle> swarm(static_cast<size_t>(this->n_par));

    // Initialize swarm
    this->init_swarm(swarm, wnd);

    static thread_local std::mt19937 rng{std::mt19937::default_seed};
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    double gbest_v = swarm.front().pbest_v;
    double gbest_w = swarm.front().pbest_w;
    double gbest_cost = swarm.front().pbest_cost;

    this->eval_cost(gbest_v, gbest_w, 1);
    this->tbest = this->tcurr;

    for (const auto &p : swarm) {
        if (p.pbest_cost < gbest_cost) {
            gbest_cost = p.pbest_cost;
            gbest_v = p.pbest_v;
            gbest_w = p.pbest_w;

            this->eval_cost(gbest_v, gbest_w, 1);
            this->tbest = this->tcurr;
        }
    }

    int stall = 0;
    double prev_gbest_cost = gbest_cost;

    for (size_t it = 0; it < this->imax; ++it) {

        const double tau = (this->imax > 1)
            ? static_cast<double>(it) / static_cast<double>(this->imax - 1)
            : 1.0;
        const double w_inertia = (1.0 - tau) * w_in + tau * w_end;

        for (auto &p : swarm) {

            const double r1 = uni01(rng);
            const double r2 = uni01(rng);
            const double r3 = uni01(rng);
            const double r4 = uni01(rng);

            p.vv = w_inertia * p.vv
                 + c1 * r1 * (p.pbest_v - p.v)
                 + c2 * r2 * (gbest_v   - p.v);

            p.vw = w_inertia * p.vw
                 + c1 * r3 * (p.pbest_w - p.w)
                 + c2 * r4 * (gbest_w   - p.w);

            p.vv = std::clamp(p.vv, -pv_max, pv_max);
            p.vw = std::clamp(p.vw, -pw_max, pw_max);

            p.v += p.vv;
            p.w += p.vw;

            p.v = std::clamp(p.v, v_min, v_max);
            p.w = std::clamp(p.w, w_min, w_max);

            p.cost = this->eval_cost(p.v, p.w, it);

            if (p.cost < p.pbest_cost) {
                p.pbest_cost = p.cost;
                p.pbest_v = p.v;
                p.pbest_w = p.w;
            }
        }

        for (const auto &p : swarm) {
            if (p.pbest_cost < gbest_cost) {
                gbest_cost = p.pbest_cost;
                gbest_v = p.pbest_v;
                gbest_w = p.pbest_w;

                this->eval_cost(gbest_v, gbest_w, it + 1);
                this->tbest = this->tcurr;
            }
        }

        const double delta = std::fabs(prev_gbest_cost - gbest_cost);
        if (delta < this->eps_cost) ++stall;
        else stall = 0;

        prev_gbest_cost = gbest_cost;

        if (stall >= this->patience) break;
    }

    geometry_msgs::msg::Twist out;
    out.linear.x  = gbest_v;
    out.angular.z = gbest_w;
    return out;
}

geometry_msgs::msg::Twist DwaPsoPlanner::grid_optimize_cmd(
    const nav_msgs::msg::Odometry& odom_local
)
{
    constexpr int NV = 31;
    constexpr int NW = 31;

    geometry_msgs::msg::Twist cmd{};

    const double v_min = this->wnd_curr.v_min;
    const double v_max = this->wnd_curr.v_max;
    const double w_min = this->wnd_curr.w_min;
    const double w_max = this->wnd_curr.w_max;

    if ((v_max - v_min) < 1e-9 || (w_max - w_min) < 1e-9) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        return cmd;
    }

    double best_v = 0.0;
    double best_w = 0.0;
    double best_J = std::numeric_limits<double>::infinity();

    for (int iw = 0; iw < NW; ++iw) {
        const double aw = static_cast<double>(iw) / static_cast<double>(NW - 1);
        const double w = w_min + aw * (w_max - w_min);

        for (int iv = 0; iv < NV; ++iv) {
            const double av = static_cast<double>(iv) / static_cast<double>(NV - 1);
            const double v = v_min + av * (v_max - v_min);

            double J_total = this->eval_cost(v,w,1);

            if (J_total < best_J) {
                best_J = J_total;
                best_v = v;
                best_w = w;
            }
        }
    }

    cmd.linear.x = best_v;
    cmd.angular.z = best_w;

    return cmd;
}

void DwaPsoPlanner::init_swarm(std::vector<Particle>& swarm, window wnd){
    if(this->RANDOM_INIT){
        static thread_local std::mt19937 rng{std::mt19937::default_seed};
        std::uniform_real_distribution<double> univ(wnd.v_min, wnd.v_max);
        std::uniform_real_distribution<double> uniw(wnd.w_min, wnd.w_max);

        for (auto &p : swarm) {
            p.v = univ(rng);
            p.w = uniw(rng);
            p.cost = this->eval_cost(p.v, p.w, 1);
            p.pbest_v = p.v;
            p.pbest_w = p.w;
            p.pbest_cost = p.cost;
        } 
    } else {
        auto closestFactor = [](size_t num){
            size_t factor = static_cast<size_t>(
                std::sqrt(static_cast<double>(num))
            );

            while(num % factor != 0 || factor < 1){
                --factor;
            };

            return factor;
        };

        size_t nv = closestFactor(this->n_par);

        bool PRIME = (this->n_par>1 && nv==1);

        if(PRIME) {
            // Reduce by 1 if prime
            nv = closestFactor(this->n_par - 1);
        }
        size_t nw = (this->n_par - 1) / nv;

        double dv = (wnd.v_max - wnd.v_min) / nv;
        double dw = (wnd.w_max - wnd.w_min) / nw;
        
        for (size_t k = 0; k < swarm.size(); ++k) {
            auto &p = swarm[k];

            if (PRIME && k == swarm.size() - 1) {
                p.v = wnd.v_min + 0.5 * (wnd.v_max - wnd.v_min);
                p.w = wnd.w_min + 0.5 * (wnd.w_max - wnd.w_min);
            } else {
                const size_t i = k / nw;
                const size_t j = k % nw;

                p.v = wnd.v_min + (i + 0.5) * dv;
                p.w = wnd.w_min + (j + 0.5) * dw;
            }

            p.cost = this->eval_cost(p.v, p.w, 1);
            p.pbest_v = p.v;
            p.pbest_w = p.w;
            p.pbest_cost = p.cost;
        }
    }
}

void DwaPsoPlanner::get_params() {

    // Declare params
    this->declare_parameter<double>("goal_x", 2.0);
    this->declare_parameter<double>("goal_y", 1.0);

    this->declare_parameter<double>("dt_ms", this->dt_ms);
    this->declare_parameter<double>("eps_goal", this->eps_goal);

    this->declare_parameter<double>("limits.max_vel.linear",  this->limits.max_vel.linear);
    this->declare_parameter<double>("limits.max_vel.angular", this->limits.max_vel.angular);
    this->declare_parameter<double>("limits.max_acc.linear",  this->limits.max_acc.linear);
    this->declare_parameter<double>("limits.max_acc.angular", this->limits.max_acc.angular);

    this->declare_parameter<double>("heading_weight", this->w_head);
    this->declare_parameter<double>("velocity_weight", this->w_vel);
    this->declare_parameter<double>("progress_weight", this->w_prog);
    this->declare_parameter<double>("clearence_weight", this->w_clear);

    this->declare_parameter<bool>("reject_oob_trajectories", this->REJECT_OOB);

    this->declare_parameter<int>("imax", static_cast<int>(this->imax));
    this->declare_parameter<int>("n_par", this->n_par);

    this->declare_parameter<double>("eps_head", this->eps_head);
    this->declare_parameter<double>("eps_cost", this->eps_cost);
    this->declare_parameter<int>("patience", this->patience);

    this->declare_parameter<double>("acc_cog", this->acc_cog);
    this->declare_parameter<double>("acc_soc", this->acc_soc);
    this->declare_parameter<double>("iner_start", this->iner_start);
    this->declare_parameter<double>("iner_end", this->iner_end);

    // Get params
    this->get_parameter("goal_x", this->goal.x);
    this->get_parameter("goal_y", this->goal.y);

    this->get_parameter("dt_ms", this->dt_ms);
    this->get_parameter("eps_goal", this->eps_goal);

    this->get_parameter("limits.max_vel.linear",  this->limits.max_vel.linear);
    this->get_parameter("limits.max_vel.angular", this->limits.max_vel.angular);
    this->get_parameter("limits.max_acc.linear",  this->limits.max_acc.linear);
    this->get_parameter("limits.max_acc.angular", this->limits.max_acc.angular);

    this->get_parameter("heading_weight", this->w_head);
    this->get_parameter("velocity_weight", this->w_vel);
    this->get_parameter("progress_weight", this->w_prog);
    this->get_parameter("clearence_weight", this->w_clear);

    this->get_parameter("reject_oob_trajectories", this->REJECT_OOB);

    int imax_tmp;
    this->get_parameter("imax", imax_tmp);
    this->imax = static_cast<size_t>(imax_tmp);

    this->get_parameter("n_par", this->n_par);

    this->get_parameter("eps_head", this->eps_head);
    this->get_parameter("eps_cost", this->eps_cost);
    this->get_parameter("patience", this->patience);
    this->get_parameter("random_init", this->RANDOM_INIT);

    this->get_parameter("acc_cog", this->acc_cog);
    this->get_parameter("acc_soc", this->acc_soc);
    this->get_parameter("iner_start", this->iner_start);
    this->get_parameter("iner_end", this->iner_end);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    DwaPsoPlanner::SharedPtr node = std::make_shared<DwaPsoPlanner>();

    // Double threaded executor
    rclcpp::executors::MultiThreadedExecutor exec(
        rclcpp::ExecutorOptions(),
        2
    );

    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}