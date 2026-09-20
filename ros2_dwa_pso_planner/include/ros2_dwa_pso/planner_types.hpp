#include <nav_msgs/msg/path.hpp>
#include <limits>

namespace planner_types {

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

struct Window {
    double v_min, v_max;
    double w_min, w_max;
};

struct Trajectory {
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

}