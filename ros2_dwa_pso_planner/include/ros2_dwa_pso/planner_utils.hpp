#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>

static double wrap_angle(double a){
    return std::atan2(std::sin(a), std::cos(a));
}

static double alpha_(const size_t k){
    if(k == 0){
        return 100.0;
    }
    return 100*std::sqrt(k);
}

static double beta_(const uint q){
    if(q > 0 && q < 1e-3){
        return 10.0;
    } else if(q <= 0.1){
        return 20.0;
    } else if(q <= 1){
        return 100.0;
    } else if(q > 1){
        return 300.0;
    }
    return 0.0;
}

static double gamma_(const uint q){
    if(q >= 1){
        return 2.0;
    }
    return 1.0;
}

static int signed_dir(const double val){
    if(val > 1e-4){
        return +1;
    }else if(val < -1e-4){
        return -1;
    }
    return 0;
}

static double get_yaw(const geometry_msgs::msg::Quaternion& orientation){
    tf2::Quaternion q {
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    };

    tf2::Matrix3x3 m{q};

    double roll, pitch, yaw;

    m.getRPY(roll, pitch, yaw);

    return yaw;
}