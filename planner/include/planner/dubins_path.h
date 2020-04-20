#pragma once

#include <cmath>
#include <string>

//#include "dynamics/vehicle_state.h"

class DubinsPath
{
    public:

        struct Path
        {
            double t;
            double p;
            double q;
            std::string mode;
        };

        double mod2pi(const double theta);
        double pi2pi(const double angle);
        Path leftStraightLeft(const double alpha, const double betaa, const double d);
        Path rightStraightRight(const double alpha, const double betaa, const double d);
        Path leftStraightRight(const double alpha, const double betaa, const double d);
        Path rightStraightLeft(const double alpha, const double betaa, const double d);
        Path rightLeftRight(const double alpha, const double betaa, const double d);
        Path leftRightLeft(const double alpha, const double betaa, const double d);

};
