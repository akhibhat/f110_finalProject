#include <cmath>
#include <string>
#include <vector>

#include "dynamics/vehicle_state.h"

class DubinsPath
{
    private:
        const double pi;

    public:
        DubinsPath(State start, State end)
        {
            pi = boost::math::constants::pi<double>();
        }

        struct Path
        {
            double t;
            double p;
            double q;
            std::string mode;
        };

        double mod2Pi(const double theta)
        {
            return theta - 2.0 * pi * floor(theta/(2 * pi));
        }

        double pi2Pi(const double angle)
        {
            return (angle + pi) % (2 * pi) - pi;
        }

        Path leftStraightLeft(const double alpha, const double betaa, const double d)
        {
            Path lsl_path;
            lsl_path.p = std::nan;
            lsl_path.t = std::nan;
            lsl_path.q = std::nan;
            lsl_path.mode = "lsl";

            const auto p_sq = 2 + (d * d) - (2 * cos(alpha-betaa)) + (2 * d * (sin(alpha) - sin(betaa)));

            if p_sq < 0
            {
                return lsl_path;
            }

            const auto tmp = atan2((cos(betaa) - cos(alpha)), d + sin(alpha) - sin(betaa));

            lsl_path.t = mod2Pi(-alpha + tmp);
            lsl_path.p = sqrt(p_sq);
            lsl_path.q = mod2Pi(betaa - tmp);

            return lsl_path;
        }

        Path rightStraightRight(const double alpha, const double betaa, const double d)
        {
            Path rsr_path;
            rsr_path.p = std::nan;
            rsr_path.q = std::nan;
            rsr_path.t = std::nan;
            rsr_path.mode = "rsr";

            const auto p_sq = 2 + (d * d) - (2 * cos(alpha-betaa)) + (2 * d * (sin(betaa) - sin(alpha)));

            if p_sq < 0
            {
                return rsr_path;
            }

            const auto tmp = atan2((cos(alpha) - cos(betaa)), d - sin(alpha) + sin(betaa));

            rsr_path.t = mod2Pi(alpha - tmp);
            rsr_path.p = sqrt(p);
            rsr_path.q = mod2Pi(-betaa + tmp);

            return rsr_path;
        }

        Path leftStraightRight(const double alpha, const double betaa, const double d)
        {
            Path lsr_path;
            lsr_path.p = std::nan;
            lsr_path.t = std::nan;
            lsr_path.q = std::nan;
            lsr_path.mode = "lsr";

            const auto p_sq = -2 + (d * d) + (2 * cos(alpha-betaa)) + (2 * d * (sin(betaa) + sin(alpha)));

            if p_sq < 0
            {
                return lsr_path;
            }

            const auto tmp = atan2((-cos(alpha) - cos(betaa)), (d + sin(alpha) + sin(betaa))) - atan2(-2, sqrt(p_sq));

            lsr_path.p = sqrt(p_sq);
            lsr_path.t = mod2Pi(-alpha + tmp);
            lsr_path.q = mod2Pi(-mod2Pi(betaa) + tmp);

            return lsr_path;
        }

        Path rightStraightLeft(const double alpha, const double betaa, const double d)
        {
            Path rsl_path;
            rsl_path.p = std::nan;
            rsl_path.t = std::nan;
            rsl_path.q = std::nan;
            rsl_path.mode = "rsl";

            const auto p_sq = (d * d) - 2 + (2 * cos(alpha-betaa)) - (2 * d * (sin(alpha) - sin(betaa)));

            if p_sq < 0
            {
                return rsl_path;
            }

            const auto tmp = atan2((cos(alpha) + cos(betaa)), (d - sin(alpha) - sin(betaa))) - atan2(2.0, sqrt(p_sq));

            rsl_path.p = sqrt(p_sq);
            rsl_path.t = mod2Pi(alpha - tmp);
            rsl_path.q = mod2Pi(betaa - tmp);

            return rsl_path
        }

        Path rightLeftRight(const double alpha, const double betaa, const double d)
        {
            Path rlr_path;
            rlr_path.p = std::nan;
            rlr_path.t = std::nan;
            rlr_path.q = std::nan;
            rlr_path.mode = "rlr";

            const auto temp_rlr = (6.0 - d * d + 2.0 * cos(alpha-betaa) + 2.0 * d * (sin(alpha) - sin(betaa)))/8.0;

            if abs(temp_rlr) > 1.0
            {
                return rlr_path;
            }

            double p = mod2Pi(2 * pi - acos(temp_rlr));

            rlr_path.p = p
            rlr_path.t = mod2Pi(alpha - atan2(cos(alpha) - cos(betaa), d - sin(alpha) + sin(betaa)) + mod2Pi(p/2.0));
            rlr_path.q = mod2Pi(alpha - betaa + mod2Pi(rlr_path.p));

            return rlr_path;
        }

        Path leftRightLeft(const double alpha, const double betaa, const double d)
        {
            Path lrl_path;
            lrl_path.p = std::nan;
            lrl_path.t = std::nan;
            lrl_path.q = std::nan;
            lrl_path.mode = "lrl";

            const double temp_lrl = (6.0 - d * d + 2.0 * cos(alpha-betaa) + 2.0 * d * (-sin(alpha) + sin(betaa))) / 8.0;

            if abs(temp_lrl) > 1.0
            {
                return lrl_path;
            }

            double p = mod2Pi(2 * pi - acos(temp_lrl));

            lrl_path.p = p;
            lrl_path.t = mod2Pi(-alpha - atan2(cos(alpha) - cos(betaa), d + sin(alpha) - sin(betaa)) + p/2.0);
            lrl_path.q = mod2Pi(mod2Pi(betaa) - alpha - t + mod2Pi(p));

            return lrl_path;
        }

}
