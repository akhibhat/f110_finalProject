#include <cmath>
#include <string>
#include <vector>

#include "dynamics/vehicle_state.h"

class DubinsPath
{
    private:
        const double pi_;
        const double curvature_;
        const double step_size_;

    public:
        DubinsPath(State start, State end)
        {
            pi_ = boost::math::constants::pi<double>();
            step_size_ = 0.1;
            curvature = 1.0;
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
            return theta - 2.0 * pi_ * floor(theta/(2 * pi_));
        }

        double pi2Pi(const double angle)
        {
            return (angle + pi_) % (2 * pi_) - pi_;
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

            double p = mod2Pi(2 * pi_ - acos(temp_rlr));

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

            double p = mod2Pi(2 * pi_ - acos(temp_lrl));

            lrl_path.p = p;
            lrl_path.t = mod2Pi(-alpha - atan2(cos(alpha) - cos(betaa), d + sin(alpha) - sin(betaa)) + p/2.0);
            lrl_path.q = mod2Pi(mod2Pi(betaa) - alpha - t + mod2Pi(p));

            return lrl_path;
        }

        std::vector<geometry_msgs::Pose2D> generateCourse(Path best_path)
        {
            double n_point = std::trunc((best_path.t + best_path.q + best_path.p)/step_size_) + 3 + 4;

            std::vector<double> path_x;
            std::vector<double> path_y;
            std::vector<double> path_theta;
            std::vector<double> directions;

            if (best_path.t > 0.0)
            {
                directions.push_back(1.0);
            }
            else
            {
                directions.push_back(-1.0);
            }

            double l1 = 0.0;

        }

        std::pair<std::vector<geometry_msgs::Pose2D, double>>fromOrigin(geometry_msgs::Pose2D rel_pose)
        {
            double d = curvature_ * sqrt(pow(rel_pose.x, 2) + pow(rel_pose.y, 2));

            double theta = mod2Pi(atan2(rel_pose.x, rel_pose.y));
            double alpha = mod2Pi(-theta);
            double betaa = mod2Pi(rel_pose.theta - theta);

            std::vector<Path> dubins_paths;

            dubins_paths.push_back(leftStraightLeft(alpha, betaa, d));
            dubins_paths.push_back(rightStraightRight(alpha, betaa, d));
            dubins_paths.push_back(leftStraightRight(alpha, betaa, d));
            dubins_paths.push_back(rightStraightLeft(alphaa, betaa, d));
            dubins_paths.push_back(rightLeftRight(alpha, betaa, d));
            dubins_paths.push_back(leftRightLeft(alpha, betaa, d));

            double best_cost = std::numeric_limits<double>::max();
            Path best_path;

            for (int i=0; i<dubins_paths.size(); i++)
            {
                if (std::isnan(dubins_paths[i].t)) continue;

                double cost = (abs(dubins_paths[i].t) + abs(dubins_paths[i].q) + abs(dubins_paths[i].p));

                if (best_cost > cost)
                {
                    best_path = dubins_paths[i];
                    best_cost = cost;
                }
            }

            std::vector<geometry_msgs::Pose2D> path_points;

            path_points = generateCourse(best_path);

            return {path_points, best_cost}
        }

        std::pair<std::vector<geometry_msgs::Pose2D, double>> dubinsCost(geometry_msgs::Pose2D start, Waypoint end)
        {
            geometry_msgs::Pose2D relative_pos;

            double ex = end.x - start.x;
            double ey = end.y - start.y;

            relative_pos.x = ex*cos(start.theta) + ey*sin(start.theta);
            relative_pos.y = ex*sin(start.theta) + ey*cos(start.theta);
            relative_pos.yaw = end.theta - start.theta;

            std::vector<geometry_msgs::Pose2D> path_rel;
            double path_cost;

            auto path_origin = fromOrigin(relative_pose);
            path_rel = path_origin.first;
            path_cost = path_origin.second;

            std::vector<geometry_msgs::Pose2D> path_;


            for (int i=0; i<path_rel.size(); i++)
            {
                geometry_msgs::Pose2D path_point;
                path_point.x = path_rel[i].x * cos(-start.theta) + path_rel[i].y * sin(-start.theta) + start.x;
                path_point.y = -path_rel[i].x * sin(-start.theta) + path_rel[i].y * cos(-start.theta) + start.y;
                path_point.theta = pi2Pi(start.theta + path_rel[i].theta);

                path_.push_back(path_point);
            }

            return {path_, path_cost};
            
        }
}
