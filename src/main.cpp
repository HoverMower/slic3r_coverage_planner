#include "coverage_planner.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto coverage_planner = std::make_shared<CoveragePlanner>("slic3r_coverage_planner");

        rclcpp::spin(coverage_planner);
        rclcpp::shutdown();

    return 0;
}
