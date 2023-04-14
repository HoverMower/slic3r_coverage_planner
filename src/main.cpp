#include "coverage_planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slic3r_coverage_planner"); 
    slic3r_coverage_planner::CoveragePlanner coverage_planner;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    while (ros::ok())
    {
    }

    spinner.stop();
    return 0;
}