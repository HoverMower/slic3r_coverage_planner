#include "coverage_planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slic3r_coverage_planner");
    
    CoveragePlanner coverage_planner();

    ros::spin();
    return 0;
}