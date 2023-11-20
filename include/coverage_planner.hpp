#ifndef COVERAGE_PLANNER_H
#define COVERAGE_PLANNER_H

#include "rclcpp/rclcpp.hpp"

#include "ExPolygon.hpp"
#include "Polyline.hpp"
#include "Fill/FillRectilinear.hpp"
#include "Fill/FillConcentric.hpp"

#include "slic3r_coverage_planner/srv/plan_path.hpp"
#include "slic3r_coverage_planner/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "Surface.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <Fill/FillPlanePath.hpp>
#include <PerimeterGenerator.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "ClipperUtils.hpp"
#include "ExtrusionEntityCollection.hpp"

class CoveragePlanner : public rclcpp::Node
{

public:
    /**
     * class constructor
     *
     * @param name name of the node
     */
    CoveragePlanner(std::string name);

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

private:
    bool visualize_plan;
    bool doPerimeterClockwise;
    bool useEquallySpacedPoints;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher;

    rclcpp::Service<slic3r_coverage_planner::srv::PlanPath>::SharedPtr path_service;

    void createLineMarkers(std::vector<Polygons> outline_groups, std::vector<Polygons> obstacle_groups, Polylines &fill_lines, visualization_msgs::msg::MarkerArray &markerArray);
    void traverse(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups);
    void planPath(const std::shared_ptr<slic3r_coverage_planner::srv::PlanPath::Request> req,
                                   std::shared_ptr<slic3r_coverage_planner::srv::PlanPath::Response> res);
};

#endif