#ifndef COVERAGE_PLANNER_H_
#define COVERAGE_PLANNER_H_

#include "ros/ros.h"

#include "ExPolygon.hpp"
#include "Polyline.hpp"
#include "Fill/FillRectilinear.hpp"
#include "Fill/FillConcentric.hpp"

#include "slic3r_coverage_planner/PlanPath.h"
#include "visualization_msgs/MarkerArray.h"
#include "Surface.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <Fill/FillPlanePath.hpp>
#include <PerimeterGenerator.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ClipperUtils.hpp"
#include "ExtrusionEntityCollection.hpp"

#include <costmap_converter/ObstacleMsg.h>
#include <costmap_converter/costmap_converter_interface.h>

namespace teb_local_planner
{

    /**
     * @class CoveragePlanner
     * @brief Implements a service to create a path, which covers a given polygonal area.
     * @todo implement costmap
     */
    class CoveragePlanner
    {

    public:
        CoveragePlanner();
        bool planPath(slic3r_coverage_planner::PlanPathRequest &req, slic3r_coverage_planner::PlanPathResponse &res);
        void dyn_callback(CoveragePlanner::CoveragePlannerConfig &config, uint32_t level);

    private:
        ros::NodeHandle nh_;
        ros::Publisher marker_array_publisher_;
        ros::ServiceServer plan_path_srv_;
        ros::Subscriber costmap_converter_sub_;

        bool visualize_plan_;
        bool doPerimeterClockwise_;
        bool use_costmap_converter_;

        costmap_converter::ObstacleArrayMsg custom_obstacle_msg_; //!< Copy of the most recent obstacle message

        boost::mutex custom_obst_mutex_; //!< Mutex that locks the obstacle array (multi-threaded)

        /**
         * @brief Callback for custom obstacles that are not obtained from the costmap
         * @param obst_msg pointer to the message containing a list of polygon shaped obstacles
         */
        void customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg);

        void createLineMarkers(std::vector<Polygons> outline_groups, std::vector<Polygons> obstacle_groups, Polylines &fill_lines, visualization_msgs::MarkerArray &markerArray);
        void traverse(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups);

        std::vector<Slic3r::Polygon> getCostmapObstacles();

        // dynamic reconfigure
        typedef dynamic_reconfigure::Server<CoveragePlanner::CoveragePlannerConfig> DynamicReconfigServer;
        boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
        DynamicReconfigServer::CallbackType param_reconfig_callback_;
    };
};
#endif