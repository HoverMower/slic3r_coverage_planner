//
// Created by Clemens Elflein on 27.08.21.
//

#include <coverage_planner.h>

namespace slic3r_coverage_planner
{
    CoveragePlanner::CoveragePlanner()
    {
        marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("slic3r_coverage_planner/path_marker_array", 100, true);
        costmap_converter_sub_ = nh_.subscribe("/costmap_converter/costmap_obstacles", 1, &CoveragePlanner::customObstacleCB, this);
        plan_path_srv_ = nh_.advertiseService("slic3r_coverage_planner/plan_path", &CoveragePlanner::planPath, this);

        param_reconfig_callback_ = boost::bind(&CoveragePlanner::dyn_callback, this, _1, _2);

        param_reconfig_server_.reset(new DynamicReconfigServer());
        param_reconfig_server_->setCallback(param_reconfig_callback_);

        tfListener = new tf2_ros::TransformListener(tf2_);

        ROS_INFO("Slic3r_coverage_planner initialized");
    }

    void CoveragePlanner::createLineMarkers(std::vector<Polygons> outline_groups, std::vector<Polygons> obstacle_groups, Polylines &fill_lines, visualization_msgs::MarkerArray &markerArray)
    {

        std::vector<std_msgs::ColorRGBA> colors;

        {
            std_msgs::ColorRGBA color;
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 1.0;
            colors.push_back(color);
        }
        {
            std_msgs::ColorRGBA color;
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 1.0;
            colors.push_back(color);
        }
        {
            std_msgs::ColorRGBA color;
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 1.0;
            colors.push_back(color);
        }
        {
            std_msgs::ColorRGBA color;
            color.r = 1.0;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 1.0;
            colors.push_back(color);
        }
        {
            std_msgs::ColorRGBA color;
            color.r = 1.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 1.0;
            colors.push_back(color);
        }
        {
            std_msgs::ColorRGBA color;
            color.r = 0.0;
            color.g = 1.0;
            color.b = 1.0;
            color.a = 1.0;
            colors.push_back(color);
        }
        {
            std_msgs::ColorRGBA color;
            color.r = 1.0;
            color.g = 1.0;
            color.b = 1.0;
            color.a = 1.0;
            colors.push_back(color);
        }

        uint32_t cidx = 0;

        for (auto &group : outline_groups)
        {
            for (auto &line : group)
            {
                {
                    visualization_msgs::Marker marker;

                    marker.header.frame_id = "map";
                    marker.ns = "mower_map_service_lines";
                    marker.id = static_cast<int>(markerArray.markers.size());
                    marker.frame_locked = true;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.type = visualization_msgs::Marker::LINE_STRIP;
                    marker.color = colors[cidx];
                    marker.pose.orientation.w = 1;
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
                    for (auto &pt : line.points)
                    {
                        geometry_msgs::Point vpt;
                        vpt.x = unscale(pt.x);
                        vpt.y = unscale(pt.y);
                        marker.points.push_back(vpt);
                    }
                    // republish the first point to get a closed polygon
                    if (!line.points.empty())
                    {
                        geometry_msgs::Point vpt;
                        vpt.x = unscale(line.points[0].x);
                        vpt.y = unscale(line.points[0].y);
                        marker.points.push_back(vpt);
                    }

                    markerArray.markers.push_back(marker);
                }
            }
            cidx = (cidx + 1) % colors.size();
        }

        for (auto &line : fill_lines)
        {
            {
                visualization_msgs::Marker marker;

                marker.header.frame_id = "map";
                marker.ns = "mower_map_service_lines";
                marker.id = static_cast<int>(markerArray.markers.size());
                marker.frame_locked = true;
                marker.action = visualization_msgs::Marker::ADD;
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.color = colors[cidx];
                marker.pose.orientation.w = 1;
                marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
                for (auto &pt : line.points)
                {
                    geometry_msgs::Point vpt;
                    vpt.x = unscale(pt.x);
                    vpt.y = unscale(pt.y);
                    marker.points.push_back(vpt);
                }

                markerArray.markers.push_back(marker);

                cidx = (cidx + 1) % colors.size();
            }
        }
        for (auto &group : obstacle_groups)
        {
            for (auto &line : group)
            {
                {
                    visualization_msgs::Marker marker;

                    marker.header.frame_id = "map";
                    marker.ns = "mower_map_service_lines";
                    marker.id = static_cast<int>(markerArray.markers.size());
                    marker.frame_locked = true;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.type = visualization_msgs::Marker::LINE_STRIP;
                    marker.color = colors[cidx];
                    marker.pose.orientation.w = 1;
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
                    for (auto &pt : line.points)
                    {
                        geometry_msgs::Point vpt;
                        vpt.x = unscale(pt.x);
                        vpt.y = unscale(pt.y);
                        marker.points.push_back(vpt);
                    }
                    // republish the first point to get a closed polygon
                    if (!line.points.empty())
                    {
                        geometry_msgs::Point vpt;
                        vpt.x = unscale(line.points[0].x);
                        vpt.y = unscale(line.points[0].y);
                        marker.points.push_back(vpt);
                    }
                    markerArray.markers.push_back(marker);
                }
            }
            cidx = (cidx + 1) % colors.size();
        }
    }

    void CoveragePlanner::traverse(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups)
    {
        for (auto &contour : contours)
        {
            if (contour.children.empty())
            {
                line_groups.push_back(Polygons());
            }
            else
            {
                traverse(contour.children, line_groups);
            }
            line_groups.back().push_back(contour.polygon);
        }
    }

    bool CoveragePlanner::planPath(slic3r_coverage_planner::PlanPathRequest &req, slic3r_coverage_planner::PlanPathResponse &res)
    {
        ROS_INFO("start planning");
        ROS_INFO_STREAM("perimeter clockwise: " << doPerimeterClockwise_);
        ROS_INFO_STREAM("add costmap obstacles: " << use_costmap_converter_);
        Slic3r::Polygon outline_poly;
        for (auto &pt : req.outline.points)
        {
            outline_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
        }

        outline_poly.make_counter_clockwise();

        // This ExPolygon contains our input area with holes.
        // an ExPolygon is a polygon with holes
        Slic3r::ExPolygon expoly(outline_poly);

        for (auto &hole : req.holes)
        {
            Slic3r::Polygon hole_poly;
            for (auto &pt : hole.points)
            {
                hole_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
            }
            hole_poly.make_clockwise();

            expoly.holes.push_back(hole_poly);
        }

        // add costmap obstacles
        if (use_costmap_converter_)
        {
            // first create a Polygon vector to store all obstacle outlines
            std::vector<Slic3r::Polygon> obstacle_polygons;

            // convert costmap to polygon
            obstacle_polygons = getCostmapObstacles(outline_poly);

            // add polygons to holes
            for (auto &obstacle : obstacle_polygons)
            {
                Slic3r::Polygon hole_poly;
                for (auto &pt : obstacle.points)
                {
                    hole_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
                }

                hole_poly.make_clockwise();

                expoly.holes.push_back(hole_poly);
            }
        }
        // Results are stored here
        std::vector<Polygons> area_outlines;
        Polylines fill_lines;
        std::vector<Polygons> obstacle_outlines;

        coord_t distance = scale_(req.distance);
        coord_t outer_distance = scale_(req.outer_offset);

        // detect how many perimeters must be generated for this island
        int loops = req.outline_count;

        ROS_INFO_STREAM("generating " << loops << " outlines");

        const int loop_number = loops - 1; // 0-indexed loops

        Polygons gaps;

        Polygons last = expoly;
        if (loop_number >= 0)
        { // no loops = -1

            std::vector<PerimeterGeneratorLoops> contours(loop_number + 1); // depth => loops
            std::vector<PerimeterGeneratorLoops> holes(loop_number + 1);    // depth => loops

            for (int i = 0; i <= loop_number; ++i)
            { // outer loop is 0
                Polygons offsets;

                if (i == 0)
                {
                    offsets = offset(
                        last,
                        -outer_distance);
                }
                else
                {
                    offsets = offset(
                        last,
                        -distance);
                }

                if (offsets.empty())
                    break;

                last = offsets;

                for (Polygons::const_iterator polygon = offsets.begin(); polygon != offsets.end(); ++polygon)
                {
                    PerimeterGeneratorLoop loop(*polygon, i);
                    loop.is_contour = polygon->is_counter_clockwise();
                    if (loop.is_contour)
                    {
                        contours[i].push_back(loop);
                    }
                    else
                    {
                        holes[i].push_back(loop);
                    }
                }
            }

            // nest loops: holes first
            for (int d = 0; d <= loop_number; ++d)
            {
                PerimeterGeneratorLoops &holes_d = holes[d];

                // loop through all holes having depth == d
                for (int i = 0; i < (int)holes_d.size(); ++i)
                {
                    const PerimeterGeneratorLoop &loop = holes_d[i];

                    // find the hole loop that contains this one, if any
                    for (int t = d + 1; t <= loop_number; ++t)
                    {
                        for (int j = 0; j < (int)holes[t].size(); ++j)
                        {
                            PerimeterGeneratorLoop &candidate_parent = holes[t][j];
                            if (candidate_parent.polygon.contains(loop.polygon.first_point()))
                            {
                                candidate_parent.children.push_back(loop);
                                holes_d.erase(holes_d.begin() + i);
                                --i;
                                goto NEXT_LOOP;
                            }
                        }
                    }

                NEXT_LOOP:;
                }
            }

            // nest contour loops
            for (int d = loop_number; d >= 1; --d)
            {
                PerimeterGeneratorLoops &contours_d = contours[d];

                // loop through all contours having depth == d
                for (int i = 0; i < (int)contours_d.size(); ++i)
                {
                    const PerimeterGeneratorLoop &loop = contours_d[i];

                    // find the contour loop that contains it
                    for (int t = d - 1; t >= 0; --t)
                    {
                        for (size_t j = 0; j < contours[t].size(); ++j)
                        {
                            PerimeterGeneratorLoop &candidate_parent = contours[t][j];
                            if (candidate_parent.polygon.contains(loop.polygon.first_point()))
                            {
                                candidate_parent.children.push_back(loop);
                                contours_d.erase(contours_d.begin() + i);
                                --i;
                                goto NEXT_CONTOUR;
                            }
                        }
                    }

                NEXT_CONTOUR:;
                }
            }
            // recursive call to collect all line segments
            traverse(contours[0], area_outlines);
            for (auto &hole : holes)
            {
                traverse(hole, obstacle_outlines);
            }

            for (auto &obstacle_group : obstacle_outlines)
            {
                std::reverse(obstacle_group.begin(), obstacle_group.end());
            }
        }

        ExPolygons expp = union_ex(last);

        // Go through the innermost poly and create the fill path using a Fill object
        // for (auto &poly : expp)
        // {
        //     Slic3r::Surface surface(Slic3r::SurfaceType::stBottom, poly);

        //     Slic3r::Fill *fill;
        //     if (req.fill_type == slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR)
        //     {
        //         fill = new Slic3r::FillRectilinear();
        //     }
        //     else
        //     {
        //         fill = new Slic3r::FillConcentric();
        //     }
        //     fill->link_max_length = scale_(1.0);
        //     fill->angle = req.angle;
        //     fill->z = scale_(1.0);
        //     fill->endpoints_overlap = 0;
        //     fill->density = 1.0;
        //     fill->dont_connect = false;
        //     fill->dont_adjust = false;
        //     fill->min_spacing = req.distance;
        //     fill->complete = false;
        //     fill->link_max_length = 0;

        //     ROS_INFO_STREAM("Starting Fill. Poly size:" << surface.expolygon.contour.points.size());

        //     Slic3r::Polylines lines = fill->fill_surface(surface);
        //     append_to(fill_lines, lines);
        //     delete fill;
        //     fill = nullptr;

        //     ROS_INFO_STREAM("Fill Complete. Polyline count: " << lines.size());
        //     for (int i = 0; i < lines.size(); i++)
        //     {
        //         ROS_INFO_STREAM("Polyline " << i << " has point count: " << lines[i].points.size());
        //     }
        // }

        if (visualize_plan_)
        {

            visualization_msgs::MarkerArray arr;
            createLineMarkers(area_outlines, obstacle_outlines, fill_lines, arr);
            marker_array_publisher_.publish(arr);
        }

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "map";
        header.seq = 0;

        // building the path for outline
        for (auto &group : area_outlines)
        {
            slic3r_coverage_planner::Path path;
            path.is_outline = true;
            path.path.header = header;
            int split_index = 0;
            for (int i = 0; i < group.size(); i++)
            {
                auto &poly = group[i];

                Polyline line;
                if (split_index < poly.points.size())
                {
                    line = poly.split_at_index(split_index);
                }
                else
                {
                    line = poly.split_at_first_point();
                    split_index = 0;
                }
                split_index += 2;
                line.remove_duplicate_points();

                auto equally_spaced_points = line.equally_spaced_points(scale_(0.1));
                if (doPerimeterClockwise_ == true)
                {
                    std::reverse(equally_spaced_points.begin(), equally_spaced_points.end());
                }
                if (equally_spaced_points.size() < 2)
                {
                    ROS_INFO("Skipping single dot");
                    continue;
                }
                ROS_INFO_STREAM("Got " << equally_spaced_points.size() << " points");

                Point *lastPoint = nullptr;
                for (auto &pt : equally_spaced_points)
                {
                    if (lastPoint == nullptr)
                    {
                        lastPoint = &pt;
                        continue;
                    }

                    // calculate pose for "lastPoint" pointing to current point

                    auto dir = pt - *lastPoint;
                    double orientation = atan2(dir.y, dir.x);
                    tf2::Quaternion q(0.0, 0.0, orientation);

                    geometry_msgs::PoseStamped pose;
                    pose.header = header;
                    pose.pose.orientation = tf2::toMsg(q);
                    pose.pose.position.x = unscale(lastPoint->x);
                    pose.pose.position.y = unscale(lastPoint->y);
                    pose.pose.position.z = 0;
                    path.path.poses.push_back(pose);
                    lastPoint = &pt;
                }

                // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
                geometry_msgs::PoseStamped pose;
                pose.header = header;
                pose.pose.orientation = path.path.poses.back().pose.orientation;
                pose.pose.position.x = unscale(lastPoint->x);
                pose.pose.position.y = unscale(lastPoint->y);
                pose.pose.position.z = 0;
                path.path.poses.push_back(pose);
            }
            res.paths.push_back(path);
        }

        // Building the path for infill
        for (int i = 0; i < fill_lines.size(); i++)
        {
            auto &line = fill_lines[i];
            slic3r_coverage_planner::Path path;
            path.is_outline = false;
            path.path.header = header;

            line.remove_duplicate_points();

            auto equally_spaced_points = line.equally_spaced_points(scale_(0.1));

            if (equally_spaced_points.size() < 2)
            {
                ROS_INFO("Skipping single dot");
                continue;
            }
            ROS_INFO_STREAM("Got " << equally_spaced_points.size() << " points");

            Point *lastPoint = nullptr;
            for (auto &pt : equally_spaced_points)
            {
                if (lastPoint == nullptr)
                {
                    lastPoint = &pt;
                    continue;
                }

                // calculate pose for "lastPoint" pointing to current point

                auto dir = pt - *lastPoint;
                double orientation = atan2(dir.y, dir.x);
                tf2::Quaternion q(0.0, 0.0, orientation);

                geometry_msgs::PoseStamped pose;
                pose.header = header;
                pose.pose.orientation = tf2::toMsg(q);
                pose.pose.position.x = unscale(lastPoint->x);
                pose.pose.position.y = unscale(lastPoint->y);
                pose.pose.position.z = 0;
                path.path.poses.push_back(pose);
                lastPoint = &pt;
            }

            // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = path.path.poses.back().pose.orientation;
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);

            res.paths.push_back(path);
        }

        // building the path for obstacles
        for (auto &group : obstacle_outlines)
        {
            slic3r_coverage_planner::Path path;
            path.is_outline = true;
            path.path.header = header;
            int split_index = 0;
            for (int i = 0; i < group.size(); i++)
            {
                auto &poly = group[i];

                Polyline line;
                if (split_index < poly.points.size())
                {
                    line = poly.split_at_index(split_index);
                }
                else
                {
                    line = poly.split_at_first_point();
                    split_index = 0;
                }
                split_index += 2;
                line.remove_duplicate_points();

                auto equally_spaced_points = line.equally_spaced_points(scale_(0.1));
                if (doPerimeterClockwise_ == true)
                {
                    std::reverse(equally_spaced_points.begin(), equally_spaced_points.end());
                }

                if (equally_spaced_points.size() < 2)
                {
                    ROS_INFO("Skipping single dot");
                    continue;
                }
                ROS_INFO_STREAM("Got " << equally_spaced_points.size() << " points");

                Point *lastPoint = nullptr;
                for (auto &pt : equally_spaced_points)
                {
                    if (lastPoint == nullptr)
                    {
                        lastPoint = &pt;
                        continue;
                    }

                    // calculate pose for "lastPoint" pointing to current point

                    auto dir = pt - *lastPoint;
                    double orientation = atan2(dir.y, dir.x);
                    tf2::Quaternion q(0.0, 0.0, orientation);

                    geometry_msgs::PoseStamped pose;
                    pose.header = header;
                    pose.pose.orientation = tf2::toMsg(q);
                    pose.pose.position.x = unscale(lastPoint->x);
                    pose.pose.position.y = unscale(lastPoint->y);
                    pose.pose.position.z = 0;
                    path.path.poses.push_back(pose);
                    lastPoint = &pt;
                }

                // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
                geometry_msgs::PoseStamped pose;
                pose.header = header;
                pose.pose.orientation = path.path.poses.back().pose.orientation;
                pose.pose.position.x = unscale(lastPoint->x);
                pose.pose.position.y = unscale(lastPoint->y);
                pose.pose.position.z = 0;
                path.path.poses.push_back(pose);
            }
            res.paths.push_back(path);
        }

        return true;
    }

    void CoveragePlanner::dyn_callback(slic3r_coverage_planner::CoveragePlannerConfig &config, uint32_t level)
    {
        doPerimeterClockwise_ = config.perimeter_clockwise;
        visualize_plan_ = config.visualize_plan;
        use_costmap_converter_ = config.costmap_converter;
    }

    void CoveragePlanner::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg)
    {
        if (!use_costmap_converter_)
            return;
        boost::mutex::scoped_lock l(custom_obst_mutex_);
        custom_obstacle_msg_ = *obst_msg;
    }

    // std::vector<Slic3r::Polygon> CoveragePlanner::getCostmapObstacles(Slic3r::ExPolygon area_polygon)
    std::vector<Slic3r::Polygon> CoveragePlanner::getCostmapObstacles(Slic3r::Polygon area_polygon)
    {
        std::vector<Slic3r::Polygon> obstacle_polygons;

        // Add custom obstacles obtained via message
        boost::mutex::scoped_lock l(custom_obst_mutex_);

        if (!custom_obstacle_msg_.obstacles.empty())
        {

            // We only use the global header to specify the obstacle coordinate system instead of individual ones
            // Eigen::Affine3d obstacle_to_map_eig;
            // try
            // {
            //     geometry_msgs::TransformStamped obstacle_to_map = tf2_.lookupTransform("map", custom_obstacle_msg_.header.frame_id, ros::Time(0), ros::Duration(2)); // cfg_.robot.transform_tolerance));
            //     obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
            // }
            // catch (tf2::TransformException ex)
            // {
            //     ROS_ERROR("%s", ex.what());
            //     obstacle_to_map_eig.setIdentity();
            // }

            for (size_t i = 0; i < custom_obstacle_msg_.obstacles.size(); ++i)
            {
                if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0) // circle
                {
                    Slic3r::Polygon poly; // new obstacle polygon
                    // new point of obstacle polygon
                    Slic3r::Point new_point(scale_(custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x),
                                            scale_(custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y));

                    // check if point lies inside the area we want to cover.
                    // if so, add new obstacle polygon
                    if (area_polygon.contains(new_point))
                    {
                        poly.points.push_back(new_point);
                        obstacle_polygons.push_back(poly);
                        ROS_INFO_STREAM("circular obstacle " << i << " inside ex_poly");
                    }
                }
                else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1) // point
                {
                    Slic3r::Polygon poly;
                    Slic3r::Point new_point(scale_(custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x),
                                            scale_(custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y));
                    if (area_polygon.contains(new_point))
                    {
                        poly.points.push_back(new_point);
                        obstacle_polygons.push_back(poly);
                        ROS_INFO_STREAM("point obstacle " << i << " inside ex_poly");
                    }
                }
                else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2) // line
                {
                    Slic3r::Polygon poly;
                    // get start point if line
                    Slic3r::Point start_point(scale_(custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x),
                                              scale_(custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y));
                    // get end_line of point
                    Slic3r::Point end_point(scale_(custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x),
                                            scale_(custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y));
                    // add the obstacle, if start and end of line lies inside
                    // TODO: trim the line, if e.g. start is outside but end is inside
                    if (area_polygon.contains(start_point) && area_polygon.contains(end_point))
                    {
                        poly.points.push_back(start_point);
                        poly.points.push_back(end_point);
                        obstacle_polygons.push_back(poly);
                        ROS_INFO_STREAM("line obstacle " << i << " inside ex_poly");
                    }
                }
                else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
                {
                    ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
                    continue;
                }
                else // polygon
                {
                    // PolygonObstacle *polyobst = new PolygonObstacle;
                    Slic3r::Polygon poly;
                    for (size_t j = 0; j < custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
                    {
                        // get a single point of the polygon
                        Slic3r::Point new_point(scale_(custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x),
                                                scale_(custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y));
                        // only take the new point, if it lies inside the area we want to cover
                        if (area_polygon.contains(new_point))
                        {
                            poly.points.push_back(new_point);
                        }
                    }
                    // polyobst->finalizePolygon();
                    if (!poly.points.empty())
                    {
                        obstacle_polygons.push_back(poly);
                        ROS_INFO_STREAM("polygonal obstacle " << i << " inside ex_poly with " << poly.points.size() << "points");
                    }
                }
            }
        }
        else
        {
            ROS_INFO("no obstacles in costmap found");
        }
        return obstacle_polygons;
    }
}
