//
// Created by Clemens Elflein on 27.08.21.
// Migratet to ROS2 by Patrick Weber on 13.11.2023
//

#include "coverage_planner.hpp"

CoveragePlanner::CoveragePlanner(std::string name) : Node(name)
{
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Publish result as marker array";
    this->declare_parameter("visualize_plan", true, param_desc);

    auto param_desc1 = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc1.description = "Follow Perimeter of area cw";
    this->declare_parameter("doPerimeterClockwise", false, param_desc1);

    auto param_desc2 = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc2.description = "build equally spaced poses";
    this->declare_parameter("equally_spaced_points", true, param_desc2);

    this->get_parameter("visualize_plan", visualize_plan);
    this->get_parameter("doPerimeterClockwise", doPerimeterClockwise);
    this->get_parameter("equally_spaced_points", useEquallySpacedPoints);

    // Register  publisher
    marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("slic3r_coverage_planner/path_marker_array", 100);

    // Register services
    path_service = this->create_service<slic3r_coverage_planner::srv::PlanPath>("slic3r_coverage_planner/plan_path", std::bind(&CoveragePlanner::planPath, this, std::placeholders::_1, std::placeholders::_2));

    // register parameter change callback handle
    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&CoveragePlanner::parametersCallback, this, std::placeholders::_1));
}

void CoveragePlanner::createLineMarkers(std::vector<Polygons> outline_groups, std::vector<Polygons> obstacle_groups, Polylines &fill_lines, visualization_msgs::msg::MarkerArray &markerArray)
{

    std::vector<std_msgs::msg::ColorRGBA> colors;

    {
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::msg::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::msg::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::msg::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::msg::ColorRGBA color;
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
                visualization_msgs::msg::Marker marker;

                marker.header.frame_id = "map";
                marker.ns = "mower_map_service_lines";
                marker.id = static_cast<int>(markerArray.markers.size());
                marker.frame_locked = true;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                marker.color = colors[cidx];
                marker.pose.orientation.w = 1;
                marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
                for (auto &pt : line.points)
                {
                    geometry_msgs::msg::Point vpt;
                    vpt.x = unscale(pt.x);
                    vpt.y = unscale(pt.y);
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
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = static_cast<int>(markerArray.markers.size());
            marker.frame_locked = true;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.color = colors[cidx];
            marker.pose.orientation.w = 1;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
            for (auto &pt : line.points)
            {
                geometry_msgs::msg::Point vpt;
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
                visualization_msgs::msg::Marker marker;

                marker.header.frame_id = "map";
                marker.ns = "mower_map_service_lines";
                marker.id = static_cast<int>(markerArray.markers.size());
                marker.frame_locked = true;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                marker.color = colors[cidx];
                marker.pose.orientation.w = 1;
                marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
                for (auto &pt : line.points)
                {
                    geometry_msgs::msg::Point vpt;
                    vpt.x = unscale(pt.x);
                    vpt.y = unscale(pt.y);
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
            // Grouping together multiple polygons can lead to issues since the segment
            // from the last point of the n polygon to the first point of the n+1 polygon might go through not authorized areas or obstacles
            // line_groups.push_back(Polygons());
        }
        else
        {
            traverse(contour.children, line_groups);
        }
        line_groups.push_back(Polygons());
        line_groups.back().push_back(contour.polygon);
    }
}

void CoveragePlanner::groupPolygons(std::vector<Polygons> &polygons, float distance_lines)
{
    Slic3r::Polygons *currentPolygons = &polygons[0];
    for (int i = 1; i < (int)polygons.size(); ++i)
    {
        Slic3r::Point lastPoint = currentPolygons->back().points.back();
        auto distance = unscale(lastPoint.distance_to(polygons[i].front().points.front()));
        if (distance < distance_lines * 3)
        {
            currentPolygons->push_back(polygons[i].front());
            polygons.erase(polygons.begin() + i);
            --i;
        }
        else
        {
            currentPolygons = &polygons[i];
        }
    }
}

void CoveragePlanner::planPath(const std::shared_ptr<slic3r_coverage_planner::srv::PlanPath::Request> req,
                               std::shared_ptr<slic3r_coverage_planner::srv::PlanPath::Response> res)
{
    RCLCPP_INFO_STREAM(get_logger(), "perimeter clockwise: " << doPerimeterClockwise);
    RCLCPP_INFO_STREAM(get_logger(), "equally_spaced points: " << useEquallySpacedPoints);

    Slic3r::Polygon outline_poly;
    for (auto &pt : req->outline.points)
    {
        outline_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
    }

    outline_poly.make_counter_clockwise();

    // This ExPolygon contains our input area with holes.
    Slic3r::ExPolygon expoly(outline_poly);

    for (auto &hole : req->holes)
    {
        Slic3r::Polygon hole_poly;
        for (auto &pt : hole.points)
        {
            hole_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
        }
        hole_poly.make_clockwise();

        expoly.holes.push_back(hole_poly);
    }

    // Results are stored here
    std::vector<Polygons> area_outlines;
    Polylines fill_lines;
    std::vector<Polygons> obstacle_outlines;

    coord_t distance = scale_(req->distance);
    coord_t outer_distance = scale_(req->outer_offset);

    // detect how many perimeters must be generated for this island
    int loops = req->outline_count;

    RCLCPP_INFO_STREAM(get_logger(), "generating " << loops << " outlines");

    const int loop_number = loops - 1; // 0-indexed loops
    const int inner_loop_number = loop_number - req->outline_overlap_count;

    Polygons gaps;

    Polygons last = expoly;
    Polygons inner = last;
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
            if (i <= inner_loop_number)
            {
                inner = last;
            }

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

        traverse(contours[0], area_outlines);
        for (auto &hole : holes)
        {
            traverse(hole, obstacle_outlines);
        }

        std::reverse(obstacle_outlines.begin(), obstacle_outlines.end());

        groupPolygons(area_outlines, req->distance);
        groupPolygons(obstacle_outlines, req->distance);
    }

    ExPolygons expp = union_ex(inner);

    // Go through the innermost poly and create the fill path using a Fill object
    for (auto &poly : expp)
    {
        Slic3r::Surface surface(Slic3r::SurfaceType::stBottom, poly);

        Slic3r::Fill *fill;
        if (req->fill_type == slic3r_coverage_planner::srv::PlanPath::Request::FILL_LINEAR)
        {
            fill = new Slic3r::FillRectilinear();
        }
        else
        {
            fill = new Slic3r::FillConcentric();
        }
        fill->link_max_length = scale_(1.0);
        fill->angle = req->angle;
        fill->z = scale_(1.0);
        fill->endpoints_overlap = 0;
        fill->density = 1.0;
        fill->dont_connect = false;
        fill->dont_adjust = false;
        fill->min_spacing = req->distance;
        fill->complete = false;
        fill->link_max_length = 0;

        RCLCPP_INFO_STREAM(get_logger(), "Starting Fill. Poly size:" << surface.expolygon.contour.points.size());

        Slic3r::Polylines lines = fill->fill_surface(surface);
        append_to(fill_lines, lines);
        delete fill;
        fill = nullptr;

        RCLCPP_INFO_STREAM(get_logger(), "Fill Complete. Polyline count: " << lines.size());
        for (int i = 0; i < lines.size(); i++)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Polyline " << i << " has point count: " << lines[i].points.size());
        }
    }

    if (visualize_plan)
    {

        visualization_msgs::msg::MarkerArray arr;
        createLineMarkers(area_outlines, obstacle_outlines, fill_lines, arr);
        marker_array_publisher->publish(arr);
    }

    std_msgs::msg::Header header;
    header.stamp = get_clock()->now();
    header.frame_id = "map";
    //  header.seq = 0;

    for (auto &group : area_outlines)
    {
        slic3r_coverage_planner::msg::Path path;
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

            Points equally_spaced_points;
            if (useEquallySpacedPoints == true)
            {
                equally_spaced_points = line.equally_spaced_points(scale_(0.1));
            }
            else
            {
                equally_spaced_points = line.points;
            }

            if (doPerimeterClockwise == true)
            {
                std::reverse(equally_spaced_points.begin(), equally_spaced_points.end());
            }
            if (equally_spaced_points.size() < 2)
            {
                RCLCPP_INFO(get_logger(), "Skipping single dot");
                continue;
            }
            RCLCPP_INFO_STREAM(get_logger(), "Got " << equally_spaced_points.size() << " points");

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
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, orientation);

                geometry_msgs::msg::PoseStamped pose;
                pose.header = header;
                pose.pose.orientation = tf2::toMsg(q);
                pose.pose.position.x = unscale(lastPoint->x);
                pose.pose.position.y = unscale(lastPoint->y);
                pose.pose.position.z = 0;
                path.path.poses.push_back(pose);
                lastPoint = &pt;
            }

            // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
            geometry_msgs::msg::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = path.path.poses.back().pose.orientation;
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);
        }
        res->paths.push_back(path);
    }

    // for (int i = 0; i < fill_lines.size(); i++)
    // {
    //     auto &line = fill_lines[i];
    //     slic3r_coverage_planner::msg::Path path;
    //     path.is_outline = false;
    //     path.path.header = header;

    //     line.remove_duplicate_points();

    //     Points equally_spaced_points;
    //     if (useEquallySpacedPoints == true)
    //     {
    //         equally_spaced_points = line.equally_spaced_points(scale_(0.1));
    //     }
    //     else
    //     {
    //         equally_spaced_points = line.points;
    //     }

    //     if (equally_spaced_points.size() < 2)
    //     {
    //         RCLCPP_INFO(get_logger(), "Skipping single dot");
    //         continue;
    //     }
    //     RCLCPP_INFO_STREAM(get_logger(), "Got " << equally_spaced_points.size() << " points");

    //     Point *lastPoint = nullptr;
    //     for (auto &pt : equally_spaced_points)
    //     {
    //         if (lastPoint == nullptr)
    //         {
    //             lastPoint = &pt;
    //             continue;
    //         }

    //         // calculate pose for "lastPoint" pointing to current point

    //         auto dir = pt - *lastPoint;
    //         double orientation = atan2(dir.y, dir.x);
    //         tf2::Quaternion q;
    //         q.setRPY(0.0, 0.0, orientation);

    //         geometry_msgs::msg::PoseStamped pose;
    //         pose.header = header;
    //         pose.pose.orientation = tf2::toMsg(q);
    //         pose.pose.position.x = unscale(lastPoint->x);
    //         pose.pose.position.y = unscale(lastPoint->y);
    //         pose.pose.position.z = 0;
    //         path.path.poses.push_back(pose);
    //         lastPoint = &pt;
    //     }

    //     // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
    //     geometry_msgs::msg::PoseStamped pose;
    //     pose.header = header;
    //     pose.pose.orientation = path.path.poses.back().pose.orientation;
    //     pose.pose.position.x = unscale(lastPoint->x);
    //     pose.pose.position.y = unscale(lastPoint->y);
    //     pose.pose.position.z = 0;
    //     path.path.poses.push_back(pose);

    //     res->paths.push_back(path);
    // }

    for (auto &group : obstacle_outlines)
    {
        slic3r_coverage_planner::msg::Path path;
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

            Points equally_spaced_points;
            if (useEquallySpacedPoints == true)
            {
                equally_spaced_points = line.equally_spaced_points(scale_(0.1));
            }
            else
            {
                equally_spaced_points = line.points;
            }

            if (doPerimeterClockwise == true)
            {
                std::reverse(equally_spaced_points.begin(), equally_spaced_points.end());
            }

            if (equally_spaced_points.size() < 2)
            {
                RCLCPP_INFO(get_logger(), "Skipping single dot");
                continue;
            }
            RCLCPP_INFO_STREAM(get_logger(), "Got " << equally_spaced_points.size() << " points");

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
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, orientation);

                geometry_msgs::msg::PoseStamped pose;
                pose.header = header;
                pose.pose.orientation = tf2::toMsg(q);
                pose.pose.position.x = unscale(lastPoint->x);
                pose.pose.position.y = unscale(lastPoint->y);
                pose.pose.position.z = 0;
                path.path.poses.push_back(pose);
                lastPoint = &pt;
            }

            // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
            geometry_msgs::msg::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = path.path.poses.back().pose.orientation;
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);
        }
        res->paths.push_back(path);
    }

    for (int i = 0; i < fill_lines.size(); i++)
    {
        auto &line = fill_lines[i];
        slic3r_coverage_planner::msg::Path path;
        path.is_outline = false;
        path.path.header = header;

        line.remove_duplicate_points();

        auto equally_spaced_points = line.equally_spaced_points(scale_(0.1));
        if (equally_spaced_points.size() < 2)
        {
            RCLCPP_INFO(get_logger(), "Skipping single dot");
            continue;
        }
        RCLCPP_INFO_STREAM(get_logger(), "Got " << equally_spaced_points.size() << " points");

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
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, orientation);

            geometry_msgs::msg::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = tf2::toMsg(q);
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);
            lastPoint = &pt;
        }

        // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
        geometry_msgs::msg::PoseStamped pose;
        pose.header = header;
        pose.pose.orientation = path.path.poses.back().pose.orientation;
        pose.pose.position.x = unscale(lastPoint->x);
        pose.pose.position.y = unscale(lastPoint->y);
        pose.pose.position.z = 0;
        path.path.poses.push_back(pose);

        res->paths.push_back(path);
    }
    return;
}

rcl_interfaces::msg::SetParametersResult CoveragePlanner::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.
    for (const auto &param : parameters)
    {
        if (param.get_name() == "visualize_plan")
        {
            this->visualize_plan = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "new value for visualize plan: %i", this->visualize_plan);
        }
        if (param.get_name() == "doPerimeterClockwise")
        {
            this->doPerimeterClockwise = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "new value for perimeter clockwise: %i", this->doPerimeterClockwise);
        }
        if (param.get_name() == "equally_spaced_points")
        {
            this->useEquallySpacedPoints = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "new value for equally spaced points: %i", this->useEquallySpacedPoints);
        }
    }

    return result;
}