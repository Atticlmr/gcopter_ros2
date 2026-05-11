#include "gcopter/flatness.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/geo_utils.hpp"
#include "gcopter/quickhull.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/voxel_map.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Eigen>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct Config
{
    std::string map_topic;
    std::string target_topic;
    std::string marker_frame;
    std::string controller_topic;

    double dilate_radius;
    double voxel_width;
    std::vector<double> map_bound;
    double timeout_rrt;
    double max_vel_mag;
    double max_bdr_mag;
    double max_tilt_angle;
    double min_thrust;
    double max_thrust;
    double vehicle_mass;
    double grav_acc;
    double horiz_drag;
    double vert_drag;
    double paras_drag;
    double speed_eps;
    double weight_t;
    std::vector<double> chi_vec;
    double smoothing_eps;
    int integral_intervs;
    double rel_cost_tol;

    double publish_rate_hz;
    bool enu_to_ned;
    double setpoint_yaw_enu;
};

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d &p)
{
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    return point;
}

double wrapPi(double angle)
{
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

std::array<float, 3> finiteVectorOrNan(const Eigen::Vector3d &v)
{
    return {
        static_cast<float>(v.x()),
        static_cast<float>(v.y()),
        static_cast<float>(v.z()),
    };
}

std::array<float, 3> enuVectorToNed(const Eigen::Vector3d &v)
{
    return {
        static_cast<float>(v.y()),
        static_cast<float>(v.x()),
        static_cast<float>(-v.z()),
    };
}

class Visualizer
{
public:
    Visualizer(rclcpp::Node *node, std::string frame_id)
        : node_(node), frame_id_(std::move(frame_id))
    {
        auto marker_qos = rclcpp::QoS(10).reliable().transient_local();
        route_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/route", marker_qos);
        waypoints_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/waypoints", marker_qos);
        trajectory_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/trajectory", marker_qos);
        mesh_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/mesh", marker_qos);
        edge_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/edge", marker_qos);
        sphere_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/spheres", marker_qos);

        speed_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/visualizer/speed", 10);
        thrust_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/visualizer/total_thrust", 10);
        tilt_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/visualizer/tilt_angle", 10);
        body_rate_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/visualizer/body_rate", 10);
    }

    template <int D>
    void visualizeTrajectory(const Trajectory<D> &traj, const std::vector<Eigen::Vector3d> &route)
    {
        visualizeRoute(route);

        visualization_msgs::msg::Marker waypoints_marker = makeMarker("waypoints", 0);
        waypoints_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        waypoints_marker.color.r = 1.0f;
        waypoints_marker.color.a = 1.0f;
        waypoints_marker.scale.x = 0.35;
        waypoints_marker.scale.y = 0.35;
        waypoints_marker.scale.z = 0.35;

        visualization_msgs::msg::Marker traj_marker = makeMarker("trajectory", 0);
        traj_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        traj_marker.color.g = 0.5f;
        traj_marker.color.b = 1.0f;
        traj_marker.color.a = 1.0f;
        traj_marker.scale.x = 0.3;

        if (traj.getPieceNum() > 0) {
            const Eigen::MatrixXd wps = traj.getPositions();
            for (int i = 0; i < wps.cols(); ++i) {
                waypoints_marker.points.push_back(toPoint(wps.col(i)));
            }
            waypoints_pub_->publish(waypoints_marker);

            const double dt = 0.01;
            Eigen::Vector3d last = traj.getPos(0.0);
            for (double t = dt; t < traj.getTotalDuration(); t += dt) {
                const Eigen::Vector3d current = traj.getPos(t);
                traj_marker.points.push_back(toPoint(last));
                traj_marker.points.push_back(toPoint(current));
                last = current;
            }
            trajectory_pub_->publish(traj_marker);
            RCLCPP_INFO(
                node_->get_logger(),
                "Published trajectory markers: waypoint_points=%zu, trajectory_segments=%zu",
                waypoints_marker.points.size(), traj_marker.points.size() / 2);
        }
    }

    void visualizeRoute(const std::vector<Eigen::Vector3d> &route)
    {
        visualization_msgs::msg::Marker route_marker = makeMarker("route", 0);
        route_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        route_marker.color.r = 1.0f;
        route_marker.color.a = 1.0f;
        route_marker.scale.x = 0.16;

        for (const auto &point : route) {
            route_marker.points.push_back(toPoint(point));
        }

        route_pub_->publish(route_marker);
        RCLCPP_INFO(
            node_->get_logger(),
            "Published route marker: route_points=%zu, line_segments=%zu",
            route_marker.points.size(), route_marker.points.size() > 1 ? route_marker.points.size() - 1 : 0);
    }

    void visualizePolytope(const std::vector<Eigen::MatrixX4d> &h_polys)
    {
        Eigen::Matrix3Xd mesh(3, 0);
        size_t skipped_polys = 0;
        for (const auto &h_poly : h_polys) {
            Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vertices;
            if (!geo_utils::enumerateVs(h_poly, vertices)) {
                ++skipped_polys;
                continue;
            }
            if (vertices.cols() < 4) {
                ++skipped_polys;
                continue;
            }

            quickhull::QuickHull<double> qh;
            const auto hull = qh.getConvexHull(vertices.data(), vertices.cols(), false, true);
            const auto &indices = hull.getIndexBuffer();
            Eigen::Matrix3Xd tris(3, static_cast<int>(indices.size()));
            for (size_t i = 0; i < indices.size(); ++i) {
                tris.col(static_cast<int>(i)) = vertices.col(indices[i]);
            }

            const Eigen::Index old_cols = mesh.cols();
            mesh.conservativeResize(3, old_cols + tris.cols());
            mesh.rightCols(tris.cols()) = tris;
        }

        visualization_msgs::msg::Marker mesh_marker = makeMarker("mesh", 0);
        mesh_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        mesh_marker.color.b = 1.0f;
        mesh_marker.color.a = 0.15f;
        mesh_marker.scale.x = 1.0;
        mesh_marker.scale.y = 1.0;
        mesh_marker.scale.z = 1.0;

        visualization_msgs::msg::Marker edge_marker = makeMarker("edge", 0);
        edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edge_marker.color.g = 1.0f;
        edge_marker.color.b = 1.0f;
        edge_marker.color.a = 1.0f;
        edge_marker.scale.x = 0.02;

        for (int i = 0; i < mesh.cols(); ++i) {
            mesh_marker.points.push_back(toPoint(mesh.col(i)));
        }

        for (int i = 0; i < mesh.cols() / 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                edge_marker.points.push_back(toPoint(mesh.col(3 * i + j)));
                edge_marker.points.push_back(toPoint(mesh.col(3 * i + (j + 1) % 3)));
            }
        }

        mesh_pub_->publish(mesh_marker);
        edge_pub_->publish(edge_marker);
        RCLCPP_INFO(
            node_->get_logger(),
            "Published corridor markers: h_polys=%zu, skipped=%zu, triangles=%zu, edge_segments=%zu",
            h_polys.size(), skipped_polys, mesh_marker.points.size() / 3, edge_marker.points.size() / 2);
    }

    void visualizeTrackingSphere(const Eigen::Vector3d &center, double radius)
    {
        visualization_msgs::msg::Marker marker = makeMarker("tracking_sphere", 100);
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        marker.scale.x = radius * 2.0;
        marker.scale.y = radius * 2.0;
        marker.scale.z = radius * 2.0;
        marker.points.push_back(toPoint(center));
        sphere_pub_->publish(marker);
    }

    void visualizeStartGoal(const Eigen::Vector3d &center, double radius, size_t index)
    {
        if (index == 0) {
            visualization_msgs::msg::Marker deleter = makeMarker("start_goal", 0);
            deleter.action = visualization_msgs::msg::Marker::DELETEALL;
            sphere_pub_->publish(deleter);
        }

        visualization_msgs::msg::Marker marker = makeMarker("start_goal", static_cast<int>(index));
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.color.r = index == 0 ? 0.0f : 1.0f;
        marker.color.g = index == 0 ? 0.8f : 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker.scale.x = radius * 2.0;
        marker.scale.y = radius * 2.0;
        marker.scale.z = radius * 2.0;
        marker.points.push_back(toPoint(center));
        sphere_pub_->publish(marker);
    }

    void publishMetrics(double speed, double thrust, double tilt, double body_rate)
    {
        std_msgs::msg::Float64 msg;
        msg.data = speed;
        speed_pub_->publish(msg);
        msg.data = thrust;
        thrust_pub_->publish(msg);
        msg.data = tilt;
        tilt_pub_->publish(msg);
        msg.data = body_rate;
        body_rate_pub_->publish(msg);
    }

private:
    visualization_msgs::msg::Marker makeMarker(const std::string &ns, int id) const
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = node_->now();
        marker.header.frame_id = frame_id_;
        marker.ns = ns;
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        return marker;
    }

    rclcpp::Node *node_;
    std::string frame_id_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr route_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoints_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edge_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sphere_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tilt_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr body_rate_pub_;
};

class GCOPTERPlannerNode : public rclcpp::Node
{
public:
    GCOPTERPlannerNode()
        : Node("gcopter_ros2_planner")
    {
        declareParameters();
        loadParameters();
        validateParameters();

        const Eigen::Vector3i voxel_count(
            static_cast<int>((config_.map_bound[1] - config_.map_bound[0]) / config_.voxel_width),
            static_cast<int>((config_.map_bound[3] - config_.map_bound[2]) / config_.voxel_width),
            static_cast<int>((config_.map_bound[5] - config_.map_bound[4]) / config_.voxel_width));
        const Eigen::Vector3d origin(config_.map_bound[0], config_.map_bound[2], config_.map_bound[4]);
        voxel_map_ = voxel_map::VoxelMap(voxel_count, origin, config_.voxel_width);

        Eigen::VectorXd physical_params(6);
        physical_params << config_.vehicle_mass, config_.grav_acc, config_.horiz_drag,
            config_.vert_drag, config_.paras_drag, config_.speed_eps;
        flatmap_.reset(
            physical_params(0), physical_params(1), physical_params(2),
            physical_params(3), physical_params(4), physical_params(5));

        visualizer_ = std::make_unique<Visualizer>(this, config_.marker_frame);
        controller_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(config_.controller_topic, 10);

        map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            config_.map_topic, rclcpp::QoS(1).reliable(),
            std::bind(&GCOPTERPlannerNode::mapCallback, this, std::placeholders::_1));
        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            config_.target_topic, rclcpp::QoS(10),
            std::bind(&GCOPTERPlannerNode::targetCallback, this, std::placeholders::_1));

        const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / config_.publish_rate_hz));
        publish_timer_ = create_wall_timer(timer_period, std::bind(&GCOPTERPlannerNode::publishTimerCallback, this));

        RCLCPP_INFO(get_logger(), "GCOPTER ROS2 planner started");
        RCLCPP_INFO(get_logger(), "Map topic: %s, target topic: %s, marker frame: %s",
                    config_.map_topic.c_str(), config_.target_topic.c_str(), config_.marker_frame.c_str());
        RCLCPP_INFO(get_logger(), "Map bound: [%.2f %.2f] [%.2f %.2f] [%.2f %.2f], voxel width %.3f",
                    config_.map_bound[0], config_.map_bound[1], config_.map_bound[2],
                    config_.map_bound[3], config_.map_bound[4], config_.map_bound[5], config_.voxel_width);
        RCLCPP_INFO(get_logger(), "Controller output: %s, ENU->NED: %s, publish rate: %.1f Hz",
                    config_.controller_topic.c_str(), config_.enu_to_ned ? "true" : "false", config_.publish_rate_hz);
    }

private:
    struct PointFields
    {
        uint32_t x_offset = 0;
        uint32_t y_offset = 0;
        uint32_t z_offset = 0;
        bool valid = false;
    };

    void declareParameters()
    {
        declare_parameter<std::string>("map_topic", "/mock_map");
        declare_parameter<std::string>("target_topic", "/move_base_simple/goal");
        declare_parameter<std::string>("marker_frame", "map");
        declare_parameter<std::string>("controller_topic", "/controller/position/output");

        declare_parameter<double>("dilate_radius", 0.5);
        declare_parameter<double>("voxel_width", 0.25);
        declare_parameter<std::vector<double>>("map_bound", {-25.0, 25.0, -25.0, 25.0, 0.0, 5.0});
        declare_parameter<double>("timeout_rrt", 0.02);
        declare_parameter<double>("max_vel_mag", 4.0);
        declare_parameter<double>("max_bdr_mag", 2.1);
        declare_parameter<double>("max_tilt_angle", 1.05);
        declare_parameter<double>("min_thrust", 2.0);
        declare_parameter<double>("max_thrust", 12.0);
        declare_parameter<double>("vehicle_mass", 0.61);
        declare_parameter<double>("grav_acc", 9.8);
        declare_parameter<double>("horiz_drag", 0.70);
        declare_parameter<double>("vert_drag", 0.80);
        declare_parameter<double>("paras_drag", 0.01);
        declare_parameter<double>("speed_eps", 0.0001);
        declare_parameter<double>("weight_t", 20.0);
        declare_parameter<std::vector<double>>("chi_vec", {1.0e4, 1.0e4, 1.0e4, 1.0e4, 1.0e5});
        declare_parameter<double>("smoothing_eps", 1.0e-2);
        declare_parameter<int>("integral_intervs", 16);
        declare_parameter<double>("rel_cost_tol", 1.0e-5);

        declare_parameter<double>("publish_rate_hz", 50.0);
        declare_parameter<bool>("enu_to_ned", true);
        declare_parameter<double>("setpoint_yaw_enu", 0.0);
    }

    void loadParameters()
    {
        config_.map_topic = get_parameter("map_topic").as_string();
        config_.target_topic = get_parameter("target_topic").as_string();
        config_.marker_frame = get_parameter("marker_frame").as_string();
        config_.controller_topic = get_parameter("controller_topic").as_string();

        config_.dilate_radius = get_parameter("dilate_radius").as_double();
        config_.voxel_width = get_parameter("voxel_width").as_double();
        config_.map_bound = get_parameter("map_bound").as_double_array();
        config_.timeout_rrt = get_parameter("timeout_rrt").as_double();
        config_.max_vel_mag = get_parameter("max_vel_mag").as_double();
        config_.max_bdr_mag = get_parameter("max_bdr_mag").as_double();
        config_.max_tilt_angle = get_parameter("max_tilt_angle").as_double();
        config_.min_thrust = get_parameter("min_thrust").as_double();
        config_.max_thrust = get_parameter("max_thrust").as_double();
        config_.vehicle_mass = get_parameter("vehicle_mass").as_double();
        config_.grav_acc = get_parameter("grav_acc").as_double();
        config_.horiz_drag = get_parameter("horiz_drag").as_double();
        config_.vert_drag = get_parameter("vert_drag").as_double();
        config_.paras_drag = get_parameter("paras_drag").as_double();
        config_.speed_eps = get_parameter("speed_eps").as_double();
        config_.weight_t = get_parameter("weight_t").as_double();
        config_.chi_vec = get_parameter("chi_vec").as_double_array();
        config_.smoothing_eps = get_parameter("smoothing_eps").as_double();
        config_.integral_intervs = get_parameter("integral_intervs").as_int();
        config_.rel_cost_tol = get_parameter("rel_cost_tol").as_double();

        config_.publish_rate_hz = get_parameter("publish_rate_hz").as_double();
        config_.enu_to_ned = get_parameter("enu_to_ned").as_bool();
        config_.setpoint_yaw_enu = get_parameter("setpoint_yaw_enu").as_double();
    }

    void validateParameters()
    {
        if (config_.map_bound.size() != 6) {
            throw std::runtime_error("map_bound must contain 6 values: [xmin, xmax, ymin, ymax, zmin, zmax]");
        }
        if (config_.chi_vec.size() != 5) {
            throw std::runtime_error("chi_vec must contain 5 penalty weights");
        }
        if (config_.voxel_width <= 0.0) {
            throw std::runtime_error("voxel_width must be positive");
        }
        if (config_.publish_rate_hz <= 0.0) {
            throw std::runtime_error("publish_rate_hz must be positive");
        }
    }

    static bool readFloat32(const sensor_msgs::msg::PointCloud2 &msg, size_t offset, float &value)
    {
        if (offset + sizeof(float) > msg.data.size()) {
            return false;
        }

        std::array<uint8_t, sizeof(float)> bytes{};
        std::memcpy(bytes.data(), msg.data.data() + offset, sizeof(float));
        if (msg.is_bigendian) {
            std::reverse(bytes.begin(), bytes.end());
        }
        std::memcpy(&value, bytes.data(), sizeof(float));
        return true;
    }

    PointFields findPointFields(const sensor_msgs::msg::PointCloud2 &msg) const
    {
        PointFields fields;
        bool has_x = false;
        bool has_y = false;
        bool has_z = false;

        for (const auto &field : msg.fields) {
            if (field.name != "x" && field.name != "y" && field.name != "z") {
                continue;
            }
            if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count < 1) {
                RCLCPP_ERROR(get_logger(), "PointCloud2 field '%s' must be FLOAT32 with count >= 1", field.name.c_str());
                return fields;
            }

            if (field.name == "x") {
                fields.x_offset = field.offset;
                has_x = true;
            } else if (field.name == "y") {
                fields.y_offset = field.offset;
                has_y = true;
            } else {
                fields.z_offset = field.offset;
                has_z = true;
            }
        }

        fields.valid = has_x && has_y && has_z;
        if (!fields.valid) {
            RCLCPP_ERROR(get_logger(), "PointCloud2 must contain x/y/z FLOAT32 fields");
        }
        return fields;
    }

    void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (map_initialized_) {
            return;
        }

        const PointFields fields = findPointFields(*msg);
        if (!fields.valid) {
            return;
        }
        if (msg->point_step == 0) {
            RCLCPP_ERROR(get_logger(), "PointCloud2 point_step is zero");
            return;
        }

        const size_t declared_points = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
        const size_t data_points = msg->data.size() / msg->point_step;
        const size_t total_points = std::min(declared_points, data_points);
        size_t valid_points = 0;
        size_t occupied_attempts = 0;

        for (size_t i = 0; i < total_points; ++i) {
            const size_t base = i * msg->point_step;
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            if (!readFloat32(*msg, base + fields.x_offset, x) ||
                !readFloat32(*msg, base + fields.y_offset, y) ||
                !readFloat32(*msg, base + fields.z_offset, z)) {
                continue;
            }

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }

            ++valid_points;
            const Eigen::Vector3d p(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
            if (p.x() >= config_.map_bound[0] && p.x() <= config_.map_bound[1] &&
                p.y() >= config_.map_bound[2] && p.y() <= config_.map_bound[3] &&
                p.z() >= config_.map_bound[4] && p.z() <= config_.map_bound[5]) {
                ++occupied_attempts;
            }
            voxel_map_.setOccupied(p);
        }

        const int dilate_voxels = static_cast<int>(std::ceil(config_.dilate_radius / voxel_map_.getScale()));
        voxel_map_.dilate(dilate_voxels);
        map_initialized_ = true;

        RCLCPP_INFO(get_logger(), "Map initialized from '%s' frame '%s'", config_.map_topic.c_str(), msg->header.frame_id.c_str());
        RCLCPP_INFO(get_logger(), "PointCloud2 points: declared=%zu parsed=%zu valid=%zu in_bound=%zu",
                    declared_points, total_points, valid_points, occupied_attempts);
        RCLCPP_INFO(get_logger(), "Voxel map origin=(%.2f, %.2f, %.2f), corner=(%.2f, %.2f, %.2f), dilate=%d voxels",
                    voxel_map_.getOrigin().x(), voxel_map_.getOrigin().y(), voxel_map_.getOrigin().z(),
                    voxel_map_.getCorner().x(), voxel_map_.getCorner().y(), voxel_map_.getCorner().z(), dilate_voxels);
    }

    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!map_initialized_) {
            RCLCPP_WARN(get_logger(), "Reject target: map is not initialized yet");
            return;
        }

        if (start_goal_.size() >= 2) {
            start_goal_.clear();
            active_trajectory_ = false;
            traj_.clear();
            RCLCPP_INFO(get_logger(), "Received new target after a completed pair; clearing start/goal");
        }

        const double map_min_z = config_.map_bound[4];
        const double map_z_range = config_.map_bound[5] - config_.map_bound[4];
        const double z_goal = map_min_z + config_.dilate_radius +
                              std::abs(msg->pose.orientation.z) * (map_z_range - 2.0 * config_.dilate_radius);
        const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, z_goal);

        if (voxel_map_.query(goal) != 0) {
            RCLCPP_WARN(get_logger(), "Infeasible position selected: (%.2f, %.2f, %.2f)",
                        goal.x(), goal.y(), goal.z());
            return;
        }

        const size_t index = start_goal_.size();
        visualizer_->visualizeStartGoal(goal, 0.5, index);
        start_goal_.push_back(goal);
        RCLCPP_INFO(get_logger(), "%s received: (%.2f, %.2f, %.2f)",
                    index == 0 ? "Start" : "Goal", goal.x(), goal.y(), goal.z());

        if (start_goal_.size() == 2) {
            plan();
        }
    }

    void plan()
    {
        std::vector<Eigen::Vector3d> route;
        const double rrt_cost = sfc_gen::planPath<voxel_map::VoxelMap>(
            start_goal_[0],
            start_goal_[1],
            voxel_map_.getOrigin(),
            voxel_map_.getCorner(),
            &voxel_map_,
            config_.timeout_rrt,
            route);

        if (route.size() <= 1 || !std::isfinite(rrt_cost)) {
            RCLCPP_WARN(get_logger(), "RRT failed or returned an invalid route, route points=%zu", route.size());
            return;
        }
        RCLCPP_INFO(get_logger(), "RRT route generated: points=%zu, cost=%.3f", route.size(), rrt_cost);
        visualizer_->visualizeRoute(route);

        std::vector<Eigen::Vector3d> surface_points;
        voxel_map_.getSurf(surface_points);
        if (surface_points.empty()) {
            RCLCPP_WARN(get_logger(), "Voxel surface point set is empty; cannot build safe flight corridor");
            return;
        }

        std::vector<Eigen::MatrixX4d> h_polys;
        sfc_gen::convexCover(
            route,
            surface_points,
            voxel_map_.getOrigin(),
            voxel_map_.getCorner(),
            7.0,
            3.0,
            h_polys);
        sfc_gen::shortCut(h_polys);
        if (h_polys.empty()) {
            RCLCPP_WARN(get_logger(), "No safe flight corridor polytopes were generated");
            return;
        }
        RCLCPP_INFO(get_logger(), "Safe flight corridor generated: surface points=%zu, h_polys=%zu",
                    surface_points.size(), h_polys.size());
        visualizer_->visualizePolytope(h_polys);

        Eigen::Matrix3d ini_state;
        Eigen::Matrix3d fin_state;
        ini_state << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
        fin_state << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

        Eigen::VectorXd magnitude_bounds(5);
        magnitude_bounds << config_.max_vel_mag, config_.max_bdr_mag, config_.max_tilt_angle,
            config_.min_thrust, config_.max_thrust;
        Eigen::VectorXd penalty_weights(5);
        penalty_weights << config_.chi_vec[0], config_.chi_vec[1], config_.chi_vec[2],
            config_.chi_vec[3], config_.chi_vec[4];
        Eigen::VectorXd physical_params(6);
        physical_params << config_.vehicle_mass, config_.grav_acc, config_.horiz_drag,
            config_.vert_drag, config_.paras_drag, config_.speed_eps;

        traj_.clear();
        gcopter::GCOPTER_PolytopeSFC optimizer;
        const bool setup_ok = optimizer.setup(
            config_.weight_t,
            ini_state,
            fin_state,
            h_polys,
            INFINITY,
            config_.smoothing_eps,
            config_.integral_intervs,
            magnitude_bounds,
            penalty_weights,
            physical_params);
        if (!setup_ok) {
            RCLCPP_WARN(get_logger(), "GCOPTER setup failed");
            return;
        }

        const double final_cost = optimizer.optimize(traj_, config_.rel_cost_tol);
        if (std::isinf(final_cost) || traj_.getPieceNum() <= 0) {
            RCLCPP_WARN(get_logger(), "GCOPTER optimize failed, cost=%f, pieces=%d", final_cost, traj_.getPieceNum());
            return;
        }

        traj_start_time_ = now();
        active_trajectory_ = true;
        visualizer_->visualizeTrajectory(traj_, route);
        RCLCPP_INFO(get_logger(), "GCOPTER trajectory ready: pieces=%d, duration=%.3f s, cost=%.3f",
                    traj_.getPieceNum(), traj_.getTotalDuration(), final_cost);
    }

    void publishTimerCallback()
    {
        if (!active_trajectory_ || traj_.getPieceNum() <= 0) {
            return;
        }

        const double t = (now() - traj_start_time_).seconds();
        if (t < 0.0) {
            return;
        }
        if (t > traj_.getTotalDuration()) {
            active_trajectory_ = false;
            RCLCPP_INFO(get_logger(), "Trajectory setpoint publishing finished at %.3f s", t);
            return;
        }

        const Eigen::Vector3d pos = traj_.getPos(t);
        const Eigen::Vector3d vel = traj_.getVel(t);
        const Eigen::Vector3d acc = traj_.getAcc(t);
        const Eigen::Vector3d jer = traj_.getJer(t);

        double thrust = 0.0;
        Eigen::Vector4d quat;
        Eigen::Vector3d body_rate;
        flatmap_.forward(vel, acc, jer, config_.setpoint_yaw_enu, 0.0, thrust, quat, body_rate);
        const double speed = vel.norm();
        const double tilt = std::acos(std::clamp(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)), -1.0, 1.0));

        visualizer_->publishMetrics(speed, thrust, tilt, body_rate.norm());
        visualizer_->visualizeTrackingSphere(pos, config_.dilate_radius);
        publishSetpoint(pos, vel, acc, jer);
    }

    void publishSetpoint(
        const Eigen::Vector3d &pos,
        const Eigen::Vector3d &vel,
        const Eigen::Vector3d &acc,
        const Eigen::Vector3d &jer)
    {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);

        if (config_.enu_to_ned) {
            msg.position = enuVectorToNed(pos);
            msg.velocity = enuVectorToNed(vel);
            msg.acceleration = enuVectorToNed(acc);
            msg.jerk = enuVectorToNed(jer);
            msg.yaw = static_cast<float>(wrapPi(kPi * 0.5 - config_.setpoint_yaw_enu));
            msg.yawspeed = 0.0f;
        } else {
            msg.position = finiteVectorOrNan(pos);
            msg.velocity = finiteVectorOrNan(vel);
            msg.acceleration = finiteVectorOrNan(acc);
            msg.jerk = finiteVectorOrNan(jer);
            msg.yaw = static_cast<float>(wrapPi(config_.setpoint_yaw_enu));
            msg.yawspeed = 0.0f;
        }

        controller_pub_->publish(msg);
    }

    Config config_;
    bool map_initialized_ = false;
    bool active_trajectory_ = false;
    voxel_map::VoxelMap voxel_map_;
    flatness::FlatnessMap flatmap_;
    std::unique_ptr<Visualizer> visualizer_;
    std::vector<Eigen::Vector3d> start_goal_;
    Trajectory<5> traj_;
    rclcpp::Time traj_start_time_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr controller_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GCOPTERPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
