#include "erl_env/environment_2d.hpp"
#include "erl_path_planning_ros/ros2/astar_node.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <memory>
#include <mutex>

template<typename Dtype, typename MapDtype>
class Astar2dNode : public AstarNode<Dtype, 2> {
public:
    using Super = AstarNode<Dtype, 2>;
    using MetricState = typename Super::MetricState;
    using Env = typename Super::Env;
    using Heuristic = typename Super::Heuristic;
    using PlanRecord = typename Super::PlanRecord;
    using GridMapInfo2D = erl::common::GridMapInfo2D<Dtype>;
    using Env2d = erl::env::Environment2D<Dtype, MapDtype>;  // occ map type is int8_t
    using Env2dSetting = typename Env2d::Setting;
    using Cost = erl::env::EuclideanDistanceCost<Dtype, 2>;

private:
    std::string m_map_topic_ = "map";
    std::string m_map_topic_reliability_ = "reliable";
    std::string m_map_topic_durability_ = "volatile";

    int m_max_axis_step_ = 1;  // max step along one axis for motion primitives
    bool m_allow_diagonal_ = true;
    std::vector<double> m_robot_metric_contour_ = {};
    std::shared_ptr<Env2dSetting> m_env_setting_ = std::make_shared<Env2dSetting>();
    std::shared_ptr<Cost> m_cost_ = std::make_shared<Cost>();

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_sub_;
    nav_msgs::msg::OccupancyGrid m_map_;
    bool m_map_received_ = false;
    std::mutex m_map_mutex_;

public:
    Astar2dNode(const std::string& node_name = "astar_2d_node")
        : Super(node_name) {

        // Declare parameters
        this->declare_parameter("map_topic", m_map_topic_);
        this->declare_parameter("map_topic_reliability", this->m_default_qos_reliability_);
        this->declare_parameter("map_topic_durability", this->m_default_qos_durability_);
        this->declare_parameter("max_axis_step", m_max_axis_step_);
        this->declare_parameter("allow_diagonal", m_allow_diagonal_);
        this->declare_parameter("robot_metric_contour", m_robot_metric_contour_);
        this->declare_parameter("obstacle_threshold", m_env_setting_->obstacle_threshold);
        this->declare_parameter("add_map_cost", m_env_setting_->add_map_cost);
        this->declare_parameter("map_cost_factor", m_env_setting_->map_cost_factor);

        // Get parameters
#define GET_PARAM(param_name, member)                           \
    if (!this->get_parameter(param_name, member)) {             \
        RCLCPP_WARN(                                            \
            this->get_logger(),                                 \
            "Failed to get parameter %s, using default value.", \
            param_name);                                        \
    }                                                           \
    (void) 0

        GET_PARAM("map_topic", m_map_topic_);
        GET_PARAM("map_topic_reliability", m_map_topic_reliability_);
        GET_PARAM("map_topic_durability", m_map_topic_durability_);
        GET_PARAM("max_axis_step", m_max_axis_step_);
        GET_PARAM("allow_diagonal", m_allow_diagonal_);
        GET_PARAM("robot_metric_contour", m_robot_metric_contour_);
        GET_PARAM("obstacle_threshold", m_env_setting_->obstacle_threshold);
        GET_PARAM("add_map_cost", m_env_setting_->add_map_cost);
        GET_PARAM("map_cost_factor", m_env_setting_->map_cost_factor);
#undef GET_PARAM

        // Print parameters
        RCLCPP_INFO(
            this->get_logger(),
            "Loaded parameters:\n"
            "map_topic: %s\n"
            "map_topic_reliability: %s\n"
            "map_topic_durability: %s\n"
            "max_axis_step: %d\n"
            "allow_diagonal: %s\n"
            "robot_metric_contour size: %lu\n"
            "obstacle_threshold: %d\n"
            "add_map_cost: %s\n"
            "map_cost_factor: %f",
            m_map_topic_.c_str(),
            m_map_topic_reliability_.c_str(),
            m_map_topic_durability_.c_str(),
            m_max_axis_step_,
            m_allow_diagonal_ ? "true" : "false",
            m_robot_metric_contour_.size(),
            m_env_setting_->obstacle_threshold,
            m_env_setting_->add_map_cost ? "true" : "false",
            m_env_setting_->map_cost_factor);

        // Set environment setting
        if (m_robot_metric_contour_.size() > 0) {
            if (m_robot_metric_contour_.size() % 2 != 0) {
                RCLCPP_FATAL(
                    this->get_logger(),
                    "robot_metric_contour size must be even, got size %ld.",
                    m_robot_metric_contour_.size());
                rclcpp::shutdown();
                exit(EXIT_FAILURE);
            }
            if (m_robot_metric_contour_.size() < 6) {
                RCLCPP_FATAL(
                    this->get_logger(),
                    "robot_metric_contour must have at least 3 points, got %ld points.",
                    m_robot_metric_contour_.size() / 2);
                rclcpp::shutdown();
                exit(EXIT_FAILURE);
            }
            const auto n_points = static_cast<long>(m_robot_metric_contour_.size() >> 1);
            Eigen::Matrix2X<Dtype> contour(2, n_points);
            for (long i = 0; i < n_points; ++i) {
                contour(0, i) = m_robot_metric_contour_[2 * i];
                contour(1, i) = m_robot_metric_contour_[2 * i + 1];
            }
            m_env_setting_->robot_metric_contour = std::move(contour);
        }
        m_env_setting_->SetGridMotionPrimitive(m_max_axis_step_, m_allow_diagonal_);

        // Initialize subscribers
        m_map_sub_ = this->template create_subscription<nav_msgs::msg::OccupancyGrid>(
            m_map_topic_,
            Super::GetQoS(m_map_topic_reliability_, m_map_topic_durability_),
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m_map_mutex_);
                m_map_ = *msg;
                m_map_received_ = true;
            });
    }

    [[nodiscard]] std::shared_ptr<Env>
    GetEnv() override {
        nav_msgs::msg::OccupancyGrid map;
        {
            std::lock_guard<std::mutex> lock(m_map_mutex_);
            if (!m_map_received_) {
                RCLCPP_ERROR(this->get_logger(), "Map not received yet!");
                return nullptr;
            }
            map = m_map_;  // copy because m_map_ will be updated in the callback
        }

        // OccupancyGrid: int8, -1: unknown, 0: free, >=1: occupied

        // Make sure width and height are odd numbers
        if (map.info.width % 2 == 0) {
            for (uint32_t i = 0; i < map.info.height; ++i) {
                map.data.insert(map.data.begin() + (i + 1) * map.info.width + i, 100);
            }
            ++map.info.width;
        }
        if (map.info.height % 2 == 0) {
            map.data.insert(map.data.end(), map.info.width, 100);
            ++map.info.height;
        }

        // Calculate map parameters
        const Dtype res = static_cast<Dtype>(map.info.resolution);
        const Dtype min_x = static_cast<Dtype>(map.info.origin.position.x);
        const Dtype min_y = static_cast<Dtype>(map.info.origin.position.y);
        Eigen::Vector2<Dtype> map_min(min_x, min_y);
        Eigen::Vector2<Dtype> map_max(
            min_x + static_cast<Dtype>(map.info.width) * res,
            min_y + static_cast<Dtype>(map.info.height) * res);
        Eigen::Vector2i map_shape(
            static_cast<int>(map.info.width),
            static_cast<int>(map.info.height));
        auto grid_map_info = std::make_shared<GridMapInfo2D>(map_shape, map_min, map_max);

        // Load map data
        cv::Mat cost_map = cv::Mat(
                               static_cast<int>(map.info.height),
                               static_cast<int>(map.info.width),
                               erl::common::CvMatType<MapDtype>(),
                               reinterpret_cast<MapDtype*>(map.data.data()))
                               .t();
        cost_map.copyTo(cost_map);  // deep copy because map is temporary

        auto env = std::make_shared<Env2d>(grid_map_info, cost_map, m_env_setting_, m_cost_);
        return env;
    }

    [[nodiscard]] MetricState
    GetStartFromPoseMsg(const geometry_msgs::msg::TransformStamped& pose_msg) const override {
        MetricState start;
        start[0] = static_cast<Dtype>(pose_msg.transform.translation.x);
        start[1] = static_cast<Dtype>(pose_msg.transform.translation.y);
        return start;
    }

    [[nodiscard]] std::shared_ptr<Heuristic>
    GetHeuristic() override {
        return nullptr;  // let the planning interface create the default heuristic
    }

    void
    LoadPathToMsg(const PlanRecord& plan_record, nav_msgs::msg::Path& path_msg) const override {
        const long n_wp = plan_record.path.cols();
        path_msg.poses.clear();
        path_msg.poses.reserve(n_wp);
        for (long i = 0; i < n_wp; ++i) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            auto p = plan_record.path.col(i);
            pose_stamped.pose.position.x = static_cast<double>(p[0]);
            pose_stamped.pose.position.y = static_cast<double>(p[1]);
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;  // No orientation information
            path_msg.poses.push_back(pose_stamped);
        }
    }

    void
    Reset() override {
        Super::Reset();
        std::lock_guard<std::mutex> lock(m_map_mutex_);
        m_map_received_ = false;
    }
};

int
main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create a node to read parameters
    auto temp_node = rclcpp::Node::make_shared("astar_2d_node");

    bool double_precision = false;
    bool unknown_as_obstacle = false;
    temp_node->declare_parameter<bool>("double_precision", false);
    temp_node->declare_parameter<bool>("unknown_as_obstacle", false);
    double_precision = temp_node->get_parameter("double_precision").as_bool();
    unknown_as_obstacle = temp_node->get_parameter("unknown_as_obstacle").as_bool();
    RCLCPP_INFO(
        temp_node->get_logger(),
        "Using %s precision, unknown_as_obstacle = %s.",
        double_precision ? "double" : "single",
        unknown_as_obstacle ? "true" : "false");
    temp_node.reset();  // release the temporary node

    std::shared_ptr<rclcpp::Node> node;
    if (double_precision) {
        if (unknown_as_obstacle) {
            node = std::make_shared<Astar2dNode<double, uint8_t>>();
        } else {
            node = std::make_shared<Astar2dNode<double, int8_t>>();
        }
    } else {
        if (unknown_as_obstacle) {
            node = std::make_shared<Astar2dNode<float, uint8_t>>();
        } else {
            node = std::make_shared<Astar2dNode<float, int8_t>>();
        }
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
