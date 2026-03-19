#include "erl_env/environment_2d.hpp"
#include "erl_path_planning_ros/ros2/astar_node.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <memory>
#include <mutex>

template<typename Dtype, typename MapDtype>
struct Astar2dNodeConfig : public erl::common::Yamlable<Astar2dNodeConfig<Dtype, MapDtype>> {
    using Ros2TopicParams = erl::common::ros_params::Ros2TopicParams;
    using Env2d = erl::env::Environment2D<Dtype, MapDtype>;
    using Env2dSetting = typename Env2d::Setting;

    Ros2TopicParams map_topic{"map"};

    int max_axis_step = 1;
    bool allow_diagonal = true;
    std::vector<double> robot_metric_contour = {};
    std::shared_ptr<Env2dSetting> env = std::make_shared<Env2dSetting>();

    ERL_REFLECT_SCHEMA(
        Astar2dNodeConfig,
        ERL_REFLECT_MEMBER(Astar2dNodeConfig, map_topic),
        ERL_REFLECT_MEMBER(Astar2dNodeConfig, max_axis_step),
        ERL_REFLECT_MEMBER(Astar2dNodeConfig, allow_diagonal),
        ERL_REFLECT_MEMBER(Astar2dNodeConfig, robot_metric_contour),
        ERL_REFLECT_MEMBER(Astar2dNodeConfig, env));
};

template<typename Dtype, typename MapDtype>
class Astar2dNode : public AstarNode<Dtype, 2> {
public:
    using Super = AstarNode<Dtype, 2>;
    using MetricState = typename Super::MetricState;
    using Env = typename Super::Env;
    using Heuristic = typename Super::Heuristic;
    using PlanRecord = typename Super::PlanRecord;
    using GridMapInfo2D = erl::common::GridMapInfo2D<Dtype>;
    using Env2d = erl::env::Environment2D<Dtype, MapDtype>;
    using Env2dSetting = typename Env2d::Setting;
    using Cost = erl::env::EuclideanDistanceCost<Dtype, 2>;

private:
    Astar2dNodeConfig<Dtype, MapDtype> m_2d_config_;
    std::shared_ptr<Cost> m_cost_ = std::make_shared<Cost>();

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_sub_;
    nav_msgs::msg::OccupancyGrid m_map_;
    bool m_map_received_ = false;
    std::mutex m_map_mutex_;

public:
    Astar2dNode(const std::string &node_name = "astar_2d_node")
        : Super(node_name) {

        // Load parameters via Yamlable interface
        ERL_ASSERTM(m_2d_config_.LoadFromRos2(this, ""), "Failed to load 2D parameters");

        RCLCPP_INFO(
            this->get_logger(),
            "Loaded 2D parameters:\n%s",
            m_2d_config_.AsYamlString().c_str());

        // Set environment setting
        auto &robot_contour = m_2d_config_.robot_metric_contour;
        if (!robot_contour.empty()) {
            if (robot_contour.size() % 2 != 0) {
                RCLCPP_FATAL(
                    this->get_logger(),
                    "robot_metric_contour size must be even, got size %ld.",
                    robot_contour.size());
                rclcpp::shutdown();
                exit(EXIT_FAILURE);
            }
            if (robot_contour.size() < 6) {
                RCLCPP_FATAL(
                    this->get_logger(),
                    "robot_metric_contour must have at least 3 points, got %ld points.",
                    robot_contour.size() / 2);
                rclcpp::shutdown();
                exit(EXIT_FAILURE);
            }
            const auto n_points = static_cast<long>(robot_contour.size() >> 1);
            Eigen::Matrix2X<Dtype> contour(2, n_points);
            for (long i = 0; i < n_points; ++i) {
                contour(0, i) = robot_contour[2 * i];
                contour(1, i) = robot_contour[2 * i + 1];
            }
            m_2d_config_.env->robot_metric_contour = std::move(contour);
        }
        m_2d_config_.env->SetGridMotionPrimitive(
            m_2d_config_.max_axis_step,
            m_2d_config_.allow_diagonal);

        // Initialize subscribers
        m_map_sub_ = this->template create_subscription<nav_msgs::msg::OccupancyGrid>(
            m_2d_config_.map_topic.path,
            m_2d_config_.map_topic.GetQoS(),
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
                               reinterpret_cast<MapDtype *>(map.data.data()))
                               .t();
        cost_map.copyTo(cost_map);  // deep copy because map is temporary

        auto env = std::make_shared<Env2d>(grid_map_info, cost_map, m_2d_config_.env, m_cost_);
        return env;
    }

    [[nodiscard]] MetricState
    GetStartFromPoseMsg(const geometry_msgs::msg::TransformStamped &pose_msg) const override {
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
    LoadPathToMsg(const PlanRecord &plan_record, nav_msgs::msg::Path &path_msg) const override {
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
main(int argc, char **argv) {
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
