#include "erl_env/environment_ltl_2d.hpp"
#include "erl_env/finite_state_automaton.hpp"
#include "erl_geometry_msgs/msg/grid_map_msg.hpp"
#include "erl_geometry_msgs/ros2/grid_map_msg_encoding.hpp"
#include "erl_path_planning/ltl_2d_heuristic.hpp"
#include "erl_path_planning_ros/ros2/astar_node.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <mutex>

template<typename Dtype, typename MapDtype>
class Astar2dLtlNode : public AstarNode<Dtype, 3> {
public:
    using Super = AstarNode<Dtype, 3>;
    using MetricState = typename Super::MetricState;
    using Env = typename Super::Env;
    using Heuristic = typename Super::Heuristic;
    using PlanRecord = typename Super::PlanRecord;
    using GridMapInfo2D = erl::common::GridMapInfo2D<Dtype>;
    using Env2dLtl = erl::env::EnvironmentLTL2D<Dtype, MapDtype>;  // occ map type is int8_t
    using Env2dLtlSetting = typename Env2dLtl::Setting;
    using LtlHeuristic = erl::path_planning::LinearTemporalLogicHeuristic2D<Dtype>;
    using Cost = erl::env::EuclideanDistanceCost<Dtype, 2>;
    using FiniteStateAutomaton = erl::env::FiniteStateAutomaton;

private:
    std::string m_label_map_topic_ = "label_map";
    std::string m_label_map_topic_qos_reliability_ = "reliable";
    std::string m_label_map_topic_qos_durability_ = "volatile";
    std::string m_occ_map_topic_ = "occ_map";
    std::string m_occ_map_topic_qos_reliability_ = "reliable";
    std::string m_occ_map_topic_qos_durability_ = "volatile";
    std::string m_aut_topic_ = "aut_spot";  // spot HOA format
    std::string m_aut_topic_qos_reliability_ = "reliable";
    std::string m_aut_topic_qos_durability_ = "volatile";
    std::string m_ap_dict_topic_ = "ap_dict";
    std::string m_ap_dict_topic_qos_reliability_ = "reliable";
    std::string m_ap_dict_topic_qos_durability_ = "volatile";

    int m_max_axis_step_ = 1;  // max step along one axis for motion primitives
    bool m_allow_diagonal_ = true;
    std::vector<double> m_robot_metric_contour_ = {};
    std::shared_ptr<Env2dLtlSetting> m_env_setting_ = std::make_shared<Env2dLtlSetting>();
    std::shared_ptr<Cost> m_cost_ = std::make_shared<Cost>();

    rclcpp::Subscription<erl_geometry_msgs::msg::GridMapMsg>::SharedPtr m_label_map_sub_;
    erl_geometry_msgs::msg::GridMapMsg m_label_map_;  // TYPE_32SC1
    bool m_label_map_received_ = false;
    std::mutex m_label_map_mutex_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_occ_map_sub_;
    nav_msgs::msg::OccupancyGrid m_occ_map_;
    bool m_occ_map_received_ = false;
    std::mutex m_occ_map_mutex_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_aut_sub_;
    std_msgs::msg::String m_aut_msg_;
    std::shared_ptr<FiniteStateAutomaton::Setting> m_fsa_setting_ = nullptr;
    bool m_aut_received_ = false;
    std::mutex m_aut_mutex_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_ap_dict_pub_;

    // The following members are protected by m_env_mutex_
    std::shared_ptr<FiniteStateAutomaton> m_fsa_ = nullptr;
    Eigen::MatrixX<uint32_t> m_label_matrix_{};
    std::shared_ptr<GridMapInfo2D> m_grid_map_info_ = nullptr;
    std::mutex m_env_mutex_;

public:
    Astar2dLtlNode(const std::string& node_name = "astar_2d_ltl_node")
        : Super(node_name) {

        // Declare parameters
        this->declare_parameter("label_map_topic", m_label_map_topic_);
        this->declare_parameter(
            "label_map_topic_qos_reliability",
            this->m_default_qos_reliability_);
        this->declare_parameter("label_map_topic_qos_durability", this->m_default_qos_durability_);

        this->declare_parameter("occ_map_topic", m_occ_map_topic_);
        this->declare_parameter("occ_map_topic_qos_reliability", this->m_default_qos_reliability_);
        this->declare_parameter("occ_map_topic_qos_durability", this->m_default_qos_durability_);

        this->declare_parameter("aut_topic", m_aut_topic_);
        this->declare_parameter("aut_topic_qos_reliability", this->m_default_qos_reliability_);
        this->declare_parameter("aut_topic_qos_durability", this->m_default_qos_durability_);

        this->declare_parameter("ap_dict_topic", m_ap_dict_topic_);
        this->declare_parameter("ap_dict_topic_qos_reliability", this->m_default_qos_reliability_);
        this->declare_parameter("ap_dict_topic_qos_durability", this->m_default_qos_durability_);

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

        GET_PARAM("label_map_topic", m_label_map_topic_);
        GET_PARAM("label_map_topic_qos_reliability", m_label_map_topic_qos_reliability_);
        GET_PARAM("label_map_topic_qos_durability", m_label_map_topic_qos_durability_);

        GET_PARAM("occ_map_topic", m_occ_map_topic_);
        GET_PARAM("occ_map_topic_qos_reliability", m_occ_map_topic_qos_reliability_);
        GET_PARAM("occ_map_topic_qos_durability", m_occ_map_topic_qos_durability_);

        GET_PARAM("aut_topic", m_aut_topic_);
        GET_PARAM("aut_topic_qos_reliability", m_aut_topic_qos_reliability_);
        GET_PARAM("aut_topic_qos_durability", m_aut_topic_qos_durability_);

        GET_PARAM("ap_dict_topic", m_ap_dict_topic_);
        GET_PARAM("ap_dict_topic_qos_reliability", m_ap_dict_topic_qos_reliability_);
        GET_PARAM("ap_dict_topic_qos_durability", m_ap_dict_topic_qos_durability_);

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
            "label_map_topic: %s\n"
            "label_map_topic_qos_reliability: %s\n"
            "label_map_topic_qos_durability: %s\n"
            "occ_map_topic: %s\n"
            "occ_map_topic_qos_reliability: %s\n"
            "occ_map_topic_qos_durability: %s\n"
            "aut_topic: %s\n"
            "aut_topic_qos_reliability: %s\n"
            "aut_topic_qos_durability: %s\n"
            "ap_dict_topic: %s\n"
            "ap_dict_topic_qos_reliability: %s\n"
            "ap_dict_topic_qos_durability: %s\n"
            "max_axis_step: %d\n"
            "allow_diagonal: %s\n"
            "robot_metric_contour size: %lu\n"
            "obstacle_threshold: %d\n"
            "add_map_cost: %s\n"
            "map_cost_factor: %f",
            m_label_map_topic_.c_str(),
            m_label_map_topic_qos_reliability_.c_str(),
            m_label_map_topic_qos_durability_.c_str(),
            m_occ_map_topic_.c_str(),
            m_occ_map_topic_qos_reliability_.c_str(),
            m_occ_map_topic_qos_durability_.c_str(),
            m_aut_topic_.c_str(),
            m_aut_topic_qos_reliability_.c_str(),
            m_aut_topic_qos_durability_.c_str(),
            m_ap_dict_topic_.c_str(),
            m_ap_dict_topic_qos_reliability_.c_str(),
            m_ap_dict_topic_qos_durability_.c_str(),
            m_max_axis_step_,
            (m_allow_diagonal_ ? "true" : "false"),
            m_robot_metric_contour_.size(),
            m_env_setting_->obstacle_threshold,
            (m_env_setting_->add_map_cost ? "true" : "false"),
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

        // Initialize publishers
        m_ap_dict_pub_ = this->template create_publisher<std_msgs::msg::String>(
            m_ap_dict_topic_,
            Super::GetQoS(m_ap_dict_topic_qos_reliability_, m_ap_dict_topic_qos_durability_));

        // Initialize subscribers
        m_label_map_sub_ = this->template create_subscription<erl_geometry_msgs::msg::GridMapMsg>(
            m_label_map_topic_,
            Super::GetQoS(m_label_map_topic_qos_reliability_, m_label_map_topic_qos_durability_),
            [this](const erl_geometry_msgs::msg::GridMapMsg::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m_label_map_mutex_);
                m_label_map_ = *msg;
                m_label_map_received_ = true;
            });

        m_occ_map_sub_ = this->template create_subscription<nav_msgs::msg::OccupancyGrid>(
            m_occ_map_topic_,
            Super::GetQoS(m_occ_map_topic_qos_reliability_, m_occ_map_topic_qos_durability_),
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m_occ_map_mutex_);
                m_occ_map_ = *msg;
                m_occ_map_received_ = true;
            });

        m_aut_sub_ = this->template create_subscription<std_msgs::msg::String>(
            m_aut_topic_,
            Super::GetQoS(m_aut_topic_qos_reliability_, m_aut_topic_qos_durability_),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m_aut_mutex_);
                m_aut_msg_ = *msg;
                m_aut_received_ = true;

                // try to load FSA
                try {
                    // recreate m_fsa_setting_ to disconnect from the old one, which may still
                    // be used by the old env, fsa and heuristic on other threads.
                    m_fsa_setting_ = std::make_shared<FiniteStateAutomaton::Setting>();
                    m_fsa_setting_->FromSpotGraphHoaString(m_aut_msg_.data, false /* complete */);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Failed to load FSA from topic %s: %s",
                        m_aut_topic_.c_str(),
                        e.what());
                    return;
                }

                // publish AP dictionary
                YAML::Node ap_dict_node = YAML::convert<std::vector<std::string>>::encode(
                    m_fsa_setting_->atomic_propositions);
                YAML::Emitter emitter;
                emitter.SetIndent(4);
                emitter.SetSeqFormat(YAML::Flow);
                emitter << ap_dict_node;
                std_msgs::msg::String ap_dict_msg;
                ap_dict_msg.data = emitter.c_str();
                m_ap_dict_pub_->publish(ap_dict_msg);
            });
    }

    [[nodiscard]] std::shared_ptr<Env>
    GetEnv() override {
        // fetch data

        // try to load FSA
        {
            std::lock_guard<std::mutex> lock(m_aut_mutex_);
            if (!m_aut_received_) {
                RCLCPP_ERROR(this->get_logger(), "Automaton not received yet!");
                return nullptr;
            }
            if (m_fsa_setting_ == nullptr) {
                RCLCPP_ERROR(this->get_logger(), "FSA setting is null!");
                return nullptr;
            }
            m_env_setting_->fsa = m_fsa_setting_;
        }

        // GridMap: TYPE_32SC1, each pixel is the label of the cell.
        // We will cast it to uint32_t directly for EnvironmentLTL2D.
        erl_geometry_msgs::msg::GridMapMsg label_map_msg;
        {
            std::lock_guard<std::mutex> lock(m_label_map_mutex_);
            if (!m_label_map_received_) {
                RCLCPP_ERROR(this->get_logger(), "Label map not received yet!");
                return nullptr;
            }
            label_map_msg = m_label_map_;
        }

        // make sure label_map is of type UINT32
        if (label_map_msg.encoding !=
            static_cast<uint8_t>(erl::geometry::GridMapEncoding::UINT32)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Label map must be of type %s, got %d.",
                "erl::geometry::GridMapEncoding::UINT32",
                label_map_msg.encoding);
            return nullptr;
        }

        // OccupancyGrid: int8, -1: unknown, 0: free, >=1: occupied
        nav_msgs::msg::OccupancyGrid occ_map;
        {
            std::lock_guard<std::mutex> lock(m_occ_map_mutex_);
            if (!m_occ_map_received_) {
                RCLCPP_ERROR(this->get_logger(), "Map not received yet!");
                return nullptr;
            }
            occ_map = m_occ_map_;  // copy because m_occ_map_ will be updated in the callback
        }

        // make sure label_map and occ_map have the same size
        if (label_map_msg.info.height != occ_map.info.height ||
            label_map_msg.info.width != occ_map.info.width) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Label map size (%d, %d) does not match occupancy map size (%u, %u)!",
                label_map_msg.info.height,
                label_map_msg.info.width,
                occ_map.info.height,
                occ_map.info.width);
            return nullptr;
        }

        if (label_map_msg.info.resolution != occ_map.info.resolution) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Label map resolution (%f) does not match occupancy map resolution (%f)!",
                label_map_msg.info.resolution,
                occ_map.info.resolution);
            return nullptr;
        }

        if (label_map_msg.info.origin.position.x != occ_map.info.origin.position.x ||
            label_map_msg.info.origin.position.y != occ_map.info.origin.position.y ||
            label_map_msg.info.origin.position.z != occ_map.info.origin.position.z ||
            label_map_msg.info.origin.orientation.x != occ_map.info.origin.orientation.x ||
            label_map_msg.info.origin.orientation.y != occ_map.info.origin.orientation.y ||
            label_map_msg.info.origin.orientation.z != occ_map.info.origin.orientation.z ||
            label_map_msg.info.origin.orientation.w != occ_map.info.origin.orientation.w) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Label map origin does not match occupancy map origin!");
            return nullptr;
        }

        // prepare label_map and cost_map
        // convert label_map to Eigen::MatrixX<uint32_t>.
        // The input label map: x to right, y down, row-major.
        // The required label matrix: x down, y to right, col-major.
        // The actual data layout is the same, so we just need to transpose it.
        m_label_matrix_.resize(label_map_msg.info.width /* x */, label_map_msg.info.height /* y */);
        std::memcpy(m_label_matrix_.data(), label_map_msg.data.data(), label_map_msg.data.size());

        // Make sure width and height are odd numbers
        if (occ_map.info.width % 2 == 0) {
            for (uint32_t i = 0; i < occ_map.info.height; ++i) {
                occ_map.data.insert(occ_map.data.begin() + (i + 1) * occ_map.info.width + i, 100);
            }
            ++occ_map.info.width;
            m_label_matrix_.conservativeResize(occ_map.info.width, occ_map.info.height);
            m_label_matrix_.bottomRows<1>().setConstant(0);  // all false.
        }
        if (occ_map.info.height % 2 == 0) {
            occ_map.data.insert(occ_map.data.end(), occ_map.info.width, 100);
            ++occ_map.info.height;
            m_label_matrix_.conservativeResize(occ_map.info.width, occ_map.info.height);
            m_label_matrix_.rightCols<1>().setConstant(0);  // all false.
        }

        // Calculate map parameters
        const Dtype res = static_cast<Dtype>(occ_map.info.resolution);
        const Dtype min_x = static_cast<Dtype>(occ_map.info.origin.position.x);
        const Dtype min_y = static_cast<Dtype>(occ_map.info.origin.position.y);
        Eigen::Vector2<Dtype> map_min(min_x, min_y);
        Eigen::Vector2<Dtype> map_max(
            min_x + static_cast<Dtype>(occ_map.info.width) * res,
            min_y + static_cast<Dtype>(occ_map.info.height) * res);
        Eigen::Vector2i map_shape(
            static_cast<int>(occ_map.info.width),
            static_cast<int>(occ_map.info.height));
        {
            std::lock_guard<std::mutex> lock(m_env_mutex_);
            m_grid_map_info_ = std::make_shared<GridMapInfo2D>(map_shape, map_min, map_max);
        }

        // Load map data
        // shape: (width, height), type: CV_8SC1
        cv::Mat cost_map = cv::Mat(
                               static_cast<int>(occ_map.info.height),
                               static_cast<int>(occ_map.info.width),
                               erl::common::CvMatType<MapDtype>(),
                               reinterpret_cast<MapDtype*>(occ_map.data.data()))
                               .t();
        cost_map.copyTo(cost_map);  // deep copy because map is temporary

        auto env = std::make_shared<Env2dLtl>(
            m_label_matrix_,
            m_grid_map_info_,
            cost_map,
            m_env_setting_,
            m_cost_);

        {
            std::lock_guard<std::mutex> lock(m_env_mutex_);
            m_fsa_ = env->GetFiniteStateAutomaton();
        }

        return env;
    }

    [[nodiscard]] MetricState
    GetStartFromPoseMsg(const geometry_msgs::msg::TransformStamped& pose_msg) const override {
        MetricState start;
        start[0] = static_cast<Dtype>(pose_msg.transform.translation.x);
        start[1] = static_cast<Dtype>(pose_msg.transform.translation.y);
        start[2] = m_fsa_->GetSetting()->initial_state;  // initial FSA state
        return start;
    }

    [[nodiscard]] std::shared_ptr<Heuristic>
    GetHeuristic() override {
        std::shared_ptr<FiniteStateAutomaton> fsa;
        Eigen::MatrixX<uint32_t> label_matrix;
        std::shared_ptr<GridMapInfo2D> grid_map_info;
        {
            // in case m_fsa_ or m_grid_map_info_ is being updated.
            // or reset is being called.
            std::lock_guard<std::mutex> lock(m_env_mutex_);
            if (m_fsa_ == nullptr || m_grid_map_info_ == nullptr) {
                RCLCPP_ERROR(this->get_logger(), "FSA or map info is not ready yet!");
                return nullptr;
            }
            fsa = m_fsa_;
            label_matrix = m_label_matrix_;
            grid_map_info = m_grid_map_info_;
        }
        auto heuristic = std::make_shared<LtlHeuristic>(fsa, label_matrix, grid_map_info);
        return heuristic;
    }

    void
    BeforeAstar(
        std::shared_ptr<Env>& /* env */,
        MetricState& start,
        std::vector<MetricState>& goals,
        std::vector<MetricState>& goals_tolerances,
        std::vector<Dtype>& terminal_costs) override {
        auto fsa_setting = m_fsa_->GetSetting();

        start[2] = fsa_setting->initial_state;  // initial FSA state

        RCLCPP_INFO(this->get_logger(), "fsa_setting: %s", fsa_setting->AsYamlString().c_str());

        const std::size_t n_acc_states = fsa_setting->accepting_states.size();
        if (n_acc_states == 1) {
            const Dtype acc_state = static_cast<Dtype>(fsa_setting->accepting_states[0]);
            RCLCPP_INFO(
                this->get_logger(),
                "Adding goal state with accepting FSA state %f.",
                acc_state);
            for (auto& goal: goals) { goal[2] = acc_state; }
        } else {
            std::vector<MetricState> new_goals;
            new_goals.reserve(goals.size() * n_acc_states);
            for (const auto& goal: goals) {
                for (const auto& acc_state: fsa_setting->accepting_states) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Adding goal state with accepting FSA state %u.",
                        acc_state);
                    MetricState new_goal = goal;
                    new_goal[2] = static_cast<Dtype>(acc_state);
                    new_goals.emplace_back(std::move(new_goal));
                }
            }
            goals = std::move(new_goals);

            if (goals_tolerances.size() > 1) {
                std::vector<MetricState> new_goals_tolerances;
                new_goals_tolerances.reserve(goals_tolerances.size() * n_acc_states);
                for (const auto& goal_tol: goals_tolerances) {
                    new_goals_tolerances.insert(new_goals_tolerances.end(), n_acc_states, goal_tol);
                }
                goals_tolerances = std::move(new_goals_tolerances);
            }

            if (terminal_costs.size() > 1) {
                std::vector<Dtype> new_terminal_costs;
                new_terminal_costs.reserve(terminal_costs.size() * n_acc_states);
                for (const auto& term_cost: terminal_costs) {
                    new_terminal_costs.insert(new_terminal_costs.end(), n_acc_states, term_cost);
                }
                terminal_costs = std::move(new_terminal_costs);
            }
        }
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
        {
            std::lock_guard<std::mutex> lock(m_occ_map_mutex_);
            m_occ_map_received_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(m_label_map_mutex_);
            m_label_map_received_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(m_aut_mutex_);
            m_aut_received_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(m_env_mutex_);
            m_fsa_ = nullptr;
            m_grid_map_info_ = nullptr;
        }
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
            // -1 is treated as large unsigned number, thus obstacle.
            node = std::make_shared<Astar2dLtlNode<double, uint8_t>>();
        } else {
            node = std::make_shared<Astar2dLtlNode<double, int8_t>>();
        }
    } else {
        if (unknown_as_obstacle) {
            node = std::make_shared<Astar2dLtlNode<float, uint8_t>>();
        } else {
            node = std::make_shared<Astar2dLtlNode<float, int8_t>>();
        }
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
