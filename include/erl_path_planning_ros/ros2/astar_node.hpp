#include "erl_path_planning/astar.hpp"
#include "erl_path_planning/heuristic.hpp"
#include "erl_path_planning/search_planning_interface.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/goals.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <memory>
#include <mutex>

template<typename Dtype, int Dim>
class AstarNode : public rclcpp::Node {
public:
    using AStar = erl::path_planning::astar::AStar<Dtype, Dim>;
    using AstarSetting = erl::path_planning::astar::AstarSetting<Dtype>;
    using AStarOutput = erl::path_planning::astar::Output<Dtype, Dim>;
    using PlanRecord = erl::path_planning::PlanRecord<Dtype, Dim>;
    using SearchPlanningInterface = erl::path_planning::SearchPlanningInterface<Dtype, Dim>;
    using Env = typename SearchPlanningInterface::Env;
    using MetricState = typename SearchPlanningInterface::MetricState;
    using Heuristic = typename SearchPlanningInterface::Heuristic;

protected:
    std::string m_default_qos_reliability_ = "reliable";  // or best_effort
    std::string m_default_qos_durability_ = "volatile";   // or transient_local

    std::string m_global_frame_ = "map";
    std::string m_robot_frame_ = "base_link";

    std::string m_start_source_ = "topic";  // "topic" or "tf"
    std::string m_start_topic_ = "start";
    std::string m_start_topic_qos_reliability_ = m_default_qos_reliability_;
    std::string m_start_topic_qos_durability_ = m_default_qos_durability_;

    std::string m_goal_topic_ = "goals";
    std::string m_goal_topic_qos_reliability_ = m_default_qos_reliability_;
    std::string m_goal_topic_qos_durability_ = m_default_qos_durability_;

    std::string m_goal_tolerance_topic_ = "goal_tolerances";
    std::string m_goal_tolerance_topic_qos_reliability_ = m_default_qos_reliability_;
    std::string m_goal_tolerance_topic_qos_durability_ = m_default_qos_durability_;

    std::string m_terminal_cost_topic_ = "terminal_costs";
    std::string m_terminal_cost_topic_qos_reliability_ = m_default_qos_reliability_;
    std::string m_terminal_cost_topic_qos_durability_ = m_default_qos_durability_;

    std::string m_path_topic_ = "path";
    std::string m_path_topic_qos_reliability_ = m_default_qos_reliability_;
    std::string m_path_topic_qos_durability_ = m_default_qos_durability_;

    std::string m_cost_topic_ = "cost";
    std::string m_cost_topic_qos_reliability_ = m_default_qos_reliability_;
    std::string m_cost_topic_qos_durability_ = m_default_qos_durability_;

    std::string m_goal_idx_topic_ = "goal_index";
    std::string m_goal_idx_topic_qos_reliability_ = m_default_qos_reliability_;
    std::string m_goal_idx_topic_qos_durability_ = m_default_qos_durability_;

    std::string m_plan_srv_name_ = "plan_path";
    std::string m_reset_srv_name_ = "reset_planner";

    std::shared_ptr<AstarSetting> m_astar_setting_ = std::make_shared<AstarSetting>();

    std::shared_ptr<rclcpp::ParameterEventHandler> m_param_event_handler_;
    rclcpp::ParameterEventCallbackHandle::SharedPtr m_param_event_cb_handle_;

    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_start_sub_;
    std_msgs::msg::Float64MultiArray m_start_;  // (Dim, ) vector
    bool m_start_received_ = false;
    std::mutex m_start_mutex_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_goal_sub_;
    std_msgs::msg::Float64MultiArray m_goals_;  // (Dim, ) vector or (Dim, N) col-major matrix
    bool m_goals_received_ = false;
    std::mutex m_goals_mutex_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_goal_tolerance_sub_;
    std_msgs::msg::Float64MultiArray m_goal_tolerances_;  // (Dim, ) vector or (Dim, N) col-major
    bool m_goal_tolerances_received_ = false;
    std::mutex m_goal_tolerances_mutex_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_terminal_cost_sub_;
    std_msgs::msg::Float64MultiArray m_terminal_costs_;  // (1, ) vector or (N, ) vector
    bool m_terminal_costs_received_ = false;
    std::mutex m_terminal_costs_mutex_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_cost_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr m_goal_idx_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_plan_srv_;   // start planning service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_reset_srv_;  // reset service

public:
    explicit AstarNode(const std::string& node_name)
        : rclcpp::Node(node_name),
          m_param_event_handler_(std::make_shared<rclcpp::ParameterEventHandler>(this)),
          m_tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          m_tf_listener_(std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_)) {

        this->declare_parameter("default_qos_reliability", m_default_qos_reliability_);
        this->declare_parameter("default_qos_durability", m_default_qos_durability_);
        this->get_parameter("default_qos_reliability", m_default_qos_reliability_);
        this->get_parameter("default_qos_durability", m_default_qos_durability_);

        // Declare parameters
        this->declare_parameter("global_frame", m_global_frame_);
        this->declare_parameter("robot_frame", m_robot_frame_);
        this->declare_parameter("start_source", m_start_source_);
        this->declare_parameter("start_topic", m_start_topic_);
        this->declare_parameter("start_topic_qos_reliability", m_default_qos_reliability_);
        this->declare_parameter("start_topic_qos_durability", m_default_qos_durability_);
        this->declare_parameter("goal_topic", m_goal_topic_);
        this->declare_parameter("goal_topic_qos_reliability", m_default_qos_reliability_);
        this->declare_parameter("goal_topic_qos_durability", m_default_qos_durability_);
        this->declare_parameter("goal_tolerance_topic", m_goal_tolerance_topic_);
        this->declare_parameter("goal_tolerance_topic_qos_reliability", m_default_qos_reliability_);
        this->declare_parameter("goal_tolerance_topic_qos_durability", m_default_qos_durability_);
        this->declare_parameter("terminal_cost_topic", m_terminal_cost_topic_);
        this->declare_parameter("terminal_cost_topic_qos_reliability", m_default_qos_reliability_);
        this->declare_parameter("terminal_cost_topic_qos_durability", m_default_qos_durability_);
        this->declare_parameter("path_topic", m_path_topic_);
        this->declare_parameter("path_topic_qos_reliability", m_default_qos_reliability_);
        this->declare_parameter("path_topic_qos_durability", m_default_qos_durability_);
        this->declare_parameter("cost_topic", m_cost_topic_);
        this->declare_parameter("cost_topic_qos_reliability", m_default_qos_reliability_);
        this->declare_parameter("cost_topic_qos_durability", m_default_qos_durability_);
        this->declare_parameter("goal_idx_topic", m_goal_idx_topic_);
        this->declare_parameter("goal_idx_topic_qos_reliability", m_default_qos_reliability_);
        this->declare_parameter("goal_idx_topic_qos_durability", m_default_qos_durability_);
        this->declare_parameter("plan_srv_name", m_plan_srv_name_);
        this->declare_parameter("reset_srv_name", m_reset_srv_name_);

        this->declare_parameter("eps", m_astar_setting_->eps);
        this->declare_parameter("max_num_iterations", m_astar_setting_->max_num_iterations);
        this->declare_parameter("log", m_astar_setting_->log);
        this->declare_parameter("reopen_inconsistent", m_astar_setting_->reopen_inconsistent);

        // Get parameters
#define GET_PARAM(param_name, member)                           \
    if (!this->get_parameter(param_name, member)) {             \
        RCLCPP_WARN(                                            \
            this->get_logger(),                                 \
            "Failed to get parameter %s, using default value.", \
            param_name);                                        \
    }                                                           \
    (void) 0

        GET_PARAM("global_frame", m_global_frame_);
        GET_PARAM("robot_frame", m_robot_frame_);
        GET_PARAM("start_source", m_start_source_);
        GET_PARAM("start_topic", m_start_topic_);
        GET_PARAM("start_topic_qos_reliability", m_start_topic_qos_reliability_);
        GET_PARAM("start_topic_qos_durability", m_start_topic_qos_durability_);
        GET_PARAM("goal_topic", m_goal_topic_);
        GET_PARAM("goal_topic_qos_reliability", m_goal_topic_qos_reliability_);
        GET_PARAM("goal_topic_qos_durability", m_goal_topic_qos_durability_);
        GET_PARAM("goal_tolerance_topic", m_goal_tolerance_topic_);
        GET_PARAM("goal_tolerance_topic_qos_reliability", m_goal_tolerance_topic_qos_reliability_);
        GET_PARAM("goal_tolerance_topic_qos_durability", m_goal_tolerance_topic_qos_durability_);
        GET_PARAM("terminal_cost_topic", m_terminal_cost_topic_);
        GET_PARAM("terminal_cost_topic_qos_reliability", m_terminal_cost_topic_qos_reliability_);
        GET_PARAM("terminal_cost_topic_qos_durability", m_terminal_cost_topic_qos_durability_);
        GET_PARAM("path_topic", m_path_topic_);
        GET_PARAM("path_topic_qos_reliability", m_path_topic_qos_reliability_);
        GET_PARAM("path_topic_qos_durability", m_path_topic_qos_durability_);
        GET_PARAM("cost_topic", m_cost_topic_);
        GET_PARAM("cost_topic_qos_reliability", m_cost_topic_qos_reliability_);
        GET_PARAM("cost_topic_qos_durability", m_cost_topic_qos_durability_);
        GET_PARAM("goal_idx_topic", m_goal_idx_topic_);
        GET_PARAM("goal_idx_topic_qos_reliability", m_goal_idx_topic_qos_reliability_);
        GET_PARAM("goal_idx_topic_qos_durability", m_goal_idx_topic_qos_durability_);
        GET_PARAM("plan_srv_name", m_plan_srv_name_);
        GET_PARAM("reset_srv_name", m_reset_srv_name_);
        GET_PARAM("eps", m_astar_setting_->eps);
        GET_PARAM("max_num_iterations", m_astar_setting_->max_num_iterations);
        GET_PARAM("log", m_astar_setting_->log);
        GET_PARAM("reopen_inconsistent", m_astar_setting_->reopen_inconsistent);
#undef GET_PARAM

        // Print parameters
        RCLCPP_INFO(
            this->get_logger(),
            "Loaded parameters:\n"
            "default_qos_reliability: %s\n"
            "default_qos_durability: %s\n"
            "global_frame: %s\n"
            "robot_frame: %s\n"
            "start_source: %s\n"
            "start_topic: %s\n"
            "start_topic_qos_reliability: %s\n"
            "start_topic_qos_durability: %s\n"
            "goal_topic: %s\n"
            "goal_topic_qos_reliability: %s\n"
            "goal_topic_qos_durability: %s\n"
            "goal_tolerance_topic: %s\n"
            "goal_tolerance_topic_qos_reliability: %s\n"
            "goal_tolerance_topic_qos_durability: %s\n"
            "terminal_cost_topic: %s\n"
            "terminal_cost_topic_qos_reliability: %s\n"
            "terminal_cost_topic_qos_durability: %s\n"
            "path_topic: %s\n"
            "path_topic_qos_reliability: %s\n"
            "path_topic_qos_durability: %s\n"
            "cost_topic: %s\n"
            "cost_topic_qos_reliability: %s\n"
            "cost_topic_qos_durability: %s\n"
            "goal_idx_topic: %s\n"
            "goal_idx_topic_qos_reliability: %s\n"
            "goal_idx_topic_qos_durability: %s\n"
            "plan_srv_name: %s\n"
            "reset_srv_name: %s\n"
            "eps: %f\n"
            "max_num_iterations: %ld\n"
            "log: %s\n"
            "reopen_inconsistent: %s",
            m_default_qos_reliability_.c_str(),
            m_default_qos_durability_.c_str(),
            m_global_frame_.c_str(),
            m_robot_frame_.c_str(),
            m_start_source_.c_str(),
            m_start_topic_.c_str(),
            m_start_topic_qos_reliability_.c_str(),
            m_start_topic_qos_durability_.c_str(),
            m_goal_topic_.c_str(),
            m_goal_topic_qos_reliability_.c_str(),
            m_goal_topic_qos_durability_.c_str(),
            m_goal_tolerance_topic_.c_str(),
            m_goal_tolerance_topic_qos_reliability_.c_str(),
            m_goal_tolerance_topic_qos_durability_.c_str(),
            m_terminal_cost_topic_.c_str(),
            m_terminal_cost_topic_qos_reliability_.c_str(),
            m_terminal_cost_topic_qos_durability_.c_str(),
            m_path_topic_.c_str(),
            m_path_topic_qos_reliability_.c_str(),
            m_path_topic_qos_durability_.c_str(),
            m_cost_topic_.c_str(),
            m_cost_topic_qos_reliability_.c_str(),
            m_cost_topic_qos_durability_.c_str(),
            m_goal_idx_topic_.c_str(),
            m_goal_idx_topic_qos_reliability_.c_str(),
            m_goal_idx_topic_qos_durability_.c_str(),
            m_plan_srv_name_.c_str(),
            m_reset_srv_name_.c_str(),
            m_astar_setting_->eps,
            m_astar_setting_->max_num_iterations,
            m_astar_setting_->log ? "true" : "false",
            m_astar_setting_->reopen_inconsistent ? "true" : "false");

        // Set parameter change callback
#define SET_PARAM_IF(param, param_name, param_type, member)  \
    if (param.get_name() == param_name) {                    \
        if (param.get_type() == param_type) {                \
            member = param.get_value<decltype(member)>();    \
            RCLCPP_INFO(                                     \
                this->get_logger(),                          \
                "Parameter %s is set to %s",                 \
                param_name,                                  \
                param.value_to_string().c_str());            \
        } else {                                             \
            RCLCPP_WARN(                                     \
                this->get_logger(),                          \
                "Parameter %s has wrong type, expected %s.", \
                param_name,                                  \
                #param_type);                                \
        }                                                    \
        continue;                                            \
    }                                                        \
    (void) 0

        m_param_event_cb_handle_ = m_param_event_handler_->add_parameter_event_callback(
            [this](const rcl_interfaces::msg::ParameterEvent& event) {
                auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
                for (auto& p: params) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Received an update to parameter \"%s\" of type %s: \"%s\"",
                        p.get_name().c_str(),
                        p.get_type_name().c_str(),
                        p.value_to_string().c_str());
                    SET_PARAM_IF(
                        p,
                        "global_frame",
                        rclcpp::ParameterType::PARAMETER_STRING,
                        this->m_global_frame_);
                    SET_PARAM_IF(
                        p,
                        "robot_frame",
                        rclcpp::ParameterType::PARAMETER_STRING,
                        this->m_robot_frame_);
                    SET_PARAM_IF(
                        p,
                        "start_source",
                        rclcpp::ParameterType::PARAMETER_STRING,
                        this->m_start_source_);
                    // topic parameters are not allowed to be changed --- IGNORE ---
                    SET_PARAM_IF(
                        p,
                        "eps",
                        rclcpp::ParameterType::PARAMETER_DOUBLE,
                        this->m_astar_setting_->eps);
                    SET_PARAM_IF(
                        p,
                        "max_num_iterations",
                        rclcpp::ParameterType::PARAMETER_INTEGER,
                        this->m_astar_setting_->max_num_iterations);
                    SET_PARAM_IF(
                        p,
                        "log",
                        rclcpp::ParameterType::PARAMETER_BOOL,
                        this->m_astar_setting_->log);
                    SET_PARAM_IF(
                        p,
                        "reopen_inconsistent",
                        rclcpp::ParameterType::PARAMETER_BOOL,
                        this->m_astar_setting_->reopen_inconsistent);
                }
            });

#undef SET_PARAM_IF

        // Initialize subscribers
        m_start_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            m_start_topic_,
            GetQoS(m_start_topic_qos_reliability_, m_start_topic_qos_durability_),
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m_start_mutex_);
                m_start_ = *msg;
                m_start_received_ = true;
            });

        m_goal_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            m_goal_topic_,
            GetQoS(m_goal_topic_qos_reliability_, m_goal_topic_qos_durability_),
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m_goals_mutex_);
                m_goals_ = *msg;
                m_goals_received_ = true;
            });

        m_goal_tolerance_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            m_goal_tolerance_topic_,
            GetQoS(m_goal_tolerance_topic_qos_reliability_, m_goal_tolerance_topic_qos_durability_),
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m_goal_tolerances_mutex_);
                m_goal_tolerances_ = *msg;
                m_goal_tolerances_received_ = true;
            });

        m_terminal_cost_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            m_terminal_cost_topic_,
            GetQoS(m_terminal_cost_topic_qos_reliability_, m_terminal_cost_topic_qos_durability_),
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m_terminal_costs_mutex_);
                m_terminal_costs_ = *msg;
                m_terminal_costs_received_ = true;
            });

        // Initialize publisher
        m_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            m_path_topic_,
            GetQoS(m_path_topic_qos_reliability_, m_path_topic_qos_durability_));
        m_cost_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            m_cost_topic_,
            GetQoS(m_cost_topic_qos_reliability_, m_cost_topic_qos_durability_));
        m_goal_idx_pub_ = this->create_publisher<std_msgs::msg::Int64>(
            m_goal_idx_topic_,
            GetQoS(m_goal_idx_topic_qos_reliability_, m_goal_idx_topic_qos_durability_));

        // Initialize services
        m_plan_srv_ = this->create_service<std_srvs::srv::Trigger>(
            m_plan_srv_name_,
            std::bind(
                &AstarNode::CallbackSrvPlan,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
        m_reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            m_reset_srv_name_,
            std::bind(
                &AstarNode::CallbackSrvReset,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
    }

    rclcpp::QoS
    GetQoS(const std::string& reliability, const std::string& durability) {
        rclcpp::QoS qos(1);
        if (reliability == "reliable") {
            qos.reliable();
        } else if (reliability == "best_effort") {
            qos.best_effort();
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown reliability %s, using reliable.",
                reliability.c_str());
            qos.reliable();
        }
        if (durability == "volatile") {
            qos.durability_volatile();
        } else if (durability == "transient_local") {
            qos.transient_local();
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown durability %s, using transient_local.",
                durability.c_str());
            qos.transient_local();
        }
        return qos;
    }

    virtual ~AstarNode() = default;

    /**
     * Create the state space for the planner. The derived class must implement this method. This
     * method will be called first when creating the planner. The derived class can prepare the
     * environment and other necessary components in this method. Other virtual methods will be
     * called after this method so that some prepared components can be used in other methods.
     * @return shared pointer to the environment.
     */
    [[nodiscard]] virtual std::shared_ptr<Env>
    GetEnv() = 0;

    /**
     * Convert a ROS pose message to a start state in the state space. The abstract class has no
     * idea the state space definition, so the derived class must implement this method.
     * @param pose_msg The ROS pose message.
     * @return The start state in the state space.
     */
    [[nodiscard]] virtual MetricState
    GetStartFromPoseMsg(const geometry_msgs::msg::TransformStamped& pose_msg) const = 0;

    /**
     * Create the heuristic for the planner. The derived class must implement this method.
     * @return shared pointer to the heuristic.
     */
    [[nodiscard]] virtual std::shared_ptr<Heuristic>
    GetHeuristic() = 0;

    /**
     * Convert the plan record to a ROS path message. The derived class must implement this method.
     * @param plan_record The plan record from the planner.
     * @param path_msg The ROS path message to be filled.
     */
    virtual void
    LoadPathToMsg(const PlanRecord& plan_record, nav_msgs::msg::Path& path_msg) const = 0;

    bool
    GetStartPoseFromTf(const rclcpp::Time& time, geometry_msgs::msg::TransformStamped& start_pose)
        const {
        // get the latest transform from the tf buffer
        try {
            start_pose = m_tf_buffer_->lookupTransform(
                m_global_frame_,
                m_robot_frame_,
                time,
                rclcpp::Duration::from_seconds(5.0));
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return false;
        }
        return true;
    }

    /**
     * Get the start state from the topic message.
     * @param start The start state to be filled.
     * @return True if the start state is successfully obtained, false otherwise.
     */
    bool
    GetStartFromTopicMsg(MetricState& start) {
        std::lock_guard<std::mutex> lock(m_start_mutex_);
        if (!m_start_received_) {
            RCLCPP_WARN(this->get_logger(), "Start is not received yet.");
            return false;
        }
        if (m_start_.layout.dim.size() != 1) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Start pose should be a (%d, ) vector, but got layout with %lu dimensions.",
                Dim,
                m_start_.layout.dim.size());
            return false;
        }
        if (m_start_.layout.dim[0].size != Dim) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Start pose should have a (%d, ) layout, but got size %u.",
                Dim,
                m_start_.layout.dim[0].size);
            return false;
        }
        if (m_start_.data.size() != Dim) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Start pose should be a (%d, ) vector, but got data with size %lu.",
                Dim,
                m_start_.data.size());
            return false;
        }
        for (int i = 0; i < Dim; ++i) { start[i] = static_cast<Dtype>(m_start_.data[i]); }
        return true;
    }

    bool
    GetGoalsFromTopicMsg(std::vector<MetricState>& goals) {
        std::lock_guard<std::mutex> lock(m_goals_mutex_);
        if (!m_goals_received_) {
            RCLCPP_WARN(this->get_logger(), "Goals are not received yet.");
            return false;
        }
        if (m_goals_.layout.dim.size() == 1) {
            // (Dim, ) vector
            if (m_goals_.layout.dim[0].size != Dim) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Goals should have a (%d, ) layout, but got size %u.",
                    Dim,
                    m_goals_.layout.dim[0].size);
                return false;
            }
            if (m_goals_.data.size() != Dim) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Goals should be a (%d, ) vector, but got data with size %ld.",
                    Dim,
                    m_goals_.data.size());
                return false;
            }
            goals.resize(1);
            for (int i = 0; i < Dim; ++i) { goals[0][i] = static_cast<Dtype>(m_goals_.data[i]); }
            return true;
        }
        if (m_goals_.layout.dim.size() != 2) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Goals should be a (%d, N) col-major matrix, but got layout with %ld dimensions.",
                Dim,
                m_goals_.layout.dim.size());
            return false;
        }
        if (m_goals_.layout.dim[0].size != Dim) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Goals should have a (%d, N) layout, but got first dimension size %u.",
                Dim,
                m_goals_.layout.dim[0].size);
            return false;
        }
        auto n_goals = static_cast<std::size_t>(m_goals_.layout.dim[1].size);
        if (m_goals_.data.size() != Dim * n_goals) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Goals should be a (%d, %ld) matrix, but got data with size %ld.",
                Dim,
                n_goals,
                m_goals_.data.size());
            return false;
        }
        goals.resize(n_goals);
        const double* data_ptr = m_goals_.data.data();
        for (std::size_t i = 0; i < n_goals; ++i) {
            for (int d = 0; d < Dim; ++d) { goals[i][d] = static_cast<Dtype>(*(data_ptr++)); }
        }
        return true;
    }

    bool
    GetGoalTolerancesFromTopicMsg(std::vector<MetricState>& goal_tolerances) {
        std::lock_guard<std::mutex> lock(m_goal_tolerances_mutex_);
        if (!m_goal_tolerances_received_) {
            RCLCPP_WARN(this->get_logger(), "Goals tolerances are not received yet.");
            return false;
        }
        if (m_goal_tolerances_.layout.dim.size() >= 3) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Goal tolerances should be a (%d, ) vector or (%d, N) col-major matrix, but got "
                "layout with %ld dimensions.",
                Dim,
                Dim,
                m_goal_tolerances_.layout.dim.size());
            return false;
        }
        if (m_goal_tolerances_.layout.dim.size() == 1) {
            // (Dim, ) vector
            if (m_goal_tolerances_.layout.dim[0].size != Dim) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Goal tolerances should have a (%d, ) layout, but got size %u.",
                    Dim,
                    m_goal_tolerances_.layout.dim[0].size);
                return false;
            }
            if (m_goal_tolerances_.data.size() != Dim) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Goal tolerances should be a (%d, ) vector, but got data with size %ld.",
                    Dim,
                    m_goal_tolerances_.data.size());
                return false;
            }
            goal_tolerances.resize(1);
            for (int i = 0; i < Dim; ++i) {
                goal_tolerances[0][i] = static_cast<Dtype>(m_goal_tolerances_.data[i]);
            }
            return true;
        }
        // (Dim, N) col-major matrix
        if (m_goal_tolerances_.layout.dim[0].size != Dim) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Goal tolerances should have a (%d, N) layout, but got first dimension size %u.",
                Dim,
                m_goal_tolerances_.layout.dim[0].size);
            return false;
        }
        auto n_goals = static_cast<std::size_t>(m_goal_tolerances_.layout.dim[1].size);
        if (m_goal_tolerances_.data.size() != Dim * n_goals) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Goal tolerances should be a (%d, %ld) matrix, but got data with size %ld.",
                Dim,
                n_goals,
                m_goal_tolerances_.data.size());
            return false;
        }
        goal_tolerances.resize(n_goals);
        const double* data_ptr = m_goal_tolerances_.data.data();
        for (std::size_t i = 0; i < n_goals; ++i) {
            for (int d = 0; d < Dim; ++d) {
                goal_tolerances[i][d] = static_cast<Dtype>(*(data_ptr++));
            }
        }
        return true;
    }

    bool
    GetTerminalCostsFromTopicMsg(std::vector<Dtype>& terminal_costs) {
        std::lock_guard<std::mutex> lock(m_terminal_costs_mutex_);
        if (!m_terminal_costs_received_) {
            RCLCPP_WARN(this->get_logger(), "Terminal costs are not received yet.");
            return false;
        }
        if (m_terminal_costs_.layout.dim.size() != 1) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Terminal costs should be a (N, ) vector, but got layout with %ld dimensions.",
                m_terminal_costs_.layout.dim.size());
            return false;
        }
        auto n_goals = static_cast<std::size_t>(m_terminal_costs_.layout.dim[0].size);
        if (m_terminal_costs_.data.size() != n_goals) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Terminal costs should be a (%ld, ) vector, but got data with size %ld.",
                n_goals,
                m_terminal_costs_.data.size());
            return false;
        }
        terminal_costs.resize(n_goals);
        for (std::size_t i = 0; i < n_goals; ++i) {
            terminal_costs[i] = static_cast<Dtype>(m_terminal_costs_.data[i]);
        }
        return true;
    }

    /**
     * Override this method to do some custom operations before running A*. For example, the
     * start state can be modified based on the environment.
     * @param env The environment.
     * @param start The start state.
     * @param goals The goal states.
     * @param goals_tolerances The goal tolerances.
     * @param terminal_costs The terminal costs for each goal.
     */
    virtual void
    BeforeAstar(
        std::shared_ptr<Env>& /* env */,
        MetricState& /* start */,
        std::vector<MetricState>& /* goals */,
        std::vector<MetricState>& /* goals_tolerances */,
        std::vector<Dtype>& /* terminal_costs */) {}

    bool
    RunAstar(const std_msgs::msg::Header& header) {
        std::shared_ptr<Env> env = GetEnv();
        if (env == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Environment for planning is not ready.");
            return false;
        }

        MetricState start;
        if (m_start_source_ == "topic") {
            if (!GetStartFromTopicMsg(start)) {
                RCLCPP_WARN(this->get_logger(), "Failed to get start from topic.");
                return false;
            }
        } else if (m_start_source_ == "tf") {
            geometry_msgs::msg::TransformStamped start_pose;
            if (!GetStartPoseFromTf(header.stamp, start_pose)) {
                RCLCPP_WARN(this->get_logger(), "Failed to get start from tf.");
                return false;
            }
            start = GetStartFromPoseMsg(start_pose);
        } else {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unknown start source: %s. Supported sources are 'topic' and 'tf'.",
                m_start_source_.c_str());
            return false;
        }

        std::vector<MetricState> goals;
        if (!GetGoalsFromTopicMsg(goals)) {
            RCLCPP_WARN(this->get_logger(), "Failed to get goals from topic message.");
            return false;
        }

        std::vector<MetricState> goals_tolerances;
        if (!GetGoalTolerancesFromTopicMsg(goals_tolerances)) {
            RCLCPP_WARN(this->get_logger(), "Failed to get goal tolerances from topic message.");
            return false;
        }
        if (goals_tolerances.size() != 1 && goals_tolerances.size() != goals.size()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Number of goal tolerances (%lu) must be 1 or equal to number of goals (%lu).",
                goals_tolerances.size(),
                goals.size());
            return false;
        }

        std::vector<Dtype> terminal_costs;
        if (!GetTerminalCostsFromTopicMsg(terminal_costs)) {
            RCLCPP_WARN(this->get_logger(), "Failed to get terminal costs from topic message.");
            return false;
        }
        if (terminal_costs.size() != 1 && terminal_costs.size() != goals.size()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Number of terminal costs (%lu) must be 1 or equal to number of goals (%lu).",
                terminal_costs.size(),
                goals.size());
            return false;
        }

        BeforeAstar(env, start, goals, goals_tolerances, terminal_costs);

        std::string fmt_str = fmt::format(
            "Start: [{}], Goals: [{}], Goal tolerances: [{}], Terminal costs: [{}]",
            fmt::join(start, ", "),
            fmt::join(goals, "], ["),
            fmt::join(goals_tolerances, "], ["),
            fmt::join(terminal_costs, ", "));
        RCLCPP_INFO(this->get_logger(), "%s", fmt_str.c_str());

        auto heuristic = GetHeuristic();
        auto planning_interface = std::make_shared<SearchPlanningInterface>(
            env,
            start,
            goals,
            goals_tolerances,
            terminal_costs,
            heuristic);

        AStar astar(planning_interface, m_astar_setting_);
        std::shared_ptr<AStarOutput> output = astar.Plan();
        const PlanRecord* plan_record = output->GetLatestRecord();
        if (plan_record == nullptr) {
            RCLCPP_WARN(this->get_logger(), "No path found.");
            return false;
        }

        // Convert to nav_msgs::msg::Path
        nav_msgs::msg::Path path_msg;
        path_msg.header = header;
        path_msg.header.frame_id = m_global_frame_;
        LoadPathToMsg(*plan_record, path_msg);

        // Publish results
        m_path_pub_->publish(path_msg);

        std_msgs::msg::Float64 cost_msg;
        cost_msg.data = plan_record->cost;
        m_cost_pub_->publish(cost_msg);

        std_msgs::msg::Int64 goal_idx_msg;
        goal_idx_msg.data = plan_record->goal_index;
        m_goal_idx_pub_->publish(goal_idx_msg);

        return true;
    }

    void
    CallbackSrvPlan(
        std_srvs::srv::Trigger::Request::ConstSharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        (void) request;
        rclcpp::Time current_time = this->now();
        std_msgs::msg::Header header;
        header.stamp = current_time;
        header.frame_id = m_global_frame_;
        if (RunAstar(header)) {
            response->success = true;
            response->message = "Planning succeeded.";
        } else {
            response->success = false;
            response->message = "Planning failed.";
        }
    }

    virtual void
    Reset() {
        {
            std::lock_guard<std::mutex> lock(m_start_mutex_);
            m_start_received_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(m_goals_mutex_);
            m_goals_received_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(m_goal_tolerances_mutex_);
            m_goal_tolerances_received_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(m_terminal_costs_mutex_);
            m_terminal_costs_received_ = false;
        }
    }

    void
    CallbackSrvReset(
        std_srvs::srv::Trigger::Request::ConstSharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        (void) request;
        Reset();
        response->success = true;
        response->message = "Reset succeeded.";
    }
};
