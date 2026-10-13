#include "rclcpp/rclcpp.hpp"
#include "collision_avoidance_interfaces/msg/collision_states.hpp"
#include "velocity_reciprocal_visualizer.hpp"
#include <cmath>
#include <map>
#include <stdexcept>

// One writer per actor id; cache only, all ORCA calculations run in actors.
class CollisionAvoidanceBlackboard : public rclcpp::Node
{
public:
    CollisionAvoidanceBlackboard() : Node("collision_avoidance_blackboard")
    {
        const auto read_only = rcl_interfaces::msg::ParameterDescriptor().set__read_only(true);
        const double period = declare_parameter("publish_period", 0.1, read_only);
        m_max_age = declare_parameter("state_max_age", 10.0, read_only);
        if (!std::isfinite(period) || period <= 0.0 ||
            !std::isfinite(m_max_age) || m_max_age <= 0.0) {
            throw std::invalid_argument("publish_period and state_max_age must be positive and finite");
        }
        declare_parameter("publish_visualization", false);
        m_publisher = create_publisher<collision_avoidance_interfaces::msg::CollisionStates>(
            "/collision_avoidance/states", rclcpp::QoS(1));
        m_subscription = create_subscription<collision_avoidance_interfaces::msg::CollisionState>(
            "/collision_avoidance/state", rclcpp::QoS(256),
            [this](collision_avoidance_interfaces::msg::CollisionState::ConstSharedPtr state) {
                if (!valid(*state)) return;
                const auto time = now();
                const auto stamp = rclcpp::Time(state->stamp, time.get_clock_type());
                const auto age = (time - stamp).seconds();
                if (age < 0.0 || age > m_max_age) return;
                const auto existing = m_states.find(state->id);
                if (existing == m_states.end() || stamp >=
                    rclcpp::Time(existing->second.stamp, time.get_clock_type())) {
                    m_states[state->id] = *state;
                }
            });
        m_timer = rclcpp::create_timer(get_node_base_interface(), get_node_timers_interface(),
            get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(period)), [this]() { publish_snapshot(); });
    }
private:
    static bool valid(const collision_avoidance_interfaces::msg::CollisionState & state)
    {
        const auto finite = [](const geometry_msgs::msg::Vector3 & v) {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        };
        return finite(state.position) && finite(state.preferred_velocity) &&
            finite(state.velocity) && finite(state.calculated_velocity) &&
            std::isfinite(state.radius) && state.radius > 0.0 &&
            std::isfinite(state.max_speed) && state.max_speed > 0.0;
    }
    void publish_snapshot()
    {
        const auto time = now();
        collision_avoidance_interfaces::msg::CollisionStates snapshot;
        std::unordered_map<uint8_t, ObjectInfo> objects;
        for (auto it = m_states.begin(); it != m_states.end();) {
            const auto stamp = rclcpp::Time(it->second.stamp, time.get_clock_type());
            const double age = (time - stamp).seconds();
            if (age < 0.0 || age > m_max_age) {
                it = m_states.erase(it);
                continue;
            }
            const auto & state = it->second;
            snapshot.states.push_back(state);
            objects[state.id] = {{state.position.x, state.position.y},
                {state.preferred_velocity.x, state.preferred_velocity.y},
                {state.velocity.x, state.velocity.y},
                {state.calculated_velocity.x, state.calculated_velocity.y},
                state.radius, state.max_speed, stamp};
            ++it;
        }
        m_publisher->publish(snapshot);
        if (get_parameter("publish_visualization").as_bool()) {
            if (!m_visualizer) {
                m_visualizer = std::make_unique<VelocityReciprocalVisualizer>(
                    get_node_topics_interface(), get_clock());
            }
            m_visualizer->publish(objects);
        } else if (m_visualizer) {
            m_visualizer->clear();
            m_visualizer.reset();
        }
    }
    double m_max_age;
    std::map<uint8_t, collision_avoidance_interfaces::msg::CollisionState> m_states;
    rclcpp::Publisher<collision_avoidance_interfaces::msg::CollisionStates>::SharedPtr m_publisher;
    rclcpp::Subscription<collision_avoidance_interfaces::msg::CollisionState>::SharedPtr m_subscription;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::unique_ptr<VelocityReciprocalVisualizer> m_visualizer;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionAvoidanceBlackboard>());
    rclcpp::shutdown();
    return 0;
}
