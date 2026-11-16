#include <gtest/gtest.h>
#include "pluginlib/class_loader.hpp"
#include "collision_avoidance_interfaces/i_collision_avoidance.hpp"
#include <cmath>

class OrcaPluginTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
    void SetUp() override
    {
        node = std::make_shared<rclcpp::Node>("orca_plugin_test");
        calculator = loader.createSharedInstance("collision_avoidance::Orca",
            node->get_node_parameters_interface());
    }
    collision_avoidance_interfaces::msg::CollisionState state(double x, double vx)
    {
        collision_avoidance_interfaces::msg::CollisionState result;
        result.position.x = x;
        result.velocity.x = vx;
        result.preferred_velocity = result.velocity;
        result.preferred_velocity.z = 0.25;
        result.radius = 0.15;
        result.max_speed = 0.8;
        return result;
    }
    pluginlib::ClassLoader<collision_avoidance_interfaces::ICollisionAvoidance> loader{
        "collision_avoidance_interfaces", "collision_avoidance_interfaces::ICollisionAvoidance"};
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<collision_avoidance_interfaces::ICollisionAvoidance> calculator;
};

TEST_F(OrcaPluginTest, PreservesFreeVelocityAndVerticalComponent)
{
    const auto self = state(0.0, 0.4);
    bool constrained = true;
    const auto result = calculator->calculate_velocity(self, {}, constrained);
    EXPECT_NEAR(result.x, 0.4, 1e-9);
    EXPECT_NEAR(result.y, 0.0, 1e-9);
    EXPECT_NEAR(result.z, 0.25, 1e-9);
    EXPECT_FALSE(constrained);
}

TEST_F(OrcaPluginTest, EnforcesSpeedLimitWithoutNeighbors)
{
    bool constrained = false;
    const auto result = calculator->calculate_velocity(state(0.0, 2.0), {}, constrained);
    EXPECT_NEAR(std::hypot(result.x, result.y), 0.8, 1e-9);
    EXPECT_TRUE(constrained);
}

TEST_F(OrcaPluginTest, ConstrainsHeadOnPairAndFiltersDistantNeighbors)
{
    auto other = state(0.8, -0.5);
    bool constrained = false;
    const auto self = state(0.0, 0.5);
    const auto result = calculator->calculate_velocity(self, {other}, constrained);
    EXPECT_TRUE(constrained);
    EXPECT_TRUE(std::isfinite(result.x) && std::isfinite(result.y));
    EXPECT_LE(std::hypot(result.x, result.y), 0.8 + 1e-9);
    other.position.x = 2.0;
    const auto free = calculator->calculate_velocity(self, {other}, constrained);
    EXPECT_FALSE(constrained);
    EXPECT_NEAR(free.x, 0.5, 1e-9);
    EXPECT_NEAR(free.y, 0.0, 1e-9);
}

TEST_F(OrcaPluginTest, RejectsInvalidRuntimeHorizon)
{
    const auto result = node->set_parameter(rclcpp::Parameter("orca.time_horizon", 0.0));
    EXPECT_FALSE(result.successful);
    EXPECT_DOUBLE_EQ(node->get_parameter("orca.time_horizon").as_double(), 1.0);
}
