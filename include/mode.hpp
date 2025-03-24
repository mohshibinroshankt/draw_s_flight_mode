// mode.hpp
#pragma once

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

using namespace std::chrono_literals;

// --- DrawSFlightMode ---
static const std::string kName = "Draw S";

class DrawSFlightMode : public px4_ros2::ModeBase
{
public:
    explicit DrawSFlightMode(rclcpp::Node& node)
        : ModeBase(node, Settings{kName}), _node{node}
    {
        _goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);
        _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
        _trajectory_setpoint_pub = this->node().create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            this->topicNamespacePrefix() + "fmu/in/trajectory_setpoint",
            1);
    }

    ~DrawSFlightMode() override = default;

    void onActivate() override
    {
        _state = State::SettlingAtStart;
        _start_position_m = _vehicle_local_position->positionNed();
    }

    void onDeactivate() override {}

    void updateSetpoint(float dt_s) override
    {
        switch (_state) {
        case State::SettlingAtStart: {
            _goto_setpoint->update(_start_position_m);
            if (positionReached(_start_position_m)) {
                _state = State::West1;
                RCLCPP_INFO(_node.get_logger(), "Starting West1 from (%f, %f)", _start_position_m.x(), _start_position_m.y());
            }
            break;
        }

        case State::West1: {
            // Move West 25m (y = -25)
            Eigen::Vector3f target = _start_position_m + Eigen::Vector3f{0.0f, -25.0f, 0.0f};
            _goto_setpoint->update(target, -M_PI_2, _horizontal_speed_m_s); // Facing West

            if (positionReached(target)) {
                _state = State::South1;
                RCLCPP_INFO(_node.get_logger(), "Reached West1 at (%f, %f), moving to South1", target.x(), target.y());
            }
            break;
        }

        case State::South1: {
            // Move South 25m (x = -25)
            Eigen::Vector3f target = _start_position_m + Eigen::Vector3f{-25.0f, -25.0f, 0.0f};
            _goto_setpoint->update(target, M_PI, _horizontal_speed_m_s); // Facing South

            if (positionReached(target)) {
                _state = State::East;
                RCLCPP_INFO(_node.get_logger(), "Reached South1 at (%f, %f), moving to East", target.x(), target.y());
            }
            break;
        }

        case State::East: {
            // Move East 25m (y = 0)
            Eigen::Vector3f target = _start_position_m + Eigen::Vector3f{-25.0f, 0.0f, 0.0f};
            _goto_setpoint->update(target, M_PI_2, _horizontal_speed_m_s); // Facing East

            if (positionReached(target)) {
                _state = State::South2;
                RCLCPP_INFO(_node.get_logger(), "Reached East at (%f, %f), moving to South2", target.x(), target.y());
            }
            break;
        }

        case State::South2: {
            // Move South 25m (x = -50)
            Eigen::Vector3f target = _start_position_m + Eigen::Vector3f{-50.0f, 0.0f, 0.0f};
            _goto_setpoint->update(target, M_PI, _horizontal_speed_m_s); // Facing South

            if (positionReached(target)) {
                _state = State::West2;
                RCLCPP_INFO(_node.get_logger(), "Reached South2 at (%f, %f), moving to West2", target.x(), target.y());
            }
            break;
        }

        case State::West2: {
            // Move West 25m (y = -25)
            Eigen::Vector3f target = _start_position_m + Eigen::Vector3f{-50.0f, -25.0f, 0.0f};
            _goto_setpoint->update(target, -M_PI_2, _horizontal_speed_m_s); // Facing West

            if (positionReached(target)) {
                _state = State::Done;
                RCLCPP_INFO(_node.get_logger(), "Reached West2 at (%f, %f), finished", target.x(), target.y());
            }
            break;
        }

        case State::Done: {
            completed(px4_ros2::Result::Success);
            break;
        }
        }
    }

private:
    rclcpp::Node& _node;
    std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_setpoint_pub;

    Eigen::Vector3f _start_position_m;

    static constexpr float _horizontal_speed_m_s = 3.0f; // Speed in m/s

    enum class State {
        SettlingAtStart = 0,
        West1,    // West 25m: (0, -25)
        South1,   // South 25m: (-25, -25)
        East,     // East 25m: (-25, 0)
        South2,   // South 25m: (-50, 0)
        West2,    // West 25m: (-50, -25)
        Done
    } _state;

    bool positionReached(const Eigen::Vector3f& target_position_m) const
    {
        static constexpr float kPositionErrorThreshold = 0.5f; // [m]
        static constexpr float kVelocityErrorThreshold = 0.3f; // [m/s]
        const Eigen::Vector3f position_error_m = target_position_m - _vehicle_local_position->positionNed();
        return (position_error_m.norm() < kPositionErrorThreshold) &&
               (_vehicle_local_position->velocityNed().norm() < kVelocityErrorThreshold);
    }
};

// --- DrawSModeExecutor ---
class DrawSModeExecutor : public px4_ros2::ModeExecutorBase
{
public:
    DrawSModeExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode)
        : ModeExecutorBase(node, {px4_ros2::ModeExecutorBase::Settings::Activation::ActivateImmediately}, owned_mode),
          _node(node)
    {
    }

    enum class State {
        Reset,
        Arming,
        TakingOff,
        DrawMode,
        RTL,
        WaitUntilDisarmed,
    };

    void onActivate() override
    {
        runState(State::Arming, px4_ros2::Result::Success);
    }

    void onDeactivate(DeactivateReason reason) override {}

    void runState(State state, px4_ros2::Result previous_result)
    {
        if (previous_result != px4_ros2::Result::Success) {
            RCLCPP_ERROR(_node.get_logger(), "State %i: previous state failed: %s", (int)state, resultToString(previous_result));
            return;
        }

        RCLCPP_DEBUG(_node.get_logger(), "Executing state %i", (int)state);

        switch (state) {
        case State::Reset: break;
        case State::Arming:
            arm([this](px4_ros2::Result result) { runState(State::TakingOff, result); });
            break;
        case State::TakingOff:
            takeoff([this](px4_ros2::Result result) { runState(State::DrawMode, result); }, 30.f);
            break;
        case State::DrawMode:
            scheduleMode(ownedMode().id(), [this](px4_ros2::Result result) { runState(State::RTL, result); });
            break;
        case State::RTL:
            rtl([this](px4_ros2::Result result) { runState(State::WaitUntilDisarmed, result); });
            break;
        case State::WaitUntilDisarmed:
            waitUntilDisarmed([this](px4_ros2::Result result) {
                RCLCPP_INFO(_node.get_logger(), "All states complete (%s)", resultToString(result));
            });
            break;
        }
    }

private:
    rclcpp::Node& _node;
};