#include "sensor_simulator/sensor_simulator.hpp"

using namespace std::chrono_literals;

int main (int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}