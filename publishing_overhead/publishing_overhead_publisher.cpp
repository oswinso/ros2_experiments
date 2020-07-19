#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  }

  void publishBenchmark() {
    const std::vector<int> kByteSizes = {
        1,     2,     3,     4,     5,      10,    15,    20,    50,
        100,   200,   300,   400,   500,    1000,  1100,  1200,  1300,
        1400,  1500,  2000,  3000,  4000,   5000,  10000, 12500, 15000,
        20000, 30000, 40000, 50000, 100000, 200000, 500000, 1000000};
    constexpr auto kRepetitions = 100;

    std::vector<uint64_t> micros(kByteSizes.size(), 0);

    // Warmup.
    constexpr int kWarmupIters = 10000;
    for (int i = 0; i < kWarmupIters; i++) {
      std_msgs::msg::String msg;
      msg.data = "Warming up. i = " + std::to_string(i);
      publisher_->publish(msg);

      std::this_thread::sleep_for(10us);
    }

    // Actual benchmark.
    for (int rep_idx = 0; rep_idx < kRepetitions; rep_idx++) {
      for (int byte_idx = 0; byte_idx < kByteSizes.size(); byte_idx++) {
        auto byte_size = kByteSizes[byte_idx];
        std_msgs::msg::String msg;

        for (int i = 0; i < byte_size; i++) {
          msg.data.push_back(i % 2 == 0 ? 'a' : 'b');
        }

        const auto start = std::chrono::high_resolution_clock::now();
        publisher_->publish(msg);
        const auto stop = std::chrono::high_resolution_clock::now();
        const auto elapsed = stop - start;
        const uint64_t microseconds =
            std::chrono::duration_cast<std::chrono::microseconds>(elapsed)
                .count();
        micros[byte_idx] += microseconds;
      }
    }

    for (int byte_idx = 0; byte_idx < kByteSizes.size(); byte_idx++) {
      const double average_microseconds =
          static_cast<double>(micros[byte_idx]) / kRepetitions;
      std::cout << std::setw(8) << kByteSizes[byte_idx] << " bytes took "
                << std::setw(10) << average_microseconds
                << " us! (Averaged over " << kRepetitions << " repetitions.)"
                << std::endl;
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  node->publishBenchmark();
  rclcpp::shutdown();
  return 0;
}
