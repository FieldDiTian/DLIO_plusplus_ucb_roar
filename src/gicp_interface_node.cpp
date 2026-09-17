/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "gicp_interface/gicp_interface_node.hpp"

#include <rclcpp/contexts/default_context.hpp>

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  // start() and the pre-shutdown drain are armed by the constructor, so the
  // composed and standalone paths behave identically.
  auto node = std::make_shared<gicp_localizer::GicpLocalizer>();

  const int configured_executor_threads = node->executorThreadCount();
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(),
      configured_executor_threads > 0
          ? static_cast<size_t>(configured_executor_threads)
          : 0u);
  RCLCPP_INFO(node->get_logger(),
              "rclcpp executor threads: %d (%s)",
              configured_executor_threads,
              configured_executor_threads > 0 ? "configured"
                                              : "rclcpp default");
  executor.add_node(node);
  int exit_code = 0;
  try {
    executor.spin();
  } catch (const std::exception& e) {
    // Legacy synchronous path: a strict-merge abort (require_all_aux past
    // budget) throws on an executor thread and lands here instead of
    // std::terminate. Report and exit nonzero after an orderly shutdown.
    RCLCPP_FATAL(node->get_logger(), "executor stopped by exception: %s", e.what());
    exit_code = 1;
  }

  rclcpp::shutdown();  // runs the pre-shutdown drain if not already run

  // A pipeline exception on the synchronizer worker is a controlled shutdown
  // (no throw escapes the thread) — surface it in the exit code.
  if (node->syncFatal()) {
    exit_code = 1;
  }

  return exit_code;
}
