// SPDX-License-Identifier: BSD-3-Clause
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>

#include "hardware_interface/component_parser.hpp"
#include "ssc32_ros2_control/inspection.hpp"

namespace {

struct Arguments {
  std::string urdf_path;
  std::optional<std::string> hardware_name;
  bool json{false};
};

void print_usage(std::ostream& output) {
  output << "Usage: ssc32_config_inspect [--json] [--hardware NAME] URDF\n";
}

Arguments parse_arguments(const int argc, char* argv[]) {
  Arguments arguments;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--json") {
      arguments.json = true;
    } else if (argument == "--hardware") {
      if (++index >= argc) {
        throw std::invalid_argument("--hardware requires a name");
      }
      arguments.hardware_name = argv[index];
    } else if (argument == "--help" || argument == "-h") {
      print_usage(std::cout);
      std::exit(0);
    } else if (!argument.empty() && argument.front() == '-') {
      throw std::invalid_argument("unknown option: " + argument);
    } else if (!arguments.urdf_path.empty()) {
      throw std::invalid_argument("only one URDF path may be supplied");
    } else {
      arguments.urdf_path = argument;
    }
  }
  if (arguments.urdf_path.empty()) {
    throw std::invalid_argument("a URDF path is required");
  }
  return arguments;
}

std::string read_file(const std::string& path) {
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("cannot open URDF: " + path);
  }
  return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

}  // namespace

int main(const int argc, char* argv[]) {
  try {
    const auto arguments = parse_arguments(argc, argv);
    const auto resources =
        hardware_interface::parse_control_resources_from_urdf(read_file(arguments.urdf_path));
    const auto configuration =
        ssc32_ros2_control::inspect_hardware_resources(resources, arguments.hardware_name);
    std::cout << (arguments.json ? ssc32_ros2_control::render_configuration_json(configuration)
                                 : ssc32_ros2_control::render_configuration_text(configuration));
    return 0;
  } catch (const std::invalid_argument& error) {
    std::cerr << "Invalid SSC-32 configuration: " << error.what() << '\n';
    print_usage(std::cerr);
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "Failed to inspect SSC-32 configuration: " << error.what() << '\n';
    return 1;
  }
}
