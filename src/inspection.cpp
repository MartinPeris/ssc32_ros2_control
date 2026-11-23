// SPDX-License-Identifier: BSD-3-Clause
#include "ssc32_ros2_control/inspection.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace ssc32_ros2_control {
namespace {

constexpr std::string_view kPluginName = "ssc32_ros2_control/Ssc32System";

std::string json_escape(const std::string_view value) {
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        output << "\\\"";
        break;
      case '\\':
        output << "\\\\";
        break;
      case '\b':
        output << "\\b";
        break;
      case '\f':
        output << "\\f";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<unsigned int>(character) << std::dec;
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  return output.str();
}

}  // namespace

HardwareConfiguration inspect_hardware_resources(
    const std::vector<hardware_interface::HardwareInfo>& resources,
    const std::optional<std::string>& requested_name) {
  std::vector<const hardware_interface::HardwareInfo*> matches;
  for (const auto& resource : resources) {
    const bool name_matches = !requested_name || resource.name == *requested_name;
    if (name_matches && resource.hardware_plugin_name == kPluginName) {
      matches.push_back(&resource);
    }
  }

  if (matches.empty()) {
    if (requested_name) {
      throw std::invalid_argument("no SSC-32 hardware named '" + *requested_name + "'");
    }
    throw std::invalid_argument("URDF contains no ssc32_ros2_control/Ssc32System hardware");
  }
  if (matches.size() > 1U) {
    throw std::invalid_argument(
        "URDF contains multiple SSC-32 hardware components; use --hardware");
  }
  return parse_hardware_configuration(*matches.front());
}

std::string render_configuration_text(const HardwareConfiguration& configuration) {
  std::ostringstream output;
  output << "VALID SSC-32 configuration\n"
         << "hardware: " << configuration.name << '\n'
         << "device: " << configuration.device << '\n'
         << "baud_rate: " << configuration.baud_rate << '\n'
         << "timeout_ms: " << configuration.timeout.count() << '\n'
         << "diagnostics_stale_after_ms: " << configuration.diagnostics_stale_after.count() << '\n'
         << "joints: " << configuration.joints.size() << '\n';
  for (const auto& joint : configuration.joints) {
    output << "  - " << joint.name << '\n'
           << "    channel: " << static_cast<unsigned int>(joint.channel) << '\n'
           << "    position: [" << joint.min_position << ", " << joint.max_position << "]\n"
           << "    pulse_envelope_us: [" << joint.min_pulse_us << ", " << joint.max_pulse_us
           << "]\n"
           << "    endpoint_pulses_us: [" << joint.position_to_pulse(joint.min_position) << ", "
           << joint.position_to_pulse(joint.max_position) << "]\n"
           << "    zero_pulse_us: " << joint.zero_pulse_us << '\n'
           << "    pulse_us_per_unit: " << joint.pulse_us_per_unit << '\n'
           << "    direction: " << joint.direction << '\n'
           << "    max_speed_us_per_second: " << joint.max_speed_us_per_second << '\n';
  }
  return output.str();
}

std::string render_configuration_json(const HardwareConfiguration& configuration) {
  std::ostringstream output;
  output << std::setprecision(17) << "{\n"
         << "  \"valid\": true,\n"
         << "  \"hardware\": \"" << json_escape(configuration.name) << "\",\n"
         << "  \"device\": \"" << json_escape(configuration.device) << "\",\n"
         << "  \"baud_rate\": " << configuration.baud_rate << ",\n"
         << "  \"timeout_ms\": " << configuration.timeout.count() << ",\n"
         << "  \"diagnostics_stale_after_ms\": " << configuration.diagnostics_stale_after.count()
         << ",\n"
         << "  \"joints\": [\n";
  for (std::size_t index = 0; index < configuration.joints.size(); ++index) {
    const auto& joint = configuration.joints[index];
    output << "    {\n"
           << "      \"name\": \"" << json_escape(joint.name) << "\",\n"
           << "      \"channel\": " << static_cast<unsigned int>(joint.channel) << ",\n"
           << "      \"position_min\": " << joint.min_position << ",\n"
           << "      \"position_max\": " << joint.max_position << ",\n"
           << "      \"pulse_min_us\": " << joint.min_pulse_us << ",\n"
           << "      \"pulse_max_us\": " << joint.max_pulse_us << ",\n"
           << "      \"pulse_at_position_min_us\": " << joint.position_to_pulse(joint.min_position)
           << ",\n"
           << "      \"pulse_at_position_max_us\": " << joint.position_to_pulse(joint.max_position)
           << ",\n"
           << "      \"zero_pulse_us\": " << joint.zero_pulse_us << ",\n"
           << "      \"pulse_us_per_unit\": " << joint.pulse_us_per_unit << ",\n"
           << "      \"direction\": " << joint.direction << ",\n"
           << "      \"max_speed_us_per_second\": " << joint.max_speed_us_per_second << "\n"
           << "    }" << (index + 1U == configuration.joints.size() ? "\n" : ",\n");
  }
  output << "  ]\n}\n";
  return output.str();
}

}  // namespace ssc32_ros2_control
