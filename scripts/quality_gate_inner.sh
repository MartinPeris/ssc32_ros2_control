#!/usr/bin/env bash
set -eo pipefail

readonly workspace=/workspace
readonly package_source="${workspace}/src/ssc32_ros2_control"
readonly coverage_lines_minimum=78
readonly coverage_branches_minimum=59

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

source_setup /opt/ros/lyrical/setup.bash
cd "${workspace}"

section() {
  printf '\n==> %s\n' "$1"
}

section 'Formatting'
mapfile -d '' formatted_sources < <(
  find "${package_source}/include" "${package_source}/src" "${package_source}/test" \
    -type f \( -name '*.cpp' -o -name '*.hpp' \) -print0
)
clang-format --dry-run --Werror "${formatted_sources[@]}"

section 'Shell lint'
shellcheck \
  "${package_source}/.githooks/pre-commit" \
  "${package_source}/scripts/install_git_hooks.sh" \
  "${package_source}/scripts/quality_gate.sh" \
  "${package_source}/scripts/quality_gate_inner.sh"

section 'GCC warnings-as-errors build and tests'
export CC=gcc CXX=g++
colcon --log-base log-gcc build --packages-select ssc32 \
  --build-base build-gcc --install-base install-gcc \
  --cmake-args \
    -DSSC32_WARNINGS_AS_ERRORS=ON \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
source_setup install-gcc/setup.bash
colcon --log-base log-gcc build --packages-select ssc32_ros2_control \
  --build-base build-gcc --install-base install-gcc \
  --cmake-args \
    -DSSC32_ROS2_CONTROL_WARNINGS_AS_ERRORS=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
source_setup install-gcc/setup.bash
colcon --log-base log-gcc test --packages-select ssc32 ssc32_ros2_control \
  --build-base build-gcc --install-base install-gcc \
  --event-handlers console_direct+
colcon test-result --test-result-base build-gcc --verbose

section 'Clang static analysis'
compile_commands="${workspace}/build-gcc/ssc32_ros2_control/compile_commands.json"
while IFS= read -r source; do
  clang-tidy --quiet -p "$(dirname "${compile_commands}")" "${source}"
done < <(find "${package_source}/src" -type f -name '*.cpp' | sort)

section 'Clang warnings-as-errors build'
export CC=clang CXX=clang++
source_setup /opt/ros/lyrical/setup.bash
source_setup install-gcc/setup.bash
colcon --log-base log-clang build --packages-select ssc32_ros2_control \
  --build-base build-clang --install-base install-clang \
  --cmake-args \
    -DSSC32_ROS2_CONTROL_WARNINGS_AS_ERRORS=ON \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

section 'AddressSanitizer and UndefinedBehaviorSanitizer'
export CC=gcc CXX=g++
source_setup /opt/ros/lyrical/setup.bash
colcon --log-base log-sanitizers build --packages-select ssc32 \
  --build-base build-sanitizers --install-base install-sanitizers \
  --cmake-args \
    -DSSC32_WARNINGS_AS_ERRORS=ON \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
source_setup install-sanitizers/setup.bash
colcon --log-base log-sanitizers build --packages-select ssc32_ros2_control \
  --build-base build-sanitizers --install-base install-sanitizers \
  --cmake-args \
    -DSSC32_ROS2_CONTROL_WARNINGS_AS_ERRORS=ON \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
source_setup install-sanitizers/setup.bash
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
  colcon --log-base log-sanitizers test --packages-select ssc32 ssc32_ros2_control \
    --build-base build-sanitizers --install-base install-sanitizers \
    --event-handlers console_direct+
colcon test-result --test-result-base build-sanitizers --verbose

section 'Coverage build, tests, and gate'
source_setup /opt/ros/lyrical/setup.bash
colcon --log-base log-coverage build --packages-select ssc32 \
  --build-base build-coverage --install-base install-coverage \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS='--coverage -O0 -g' \
    -DCMAKE_EXE_LINKER_FLAGS=--coverage \
    -DCMAKE_SHARED_LINKER_FLAGS=--coverage \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
source_setup install-coverage/setup.bash
colcon --log-base log-coverage build --packages-select ssc32_ros2_control \
  --build-base build-coverage --install-base install-coverage \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS='--coverage -O0 -g' \
    -DCMAKE_EXE_LINKER_FLAGS=--coverage \
    -DCMAKE_SHARED_LINKER_FLAGS=--coverage \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
source_setup install-coverage/setup.bash
colcon --log-base log-coverage test --packages-select ssc32 ssc32_ros2_control \
  --build-base build-coverage --install-base install-coverage \
  --event-handlers console_direct+
colcon test-result --test-result-base build-coverage --verbose
gcovr \
  --root "${package_source}" \
  --object-directory "${workspace}/build-coverage/ssc32_ros2_control" \
  --gcov-exclude-directories '.*/CMakeFiles/.*/CompilerIdCXX$' \
  --filter "${package_source}/src/" \
  --filter "${package_source}/include/ssc32_ros2_control/" \
  --exclude-unreachable-branches \
  --exclude-throw-branches \
  --print-summary \
  --fail-under-line "${coverage_lines_minimum}" \
  --fail-under-branch "${coverage_branches_minimum}" \
  --xml "${workspace}/coverage.xml" \
  --xml-pretty

section 'Quality gate passed'
