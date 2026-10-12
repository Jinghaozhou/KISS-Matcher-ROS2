#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
vendor="$workspace/vendor"
ros_setup="/opt/ros/humble/setup.bash"
algorithm_build="$workspace/build_algorithms"
prefix="$workspace/external"
jobs="${JOBS:-4}"

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS 2 setup file not found: $ros_setup" >&2
  exit 1
fi

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
set +u
source "$ros_setup"
set -u

cmake -S "$vendor/robin" -B "$algorithm_build/robin" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON \
  -DROBIN_VENDOR_ROOT="$vendor" \
  -DBUILD_TESTS=OFF \
  -DPMC_BUILD_SHARED=OFF
cmake --build "$algorithm_build/robin" --parallel "$jobs"
cmake --install "$algorithm_build/robin"

cmake -S "$vendor/kiss_matcher" -B "$algorithm_build/kiss_matcher" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DCMAKE_PREFIX_PATH="$prefix" \
  -DUSE_SYSTEM_EIGEN3=ON \
  -DUSE_SYSTEM_TBB=ON \
  -DUSE_SYSTEM_ROBIN=ON
cmake --build "$algorithm_build/kiss_matcher" --parallel "$jobs"
cmake --install "$algorithm_build/kiss_matcher"

cmake -S "$vendor/small_gicp" -B "$algorithm_build/small_gicp" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DBUILD_HELPER=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_BENCHMARKS=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_WITH_MARCH_NATIVE=OFF
cmake --build "$algorithm_build/small_gicp" --parallel "$jobs"
cmake --install "$algorithm_build/small_gicp"

cd "$workspace"
export CMAKE_PREFIX_PATH="$prefix:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$prefix/lib:${LD_LIBRARY_PATH:-}"
colcon build --packages-select \
  kiss_matcher_localization pcd_cloud_publisher localization_visualizer
