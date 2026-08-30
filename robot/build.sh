#! /bin/bash

set -e

source /opt/ros/humble/setup.bash

# Source/build ownership is independent from robot runtime data ownership.
export ROAMERX_DATA_ROOT="${ROAMERX_DATA_ROOT:-/home/dogrobot/runtime/nx-edge/data/jszr}"
BUILD_WORKERS="${ROAMERX_BUILD_WORKERS:-1}"
BUILD_JOBS="${ROAMERX_BUILD_JOBS:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j${BUILD_JOBS}}"

# All builds of this workspace must use the same install mode.  In particular,
# reusing a CMake cache created by `colcon build --symlink-install` can make
# ament try to replace generated Python directories with symlinks.
BUILD_LOCK_FILE="/tmp/roamerx-robot-build-${UID}.lock"
exec 9>"${BUILD_LOCK_FILE}"
if ! flock -n 9; then
  echo "[jszr shell error] => another robot workspace build is already running"
  echo "[jszr shell error] => lock: ${BUILD_LOCK_FILE}"
  exit 2
fi

usage() {
  echo "Usage: $0 [clean] all [debug|relwithdebinfo]"
  echo "./build.sh all             [build all project packages]"
  echo "./build.sh all debug       [build all packages in Debug mode]"
  echo "./build.sh all relwithdebinfo [build all packages with symbols and optimization]"
  echo "./build.sh clean all       [clean build/install/log, then build all]"
  echo "./build.sh clean all debug [clean then build all in Debug mode]"
}

# 检查是否传入了参数
if [ $# -eq 0 ]; then
  usage
  exit 1
fi

DO_CLEAN=0
BUILD_TARGET=""
CMAKE_BUILD_ARGS=(
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
)

for arg in "$@"; do
  case "$arg" in
    clean)
      DO_CLEAN=1
      ;;
    all)
      BUILD_TARGET="all"
      ;;
    debug)
      CMAKE_BUILD_ARGS+=("-DCMAKE_BUILD_TYPE=Debug")
      ;;
    relwithdebinfo)
      CMAKE_BUILD_ARGS+=(
        "-DCMAKE_BUILD_TYPE=RelWithDebInfo"
        "-DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer"
      )
      ;;
    *)
      echo "Unknown argument: $arg"
      usage
      exit 1
      ;;
  esac
done

if [ -z "$BUILD_TARGET" ]; then
  usage
  exit 1
fi

if [ "$DO_CLEAN" -eq 1 ]; then
  echo "[jszr shell log] => clean build/install/log ..."
  rm -rf build install log
else
  # CMake keeps AMENT_CMAKE_SYMLINK_INSTALL in its cache even when a later
  # colcon invocation omits --symlink-install.  Fail before compilation with
  # an actionable error instead of stopping deep inside robots_dog_msgs.
  SYMLINK_CACHE="$(
    rg -l '^AMENT_CMAKE_SYMLINK_INSTALL:BOOL=(1|ON|TRUE)$' \
      build/*/CMakeCache.txt 2>/dev/null | head -n 1 || true
  )"
  if [ -n "$SYMLINK_CACHE" ]; then
    echo "[jszr shell error] => incompatible symlink-install cache: ${SYMLINK_CACHE}"
    echo "[jszr shell error] => this workspace uses normal (copy) install mode"
    echo "[jszr shell error] => run './build.sh clean all [debug|relwithdebinfo]' or remove that package's build/install directories"
    exit 2
  fi
fi

case "$BUILD_TARGET" in
all)
  echo "[jszr shell log] => will compile all project..."
  echo "[jszr shell log] => "

  colcon build --cmake-args "${CMAKE_BUILD_ARGS[@]}" --packages-select \
    robots_dog_msgs \
    robot_slam \
    navigo_behavior_tree \
    navigo_behaviors \
    navigo_bt_navigator \
    navigo_collision_monitor \
    navigo_core \
    navigo_costmap_2d \
    navigo_map_server \
    navigo_mppi_controller \
    navigo_path_controller \
    navigo_navfn_planner \
    navigo_path_planner \
    navigo_util \
    navigo_velocity_optimizer \
    navigo_waypoint_follower \
    fast_gicp \
    ndt_omp \
    localization \
    robot_navigo  --parallel-workers "${BUILD_WORKERS}"
  echo "[zsibot shell log] => "
  echo "[zsibot shell log] => OK"
  ;;
*)
  usage
  exit 1
  ;;
esac
