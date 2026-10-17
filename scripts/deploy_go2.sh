#!/usr/bin/env bash
# Local preparation only. Build/install never starts runtime nodes or services.
# Validation runs isolated test graphs. No mode contacts a remote device.
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
STATE="$ROOT/.deployment"
MODE=${1:---dry-run}
if (( $# )); then shift; fi
UNITREE_UNDERLAY=""
WHEELHOUSE=""
while (( $# )); do
    case "$1" in
        --state-dir|--unitree-underlay|--wheelhouse)
            if (( $# < 2 )) || [[ -z "$2" || "$2" == --* ]]; then
                echo "Missing value for $1" >&2
                exit 2
            fi ;;
    esac
    case "$1" in
        --state-dir) STATE=$(realpath -m "$2"); shift 2 ;;
        --unitree-underlay) UNITREE_UNDERLAY=$(realpath "$2"); shift 2 ;;
        --wheelhouse) WHEELHOUSE=$(realpath "$2"); shift 2 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done
case "$MODE" in
    --help)
        cat <<'HELP'
Usage: scripts/deploy_go2.sh MODE [--state-dir DIR] [--unitree-underlay DIR]
                                  [--wheelhouse DIR]
  --dry-run       Read-only plan and source/config validation (default; no ROS needed).
  --check         Check Ubuntu 22.04, Humble, dependencies and Python isolation.
  --install-deps  Explicit apt dependency installation (requires configured ROS apt).
  --build         Create isolated Python env and colcon build, then verify install.
  --validate      Offline Python, C++ and launch checks against that build.
  --help          Show this help.
Build output defaults to .deployment/{build,install,log,venv}. Existing workspace
build/install/log are preserved. A supplied wheelhouse disables pip networking.
Remote installation is a separate command: scripts/deploy_go2_remote.py --help.
HELP
        exit 0 ;;
    --dry-run)
        /usr/bin/python3 -s "$ROOT/scripts/check_deployment.py" --source "$ROOT"
        printf 'PLAN: Ubuntu 22.04 / ROS 2 Humble, native amd64 or arm64\n'
        printf 'PLAN: state=%s\nPLAN: check -> explicit dependencies -> build -> validate\n' "$STATE"
        printf 'PLAN: config=helix_go2.launch.py (isolated topics, sink dry_run, recovery disabled)\n'
        printf 'PLAN: no remote access, node startup, services, or motion\n'
        exit 0 ;;
    --check|--install-deps|--build|--validate) ;;
    *) echo "Unknown mode: $MODE (see --help)" >&2; exit 2 ;;
esac
cd "$ROOT"
. /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
    echo 'Requires Ubuntu 22.04; use a ros:humble-ros-base-jammy development container.' >&2
    exit 1
fi
if [[ "$MODE" == --install-deps ]]; then
    # Never upgrades the OS, JetPack, firmware, DDS source trees or existing services.
    sudo apt-get update
    sudo apt-get install -y --no-install-recommends build-essential cmake \
        python3-venv python3-pip python3-colcon-common-extensions python3-rosdep \
        python3-numpy python3-yaml python3-requests python3-pytest \
        ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp ros-humble-twist-mux \
        ros-humble-rclcpp-lifecycle ros-humble-rclcpp-components \
        ros-humble-rosidl-generator-dds-idl \
        ros-humble-ament-cmake-gtest ros-humble-ament-lint-common
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        sudo rosdep init
    fi
    rosdep update --rosdistro humble
    exit 0
fi
[[ -f /opt/ros/humble/setup.bash ]] || {
    echo 'ROS 2 Humble absent. Configure the official ROS apt repository, then --install-deps.' >&2
    exit 1
}
# Discard inherited overlays and user-site Python packages for reproducible builds.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION VIRTUAL_ENV
export PATH=/usr/bin:/bin:/usr/sbin:/sbin PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
if [[ -n "$UNITREE_UNDERLAY" ]]; then
    [[ -f "$UNITREE_UNDERLAY/local_setup.bash" ]] || { echo 'Invalid Unitree install prefix' >&2; exit 1; }
    source "$UNITREE_UNDERLAY/local_setup.bash"
fi
command -v colcon >/dev/null
command -v rosdep >/dev/null
rosdep check --from-paths src --ignore-src --rosdistro humble
if [[ -d "$ROOT/.deployment/unitree-src/unitree_api" ]]; then
    ros2 pkg prefix rosidl_generator_dds_idl >/dev/null
fi
if [[ "$MODE" == --check ]]; then
    /usr/bin/python3 "$ROOT/scripts/check_deployment.py" --source "$ROOT" --ros
    exit 0
fi
if [[ "$MODE" == --build ]]; then
    mkdir -p "$STATE"
    /usr/bin/python3 -m venv --system-site-packages "$STATE/venv"
    PIP_ARGS=()
    if [[ -z "$WHEELHOUSE" && -d "$ROOT/.deployment/wheels" ]]; then
        WHEELHOUSE="$ROOT/.deployment/wheels"
    fi
    if [[ -n "$WHEELHOUSE" ]]; then PIP_ARGS=(--no-index --find-links "$WHEELHOUSE"); fi
    "$STATE/venv/bin/python" -m pip install "${PIP_ARGS[@]}" -r scripts/requirements-deployment.txt
fi
[[ -f "$STATE/venv/bin/activate" ]] || { echo 'Run --build first' >&2; exit 1; }
source "$STATE/venv/bin/activate"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
if [[ "$MODE" == --build ]]; then
    SOURCES=(src)
    if [[ -d "$ROOT/.deployment/unitree-src/unitree_api" ]]; then
        SOURCES+=("$ROOT/.deployment/unitree-src")
    fi
    python -m colcon --log-base "$STATE/log" build --base-paths "${SOURCES[@]}" \
        --build-base "$STATE/build" --install-base "$STATE/install" \
        --symlink-install --executor sequential \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
        -DPython3_EXECUTABLE="$STATE/venv/bin/python"
fi
source "$STATE/install/local_setup.bash"
python "$ROOT/scripts/check_deployment.py" --source "$ROOT" --ros --installed
mkdir -p "$STATE/evidence"
python -m pip freeze --local > "$STATE/evidence/python-versions.txt"
dpkg-query -W 'ros-humble-*' python3-numpy python3-yaml python3-requests \
    > "$STATE/evidence/apt-versions.txt"
if [[ "$MODE" == --build ]]; then
    # local_setup avoids re-sourcing unrelated build-time overlays.
    {
        printf 'export PYTHONNOUSERSITE=1\nsource /opt/ros/humble/setup.bash\n'
        if [[ -n "$UNITREE_UNDERLAY" ]]; then printf 'source %q/local_setup.bash\n' "$UNITREE_UNDERLAY"; fi
        printf 'source %q/venv/bin/activate\nsource %q/install/local_setup.bash\n' "$STATE" "$STATE"
        printf 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n'
    } > "$STATE/env.bash"
    echo "BUILD VERIFIED. Environment: $STATE/env.bash. No nodes started."
    exit 0
fi
# Explicit offline test isolation; do not inherit a robot-facing DDS configuration.
# Explicit Cyclone loopback binding already isolates transport. Humble's
# ROS_LOCALHOST_ONLY=1 also adds lo and fails with "same interface ... twice".
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=79
export CYCLONEDDS_URI="file://$ROOT/src/helix_bringup/config/cyclonedds_offline.xml" ROS_LOG_DIR="$STATE/log/ros"
python -m pytest tests/ --ignore=tests/sim_integration -q -p pytest_timeout \
    --timeout=120 --junitxml="$STATE/evidence/root.xml"
for pkg in helix_core helix_adapter helix_diagnosis helix_recovery helix_explanation helix_arbiter helix_bringup; do
    python -m pytest "src/$pkg/test" -q -p pytest_timeout --timeout=120 \
        --junitxml="$STATE/evidence/$pkg.xml"
done
python -m colcon --log-base "$STATE/log" test --build-base "$STATE/build" \
    --install-base "$STATE/install" --packages-select helix_sensing_cpp \
    --return-code-on-test-failure
python -m colcon test-result --test-result-base "$STATE/build" --verbose
echo "OFFLINE VALIDATION COMPLETE. See $STATE/evidence and colcon results for pass/skip counts."
