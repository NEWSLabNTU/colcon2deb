#!/usr/bin/env bash
set -o pipefail
cd "$colcon_work_dir"

# Note: ROS environment should already be set up by /colcon2deb-setup.sh in entry.sh

if [ "$colcon_build" = y ]; then
    # Build cmake args: always include Release build type
    cmake_args="-DCMAKE_BUILD_TYPE=Release"
    if [ "${COLCON2DEB_SKIP_TESTS:-0}" = "1" ]; then
        cmake_args="$cmake_args -DBUILD_TESTING=OFF"
    fi
    if [ -n "$COLCON2DEB_CMAKE_ARGS" ]; then
        cmake_args="$cmake_args $COLCON2DEB_CMAKE_ARGS"
    fi

    # Determine parallel workers from config or auto-detect
    config_parallel="${COLCON2DEB_PARALLEL_JOBS:-0}"
    if [ "$config_parallel" -gt 0 ]; then
        parallel_workers="$config_parallel"
    else
        parallel_workers="$(nproc)"
    fi

    # Build (stdout+stderr captured by run_script tee to phase log)
    colcon build --base-paths src \
        --parallel-workers "$parallel_workers" \
        --cmake-args $cmake_args \
        --event-handlers console_direct+ \
        2>&1
    build_status=$?

    if [ "$build_status" -ne 0 ]; then
        echo 'error: colcon build failed' >&2
        exit 1
    fi
fi
