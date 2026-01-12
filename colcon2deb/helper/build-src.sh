cd "$colcon_work_dir"

# Note: ROS environment should already be set up by /colcon2deb-setup.sh in entry.sh

if [ "$colcon_build" = y ]; then
    colcon_log="$log_logs_dir/phase4_colcon_build.log"

    # Build cmake args: always include Release build type
    # Users can add custom args via COLCON2DEB_CMAKE_ARGS environment variable
    cmake_args="-DCMAKE_BUILD_TYPE=Release"
    if [ -n "$COLCON2DEB_CMAKE_ARGS" ]; then
        cmake_args="$cmake_args $COLCON2DEB_CMAKE_ARGS"
    fi

    # Build with output to both terminal and log file
    # Use stdbuf for unbuffered output so TUI shows progress in real-time
    stdbuf -oL colcon build --base-paths src \
        --cmake-args $cmake_args \
        --event-handlers console_direct+ \
        2>&1 | tee "$colcon_log"
    build_status=${PIPESTATUS[0]}

    # Check if colcon build succeeded
    if [ "$build_status" -ne 0 ]; then
        echo 'error: colcon build failed' >&2
        tail -n 20 "$colcon_log" >&2
        exit 1
    fi
fi
