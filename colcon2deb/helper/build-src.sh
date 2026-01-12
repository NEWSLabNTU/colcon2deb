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

    # Redirect detailed output to log file
    colcon build --base-paths src \
        --cmake-args $cmake_args \
        --event-handlers console_direct+ \
        > "$colcon_log" 2>&1 || {
        echo 'error: colcon build failed' >&2
        tail -n 20 "$colcon_log" >&2
        exit 1
    }
fi
