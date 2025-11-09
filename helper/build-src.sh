cd "$colcon_work_dir"
source /opt/ros/humble/setup.bash

if [ "$colcon_build" = y ]; then
    echo 'info: compiling packages (this may take a while...)'

    # Redirect detailed output to log file, show progress only
    colcon build --base-paths src \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_direct+ \
        > "$log_dir/colcon_build.log" 2>&1 || {
        echo 'error: colcon build failed' >&2
        echo "  See log: $log_dir/colcon_build.log" >&2
        tail -n 20 "$log_dir/colcon_build.log" >&2
        exit 1
    }

    echo '  ✓ Compilation complete'
else
    echo 'info: skip compiling packages'
fi
