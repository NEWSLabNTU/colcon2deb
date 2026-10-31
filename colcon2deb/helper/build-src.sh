#!/usr/bin/env bash
set -o pipefail
cd "$colcon_work_dir"

# Note: ROS environment should already be set up by /colcon2deb-setup.sh in entry.sh

if [ "$colcon_build" = y ]; then
    # The colcon tree (build/, install/, log/) persists in the output dir
    # between runs, and colcon's incremental build trusts it. That is only
    # sound under the image that produced it: a different builder image can
    # change compilers, flags in the environment (NVCC_APPEND_FLAGS, CUDAARCHS)
    # and system headers without touching any file colcon tracks, and a run
    # killed mid-build under the old image leaves half-generated sources
    # behind (rosidl output whose types are missing, vendor headers that are
    # not installed). Record the image the tree was built under and discard
    # the tree when it changes. Phase 8 already treats the image ID as a
    # fingerprint input; this applies the same rule to phase 4.
    image_stamp="$colcon_work_dir/.colcon2deb-image-id"
    image_id="${COLCON2DEB_IMAGE_ID:-unknown}"
    if [ -d build ] || [ -d install ]; then
        if [ ! -f "$image_stamp" ] || [ "$(cat "$image_stamp")" != "$image_id" ]; then
            echo "colcon tree was built under a different builder image; discarding build/ install/ log/"
            rm -rf build install log
        fi
    fi
    printf '%s\n' "$image_id" > "$image_stamp"

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
