# Generate a rosdep file to include packages and perform `rosdep
# update`. The step is necessary for later bloom-generate.
cd "$colcon_work_dir"

rosdep_yaml_file="$top_work_dir/rosdep.yaml"
rosdep_list_file="/etc/ros/rosdep/sources.list.d/99-autoware.list"

if [ "$gen_rosdep_list" = y ]; then
    colcon list --base-paths src | cut -f1 | \
	sort | \
	while read pkg_name; do
	    pkg_name_dashed="${pkg_name//_/-}"
	    key="ros-${ROS_DISTRO}-${pkg_name_dashed}"
	    echo "\
${pkg_name}:
  ubuntu: [${key}]"
	done > "$rosdep_yaml_file"

    sudo sh -c "echo 'yaml file://${rosdep_yaml_file}' > '$rosdep_list_file'"
    # Update rosdep to load the newly created custom package list
    rosdep update > /dev/null
fi
