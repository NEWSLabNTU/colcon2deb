cd "$colcon_work_dir"

if [ "$copy_src" = y ]; then
    rsync -a --delete "$workspace_dir/src/" "$colcon_work_dir/src" || {
	echo "error: fail to copy source files" >&2
	exit 1
    }
fi
