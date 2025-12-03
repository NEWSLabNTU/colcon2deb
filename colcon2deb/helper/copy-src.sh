cd "$colcon_work_dir"

if [ "$copy_src" = y ]; then
    echo 'info: copy source files'
    rsync -avP --delete "$workspace_dir/src/" "$colcon_work_dir/src" || {
	echo "error: fail to copy source files" >&2
	exit 1
    }
else
    echo 'info: skip copying source files'
fi
