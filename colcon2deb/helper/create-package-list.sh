cd "$colcon_work_dir"

# Detect Ubuntu codename
ubuntu_codename=$(lsb_release -cs 2>/dev/null || echo "jammy")

colcon info --base-paths src | awk -v codename="$ubuntu_codename" '\
$0 ~ /^  name: / {
  name = $2
  gsub("_", "-", name)
  name = "ros-humble-" name
}

$0 ~ /^    version: / {
  version=$2
  printf "%s=%s-0%s\n", name, version, codename
}
'  > "$deb_pkgs_file"
