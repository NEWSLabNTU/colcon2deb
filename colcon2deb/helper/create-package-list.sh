#!/usr/bin/env bash
set -eo pipefail
cd "$colcon_work_dir"

# Detect Ubuntu codename
ubuntu_codename=$(lsb_release -cs 2>/dev/null || echo "jammy")

# Deb name: ros-<distro>-<name-dashed>[-<suffix>]=<version>-0<codename>
colcon info --base-paths src | awk \
    -v codename="$ubuntu_codename" \
    -v distro="${ROS_DISTRO:-humble}" \
    -v suffix="${ROS_PACKAGE_SUFFIX:-}" '
$0 ~ /^  name: / {
  name = $2
  gsub("_", "-", name)
  name = "ros-" distro "-" name
  if (suffix != "") {
    name = name "-" suffix
  }
}

$0 ~ /^    version: / {
  version=$2
  printf "%s=%s-0%s\n", name, version, codename
}
'  > "$deb_pkgs_file"
