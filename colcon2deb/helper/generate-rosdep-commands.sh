#!/usr/bin/env bash
set -eo pipefail

cd "$colcon_work_dir"
# Temporarily unset -u for ROS setup script which references undefined variables
set +u
source "/opt/ros/humble/setup.bash"
set -u

echo "info: collecting rosdep keys from packages" >&2
keys_count=$(rosdep keys --from-paths src --ignore-src 2>/dev/null | wc -l || echo 0)
echo "info: found $keys_count dependency keys to resolve" >&2

if [ "$keys_count" -eq 0 ]; then
    echo "warning: no rosdep keys found, checking if src directory exists" >&2
    if [ ! -d src ]; then
        echo "error: src directory not found at $colcon_work_dir/src" >&2
        exit 1
    fi
    ls -la src/ | head -10 >&2
fi

rosdep keys --from-paths src --ignore-src 2>/dev/null | \
    sort -u | \
    while read -r key; do
	# Note: we redirect stderr to /dev/null per key, not for the whole pipeline
	echo "rosdep resolve \"$key\" 2>/dev/null || echo \"warning: failed to resolve $key\" >&2"
    done | \
	parallel --group 2>/dev/null | \
	awk '\
BEGIN {
  mode = ""
  n_apt = 0
  n_pip = 0
}

/^#apt$/ { mode = "apt" }
/^#pip$/ { mode = "pip" }

/^[^#]/ {
  if (mode == "apt") {
    apt[n_apt] = $0
    n_apt += 1
  } else if (mode == "pip") {
    pip[n_pip] = $0
    n_pip += 1
  } else {
    print "Error: Expect #apt or #pip before a key" > "/dev/stderr"
    exit 1
  }
}

END {
  print "#!/usr/bin/env bash"
  print "set -e  # Exit on error"

  if (n_apt > 0) {
    printf "echo \"Installing %d APT packages...\" >&2\n", n_apt
    printf "sudo apt install -y"
    for (i = 0; i < n_apt; i += 1) {
      printf " %s", apt[i]
    }
    printf "\n"
  }

  if (n_pip > 0) {
    printf "echo \"Installing %d pip packages...\" >&2\n", n_pip
    printf "pip install -U"
    for (i = 0; i < n_pip; i += 1) {
          printf " %s", pip[i]
    }
    printf "\n"
  }

  if (n_apt == 0 && n_pip == 0) {
    print "echo \"No dependencies to install\" >&2"
  }
}
'
