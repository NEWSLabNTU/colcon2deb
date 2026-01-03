#!/usr/bin/make -f
# -*- makefile -*-
# Sample debian/rules that uses debhelper.
# This file was originally written by Joey Hess and Craig Small.
# As a special exception, when this file is copied by dh-make into a
# dh-make output file, you may use that output file without restriction.
# This special exception was added by Craig Small in version 0.37 of dh-make.

# Uncomment this to turn on verbose mode.
export DH_VERBOSE=1
# TODO: remove the LDFLAGS override.  It's here to avoid esoteric problems
# of this sort:
#  https://code.ros.org/trac/ros/ticket/2977
#  https://code.ros.org/trac/ros/ticket/3842
export LDFLAGS=
export PKG_CONFIG_PATH=@(InstallationPrefix)/lib/pkgconfig
# Explicitly enable -DNDEBUG, see:
# 	https://github.com/ros-infrastructure/bloom/issues/327
export DEB_CXXFLAGS_MAINT_APPEND=-DNDEBUG

# Python package installation variables
export PYBUILD_INSTALL_ARGS=--prefix "@(InstallationPrefix)" \
	--install-lib "\$$base/lib/{interpreter}/site-packages" \
	@[ if pass_install_scripts ] --install-scripts "\$$base/bin" @[end if]

DEB_HOST_GNU_TYPE ?= $(shell dpkg-architecture -qDEB_HOST_GNU_TYPE)

%:
	dh $@@ -v --buildsystem=pybuild --with python3 --builddirectory=.obj-$(DEB_HOST_GNU_TYPE)

override_dh_auto_configure:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree and source it.  It will set things like
	# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	if [ -f "@(InstallationPrefix)/setup.sh" ]; then . "@(InstallationPrefix)/setup.sh"; fi && \
	dh_auto_configure

override_dh_auto_build:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree and source it.  It will set things like
	# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	if [ -f "@(InstallationPrefix)/setup.sh" ]; then . "@(InstallationPrefix)/setup.sh"; fi && \
	dh_auto_build

override_dh_auto_test:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree and source it.  It will set things like
	# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	echo -- Running tests. Even if one of them fails the build is not canceled.
	if [ -f "@(InstallationPrefix)/setup.sh" ]; then . "@(InstallationPrefix)/setup.sh"; fi && \
	dh_auto_test || true

override_dh_shlibdeps:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree and source it.  It will set things like
	# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	# Use --ignore-missing-info to handle libraries not installed via APT (e.g., CUDA from NVIDIA base images)
	if [ -f "@(InstallationPrefix)/setup.sh" ]; then . "@(InstallationPrefix)/setup.sh"; fi && \
	dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info -l$(CURDIR)/debian/@(Package)/@(InstallationPrefix)/lib/

override_dh_auto_install:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree and source it.  It will set things like
	# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	if [ -f "@(InstallationPrefix)/setup.sh" ]; then . "@(InstallationPrefix)/setup.sh"; fi && \
	dh_auto_install
	# Generate ament environment hooks for Python packages
	# These are normally created by colcon but pybuild doesn't do this
	# Get the Python version (e.g., python3.10)
	PYTHON_INTERP=$$(python3 -c "import sys; print(f'python{sys.version_info.major}.{sys.version_info.minor}')") && \
	PYTHON_SITEPKG_REL="lib/$$PYTHON_INTERP/site-packages" && \
	SHARE_DIR=$(CURDIR)/debian/@(Package)@(InstallationPrefix)/share/@(Name) && \
	HOOK_DIR=$$SHARE_DIR/hook && \
	mkdir -p $$HOOK_DIR && \
	echo "prepend-non-duplicate;PYTHONPATH;$$PYTHON_SITEPKG_REL" > $$HOOK_DIR/pythonpath.dsv && \
	printf '%s\n' '# generated from ament_package' "ament_prepend_unique_value PYTHONPATH \"@(InstallationPrefix)/$$PYTHON_SITEPKG_REL\"" > $$HOOK_DIR/pythonpath.sh && \
	echo "prepend-non-duplicate;AMENT_PREFIX_PATH;" > $$HOOK_DIR/ament_prefix_path.dsv && \
	printf '%s\n' '# generated from ament_package' 'ament_prepend_unique_value AMENT_PREFIX_PATH "@(InstallationPrefix)"' > $$HOOK_DIR/ament_prefix_path.sh && \
	printf '%s\n' "source;share/@(Name)/hook/pythonpath.dsv" "source;share/@(Name)/hook/ament_prefix_path.dsv" > $$SHARE_DIR/local_setup.dsv && \
	echo "source;share/@(Name)/local_setup.dsv" > $$SHARE_DIR/package.dsv && \
	printf '%s\n' '# generated from ament_package' 'source "$$(dirname "$${BASH_SOURCE[0]}")/local_setup.sh"' > $$SHARE_DIR/local_setup.bash && \
	printf '%s\n' '# generated from ament_package' 'source "$${0:a:h}/local_setup.sh"' > $$SHARE_DIR/local_setup.zsh && \
	printf '%s\n' '# generated from ament_package' 'ament_prepend_unique_value() {' '  local var_name="$$1"' '  local value="$$2"' '  eval "local current_value=\"\$$$$var_name\""' '  if [ -z "$$current_value" ]; then' '    eval "export $$var_name=\"$$value\""' '  elif [[ ":$$current_value:" != *":$$value:"* ]]; then' '    eval "export $$var_name=\"$$value:$$current_value\""' '  fi' '}' "source \"@(InstallationPrefix)/share/@(Name)/hook/ament_prefix_path.sh\"" "source \"@(InstallationPrefix)/share/@(Name)/hook/pythonpath.sh\"" > $$SHARE_DIR/local_setup.sh
