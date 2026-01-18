#!/usr/bin/make -f
# -*- makefile -*-
# Sample debian/rules that uses debhelper.
# This file was originally written by Joey Hess and Craig Small.
# As a special exception, when this file is copied by dh-make into a
# dh-make output file, you may use that output file without restriction.
# This special exception was added by Craig Small in version 0.37 of dh-make.

# Use bash for all shell commands (required for sourcing colcon setup.bash)
SHELL := /bin/bash

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
ifneq ($(filter nocheck,$(DEB_BUILD_OPTIONS)),)
	BUILD_TESTING_ARG=-DBUILD_TESTING=OFF
endif

# Extract parallel job count from DEB_BUILD_OPTIONS (e.g., "parallel=4")
# Default to 1 if not specified
ifneq (,$(filter parallel=%,$(DEB_BUILD_OPTIONS)))
	PARALLEL_JOBS := $(patsubst parallel=%,%,$(filter parallel=%,$(DEB_BUILD_OPTIONS)))
else
	PARALLEL_JOBS := 1
endif

DEB_HOST_GNU_TYPE ?= $(shell dpkg-architecture -qDEB_HOST_GNU_TYPE)

# Colcon workspace install path (set by colcon2deb, defaults to empty)
COLCON_INSTALL_PATH ?= @(ColconInstallPath)

# ROS base path (where ROS2 is installed, e.g. /opt/ros/humble)
ROS_BASE_PATH ?= /opt/ros/humble

# Build combined prefix paths as a proper CMake list (semicolon-separated)
# Note: We construct this path carefully to work with cmake's list format
ifneq ($(COLCON_INSTALL_PATH),)
	PATH_COLCON = $(COLCON_INSTALL_PATH)
	PATH_INSTALL = @(InstallationPrefix)
	PATH_ROS = $(ROS_BASE_PATH)
	# Use local_setup.bash to avoid chaining to other prefixes
	SETUP_SCRIPT = $(COLCON_INSTALL_PATH)/local_setup.bash
else
	PATH_COLCON =
	PATH_INSTALL = @(InstallationPrefix)
	PATH_ROS = $(ROS_BASE_PATH)
	SETUP_SCRIPT = @(InstallationPrefix)/local_setup.bash
endif

# Export CMAKE_PREFIX_PATH and AMENT_PREFIX_PATH as environment variables
# These will be inherited by cmake when dh_auto_configure runs
export CMAKE_PREFIX_PATH := $(PATH_COLCON):$(PATH_INSTALL):$(PATH_ROS)
export AMENT_PREFIX_PATH := $(PATH_COLCON):$(PATH_INSTALL):$(PATH_ROS)

%:
	dh $@@ -v --buildsystem=cmake --builddirectory=.obj-$(DEB_HOST_GNU_TYPE)

override_dh_auto_configure:
	# Source the colcon workspace local_setup.bash first to set up CMAKE_PREFIX_PATH
	# with all individual package directories (colcon uses isolated installs).
	# We then PREPEND (not replace) additional paths to preserve the package-level paths.
	source "$(SETUP_SCRIPT)" 2>/dev/null || true && \
	export CMAKE_PREFIX_PATH="$(PATH_INSTALL):$(PATH_ROS):$$CMAKE_PREFIX_PATH" && \
	export AMENT_PREFIX_PATH="$(PATH_INSTALL):$(PATH_ROS):$$AMENT_PREFIX_PATH" && \
	mkdir -p .obj-$(DEB_HOST_GNU_TYPE) && \
	cd .obj-$(DEB_HOST_GNU_TYPE) && \
	cmake .. \
		-DCMAKE_INSTALL_PREFIX=@(InstallationPrefix) \
		-DCMAKE_BUILD_TYPE=None \
		-DCMAKE_VERBOSE_MAKEFILE=ON \
		-DCMAKE_INSTALL_LIBDIR=lib \
		$(BUILD_TESTING_ARG)

override_dh_auto_build:
	# Source setup script for build environment (preserves package-level paths from local_setup.bash)
	source "$(SETUP_SCRIPT)" 2>/dev/null || true && \
	export CMAKE_PREFIX_PATH="$(PATH_INSTALL):$(PATH_ROS):$$CMAKE_PREFIX_PATH" && \
	export AMENT_PREFIX_PATH="$(PATH_INSTALL):$(PATH_ROS):$$AMENT_PREFIX_PATH" && \
	$(MAKE) -j$(PARALLEL_JOBS) -C .obj-$(DEB_HOST_GNU_TYPE)

override_dh_auto_test:
	# Source setup script for test environment (preserves package-level paths from local_setup.bash)
	echo "-- Running tests. Even if one of them fails the build is not canceled."
	source "$(SETUP_SCRIPT)" 2>/dev/null || true && \
	export CMAKE_PREFIX_PATH="$(PATH_INSTALL):$(PATH_ROS):$$CMAKE_PREFIX_PATH" && \
	export AMENT_PREFIX_PATH="$(PATH_INSTALL):$(PATH_ROS):$$AMENT_PREFIX_PATH" && \
	$(MAKE) -C .obj-$(DEB_HOST_GNU_TYPE) test || true

override_dh_shlibdeps:
	# Source setup script for shared library dependency resolution
	# Use --ignore-missing-info to handle libraries not installed via APT (e.g., CUDA from NVIDIA base images)
	source "$(SETUP_SCRIPT)" 2>/dev/null || true; \
	dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info $(EXTRA_LIB_PATHS) -l$(CURDIR)/debian/@(Package)/@(InstallationPrefix)/lib/:$(CURDIR)/debian/@(Package)/@(InstallationPrefix)/opt/@(Name)/lib/

override_dh_auto_install:
	# Source setup script for install environment (preserves package-level paths from local_setup.bash)
	source "$(SETUP_SCRIPT)" 2>/dev/null || true && \
	export CMAKE_PREFIX_PATH="$(PATH_INSTALL):$(PATH_ROS):$$CMAKE_PREFIX_PATH" && \
	export AMENT_PREFIX_PATH="$(PATH_INSTALL):$(PATH_ROS):$$AMENT_PREFIX_PATH" && \
	DESTDIR=$(CURDIR)/debian/@(Package) $(MAKE) -C .obj-$(DEB_HOST_GNU_TYPE) install
