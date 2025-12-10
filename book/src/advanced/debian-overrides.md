# Debian Overrides

## When Needed

Override auto-generated Debian metadata when:
- Bloom fails to detect dependencies correctly
- Custom build rules are required
- Package needs specific install scripts

## Directory Structure

```
debian-overrides/
└── my_package/
    └── debian/
        ├── control      # Package metadata
        ├── rules        # Build rules
        └── changelog    # Version history
```

Reference in config.yaml:

```yaml
packages:
  directory: ./debian-overrides
```

## Example Files

### control

```
Source: my-package
Section: misc
Priority: optional
Maintainer: Your Name <you@example.com>
Build-Depends: debhelper (>= 9), ros-humble-rclcpp

Package: ros-humble-my-package
Architecture: any
Depends: ${shlibs:Depends}, ${misc:Depends}, ros-humble-rclcpp
Description: My ROS 2 package
 A brief description of what this package does.
```

### rules

```makefile
#!/usr/bin/make -f

%:
	dh $@

override_dh_auto_configure:
	# Custom configuration if needed

override_dh_auto_build:
	# Custom build steps if needed
```

### changelog

```
my-package (1.0.0-1) unstable; urgency=low

  * Initial release

 -- Your Name <you@example.com>  Mon, 01 Jan 2024 00:00:00 +0000
```

## Real Example

See `examples/*/debian-overrides/` for working examples from Autoware packages.
