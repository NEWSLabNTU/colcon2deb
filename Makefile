.PHONY: default prepare build save tarball clean deb

IMAGE_NAME = autoware_rosdebian_builder
VERSION = 0.2.0

default:
	@echo 'Usage:'
	@echo
	@echo 'make prepare'
	@echo '    Install dependent pakcages and configure the'
	@echo '    environment to build the container image.'
	@echo
	@echo 'make build'
	@echo '    Build the container image.'
	@echo
	@echo 'make run'
	@echo '    Enter the container shell.'
	@echo
	@echo 'make save'
	@echo '    Save a Docker image file.'
	@echo
	@echo 'make tarball'
	@echo '    Create a source tarball for packaging.'
	@echo
	@echo 'make deb'
	@echo '    Create a Debian package using makedeb.'

wheel:
	@echo "Building Python wheel package..."
	@uv build --wheel
	@echo "Wheel package created in dist/"

clean:
	@rm -rf pkg/
	@rm -rf src/
	@rm -rf dist/
	@rm -f *.deb
	@rm -f *.tar.gz
	@echo "Cleaned up build artifacts"

setup-apt:
	@echo "Setting up apt_pkg in virtual environment..."
	@if [ -d .venv/lib/python3.10/site-packages ]; then \
		ln -sf /usr/lib/python3/dist-packages/apt_pkg.cpython-310-x86_64-linux-gnu.so .venv/lib/python3.10/site-packages/ 2>/dev/null || true; \
		ln -sf /usr/lib/python3/dist-packages/apt_inst.cpython-310-x86_64-linux-gnu.so .venv/lib/python3.10/site-packages/ 2>/dev/null || true; \
		ln -sf /usr/lib/python3/dist-packages/apt .venv/lib/python3.10/site-packages/ 2>/dev/null || true; \
	fi

deb: wheel setup-apt
	@echo "Building Debian package with makedeb..."
	@rm -f makedeb/*.deb makedeb/*.whl
	@cp dist/colcon2deb-$(VERSION)-py3-none-any.whl makedeb/
	@cd makedeb && makedeb
	@mkdir -p dist
	@cp makedeb/colcon2deb_$(VERSION)-1_all.deb dist/
	@echo "Debian package created successfully!"
