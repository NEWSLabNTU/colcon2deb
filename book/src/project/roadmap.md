# Project Roadmap

## Current Status

Version 0.2.0 has been released with major improvements:
- Migration from Rye to uv for package management
- Automated Debian package building
- Remote Dockerfile support
- Streamlined examples without submodules

## Completed Phases

### Phase 1: Critical Bug Fixes ✅

All critical issues have been resolved:
- Fixed missing shebangs in scripts
- Resolved race conditions in build process
- Fixed rosdep update conflicts
- Corrected error handling throughout

### Phase 2: Code Quality ✅

Code quality improvements implemented:
- Removed debug code
- Improved quoting consistency
- Optimized parallel job allocation
- Added comprehensive documentation

## Active Development

### Phase 3: Architecture Improvements

Currently working on:

#### Robustness Enhancements
- [ ] **Dependency validation** - Pre-flight checks before build
- [ ] **Configuration validation** - YAML schema validation
- [ ] **Error recovery** - Resume partial builds
- [ ] **Build caching** - Incremental build support

#### Testing Infrastructure
- [ ] **Unit tests** - Test individual components
- [ ] **Integration tests** - End-to-end testing
- [ ] **Performance benchmarks** - Track build metrics
- [ ] **CI/CD pipeline** - Automated testing

## Upcoming Features

### Phase 4: Feature Enhancements

#### Multi-Architecture Support
- [ ] Cross-compilation support
- [ ] QEMU integration for emulation
- [ ] Multi-arch Docker manifests
- [ ] Native ARM64 builds

#### Incremental Builds
- [ ] Change detection
- [ ] Dependency tracking
- [ ] Partial rebuilds
- [ ] Cache management

#### Build Profiles
- [ ] Debug/Release configurations
- [ ] Minimal builds
- [ ] Custom CMake arguments
- [ ] Optimization levels

### Phase 5: Repository Management

#### APT Repository Creation
- [ ] Repository generation tools
- [ ] Package signing
- [ ] Metadata generation
- [ ] Mirror support

#### Version Management
- [ ] Automatic versioning
- [ ] Changelog generation
- [ ] Git tag integration
- [ ] Release automation

### Phase 6: Cloud Integration

#### Container Registry Support
- [ ] Push images to registries
- [ ] Multi-stage builds
- [ ] Layer optimization
- [ ] Security scanning

#### CI/CD Integration
- [ ] GitHub Actions workflows
- [ ] GitLab CI templates
- [ ] Jenkins pipelines
- [ ] Build status badges

## Long-term Goals

### Performance Optimization
- Distributed builds across multiple machines
- Build farm support
- Caching service integration
- Parallel Docker builds

### Platform Expansion
- Windows WSL2 support
- macOS Docker Desktop support
- Podman compatibility
- Kubernetes job support

### Developer Experience
- Web UI for build monitoring
- VSCode extension
- Real-time build progress
- Build analytics dashboard

### Community Features
- Public package repository
- Build service hosting
- Package search interface
- Dependency visualization

## Contributing

### How to Contribute

1. **Report Issues**: File bugs on GitHub
2. **Submit PRs**: Follow development guide
3. **Test Features**: Try development builds
4. **Documentation**: Improve guides and examples

### Priority Areas

High-impact contributions needed:
- Test coverage improvement
- ARM64 testing
- Performance optimization
- Documentation translation

### Development Process

1. Discussion in GitHub issues
2. Design proposal if needed
3. Implementation with tests
4. Code review and iteration
5. Documentation update
6. Release inclusion

## Release Schedule

### Version 0.3.0 (Q2 2024)
- Testing infrastructure
- Configuration validation
- Error recovery improvements

### Version 0.4.0 (Q3 2024)
- Multi-architecture support
- Incremental builds
- Build profiles

### Version 0.5.0 (Q4 2024)
- APT repository tools
- Cloud integration
- Performance optimizations

### Version 1.0.0 (2025)
- Production ready
- Stable API
- Complete documentation
- Enterprise features

## Breaking Changes

### Planned Deprecations

Future versions may deprecate:
- Direct script invocation (use CLI only)
- Old configuration format
- Python 3.10 support (move to 3.11+)

### Migration Guides

Guides will be provided for:
- Configuration format changes
- API updates
- Docker image changes
- Script interface changes

## Support Lifecycle

### Version Support

| Version | Status | Support Until |
|---------|--------|--------------|
| 0.1.x | EOL | December 2023 |
| 0.2.x | Active | December 2024 |
| 0.3.x | Planned | December 2025 |
| 1.0.x | Future | LTS |

### Compatibility Matrix

| colcon2deb | Ubuntu | ROS 2 | Docker |
|------------|--------|-------|--------|
| 0.2.x | 22.04 | Humble | 20.10+ |
| 0.3.x | 22.04/24.04 | Humble/Iron | 20.10+ |
| 1.0.x | 24.04 | Iron/Jazzy | 24.0+ |

## Feedback

### Contact Channels

- GitHub Issues: Bug reports and features
- Discussions: General questions
- Discord: Real-time chat
- Email: contact@autoware.org

### User Survey

Help shape the roadmap:
- What features do you need?
- What problems do you face?
- How can we improve?

Submit feedback through GitHub Discussions or our user survey.