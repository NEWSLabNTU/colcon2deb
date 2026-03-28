from setuptools import setup

package_name = 'test_py_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Test Maintainer',
    maintainer_email='test@example.com',
    description='Test Python package for colcon2deb',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_subscriber = test_py_pkg.test_subscriber:main'
        ],
    },
)
