from setuptools import setup
from setuptools import find_packages

package_name = 'zoedepth'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=["test"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'zoedepth @ git+https://github.com/isl-org/ZoeDepth.git'
    ],
    zip_safe=True,
    maintainer='Victor Talpaert',
    maintainer_email='victor.talpaert@gmail.com',
    description='ROS2 package for monocular depth estimation using ZoeDepth',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_estimator = zoedepth.depth_estimator_node:main',
            'webcam_publisher = zoedepth.webcam_publisher_node:main',
        ],
    },
)
