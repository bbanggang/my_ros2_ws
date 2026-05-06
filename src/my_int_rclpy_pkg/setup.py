from setuptools import find_packages, setup

package_name = 'my_int_rclpy_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linux',
    maintainer_email='hungue6559@naver.com',
    description='ROS 2 Python package that publishes and subscribes integer values',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'int_pub = my_int_rclpy_pkg.int_publisher:main',
            'int_sub = my_int_rclpy_pkg.int_subscriber:main',
        ],
    },
)
