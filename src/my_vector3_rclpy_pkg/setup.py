from setuptools import find_packages, setup

package_name = 'my_vector3_rclpy_pkg'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vector3_pub = my_vector3_rclpy_pkg.vector3_publisher:main',
            'vector3_sub = my_vector3_rclpy_pkg.vector3_subscriber:main',
        ],
    },
)
