from setuptools import setup

package_name = 'obstacle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrblack',
    maintainer_email='ahmedharbii10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom = obstacle.odom_sub:main',
            'lidar = obstacle.lidar_sub:main',
            'vel = obstacle.vel_pub:main',
            'avoidance = obstacle.avoidance:main',
        ],
    },
)

#'talker = py_odom.publisher_member_function:main',