from setuptools import setup

package_name = 'depth2grid'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/depth2grid_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='igvc',
    maintainer_email='your_email@domain.com',
    description='Converts point cloud to occupancy grid',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth2grid_node = depth2grid.pointcloud_to_grid:main'
        ],
    },
)
