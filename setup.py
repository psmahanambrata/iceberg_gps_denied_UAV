from setuptools import find_packages, setup

package_name = 'gps_denied_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['gps_denied_navigation/resource/gps_denied_navigation']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planner.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project Maintainer',
    maintainer_email='maintainer@example.com',
    description='Grid-based path planning scaffold for GPS-denied drone navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['astar_planner = gps_denied_navigation.astar_planner:main']},
)
