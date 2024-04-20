import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'slam_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*launch.[pxy][yma]*"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config/*.yaml"))),
        (os.path.join("share", package_name, "map"), glob(os.path.join("map/*"))),
        (os.path.join("share", package_name, "rviz"), glob(os.path.join("rviz/*.rviz"))),
        # copy the bag directories while maintaining the directory structure
        *[(os.path.join("share", package_name, d), glob(os.path.join(d, "*"))) for d in glob("bag/*/", recursive=True)]
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='christiaan',
    maintainer_email='christiaantheunisse@hotmail.nl',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
