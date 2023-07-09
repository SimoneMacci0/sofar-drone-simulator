import os
from glob import glob
from setuptools import setup

package_name = 'sofar_drone_simulator'
lib = 'sofar_drone_simulator/lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, lib],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "resource"), glob("resource/*.png")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simone',
    maintainer_email='simone.maccio2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_sim_node = sofar_drone_simulator.drone_sim_node:main',
            #'drone_controller_node = sofar_drone_simulator.drone_controller:main',
            #'drone_logic_node = sofar_drone_simulator.drone_logic:main'
        ],
    },
)
