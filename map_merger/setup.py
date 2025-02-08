from setuptools import setup
import os
from glob import glob

package_name = 'map_merger'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='balaji',
    maintainer_email='rajalbandi2balaji@gmail.com',
    description='Decentralized map merging for AMRs',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_merger_node = map_merger.decentralized_map_merger:main',
            'map_update_broadcast = map_merger.update_broadcast:main',
            'merge_map_node = map_merger.merge_map:main',
        ],
    },
)