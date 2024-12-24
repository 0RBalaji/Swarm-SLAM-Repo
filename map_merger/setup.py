from setuptools import find_packages, setup

package_name = 'map_merger'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/MapUpdate.msg']),
        ('share/' + package_name + '/launch', ['launch/mapmerger.launch.py']),
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
            'map_merger_node = map_merger.decentralised_map_merger:main',
            'map_update_broadcast = map_merger.update_broadcast:main',
            'map_merging_node = map_merger.decentralised_mapmerger:main',
        ],
    },
)