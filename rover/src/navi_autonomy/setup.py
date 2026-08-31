import os
from glob import glob

from setuptools import setup

package_name = 'navi_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='star',
    maintainer_email='oxe.pxs@gmail.com',
    description='Tile aggregation and traversability for the Orin.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tile_aggregator = navi_autonomy.tile_aggregator:main',
            'traversability_layer = navi_autonomy.traversability_layer:main',
            'goal_relay = navi_autonomy.goal_relay:main',
        ],
    },
)
