import os
from glob import glob

from setuptools import setup

package_name = 'navi_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='star',
    maintainer_email='oxe.pxs@gmail.com',
    description='Rover pose and localisation health from the front ZED 2i.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_status = navi_localization.localization_status:main',
            'elevation_mapper = navi_localization.elevation_mapper:main',
            'site_anchor = navi_localization.site_anchor:main',
            'site_probe = navi_localization.site_probe:main',
        ],
    },
)
