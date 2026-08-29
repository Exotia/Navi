from setuptools import setup

package_name = 'navi_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='star',
    maintainer_email='oxe.pxs@gmail.com',
    description="Teleoperation bridge nodes for the Asterope rover.",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_twist_listener = navi_teleop.manual_twist_listener:main',
            'video_sender = navi_teleop.video_sender:main',
        ],
    },
)
