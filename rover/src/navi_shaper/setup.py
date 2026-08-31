from setuptools import setup

package_name = 'navi_shaper'

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
    description="Feasibility clamp between /rover_twist and the chassis.",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_shaper = navi_shaper.twist_shaper:main',
        ],
    },
)
