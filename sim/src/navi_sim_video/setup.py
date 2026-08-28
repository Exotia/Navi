from setuptools import setup

package_name = "navi_sim_video"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Ole Peters",
    maintainer_email="ole.peters@star-dresden.de",
    description="Streams the simulation's chase camera to the ground station.",
    license="Proprietary",
    entry_points={"console_scripts": [
        "sim_video_sender = navi_sim_video.sender:main"]},
)
