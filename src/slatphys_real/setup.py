from glob import glob

from setuptools import setup

package_name = "slatphys_real"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Real UR3e + Hand-E probing for the SLAT-Phys real2sim experiment",
    license="MIT",
    entry_points={
        "console_scripts": [
            "probe = slatphys_real.probe:main",
            "place = slatphys_real.place:main",
            "capture = slatphys_real.capture:main",
            "calibrate_camera = slatphys_real.calibrate_camera:main",
            "calibrate_gripper = slatphys_real.calibrate_gripper:main",
            "run_tasks = slatphys_real.run_tasks:main",
            "answer = slatphys_real.tasks:main",
        ],
    },
)
