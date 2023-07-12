import glob

from setuptools import setup

package_name = "lbr_fri_ros2_advanced_python_demos"

setup(
    name=package_name,
    version="1.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.py")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.rviz")),        
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="martin.huber@kcl.ac.uk",
    description="Advanced Python demos for the lbr_fri_ros2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "admittance_control_node = lbr_fri_ros2_advanced_python_demos.admittance_control_node:main",
            "joint_state_remap = lbr_fri_ros2_advanced_python_demos.joint_state_remap:main",
            "visualization = lbr_fri_ros2_advanced_python_demos.visualization:main",
            "fake_lbr = lbr_fri_ros2_advanced_python_demos.fake_lbr:main",
        ],
    },
)
