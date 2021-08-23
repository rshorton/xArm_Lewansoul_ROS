# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from glob import glob

from setuptools import setup

package_name = "xarm_test_nodes"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.launch.py")),
        ("share/" + package_name + "/configs", glob("configs/*.*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="None Štogl",
    author_email="denis@stogl.de",
    maintainer="None",
    maintainer_email="none@none.com",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Demo nodes for showing and testing xarm robot arm.",
    long_description="""\
Demo nodes for showing and testing functionalities of the xarm robot arm.""",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "publisher_forward_position_controller = \
                xarm_test_nodes.publisher_forward_position_controller:main",
            "publisher_joint_trajectory_controller = \
                xarm_test_nodes.publisher_joint_trajectory_controller:main",
        ],
    },
)
