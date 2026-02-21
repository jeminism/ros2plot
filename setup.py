from setuptools import setup, find_packages

package_name = 'ros2plot'
setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(),
    install_requires=[
        "asciimatics>=1.15,<2.0",       # latest 1.x series; 2.x not yet released
        "numpy>=1.23,<2.0",             # 1.23+ works on py3.10–3.12; avoid 2.x unless needed
        "attrs>=21.4,<26.0",            # modern attrs for 3.10–3.12
        "PyYAML>=6.0,<7.0",             # latest stable, compatible with 3.10–3.12
        "psutil>=5.8,<8.0",
    ],
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'ros2plot = ros2plot.main:main',
        ],
    },
    author='Jeremy Tan',
    description='Terminal-based real-time plotting tool for ROS2 topics',
    url='https://github.com/jeminism/ros2plot',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License'
    ],
    python_requires='>=3.6',
)