from setuptools import setup, find_packages

package_name = 'ros2plot'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'setuptools',
        'asciimatics',
        'numpy',
        'attrs',
        'pyyaml',
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
    author='Your Name',
    author_email='your.email@example.com',
    description='Terminal-based real-time plotting tool for ROS2 topics',
    url='https://github.com/yourusername/ros2graph',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)