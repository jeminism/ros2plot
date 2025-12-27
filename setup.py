from setuptools import setup, find_packages

setup(
    name='ros2plot',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'setuptools',
        'asciimatics',
        'numpy',
        'attrs',
        'pyyaml',
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
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/yourusername/ros2graph',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)