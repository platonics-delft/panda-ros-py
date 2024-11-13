from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

# Generate ROS-specific setup configuration
ros_setup_args = generate_distutils_setup(
    packages=find_packages(),
    package_dir={'': '.'}
)

# Merge additional setuptools-specific configuration
setuptools_args = dict(
    name='panda_ros',
    version='0.2.0',
    description='Panda function in Python for ROS',
    author='Giovanni Franzese',
    author_email='g.franzese@tudelft.nl',
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ],
)

# Combine both configurations, allowing ROS-specific settings to override
combined_args = {**setuptools_args, **ros_setup_args}

# Use the combined arguments for the setup function
setup(**combined_args)

