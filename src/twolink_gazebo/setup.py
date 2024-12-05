from setuptools import setup

package_name = 'twolink_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'src.joint_publisher',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Action example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_publisher = twolink_gazebo.joint_publisher:main',
            'dummy_joint_publisher = twolink_gazebo.dummy_joint_publisher:main',
        ],
    },
)
