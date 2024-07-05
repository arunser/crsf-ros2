from setuptools import find_packages, setup

package_name = 'crsf_ros2'
submodules = 'crsf_ros2/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arunser',
    maintainer_email='stormbreaker.004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crsf_ros = crsf_ros2.ros2_crsf:main',
        ],
    },
)
