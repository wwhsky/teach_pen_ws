from setuptools import find_packages, setup

package_name = 'calibration'

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
    maintainer='wwh',
    maintainer_email='wuwenhao@bochu.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'calibrate_vr_to_robot = calibration.calibrate_vr_to_robot:main',
            'calibrate_tool = calibration.calibrate_tool:main',
            'calibrate_tool_vr_joint = calibration.calibrate_tool_vr_joint:main',
            'calibrate_workpiece = calibration.calibrate_workpiece:main',
            'publish_calibration_tf = calibration.publish_calibration_tf:main',
        ],
    },
)
