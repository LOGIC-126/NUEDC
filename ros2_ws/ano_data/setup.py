from setuptools import find_packages, setup

package_name = 'ano_data'

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
    maintainer='ilogic',
    maintainer_email='j15092924790@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_ano_data = ano_data.ANO_Data:main',
            'node_GimbalNode_Ctrl = ano_data.Gimbal:main',
            'node_pose_listener = ano_data.pose_listener:main',
            'node_fly_mission = ano_data.mission:main',
            'node_rknn_link = ano_data.rknn_link:main',
            'node_normal_fly = ano_data.normal_mode:main',
            'node_imu_camera_cal = ano_data.cam_to_imu:main'
        ],
    },
)
