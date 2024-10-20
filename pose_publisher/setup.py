from setuptools import find_packages, setup

package_name = 'pose_publisher'

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
    maintainer='root',
    maintainer_email='takahiro-kawai@outlook.com',
    description='Publish specified pose for rviz',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PosePublisher = pose_publisher.PosePublisher:main',
            'PoseArrayPublisher = pose_publisher.PoseArrayPublisher:main',
            'PoseMarkerArrayPublisher = pose_publisher.PoseMarkerArrayPublisher:main'
        ],
    },
)
