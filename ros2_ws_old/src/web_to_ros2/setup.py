from setuptools import setup

package_name = 'web_to_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['index.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='HTTP to ROS2 publisher server',
    license='MIT',
    entry_points={
        'console_scripts': [
            'talker_server = web_to_ros2.talker_server:main'
        ],
    },
)
