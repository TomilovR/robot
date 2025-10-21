from setuptools import find_packages, setup

package_name = 'turtle_tf_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtle_tf_follow.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roman',
    maintainer_email='thfjfjfjdjfh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'carrot_tf_broadcaster = turtle_tf_follow.carrot_tf_broadcaster:main',
            'follower = turtle_tf_follow.follower:main',
            'turtlesim_tf_broadcaster = turtle_tf_follow.turtlesim_tf_broadcaster:main',
        ],
    },
)
