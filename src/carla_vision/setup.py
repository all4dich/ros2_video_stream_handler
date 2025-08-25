from setuptools import find_packages, setup

package_name = 'carla_vision'

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
    maintainer='Sunjoo Park',
    maintainer_email='all4dich@gmail.com',
    description='Carla Vision Example to get traffic light camera images',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_get_video = carla_vision.main_get_video:main'
        ],
    },
)
