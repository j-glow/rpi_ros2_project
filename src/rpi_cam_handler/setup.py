from setuptools import setup

package_name = 'rpi_cam_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qbaaa-0',
    maintainer_email='ntfr123@gmail.com',
    description='recieve image message from rpi camera and convert it to opencv, then find aruco markers',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpi_cam_aruco = rpi_cam_handler.rpi_cam_aruco:main'
        ],
    },
)
