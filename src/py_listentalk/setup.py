from setuptools import setup

package_name = 'py_listentalk'

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
    maintainer='imię',
    maintainer_email='email@email.pl',
    description='krótki opis działania',
    license='rodzaj licencji',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_listentalk.publisher_member_function:main',
        ],
    },
)
