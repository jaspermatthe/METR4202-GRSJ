from setuptools import find_packages, setup

package_name = 'map_explorer'

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
    maintainer='jasper',
    maintainer_email='matthejasper@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_cycler = map_explorer.waypoint_cycler:main',
            'ramtintest = map_explorer.ramtintest:main',
            'map_explorer = map_explorer.map_explorer:main'
        ],
    },
)
