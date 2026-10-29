from setuptools import find_packages, setup

package_name = 'orbit_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kikifachrizi',
    maintainer_email='kikifachrizi235@gmail.com',
    description='Fleet adapter RMF ORBIT dengan backend per robot (ADR-002)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fleet_adapter = orbit_adapter.fleet_adapter:main',
            'dispatch_task = orbit_adapter.dispatch_task:main',
        ],
    },
)
