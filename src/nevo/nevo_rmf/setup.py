from glob import glob
import os

from setuptools import setup

package_name = 'nevo_rmf'


def map_files():
    """maps/<building>/... keeping the folder layout; skips old/ and hidden runtime files
    (rmf_traffic_schedule writes .rmf_schedule_node.yaml into its working directory)."""
    out = []
    for root, dirs, files in os.walk('maps'):
        dirs[:] = [d for d in dirs if d != 'old' and not d.startswith('.')]
        files = [os.path.join(root, f) for f in files if not f.startswith('.')]
        if files:
            out.append((os.path.join('share', package_name, root), files))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
    ] + map_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arya Delphi',
    maintainer_email='delphiarya@gmail.com',
    description='Open-RMF fleet adapter for the NEVO Nav2 robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fleet_adapter = nevo_rmf.fleet_adapter:main',
        ],
    },
)
