from setuptools import setup
import os
from glob import glob

package_name = 'simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'worlds/shihou_world'), glob(os.path.join('worlds', 'shihou_world', '*.model'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.sdf'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.config'))),
        (os.path.join('share', package_name, 'models/meshes'), glob(os.path.join('models', 'meshes', '*.dae'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='masayaokada',
    maintainer_email='masayaokada@jp.honda',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_publisher=simulator.line_publish:main',
        ],
    },
)
