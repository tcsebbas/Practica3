from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'practica3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/practica3/msg', glob('msg/*.msg')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tcsebbas',
    maintainer_email='carlossebastianterrerocastillo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navegar   = practica3.navegar:main',
            'dibujar   = practica3.dibujar:main',
        ],
    },
)
