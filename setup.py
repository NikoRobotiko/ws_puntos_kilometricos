from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puntos_kilometricos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ngromero',
    maintainer_email='ngromero@pa.uc3m.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coord_ppkk_v2 = puntos_kilometricos.coord_ppkk_v2:main',
        ],
    },
)
