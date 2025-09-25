from setuptools import find_packages, setup
from glob import glob

package_name = 'mdnuc_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mlarrazabal',
    maintainer_email='mlarrazabal@azti.es',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mdnuc_client = mdnuc_client.mdnuc_client:main',
        ],
    },
)
