from setuptools import find_packages, setup

package_name = 'nuc_client'

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
    maintainer='mlarrazabal',
    maintainer_email='mlarrazabal@azti.es',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nuc_client = nuc_client.nuc_client:main',
            'path_reader = nuc_client.path_reader:main'
        ],
    },
)
