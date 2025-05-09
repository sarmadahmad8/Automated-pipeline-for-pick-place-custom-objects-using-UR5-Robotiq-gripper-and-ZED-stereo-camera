from setuptools import find_packages, setup
import os
package_name = 'automated_fhr_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [
            'launch/background.launch.py',  # your main launch file
            'launch/active.launch.py',  # new launch file with nodes
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sarmadahmad8',
    maintainer_email='sarmad.ahmed11@gmail.com',
    description='launch all nodes of my pipeline',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
