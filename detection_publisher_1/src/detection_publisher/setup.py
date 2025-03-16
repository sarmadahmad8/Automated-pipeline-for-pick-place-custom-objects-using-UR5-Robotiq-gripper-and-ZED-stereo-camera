from setuptools import find_packages, setup

package_name = 'detection_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'inference-gpu==0.16.3',
        'numpy==1.26.4',
        'cython==3.0.0',
        'pyzed==4.2'
    ],
    zip_safe=True,
    maintainer='sarmadahmad8',
    maintainer_email='sarmadahmad8@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "move_group_publisher = detection_publisher.move_group_publisher:main"
        ],
    },
)
