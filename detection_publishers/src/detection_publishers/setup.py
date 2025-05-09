from setuptools import find_packages, setup

package_name = 'detection_publishers'

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
    maintainer='sarmadahmad8',
    maintainer_email='sarmad.ahmed11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "manual_detection_publisher = detection_publishers.manual_detection_publisher:main",
            "automatic_detection_publisher = detection_publishers.automatic_detection_publisher:main"
        ],
    },
)
