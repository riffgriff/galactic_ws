from setuptools import find_packages, setup

package_name = 'galactic_navigate_to_goal'

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
    maintainer='riffgriff',
    maintainer_email='griffinmartin02@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'go_to_goal=galactic_navigate_to_goal.go_to_goal:main',
            'drive_carefully=galactic_navigate_to_goal.drive_carefully:main',
            'get_object_range=galactic_navigate_to_goal.get_object_range:main',
        ],
    },
)
