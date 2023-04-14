from setuptools import setup

package_name = 'bachelor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andma20',
    maintainer_email='andma20@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'webcam_pub = bachelor.webcam_pub:main',
        'camera_node = bachelor.camera_node:main',
        'system_node = bachelor.system_node:main',
        ],
    },
)
