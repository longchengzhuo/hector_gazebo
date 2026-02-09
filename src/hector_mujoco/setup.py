from setuptools import find_packages, setup
from glob import glob

package_name = 'hector_mujoco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/assets/hector_v2', glob('hector_mujoco/assets/hector_v2/*.urdf')),  # Install URDF files
        ('share/' + package_name + '/assets/hector_v2/meshes', glob('hector_mujoco/assets/hector_v2/meshes/*')),  # Install mesh files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hector',
    maintainer_email='hector@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hector_mujoco_node = hector_mujoco.src.ros_node:main',
        ],
    },
)
