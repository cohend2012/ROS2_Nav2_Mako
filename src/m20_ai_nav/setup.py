from glob import glob
from setuptools import setup
package_name = 'm20_ai_nav'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],          # sim/ (training env) is deliberately NOT installed
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Learned (PPO) local navigator emitting body velocity; 3 Nav2 integration modes.',
    license='MIT',
    entry_points={'console_scripts': ['ai_controller = m20_ai_nav.controller_node:main']},
)
