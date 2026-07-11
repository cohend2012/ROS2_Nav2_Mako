from setuptools import setup, find_packages
package_name = 'm20_behaviors'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Behavior Engine for the M20 autonomy stack.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['behavior_engine = m20_behaviors.engine:main']},
)
