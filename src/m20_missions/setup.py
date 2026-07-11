from setuptools import setup
package_name = 'm20_missions'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Mission server and AI agent interface. Requests goals; Commander can veto.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['mission_server = m20_missions.mission_server:main']},
)
