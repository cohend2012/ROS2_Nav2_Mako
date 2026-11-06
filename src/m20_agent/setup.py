from glob import glob
from setuptools import setup
package_name = 'm20_agent'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='M20-Brain L2/L3: landmark memory + VLM mission agent (Claude / Cosmos / offline).',
    license='MIT',
    entry_points={'console_scripts': ['agent = m20_agent.agent_node:main']},
)
