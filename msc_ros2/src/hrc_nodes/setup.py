from setuptools import setup
from glob import glob
setup(name='hrc_nodes', version='0.1.0', packages=['hrc_nodes'],
 data_files=[('share/ament_index/resource_index/packages',['resource/hrc_nodes']),
 ('share/hrc_nodes',['package.xml']),('share/hrc_nodes/launch',glob('launch/*.py')),
 ('share/hrc_nodes/config',glob('config/*.yaml'))],
 entry_points={'console_scripts':[
 'stvl53l7cx_driver = hrc_nodes.driver:main',
 'human_detection = hrc_nodes.human_detection:main',
 'sensor_health = hrc_nodes.health:main',
 'speed_scaling = hrc_nodes.speed_scaling:main',
 'decision_making = hrc_nodes.decision:main',
 'led_lighting = hrc_nodes.led:main']})
