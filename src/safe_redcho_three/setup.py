from glob import glob
from setuptools import setup

setup(name='safe_redcho_three', version='0.1.0', packages=['safe_redcho_three'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/safe_redcho_three']),
                  ('share/safe_redcho_three', ['package.xml']),
                  ('share/safe_redcho_three/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'], zip_safe=True,
      maintainer='Safe REDCHO project', maintainer_email='maintainer@example.invalid',
      description='Three-agent dual REDCHO + reciprocal HOCBF-QP reference governor for Crazyflie commissioning',
      license='MIT',
      entry_points={'console_scripts': [
          'redcho_capture = safe_redcho_three.capture:main',
          'redcho_agent = safe_redcho_three.agent:main',
          'redcho_mission = safe_redcho_three.mission:main',
          'redcho_watchdog = safe_redcho_three.watchdog:main',
      ]})
