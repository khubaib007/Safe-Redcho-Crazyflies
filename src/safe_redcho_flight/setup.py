from glob import glob
from setuptools import setup

setup(name='safe_redcho_flight', version='0.1.0', packages=['safe_redcho_flight'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/safe_redcho_flight']),
                  ('share/safe_redcho_flight', ['package.xml']),
                  ('share/safe_redcho_flight/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'], zip_safe=True,
      maintainer='Safe REDCHO project', maintainer_email='maintainer@example.invalid',
      description='Six-agent dual REDCHO + reciprocal HOCBF-QP reference governor for Crazyflie commissioning',
      license='MIT',
      entry_points={'console_scripts': [
          'redcho_agent = safe_redcho_flight.agent:main',
          'redcho_mission = safe_redcho_flight.mission:main',
          'redcho_watchdog = safe_redcho_flight.watchdog:main',
          'redcho_offline = safe_redcho_flight.offline:main',
          'redcho_figures = safe_redcho_flight.figures:main',
      ]})
