from glob import glob
from setuptools import setup

setup(name='safe_redcho_three_telemetry',version='0.1.0',packages=['safe_redcho_three_telemetry'],
      data_files=[('share/ament_index/resource_index/packages',['resource/safe_redcho_three_telemetry']),
                  ('share/safe_redcho_three_telemetry',['package.xml']),
                  ('share/safe_redcho_three_telemetry/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools','numpy','PyYAML'],zip_safe=True,
      maintainer='Safe REDCHO project',maintainer_email='maintainer@example.invalid',
      description='Four read-only live telemetry and CSV recorder nodes',license='MIT',
      entry_points={'console_scripts':[
          'flight_telemetry = safe_redcho_three_telemetry.flight_recorder:main',
          'reference_telemetry = safe_redcho_three_telemetry.reference_recorder:main',
          'redcho_telemetry = safe_redcho_three_telemetry.redcho_recorder:main',
          'control_telemetry = safe_redcho_three_telemetry.control_recorder:main']})
