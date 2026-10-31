from glob import glob
from setuptools import find_packages, setup

setup(
    name='safe_redcho_bringup', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/safe_redcho_bringup']),
                ('share/safe_redcho_bringup', ['package.xml']),
                ('share/safe_redcho_bringup/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Safe REDCHO project', maintainer_email='maintainer@example.com',
    description='First hardware commissioning with OptiTrack', license='MIT',
    entry_points={'console_scripts': [
        'pose_check = safe_redcho_bringup.pose_check:main',
        'single_hover = safe_redcho_bringup.single_hover:main',
        'fleet_capture = safe_redcho_bringup.fleet_capture:main',
        'fleet_plan = safe_redcho_bringup.swap_planner:main',
        'fleet_mission = safe_redcho_bringup.fleet_mission:main',
        'backend_audit = safe_redcho_bringup.backend_audit:main',
    ]},
)
