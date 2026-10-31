from glob import glob
from setuptools import setup

package = 'safe_redcho_three_liveplots'
setup(name=package, version='0.1.0', packages=[package],
    data_files=[('share/ament_index/resource_index/packages',['resource/'+package]),
                ('share/'+package,['package.xml','README.md','SIGNALS.md']),
                ('share/'+package+'/config',glob('config/*.yaml')),
                ('share/'+package+'/launch',glob('launch/*.launch.py'))],
    install_requires=['setuptools','numpy','PyYAML'], zip_safe=True,
    maintainer='Safe REDCHO project', maintainer_email='maintainer@example.invalid',
    description='Independent read-only runtime plots for three-Crazyflie REDCHO hardware', license='MIT',
    entry_points={'console_scripts':[
        'live_capture = '+package+'.capture:main',
        *[key+'_plot = '+package+'.gui:'+key for key in (
            'flight','battery','timing','reference','velocity_redcho','position_redcho','control','hocbf')]]})
