from glob import glob
from pathlib import Path
from setuptools import setup

name = 'tb4_voice_navigation'

# Giữ cây thư mục model để đường dẫn mesh và texture không bị hỏng.
model_data = []
for model_file in sorted(Path('models').rglob('*')):
    if model_file.is_file():
        model_data.append(('share/'+name+'/'+str(model_file.parent), [str(model_file)]))
setup(
    name=name,
    version='0.1.0',
    packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+name]),
        ('share/'+name, ['package.xml', 'README.md']),
        ('share/'+name+'/maps', glob('maps/*')),
        ('share/'+name+'/config', glob('config/*.yaml')),
        ('share/'+name+'/launch', glob('launch/*.launch.py')),
        ('share/'+name+'/worlds', glob('worlds/*.sdf') + glob('worlds/*LICENSE.txt')),
        ('share/'+name+'/rviz', glob('rviz/*.rviz')),
    ] + model_data,
    install_requires=['setuptools'],
    maintainer='TurtleBot 4 student',
    maintainer_email='student@example.com',
    description='Bài 7: phân vùng bản đồ và điều hướng bằng giọng nói',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'segment_map = '+name+'.segment_map:main',
        'parse_command = '+name+'.commands:main',
        'voice_navigation_node = '+name+'.navigation_node:main',
        'speech_node = '+name+'.speech_node:main',
        'voice_control_gui = '+name+'.voice_control_gui:main',
        'download_voice_model = '+name+'.voice_model:main',
    ]},
)
