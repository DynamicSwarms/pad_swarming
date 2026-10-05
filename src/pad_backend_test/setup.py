from glob import glob
from setuptools import setup

setup(name='pad_backend_test', version='0.1.0', packages=['pad_backend_test'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/pad_backend_test']),
                  ('share/pad_backend_test', ['package.xml', 'README.md']),
                  ('share/pad_backend_test/launch', glob('launch/*.py')),
                  ('share/pad_backend_test/config', glob('config/*.json'))],
      install_requires=['setuptools'], entry_points={'console_scripts': [
          'velocity_test = pad_backend_test.runner:main',
          'simulation_clock = pad_backend_test.simulation_clock:main']})
