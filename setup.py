from setuptools import find_packages, setup

package_name = 'gyakuenki'

setup(
  name=package_name,
  version='0.0.0',
  packages=find_packages(exclude=['test']),
  data_files=[
    ('share/ament_index/resource_index/packages',
      ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='nuna',
  maintainer_email='hanunshaka02@gmail.com',
  description='Inverse Perspective Mapping package for soccer legacy code.',
  license='MIT License',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      'gyakuenki = gyakuenki.gyakuenki_main:main',
    ],
  },
)
