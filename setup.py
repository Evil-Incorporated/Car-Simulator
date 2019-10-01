from setuptools import setup
setup(name='car_simulator',
      version='0.0.1',
      include_package_data=True,
      install_requires=['gym','opencv-python', 'Pillow', 'Panda3D']  # And any other dependencies foo needs
      # install_requires=['gym']
)