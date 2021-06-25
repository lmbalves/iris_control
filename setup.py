from distutils.core import setup
 
setup(
    version='0.0.0',
    scripts=['src/path_motor_control2.py','src/control_vision.py','src/control_rotors.py'],
    packages=['iris_control'],
    package_dir={'': 'src'}
)
