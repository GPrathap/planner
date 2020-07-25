## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    version='0.1.0',
    scripts=['bin/drone_reg_node'],
    packages=['drone_reg'],
    package_dir={'': 'scripts'}
)

setup(**setup_args)
