from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['learned_to_executed_trajectory'],
    package_dir={'': 'src'}
    )
setup(**d)