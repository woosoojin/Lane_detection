from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['my_first_python'],
    package_dir={'': 'src'}
    requires=['blob_param_siljun.py', 'turtle_video_siljun.py']
)

setup(**setup_args)
