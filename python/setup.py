import os
import sys
import warnings
from glob import glob

import pkgconfig
from pybind11.setup_helpers import ParallelCompile, naive_recompile
from pybind11.setup_helpers import Pybind11Extension
from setuptools import setup

# names of the environment variables that define osqp and openrobots include directories
osqp_path_var = 'OSQP_INCLUDE_DIR'

__version__ = "9.2.0"
__libraries__ = ['state_representation', 'clproto', 'controllers', 'dynamical_systems', 'robot_model', 'communication_interfaces']
__include_dirs__ = ['include']

__install_clproto_module__ = True
__install_controllers_module__ = True
__install_dynamical_systems_module__ = True
__install_robot_model_module__ = True
__install_communication_interfaces_module__ = True

# check that necessary libraries can be found
try:
    eigen_dir = pkgconfig.cflags('eigen3')
    if eigen_dir.startswith('-I'):
        __include_dirs__.append(eigen_dir.lstrip('-I'))
    else:
        raise Exception('Could not find Eigen3 package!')

    for lib in __libraries__:
        status = pkgconfig.installed(lib, f'>= {__version__}')
        if not status:
            msg = f'Could not find library {lib}!'
            if lib == 'clproto':
                warnings.warn(f'{msg} The clproto module will not be installed.')
                __install_clproto_module__ = False
            elif lib == 'dynamical_systems':
                warnings.warn(f'{msg} The dynamical_systems module will not be installed.')
                __install_dynamical_systems_module__ = False
            elif lib == 'robot_model':
                warnings.warn(f'{msg} The robot_model module will not be installed.')
                __install_robot_model_module__ = False
            elif lib == 'controllers':
                warnings.warn(f'{msg} The controllers module will not be installed.')
                __install_controllers_module__ = False
            else:
                raise Exception(msg)

    if __install_robot_model_module__:
        if osqp_path_var in os.environ.keys():
            __include_dirs__.append(os.environ[osqp_path_var])
        else:
            __include_dirs__.append('/usr/local/include/osqp')
            __include_dirs__.append('/usr/include/osqp')

    if __install_controllers_module__ and not __install_robot_model_module__:
        warnings.warn(
            'The robot model module is required to build the controllers module! '
            'The controllers module will not be installed with the current settings.')
        __install_controllers_module__ = False

except Exception as e:
    msg = f'Error with control library dependencies: {e.args[0]} Ensure the control libraries are properly installed.'
    warnings.warn(msg)
    sys.exit(1)

ParallelCompile('NPY_NUM_BUILD_JOBS', needs_recompile=naive_recompile).install()

ext_modules = [
    Pybind11Extension('state_representation',
                      sorted(glob('source/state_representation/*.cpp') + glob('source/common/*.cpp')),
                      cxx_std=17,
                      include_dirs=__include_dirs__,
                      libraries=['state_representation'],
                      define_macros=[('MODULE_VERSION_INFO', __version__)],
                      )
]

if __install_clproto_module__:
    ext_modules.append(
        Pybind11Extension('clproto',
                          sorted(glob('source/clproto/*.cpp') + glob('source/common/*.cpp')),
                          cxx_std=17,
                          include_dirs=__include_dirs__,
                          libraries=['state_representation', 'clproto'],
                          define_macros=[('MODULE_VERSION_INFO', __version__)],
                          )
    )

if __install_dynamical_systems_module__:
    ext_modules.append(
        Pybind11Extension("dynamical_systems",
                          sorted(glob("source/dynamical_systems/*.cpp") + glob("source/common/*.cpp")),
                          cxx_std=17,
                          include_dirs=__include_dirs__,
                          libraries=['state_representation', 'dynamical_systems'],
                          define_macros=[('MODULE_VERSION_INFO', __version__)],
                          )
    )

if __install_robot_model_module__:
    ext_modules.append(
        Pybind11Extension('robot_model',
                          sorted(glob('source/robot_model/*.cpp')),
                          cxx_std=17,
                          include_dirs=__include_dirs__,
                          libraries=['state_representation', 'robot_model'],
                          define_macros=[('MODULE_VERSION_INFO', __version__)],
                          extra_compile_args=['-DPINOCCHIO_WITH_HPP_FCL']
                          )
    )

if __install_controllers_module__:
    ext_modules.append(
        Pybind11Extension('controllers',
                          sorted(glob('source/controllers/*.cpp') + glob("source/common/*.cpp")),
                          cxx_std=17,
                          include_dirs=__include_dirs__,
                          libraries=['state_representation', 'controllers', 'robot_model'],
                          define_macros=[('MODULE_VERSION_INFO', __version__)],
                          )
    )

if __install_communication_interfaces_module__:
    ext_modules.append(
        Pybind11Extension('communication_interfaces',
                          sorted(glob('source/communication_interfaces/*.cpp')),
                          cxx_std=17,
                          include_dirs=__include_dirs__,
                          libraries=['communication_interfaces'],
                          define_macros=[('MODULE_VERSION_INFO', __version__)],
                          )
    )

setup(
    name='control-libraries',
    version=__version__,
    author='Enrico Eberhard',
    author_email='enrico@aica.tech',
    url='https://github.com/aica-technology/control-libraries',
    description='Python bindings for the C++ control libraries',
    long_description='',
    ext_modules=ext_modules,
    test_suite='tests',
    python_requires='>=3',
    install_requires=[
        'pyquaternion>=0.9.9'
    ],
    license='GNU GPL v3',
    zip_safe=False,
)
