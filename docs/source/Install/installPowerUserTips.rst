.. toctree::
   :hidden:


.. _installPowerUserTips:



Power User Installation Tips
============================

The project can be configured and built from the command line via CMake. Command line operations are run using the following setup.py script and parameters::

	python setup.py <command_1 command_2 etc.>

`clean`: removes ‘dist/build’ and build artifacts.

`cmake`: configure the project and generate an XCode project file to the ``dist`` directory. This parameter also installs Basilisk on your local path to be found along with other Python packages. This requires CMake command line version to be installed.

`xcode`: execute the project build scheme with XCode where the generated Basilisk python package is output to ‘dist3/’ or ‘dist/’.

`test`: run pytest on the Basilisk project. This parameter takes
additional user options, via ‘pytest-args=’ to pass through to pytest.

`docs`: build the documentation with doxygen. The generated html
documentation is found in ``src/html``. This requires doxygen command line version to be installed.

