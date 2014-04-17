===
yak
===
-------------------------
Yet Another Kalman filter
-------------------------

:Author: Tamas Nepusz

This is a header-only C++ template library that implements a Kalman filter [1]_.

Requirements
------------

You only need a recent C++ compiler (e.g., ``g++`` or ``clang``) and the Eigen_ library
to use ``yak``.

.. _Eigen: http://eigen.tuxfamily.org

Compilation
-----------

What compilation? :) Since ``yak`` is header-only, you don't have to compile anything - just
``#include <yak/yak.hpp>`` in your source files and you are ready to go (assuming that
Eigen_ is also installed properly.

The only thing that you can compile from this source distribution is the test suite. You
will need CMake_ and ``make`` to compile the tests. Assuming that both of them are properly
installed, just do this from the command line::

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ cd ..
    
.. _CMake: http://www.cmake.org

The test executables will be compiled into ``build/test``. Each test executable tests the
Kalman filter on a simple dynamical model by simulating the model for a given number of time
steps and displaying the actual, measured and estimated states of the model in a simple
tabular format. If you want to create nice plots from the output, you can use my
``qplot`` script from the ``swissknife`` repo (see here_) as follows::

    $ build/test/test_cannonball | qplot -f 2-7 -t scatter
    $ build/test/test_vehicle_accelerometer | qplot -f 2-7 -t scatter
    $ build/test/test_voltage_reading | qplot

See the comments in the source files of the test cases in ``test/`` for more details about
the dynamical models.

.. _here: https://github.com/ntamas/swissknife

Usage
-----

For the time being, take a look at the source files in ``test/`` for inspiration. The
pattern is usually the same. First you add ``typedefs`` for the state vector, the state
estimate, the control vector and the measurement vector of your model. Next, you have
to implement the *process model* and the *measurement model* classes. Finally, you
instantiate a Kalman filter template that uses the process model and the measurement
model that you have implemented, and feed it with data using its ``update()`` method.
After each ``update()`` call, the estimate of the Kalman filter can be retrieved from
its ``lastEstimate.mean`` and ``lastEstimate.covariance`` properties.

References
----------

.. [1] Kalman RE: A New Approach to Linear Filtering and Prediction Problems. Journal of
       Basic Engineering 82(1):35-45, 1960.
