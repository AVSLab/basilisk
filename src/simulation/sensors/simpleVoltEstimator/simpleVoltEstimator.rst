Executive Summary
-----------------

The Simple Voltage Estimator module is used to provide error-ed truth (or truth) spacecraft electric potentials,
similar to the :ref:`simpleNav` module that provides error-ed truth (or truth) spacecraft states.  This class is used
to perturb the truth state away using a gauss-markov
error model.  It is designed to look like a random walk process put on top of
the nominal spacecraft voltage.  This is meant to
be used in place of the nominal voltage output.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - voltOutMsg
      - :ref:`VoltMsgPayload`
      - spacecraft voltage output msg
    * - voltInMsg
      - :ref:`VoltMsgPayload`
      - spacecraft voltage input msg


User Guide
----------
This module is set up similarly to the :ref:`simpleNav` module. It is constructed using ``sVoltObject =
simpleVoltEstimator.SimpleVoltEstimator()``. The random walk bound is specified using ``sVoltObject.walkBounds`` and
the standard deviation is specified using ``sVoltObject.PMatrix``. Note that the input for the walk bound and
standard deviation must be a list to work for the :ref:`gauss_markov` module.

If no walk bound or standard deviation is specified, then the voltage measurement will not be corrupted with noise.