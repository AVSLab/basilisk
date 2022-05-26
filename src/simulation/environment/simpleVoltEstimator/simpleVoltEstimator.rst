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
