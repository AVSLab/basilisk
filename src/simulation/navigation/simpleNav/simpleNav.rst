Executive Summary
-----------------

Simple navigation model used to provide error-ed truth (or truth). This class is used to perturb the truth state away using a gauss-markov
error model.  It is designed to look like a random walk process put on top of
the nominal position, velocity, attitude, and attitude rate.  This is meant to
be used in place of the nominal navigation system output.

The module
:download:`PDF Description </../../src/simulation/navigation/simpleNav/_Documentation/Basilisk-SIMPLE_NAV20170712.pdf>`
contains further information on this module's function,
how to run it, as well as testing.


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
    * - attOutMsg
      - :ref:`NavAttMsgPayload`
      - attitude navigation output msg
    * - transOutMsg
      - :ref:`NavTransMsgPayload`
      - translation navigation output msg
    * - scStateInMsg
      - :ref:`SCStatesMsgPayload`
      - spacecraft state input msg
    * - sunStateInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - sun state input input msg (optional)
