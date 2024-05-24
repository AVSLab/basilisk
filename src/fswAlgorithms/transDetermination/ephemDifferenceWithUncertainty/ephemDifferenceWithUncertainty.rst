Executive Summary
-----------------

Module reads in two ephemeris messages containing the position and velocity of the two corresponding objects (Base and
Secondary). The module then computes the relative position and velocity of the Secondary with respect to the Base, and
writes this to a translational navigation message. Additionally, the module takes the 6x6 state covariance of the two
bodies as input parameters via setter functions, and writes the relative state vector as well as the relative covariance
matrix to a filter message.

The time tag that is written to the navigation and filter messages is taken from the Secondary (the base ephemeris may
not be updated as frequently).

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for:

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - ephemBaseInMsg
      - :ref:`EphemerisMsgPayload`
      - Input ephemeris message containing the position and velocity of the base object in the inertial frame
    * - ephemSecondaryInMsg
      - :ref:`EphemerisMsgPayload`
      - Input ephemeris message containing the position and velocity of the secondary object in the inertial frame
    * - navTransOutMsg
      - :ref:`NavTransMsgPayload`
      - Output navigation message containing the relative position and velocity in the inertial frame
    * - filterOutMsg
      - :ref:`FilterMsgPayload`
      - Output filter message containing the relative state and covariance matrix

Detailed Module Description
---------------------------

The relative position and velocity of the Secondary with respect to the Base are computed by

.. math::

    \mathbf{r}_{21}^N &= \mathbf{r}_{2}^N - \mathbf{r}_{1}^N \\
    \mathbf{v}_{21}^N &= \mathbf{v}_{2}^N - \mathbf{v}_{1}^N

where :math:`\mathbf{r}_{1}^N` and :math:`\mathbf{v}_{1}^N` are the position and velocity vectors of the base object,
and :math:`\mathbf{r}_{2}^N` and :math:`\mathbf{v}_{2}^N` are the position and velocity vectors of the secondary object,
respectively. The relative state vector is

.. math::

    \mathbf{X}_{21} = \mathbf{X}_{2} - \mathbf{X}_{1} = [(\mathbf{r}_{21}^N)^T, (\mathbf{v}_{21}^N)^T]^T

The relative covariance is found by:

.. math::

    Cov(\mathbf{X}_{21}, \mathbf{X}_{21}) &= Cov(\mathbf{X}_{2}, \mathbf{X}_{2}) - Cov(\mathbf{X}_{2}, \mathbf{X}_{1})
                                        - Cov(\mathbf{X}_{1}, \mathbf{X}_{2}) + Cov(\mathbf{X}_{1}, \mathbf{X}_{1}) \\
                                        &= Cov(\mathbf{X}_{1}, \mathbf{X}_{1}) + Cov(\mathbf{X}_{2}, \mathbf{X}_{2})

where :math:`Cov(\mathbf{X}_{1}, \mathbf{X}_{1})` and :math:`Cov(\mathbf{X}_{2}, \mathbf{X}_{2})` are the covariance
matrices of the base and secondary, respectively. Because the two objects are assumed to be independent,
:math:`Cov(\mathbf{X}_{2}, \mathbf{X}_{1}) = Cov(\mathbf{X}_{1}, \mathbf{X}_{2}) = 0`.

User Guide
----------
This section is to outline the steps needed to setup the module in Python.

#. Import the module::

    from Basilisk.fswAlgorithms import ephemDifferenceWithUncertainty

#. Create an instantiation of the class::

    module = ephemDifferenceWithUncertainty.EphemDifferenceWithUncertainty()

#. The covariance matrices are set by::

    module.setCovarianceBase(covar_1)
    module.setCovarianceSecondary(covar_2)

#. Subscribe to the messages::

    module.ephemBaseInMsg.subscribeTo(ephem1InMsg)
    module.ephemSecondaryInMsg.subscribeTo(ephem2InMsg)

#. Add model to task::

    sim.AddModelToTask(taskName, module)
