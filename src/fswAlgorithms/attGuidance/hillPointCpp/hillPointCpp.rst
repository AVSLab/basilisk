Executive Summary
-----------------

This attitude guidance module computes the orbital Hill reference frame states.

The orbit can be any type of Keplerian motion, including circular, elliptical or hyperbolic.
The module

Module Input and Output
=======================

Table `1 <#tab:inputNavTable>`__ shows the input message from the
navigation system.

.. container::
   :name: tab:inputNavTable

   .. table:: Input Navigation Message

      +--------------------------+-----------+--------+------------------------------+
      | Name                     | Type      | Length | Description                  |
      +==========================+===========+========+==============================+
      | :math:`\boldsymbol{R}_S` | double [] | 3      | Position vector of           |
      |                          |           |        | the spacecraft               |
      |                          |           |        | body-point with              |
      |                          |           |        | respect to the               |
      |                          |           |        | inertial frame in            |
      |                          |           |        | inertial frame               |
      |                          |           |        | components                   |
      |                          |           |        | (:math:`{}^{\mathcal{N}}     |
      |                          |           |        | {\boldsymbol{r}_{B/N}}`)     |
      +--------------------------+-----------+--------+------------------------------+
      | :math:`\boldsymbol{v}_S` | double [] | 3      | Velocity vector of           |
      |                          |           |        | the spacecraft point         |
      |                          |           |        | with respect to the          |
      |                          |           |        | inertial frame in            |
      |                          |           |        | inertial frame               |
      |                          |           |        | components                   |
      |                          |           |        | (:math:`{}^{\mathcal{N}}     |
      |                          |           |        | {\boldsymbol{v}}_{B/N}`).    |
      +--------------------------+-----------+--------+------------------------------+

Table `2 <#tab:inputCelTable>`__ shows the input message from Spice
about the main celestial body.

.. container::
   :name: tab:inputCelTable

   .. table:: Input Spice Planet Message

      +--------------------------+-----------+--------+----------------------+
      | Name                     | Type      | Length | Description          |
      +==========================+===========+========+======================+
      | :math:`\boldsymbol{R}_P` | double [] | 3      | Position vector of   |
      |                          |           |        | the main celestial   |
      |                          |           |        | object with respect  |
      |                          |           |        | to the inertial      |
      |                          |           |        | frame in inertial    |
      |                          |           |        | frame components .   |
      +--------------------------+-----------+--------+----------------------+
      | :math:`\boldsymbol{v}_P` | double [] | 3      | Velocity vector of   |
      |                          |           |        | the main celestial   |
      |                          |           |        | object with respect  |
      |                          |           |        | to the inertial      |
      |                          |           |        | frame in inertial    |
      |                          |           |        | frame components .   |
      +--------------------------+-----------+--------+----------------------+

Table `3 <#tab:outputTable>`__ shows the Attitude Reference output
message of the module Hill Point.

.. container::
   :name: tab:outputTable

   .. table:: Output Attitude Reference Message

      +-----------------------------+-----------+--------+----------------------+
      | Name                        | Type      | Length | Description          |
      +=============================+===========+========+======================+
      | :math:`\sigma_{R/N}`        | double [] | 3      | MRP attitude set of  |
      |                             |           |        | the reference frame  |
      |                             |           |        | with respect to the  |
      |                             |           |        | reference.           |
      +-----------------------------+-----------+--------+----------------------+
      | :math:`\omega_{R/N}`        | double [] | 3      | Angular rate vector  |
      |                             |           |        | of the reference     |
      |                             |           |        | frame with respect   |
      |                             |           |        | to the inertial      |
      |                             |           |        | expressed in         |
      |                             |           |        | inertial frame       |
      |                             |           |        | components.          |
      +-----------------------------+-----------+--------+----------------------+
      | :math:`\dot{\omega}_{R/N}`  | double [] | 3      | Angular acceleration |
      |                             |           |        | vector of the        |
      |                             |           |        | reference frame with |
      |                             |           |        | respect to the       |
      |                             |           |        | inertial expressed   |
      |                             |           |        | in inertial frame    |
      |                             |           |        | components.          |
      +-----------------------------+-----------+--------+----------------------+

Hill Frame Definition
=====================

The Hill reference frame takes the spacecraft’s orbital plane as the
principal one and has origin in the centre of the spacecraft. It is
defined by the right-handed set of axes
:math:`\mathcal{H}:\{ \hat{\boldsymbol\imath}_{r}, \hat{\boldsymbol\imath}_{\theta}, \hat{\boldsymbol\imath}_{h} \}`,
where

:math:`\hat {\boldsymbol\imath}_{r}` points radially outward in the direction
that connects the center of the planet with the spacecraft.

:math:`\hat {\boldsymbol\imath}_{h}` is defined normal to the orbital plane in
the direction of the angular momentum.

:math:`\hat {\boldsymbol\imath}_{\theta}` completes the right-handed triode.

Illustration of the Hill orbit frame
:math:`\mathcal{H}:\{ \hat{\boldsymbol\imath}_{r}, \hat{\boldsymbol\imath}_{\theta}, \hat{\boldsymbol\imath}_{h} \}`,
and the inertial frame
:math:`\mathcal{N}:\{ \hat{\boldsymbol n}_{1}, \hat{\boldsymbol n}_{2}, \hat{\boldsymbol n}_{3} \}`.

Introduction
============

In this module, the output reference frame :math:`\mathcal{R}` is to be
aligned with the Hill reference frame :math:`\mathcal{H}`. Note that the
presented technique does not require the planet-fixed frame to coincide
with the inertial frame
:math:`\mathcal{N}:\{ \hat{\boldsymbol n}_{1}, \hat{\boldsymbol n}_{2}, \hat{\boldsymbol n}_{3} \}`.
Figure 1 illustrates the general situation in which :math:`\boldsymbol{R}_{s}`
is the position vector of the spacecraft with respect to the inertial
frame and :math:`\boldsymbol{R_{p}}` is the position vector of the celestial
body with respect to the inertial frame as well. The relative position
of the spacecraft with respect to the planet is obtained by simple
subtraction:

.. math::

   \label{eq:r}
   	\boldsymbol r = \boldsymbol R_{s} -  \boldsymbol R_{p}

The same methodology is applied to compute the relative velocity vector:

.. math::

   \label{eq:v}
   	\boldsymbol v = \boldsymbol v_{s} -  \boldsymbol v_{p}

Note that the position and velocity vectors of the spacecraft and the
celestial body, :math:`\boldsymbol{R}_S`, :math:`\boldsymbol{R}_P`, :math:`\boldsymbol{v}_S` and
:math:`\boldsymbol{v}_P` are the only inputs that this module requires. Having
:math:`\boldsymbol r` and :math:`\boldsymbol v`, the Hill frame orientation is
completely defined:

.. math::

   \begin{equation}
   	\hat{\boldsymbol\imath}_{r} = \frac{\boldsymbol r}{r}
   	\end{equation}

.. math::
   	\begin{equation}
   	\hat{\boldsymbol\imath}_{h} = \frac{\boldsymbol{r}\times{\boldsymbol{v}}}{r v}
   	\end{equation}

.. math::
   	\begin{equation}
   	\hat{\boldsymbol\imath}_{\theta} = \hat{\boldsymbol\imath}_{h} \times \hat{\boldsymbol\imath}_{r}
   	\end{equation}

And the Direction Cosine Matrix to map from the reference frame to the
inertial is obtained:

.. math::

   =  \begin{bmatrix}
          		\hat{\boldsymbol\imath}_{r} \\
   		\hat{\boldsymbol\imath}_{\theta} \\
   		\hat{\boldsymbol\imath}_{h}
         \end{bmatrix}

The corresponding MRP attitude set is computed using the following
function from the Rigid Body Kinematics library of Reference :

.. math:: [RN] = \textrm{C2MRP}(\boldsymbol\sigma_{R/N})

Angular Velocity Descriptions
=============================

Let :math:`\mathcal{R}_{0}` reference the Hill orbit frame. The orbit
frame angular rate and acceleration vectors are given by

.. math::

   \label{eq:omega_R0}
   	\boldsymbol\omega_{R_{0}/N} = \dot f \hat{\boldsymbol\imath}_{h}

.. math::

   \label{eq:domega_R0}
   	\dot{\boldsymbol\omega}_{R_{0}/N} = \ddot f \hat{\boldsymbol\imath}_{h}

where :math:`f` is the true anomaly, whose variation is determined
through the general standard astrodynamics relations:

.. math::

   \begin{aligned}
     \dot f &= \frac{h}{r^{2}}
     \\
     \ddot f &= - 2 \frac{\boldsymbol v \cdot \hat{\boldsymbol\imath}_{r}}{r} \dot f
   \end{aligned}

The angular rate :math:`\boldsymbol\omega_{R/N}` and acceleration
:math:`\dot{\boldsymbol\omega}_{R/N}` of the output reference frame
:math:`\mathcal{R}` still need to be computed. Since the desired
attitude is a fixed-pointing one, :math:`\mathcal{R}` does not move
relative to :math:`\mathcal{R}_{0}`. Thus, the angular velocity of the
reference frame happens to be

.. math::

   \label{eq:omega_R}
   	\boldsymbol\omega_{R/N} = \boldsymbol\omega_{R/R_{0}}
    + \boldsymbol\omega_{R_{0}/N} = \dot{f} \hat{\boldsymbol\imath}_{h}

Again, given that :math:`\hat{\boldsymbol\imath}_{h}` is fixed as seen by the
reference frame :math:`R`, the acceleration vector of the reference
frame expressed in the reference frame simply becomes:

.. math::

   \label{eq:domega_R}
   	\dot\omega_{R/N} = \ddot{f} \hat{\boldsymbol\imath}_{h}

Both :math:`\boldsymbol\omega_{R/N}` and :math:`\dot\omega_{R/N}` need to be
expressed in the inertial frame :math:`N`. Given

.. math::

   \begin{equation}
   {}^{\mathcal{R}}{\boldsymbol \omega_{R/N} } =
         \begin{bmatrix}
          0\\ 0 \\ \dot{f}
         \end{bmatrix}
   \end{equation}

.. math::
   \begin{equation}
    {}^{\mathcal{R}}{\dot{\boldsymbol {\omega}}_{R/N}} =
         \begin{bmatrix}
          0\\ 0 \\ \ddot{f}
         \end{bmatrix}
   \end{equation}

Then,

.. math::

   \begin{equation}
   	{}^{\mathcal{N}}{\boldsymbol{\omega}_{R/N}} =  [NR] \textrm{ } {}^{\mathcal{R}}{\boldsymbol\omega_{R/N} }
   \end{equation}

.. math::

   \begin{equation}
   	{}^{\mathcal{N}}{\dot{\boldsymbol \omega}_{R/N}}=[NR] \textrm{ } {}^{\mathcal{R}}{\dot{\boldsymbol \omega}_{R/N}}
   \end{equation}

Where :math:`[NR] = [RN]^T`.

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
    * - attRefOutMsg
      - :ref:`AttRefMsgPayload`
      - attitude reference output message
    * - transNavInMsg
      - :ref:`NavTransMsgPayload`
      - incoming spacecraft translational state message
    * - celBodyInMsg
      - :ref:`EphemerisMsgPayload`
      - (optional) primary celestial body information input message
