Executive Summary
-----------------
This module computes attitude guidance and reference messages for two imaging modes:

- **Location Pointing Mode**: An attitude guidance message is generated to make a specified spacecraft pointing vector point at an inertial target location. This location can be on a planet if this module is connected with :ref:`GroundStateMsgPayload`, on a celestial object center using :ref:`EphemerisMsgPayload`, or on a spacecraft using :ref:`NavTransMsgPayload`.
- **Strip Imaging Mode**: When connected to a :ref:`StripStateMsgPayload` message, the module first points the body-fixed pointing vector at the moving target on the strip centerline. It then performs an additional rotation about this pointing direction so that a secondary body-fixed vector becomes orthogonal to the strip scanning direction. This secondary rotation is applied only if the pointing vector and the strip scanning direction fulfill the non-collinearity constraint.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg connection is set by the user from python.
The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - scAttInMsg
      - :ref:`NavAttMsgPayload`
      - input msg with inertial spacecraft attitude states
    * - scTransInMsg
      - :ref:`NavTransMsgPayload`
      - input msg with inertial spacecraft translational states
    * - locationInMsg
      - :ref:`GroundStateMsgPayload`
      - input msg containing the inertial point location of interest
    * - locationstripInMsg
      - :ref:`StripStateMsgPayload`
      - (alternative) input msg from :ref:`stripLocation` containing the inertial position and velocity of the current strip target point
    * - celBodyInMsg
      - :ref:`EphemerisMsgPayload`
      - (alternative) input msg containing the inertial point location of a celestial body of interest
    * - scTargetInMsg
      - :ref:`NavTransMsgPayload`
      - (alternative) input msg with inertial target spacecraft translational states
    * - attGuidOutMsg
      - :ref:`AttGuidMsgPayload`
      - output message with the attitude guidance
    * - attRefOutMsg
      - :ref:`AttRefMsgPayload`
      - output message with the attitude reference



Detailed Module Description
---------------------------

Location Pointing Mode
^^^^^^^^^^^^^^^^^^^^^^
The inertial location of interest w.r.t the inertial origin is given by :math:`{\bf r}_{L/N}` and can be either extracted from ``locationInMsg`` when
a location on a planet is provided,  ``celBodyInMsg`` when a celestial body's ephemeris location is provided (for pointing
at the Sun or the Earth), ``scTargetInMsg`` when pointing at another spacecraft, or ``locationstripInMsg`` when performing
strip imaging.
The vector pointing from the satellite location :math:`{\bf r}_{S/N}` to this location is then

.. math::
    {\bf r}_{L/S} = {\bf r}_{L/N} - {\bf r}_{S/N}

Let :math:`\hat{\bf r}_{L/S}` be the normalized heading vector to this location.

The unit vector :math:`\hat{\bf p}_B` is a body-fixed vector and denotes the body axis which is to point towards
the desired location :math:`L`.  Thus this mode performs a 2-degree of freedom attitude guidance and
control solution.

The eigen-axis to rotate :math:`\hat{\bf p}_B` towards :math:`\hat{\bf r}_{L/S}` is given by

.. math::

    \hat{\bf e} = \frac{\hat{\bf p}_B \times \hat{\bf r}_{L/S}}{|\hat{\bf p}_B \times \hat{\bf r}_{L/S}|}

The principle rotation angle :math:`\phi` is

.. math::

    \phi = \arccos (\hat{\bf p}_B \cdot \hat{\bf r}_{L/S} )

The attitude tracking error :math:`{\pmb\sigma}_{B/R}` is then given by

.. math::

    {\pmb\sigma}_{B/R} = - \tan(\phi/4) \hat{\bf e}

The reference attitude :math:`{\pmb\sigma}_{R/N}` is obtained by composing the spacecraft attitude :math:`{\pmb\sigma}_{B/N}` with the tracking error:

.. math::

    {\pmb\sigma}_{R/N} = {\pmb\sigma}_{B/N} \oplus {\pmb\sigma}_{R/B}

where :math:`\oplus` denotes the MRP addition operator.

When the module is **not** in strip imaging mode the outputs are set directly:

.. math::

    {\pmb\sigma}_{B/R}^{\text{out}} &= {\pmb\sigma}_{B/R} \\
    {\pmb\sigma}_{R/N}^{\text{out}} &= {\pmb\sigma}_{R/N}


Strip Imaging Mode
^^^^^^^^^^^^^^^^^^
When the ``locationstripInMsg`` input from the :ref:`stripLocation` module is connected, the module enters
strip imaging mode.  The :ref:`StripStateMsgPayload` message provides the inertial position w.r.t to the inertial origin
:math:`{\bf r}_{L/N}` and the inertial velocity w.r.t the planet center
:math:`{\bf v}_{LP/N}` of the current target point moving along the central line of the strip.

The standard location pointing computation described above is first performed to obtain
:math:`{\pmb\sigma}_{R/N}` (the reference that points :math:`\hat{\bf p}_B` at the current target).
An additional rotation about the pointing axis :math:`\hat{\bf p}_B` is then computed so that a
secondary body-fixed vector :math:`\hat{\bf c}_B` specified by the user and in the plane orthogonal to the pointing axis becomes perpendicular to the strip scanning
direction.  This ensures that the instrument's cross-track axis is correctly aligned during image acquisition.

**Step 1 – Compute the scanning direction in the reference body frame.**
The normalized target velocity :math:`\hat{\bf v}_{LP/N}` is expressed in the reference frame R using the
DCM :math:`[RN]` obtained from :math:`{\pmb\sigma}_{R/N}`:

.. math::

    \hat{\bf v}_{LP/R} = [RN] \, \hat{\bf v}_{LP/N}

The component of :math:`\hat{\bf v}_{LP/R}` perpendicular to the pointing axis is then:

.. math::

    {\bf v}_{\perp} = \hat{\bf p}_B \times \hat{\bf v}_{LP/R}

and is normalized to :math:`\hat{\bf v}_{\perp}`.

If :math:`|\hat{\bf p}_B \times \hat{\bf v}_{LP/R}|` is smaller than a user-defined threshold ``alignmentThreshold``
(default 0.1), then :math:`\hat{\bf p}_B` and :math:`\hat{\bf v}_{LP/R}` are nearly collinear and the perpendicular
direction is undefined or unstable.  In that case no extra rotation is applied and the outputs
fall back to the standard location-pointing values.

**Step 2 – Choose the sign of** :math:`\hat{\bf v}_{\perp}`.
Both :math:`+\hat{\bf v}_{\perp}` and :math:`-\hat{\bf v}_{\perp}` are valid perpendicular-to-strip directions.
The sign is chosen so that the rotation angle is minimized, i.e. the sign that satisfies
:math:`\hat{\bf c}_B \cdot \hat{\bf v}_{\perp} > 0` is selected.

**Step 3 – Compute the extra rotation MRP** :math:`{\pmb\sigma}_{R_2/R}`.
The rotation that aligns :math:`\hat{\bf c}_B` with :math:`\hat{\bf v}_{\perp}` about :math:`\hat{\bf p}_B` is
computed analogously to the standard pointing rotation.  The eigen-axis is:

.. math::

    \hat{\bf e}_{2} = \frac{\hat{\bf c}_B \times \hat{\bf v}_{\perp}}{|\hat{\bf c}_B \times \hat{\bf v}_{\perp}|}

and the rotation angle is:

.. math::

    \alpha = \arccos\!\left(\hat{\bf c}_B \cdot \hat{\bf v}_{\perp}\right)

The MRP of this extra rotation is then:

.. math::

    {\pmb\sigma}_{R_2/R} = \tan(\alpha / 4) \; \hat{\bf e}_{2}

**Step 4 – Compose to get the final reference.**
The combined reference attitude that both points :math:`\hat{\bf p}_B` at the target **and** aligns
:math:`\hat{\bf c}_B` perpendicular to the strip scanning direction is:

.. math::

    {\pmb\sigma}_{R_2/N} = {\pmb\sigma}_{R/N} \oplus {\pmb\sigma}_{R_2/R}

**Step 5 – Compute the final tracking error.**
The tracking error is recomputed with respect to the updated reference:

.. math::

    {\pmb\sigma}_{B/R_2} = {\pmb\sigma}_{B/N} \ominus {\pmb\sigma}_{R_2/N}

where :math:`\ominus` denotes MRP subtraction.

The outputs in strip imaging mode are therefore:

.. math::

    {\pmb\sigma}_{B/R}^{\text{out}} &= {\pmb\sigma}_{B/R_2} \\
    {\pmb\sigma}_{R/N}^{\text{out}} &= {\pmb\sigma}_{R_2/N}


Angular Rate Computation
^^^^^^^^^^^^^^^^^^^^^^^^
The tracking error rates :math:`{\pmb\omega}_{B/R}` are obtained through numerical differentiation of the
MRP values.  During the first module ``Update`` evaluation the numerical differencing is not possible and
this value is thus set to zero.

.. note::

    The module checks for several conditions such as heading vectors
    being collinear, the MRP switching during the numerical differentiation, etc.
    In strip imaging mode, the non-collinearity constraint between :math:`\hat{\bf p}_B` and the
    target velocity is also checked.  When the two vectors are nearly parallel, the extra strip
    rotation is skipped and the module falls back to pure location pointing.


User Guide
----------
For **location pointing**, the one required variable that must be set is ``pHat_B``.  This is body-fixed unit vector which is to be
pointed at the desired inertial location. The user should only connect one location of interest input message, either ``locationInMsg``,
``celBodyInMsg`` or ``scTargetInMsg``. Connecting multiple will result in a warning and the module defaults to using the ground location,
planet location or spacecraft location, in that order.

By default this is a 2D attitude control module in attitude and a 2D rate control.  In particular, the rates about the
desired heading axis are not damped.  By setting the module variable ``useBoresightRateDamping`` to 1,
the body rates about about the desired heading
angle are added to the rate tracking error yielding a 3D rate control implementation.
Note that ``useBoresightRateDamping`` is ignored during strip imaging, as strip imaging already performs
full 3D attitude control.

For **strip imaging**, the user should connect ``locationstripInMsg`` (a :ref:`StripStateMsgPayload` from :ref:`stripLocation`)
and additionally set the ``cHat_B`` body-fixed vector.  This vector must be perpendicular to ``pHat_B`` and
represents the instrument cross-track axis that needs to be aligned perpendicular to the strip scanning direction.
The variable ``alignmentThreshold`` (default 0.1) controls the collinearity threshold: if
:math:`|\hat{\bf p}_B \times \hat{\bf v}_{LP/R}|` is below this value the extra strip rotation is skipped.
The variable ``stripInertialSpeedThreshold`` (default :math:`10^{-12}` m/s) defines the minimum inertial target velocity magnitude (relative to planet origin vector)
required to compute a valid scanning direction; if the strip speed magnitude is below this threshold, the module
falls back to pure location pointing.

This module in both modes provides two output messages in the form of :ref:`attGuidMsgPayload` and :ref:`attRefMsgPayload`.
The first guidance message, describing body relative to reference tracking errors,
can be directly connected to an attitude control module.  However, at times we need to have the
attitude reference message as the output to feed to :ref:`attTrackingError`.

The variable ``smallAngle`` defines an angular threshold (in radians, default zero) below which a rotation is
set to zero.  It applies separately to the pointing rotation and the strip rotation.  This is to avoid numerical issues when the angles are very small.
