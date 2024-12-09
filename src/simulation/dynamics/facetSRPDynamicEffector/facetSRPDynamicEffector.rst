Executive Summary
-----------------
This dynamic effector module uses a faceted spacecraft model to calculate the force and torque acting on a spacecraft
due to solar radiation pressure (SRP). The force and torque are calculated about the spacecraft body frame origin
point :math:`B`. The module can be configured for either a static spacecraft or a spacecraft with any number of
articulating facets. For example, a spacecraft with two articulating solar arrays can be configured using 4
articulating facets. The unit test for this module shows how to set up this particular configuration.

.. warning::
    The module variables ``numFacets`` and ``numArticulatedFacets`` will be moved to private module variables
    Dec 2025. Setters and getters are now added to access these variables.

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
    * - sunInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - Input msg for the Sun state information
    * - articulatedFacetDataInMsgs
      - :ref:`HingedRigidBodyMsgPayload`
      - (Optional) Input msg vector containing the current articulated facet angles

Detailed Module Description
---------------------------

Mathematical Modeling
^^^^^^^^^^^^^^^^^^^^^
The spacecraft is represented as a collection of :math:`N` total facets with negligible thickness. Each facet is
characterized by an area :math:`A`, an attitude direction cosine matrix (DCM) :math:`[\mathcal{F_0B}]` relative
to the spacecraft hub frame :math:`\mathcal{B}`, a unit vector normal to its surface
:math:`\boldsymbol{\hat{n}}`, an articulation axis (optional) :math:`\boldsymbol{\hat{s}}`, the facet center of pressure (COP)
location relative to the spacecraft body frame origin point :math:`B`, and three optical coefficients representing
the interaction of impinging photons with the facet surface. The fraction of specularly reflected, diffusely scattered,
and absorbed photons are represented using the coefficients :math:`\delta, \rho,` and :math:`\alpha`, respectively.

.. Note::
    - The facet normal vectors and rotation axes are required to be specified in the facet frame :math:`\mathcal{F}`.

    - The articulating facets are assumed to rotate with 1 degree-of-freedom (DOF) about the specified articulation axis :math:`\boldsymbol{\hat{s}}`.

    - The facet COP vectors are required to be defined in the spacecraft body frame :math:`\mathcal{B}`. This module assumes that each facet COP vector is fixed in the spacecraft body frame for both static and articulating facets. Therefore, it is assumed that the facet COP is the point where the facet normal vector and facet rotation axis intersect.

For each articulating facet, the current facet normal vector must be computed in the spacecraft body
frame :math:`\mathcal{B}`. Using the facet articulation axis :math:`\boldsymbol{\hat{s}}` and the current
facet articulation angle :math:`\phi`, the DCM required to rotate from the initial facet attitude to the final
facet attitude is obtained using the principal rotation vector (PRV) transformation:

.. math::
    [\mathcal{F}\mathcal{F}_0] = \text{PRV2C}(\phi, {}^\mathcal{F} \boldsymbol{\hat{s}})

The transformation from the spacecraft body frame to the current facet attitude can next be determined:

.. math::
    [\mathcal{F}\mathcal{B}] = [\mathcal{F}\mathcal{F}_0] [\mathcal{F}_0\mathcal{B}]

The facet normal vector can now be transformed to the spacecraft body frame using the `active` transformation:

.. math::
    {}^\mathcal{B} \boldsymbol{\hat{n}} = [\mathcal{BF}] {}^{\mathcal{F}} \boldsymbol{\hat{n}}

Next, using the provided facet information and spacecraft Sun-relative position vector
:math:`\boldsymbol{r}_{\text{sc} / \odot }`, the estimated SRP force acting on the spacecraft is calculated
by summing the SRP force contribution from all :math:`N` facets:

.. math::
    \boldsymbol{F}_{\text{SRP}} = \sum_{i = 1}^{N} \boldsymbol{F}_{\text{SRP}_i} = -P(|\boldsymbol{r}_{\text{sc} / \odot }|) \sum_{i = 1}^{N} A_i \cos(\theta_i) \left [ (1 - \delta_i) \boldsymbol{\hat{s}} + 2 \left ( \frac{\rho_i}{3} + \delta_i \cos(\theta_i) \right ) \boldsymbol{\hat{n}}_{i}\right ]

Note that all values in the above expression must be specified in the spacecraft body frame. :math:`\theta`
is defined as the incidence angle between each facet normal vector and the Sun-direction vector.
:math:`P(|\boldsymbol{r}_{\text{sc}/ \odot\ }|)` represents the pressure acting on the spacecraft
scaled by the spacecraft heliocentric distance. :math:`\boldsymbol{\hat{s}}` is the unit direction
vector pointing radially towards the Sun from the spacecraft body frame origin point :math:`B`.
This vector is found by subtracting the current spacecraft inertial position from the Sun position:

.. math::
    {}^\mathcal{B} \boldsymbol{\hat{s}} = [\mathcal{BN}] ( {}^\mathcal{N} \boldsymbol{r}_{\odot / N} - {}^\mathcal{N} \boldsymbol{r}_{\text{sc} / N})

The total SRP torque acting on the spacecraft about point :math:`B` is calculated by summing the torque
contributions over all :math:`N` facets:

.. math::
    \boldsymbol{L}_{\text{SRP},B} = \sum_{i = 1}^{N} \boldsymbol{L}_{{\text{SRP},B}_i} = \sum_{i = 1}^{N} \boldsymbol{r}_{{COP_i}/B} \times \boldsymbol{F}_{\text{SRP}_i}

Module Testing
^^^^^^^^^^^^^^
The unit test for this module ensures that the calculated SRP force and torque acting on the spacecraft about the
body-fixed point B is properly computed for either a static spacecraft or a spacecraft with any number of articulating
facets. The spacecraft geometry defined in the test consists of a cubic hub and two circular solar arrays.
Six static square facets represent the cubic hub and four articulated circular facets describe the articulating
solar arrays. To verify the module functionality, the final SRP force and torque simulation values are checked with the
true values computed in python.

User Guide
----------
The following steps are required to set up the ``facetSRPDynamicEffector`` module in python.

.. important::
    Be sure to include the Sun as a gravitational body in the simulation to use this module.

#. First import the ``facetSRPDynamicEffector`` class::

    from Basilisk.simulation import facetSRPDynamicEffector

#. Next, create an instantiation of the SRP dynamic effector::

    SRPEffector = facetSRPDynamicEffector.FacetSRPDynamicEffector()
    SRPEffector.ModelTag = "SRPEffector"

#. The user is required to set the total number of spacecraft facets and the number of articulated facets. For example, if the user wants to create a spacecraft with 10 total facets, four of which articulate::

    SRPEffector.setNumFacets(10)
    SRPEffector.setNumArticulatedFacets(4)

.. warning::
    The module variables ``numFacets`` and ``numArticulatedFacets`` will be moved to private module variables
    Dec 2025. Setters and getters are now added to access these variables.

#. If the spacecraft contains articulated facets, a ``HingedRigidBodyMsgPayload`` articulation angle message must be configured for each articulated facet. An example using two stand-alone messages is provided below::

    facetRotAngle1 = macros.D2R * 10.0  # [rad]
    facetRotAngle2 = macros.D2R * -10.0  # [rad]

    facetRotAngle1MessageData = messaging.HingedRigidBodyMsgPayload()
    facetRotAngle1MessageData.theta = facetRotAngle1  # [rad]
    facetRotAngle1MessageData.thetaDot = 0.0  # [rad]
    facetRotAngle1Message = messaging.HingedRigidBodyMsg().write(facetRotAngle1MessageData)

    facetRotAngle2MessageData = messaging.HingedRigidBodyMsgPayload()
    facetRotAngle2MessageData.theta = facetRotAngle2  # [rad]
    facetRotAngle2MessageData.thetaDot = 0.0  # [rad]
    facetRotAngle2Message = messaging.HingedRigidBodyMsg().write(facetRotAngle2MessageData)


#. For articulating facets, the user must configure the module's optional ``articulatedFacetDataInMsgs`` input message by calling the ``addArticulatedFacet()`` method with each facet's ``HingedRigidBodyMsgPayload`` articulation angle input message::

    srpEffector.addArticulatedFacet(facetRotAngle1Message)
    srpEffector.addArticulatedFacet(facetRotAngle1Message)
    srpEffector.addArticulatedFacet(facetRotAngle2Message)
    srpEffector.addArticulatedFacet(facetRotAngle2Message)

#. Next, define the spacecraft facet geometry information that is contained in the module's ``FacetedSRPSpacecraftGeometryData`` structure::

    # Define facet areas
    area1 = 1.5 * 1.5
    area2 = np.pi * (0.5 * 7.5) * (0.5 * 7.5)
    facetAreaList = [area1, area1, area1, area1, area1, area1, area2, area2, area2, area2]

    # Define the initial facet attitudes relative to B frame
    sigma_F01B = (macros.D2R * -90.0) * np.array([0.0, 0.0, 1.0])
    sigma_F02B = (macros.D2R * 0.0) * np.array([0.0, 0.0, 1.0])
    sigma_F03B = (macros.D2R * 90.0) * np.array([0.0, 0.0, 1.0])
    sigma_F04B = (macros.D2R * 180.0) * np.array([0.0, 0.0, 1.0])
    sigma_F05B = (macros.D2R * 90.0) * np.array([1.0, 0.0, 0.0])
    sigma_F06B = (macros.D2R * -90.0) * np.array([1.0, 0.0, 0.0])
    sigma_F07B = (macros.D2R * 0.0) * np.array([1.0, 0.0, 0.0])
    sigma_F08B = (macros.D2R * 180.0) * np.array([1.0, 0.0, 0.0])
    sigma_F09B = (macros.D2R * 0.0) * np.array([1.0, 0.0, 0.0])
    sigma_F010B = (macros.D2R * 180.0) * np.array([1.0, 0.0, 0.0])
    facetDcm_F0BList = [rbk.MRP2C(sigma_F01B),
                        rbk.MRP2C(sigma_F02B),
                        rbk.MRP2C(sigma_F03B),
                        rbk.MRP2C(sigma_F04B),
                        rbk.MRP2C(sigma_F05B),
                        rbk.MRP2C(sigma_F06B),
                        rbk.MRP2C(sigma_F07B),
                        rbk.MRP2C(sigma_F08B),
                        rbk.MRP2C(sigma_F09B),
                        rbk.MRP2C(sigma_F010B)]

    # Define the facet normal vectors in B frame components
    facetNHat_FList = [np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0])]

    # Define facet articulation axes in B frame components
    facetRotHat_FList = [np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([1.0, 0.0, 0.0]),
                         np.array([-1.0, 0.0, 0.0]),
                         np.array([1.0, 0.0, 0.0]),
                         np.array([-1.0, 0.0, 0.0])]

    # Define facet center of pressure locations relative to point B
    facetR_CopB_BList = [np.array([0.75, 0.0, 0.0]),
                         np.array([0.0, 0.75, 0.0]),
                         np.array([-0.75, 0.0, 0.0]),
                         np.array([0.0, -0.75, 0.0]),
                         np.array([0.0, 0.0, 0.75]),
                         np.array([0.0, 0.0, -0.75]),
                         np.array([4.5, 0.0, 0.75]),
                         np.array([4.5, 0.0, 0.75]),
                         np.array([-4.5, 0.0, 0.75]),
                         np.array([-4.5, 0.0, 0.75])]

    # Define facet optical coefficients
    facetDiffuseCoeffList = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    facetSpecularCoeffList = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

.. important::
    While setting up the ``FacetedSRPSpacecraftGeometryData`` structure:

    - Be sure to configure all data contained in the geometry data structure. For all static facets, the articulation axes may be set to zero.
    - Note that the module requires the articulated facet data to be added at the end of the data vectors.

#. Next, populate the module's ``FacetedSRPSpacecraftGeometryData`` structure with the spacecraft facet information using the ``addFacet()`` method::

    for i in range(numFacets):
        srpEffector.addFacet(facetAreaList[i],
                             facetDcm_F0BList[i],
                             facetNHat_FList[i],
                             facetRotHat_FList[i],
                             facetR_CopB_BList[i],
                             facetDiffuseCoeffList[i],
                             facetSpecularCoeffList[i])

#. Connect the Sun's ephemeris message to the SRP module::

    SRPEffector.sunInMsg.subscribeTo(sunMsg)

#. Add the SRP dynamic effector to the spacecraft::

    scObject.addDynamicEffector(SRPEffector)

   See :ref:`spacecraft` documentation on how to set up a spacecraft object.

#. Finally, add the SRP effector module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, SRPEffector)

.. note::
    See the example script :ref:`scenarioSepMomentumManagement`, which illustrates how to set up a spacecraft with articulated panels for SRP calculation.
