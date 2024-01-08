Executive Summary
-----------------
This dynamic effector module uses a faceted spacecraft model to calculate the force and torque acting on a spacecraft
due to solar radiation pressure (SRP). The force and torque are calculated about the spacecraft body frame origin
point :math:`B`. The module can be configured for either a static spacecraft or a spacecraft with any number of
articulating facets. For example, a spacecraft with two articulating solar arrays can be configured using 4
articulating facets. The unit test for this module shows how to set up this particular configuration.

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
      - Input msg with the Sun state information
    * - articulatedFacetDataInMsgs
      - :ref:`HingedRigidBodyMsgPayload`
      - (Optional) Input message with facet articulation angle information

Detailed Module Description
---------------------------

Mathematical Modeling
^^^^^^^^^^^^^^^^^^^^^
The spacecraft is represented as a collection of :math:`N` total facets with negligible thickness. Each facet is
characterized by an area :math:`A`, a unit vector normal to its surface :math:`\boldsymbol{\hat{n}}` expressed in
:math:`\mathcal{B}` frame components, a position vector from the spacecraft body frame origin point :math:`B` to the
facet center of pressure :math:`\boldsymbol{r}_{COP/B}` expressed in :math:`\mathcal{B}` frame components,
an optional unit vector expressed in :math:`\mathcal{B}` frame components representing the articulation axis of any
additional articulating facets, and three optical coefficients representing the interaction of impinging photons with
the facet surface. The fraction of specularly reflected, diffusely scattered, and absorbed photons are represented
using the coefficients :math:`\delta, \rho,` and :math:`\alpha`, respectively.

For each articulating facet, the current facet normal vector is computed using the facet articulation axis
:math:`\boldsymbol{\hat{a}}` and the corresponding facet articulation angle :math:`\phi`. The facet articulation
angle is obtained from the ``articulatedFacetDataInMsgs`` input message data. The direction cosine matrix (DCM)
required to rotate the given facet normal vector through the current facet articulation angle is obtained using a
principal rotation vector (PRV) transformation where:

.. math::
    [\mathcal{B}_0\mathcal{B}] = \text{PRV2C}(\phi, \boldsymbol{\hat{a}})

and

.. math::
    {}^\mathcal{B} \boldsymbol{\hat{n}} = [\mathcal{B}\mathcal{B}_0] {}^{\mathcal{B}_0} \boldsymbol{\hat{n}}

Using the spacecraft facet information and spacecraft Sun-relative position vector
:math:`\boldsymbol{r}_{\text{sc} / \odot }`, the estimated SRP force acting on the spacecraft is calculated
by summing the SRP force contribution from all :math:`N` facets:

.. math::
    \boldsymbol{F}_{\text{SRP}} = \sum_{i = 1}^{N} \boldsymbol{F}_{\text{SRP}_i} = -P(|\boldsymbol{r}_{\text{sc} / \odot }|) \sum_{i = 1}^{N} A_i \cos(\theta_i) \left [ (1 - \delta_i) \boldsymbol{\hat{s}} + 2 \left ( \frac{\rho_i}{3} + \delta_i \cos(\theta_i) \right ) \boldsymbol{\hat{n}}_{i}\right ]

Where :math:`\theta` is defined as the incidence angle between each facet normal vector and the
Sun-direction vector, and :math:`P(|\boldsymbol{r}_{\text{sc}/ \odot\ }|)` is the pressure acting on the spacecraft
scaled by the spacecraft heliocentric distance. Because the Sun and spacecraft inertial positions are given in
inertial frame components, the current spacecraft attitude :math:`\sigma_{\mathcal{B} / \mathcal{N}}` is used
to determine the current DCM of the spacecraft body frame relative to the inertial frame, :math:`[\mathcal{BN}]`.
Note that all vector quantities must be expressed in the spacecraft body frame for the SRP force calculation.
:math:`\boldsymbol{\hat{s}}` is the unit direction vector pointing radially towards the Sun from the
spacecraft body frame origin point :math:`B`. This vector is found by subtracting the current spacecraft inertial
position from the Sun position given from the Sun Spice input message:

.. math::
    {}^\mathcal{B} \boldsymbol{\hat{s}} = [\mathcal{BN}] ( {}^\mathcal{N} \boldsymbol{r}_{\odot / N} - {}^\mathcal{N} \boldsymbol{r}_{\text{sc} / N})

The total torque acting on the spacecraft about point :math:`B` due to SRP is calculated by summing the torque
contributions over all :math:`N` facets:

.. math::
    \boldsymbol{L}_{\text{SRP},B} = \sum_{i = 1}^{N} \boldsymbol{L}_{{\text{SRP},B}_i} = \sum_{i = 1}^{N} \boldsymbol{r}_{{COP_i}/B} \times \boldsymbol{F}_{\text{SRP}_i}

Module Testing
^^^^^^^^^^^^^^
The unit test for this module ensures that the calculated SRP force and torque acting on the spacecraft about the
body-fixed point B is properly computed for either a static spacecraft or a spacecraft with any number of articulating
facets. The spacecraft geometry defined in the test consists of a cubic hub and two circular solar arrays.
Six static square facets represent the cubic hub and four articulated circular facets describe the articulating
solar arrays. To validate the module functionality, the final SRP force simulation value is checked with the
true value computed in python.

User Guide
----------
The following steps are required to set up the faceted SRP dynamic effector in python using Basilisk. Be sure to include
the Sun as a gravitational body in the simulation to use this module.

#. First import the facetSRPDynamicEffector class::

    from Basilisk.simulation import facetSRPDynamicEffector

#. Next, create an instantiation of the SRP dynamic effector::

    SRPEffector = facetSRPDynamicEffector.FacetSRPDynamicEffector()
    SRPEffector.ModelTag = "SRPEffector"

#. The user is required to set the total number of spacecraft facets and the number of articulated facets. For example, if the user wants to create a spacecraft with 10 total facets, four of which articulate; the user would set these module variables to::

    SRPEffector.numFacets = 10
    SRPEffector.numArticulatedFacets = 4

#. If the spacecraft contains articulated facets, a ``HingedRigidBodyMsgPayload`` articulation angle message must be configured for each articulated facet. An example using two constant stand-alone messages is provided below::

    facetRotAngle1 = macros.D2R * 10.0  # [rad]
    facetRotAngle2 = macros.D2R * -10.0  # [rad]

    facetRotAngle1MessageData = messaging.HingedRigidBodyMsgPayload()
    facetRotAngle1MessageData.theta = facetRotAngle1
    facetRotAngle1MessageData.thetaDot = 0.0
    facetRotAngle1Message = messaging.HingedRigidBodyMsg().write(facetRotAngle1MessageData)

    facetRotAngle2MessageData = messaging.HingedRigidBodyMsgPayload()
    facetRotAngle2MessageData.theta = facetRotAngle2
    facetRotAngle2MessageData.thetaDot = 0.0
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
    facetAreas = [area1, area1, area1, area1, area1, area1, area2, area2, area2, area2]

    # Define the facet normal vectors in B frame components
    facetNormals_B = [np.array([1.0, 0.0, 0.0]),
                          np.array([0.0, 1.0, 0.0]),
                          np.array([-1.0, 0.0, 0.0]),
                          np.array([0.0, -1.0, 0.0]),
                          np.array([0.0, 0.0, 1.0]),
                          np.array([0.0, 0.0, -1.0]),
                          np.array([0.0, 1.0, 0.0]),
                          np.array([0.0, -1.0, 0.0]),
                          np.array([0.0, 1.0, 0.0]),
                          np.array([0.0, -1.0, 0.0])]

    # Define facet center of pressure locations relative to point B
    locationsPntB_B = [np.array([0.75, 0.0, 0.0]),
                       np.array([0.0, 0.75, 0.0]),
                       np.array([-0.75, 0.0, 0.0]),
                       np.array([0.0, -0.75, 0.0]),
                       np.array([0.0, 0.0, 0.75]),
                       np.array([0.0, 0.0, -0.75]),
                       np.array([4.5, 0.0, 0.75]),
                       np.array([4.5, 0.0, 0.75]),
                       np.array([-4.5, 0.0, 0.75]),
                       np.array([-4.5, 0.0, 0.75])]

    # Define facet articulation axes in B frame components
    rotAxes_B = [np.array([0.0, 0.0, 0.0]),
                 np.array([0.0, 0.0, 0.0]),
                 np.array([0.0, 0.0, 0.0]),
                 np.array([0.0, 0.0, 0.0]),
                 np.array([0.0, 0.0, 0.0]),
                 np.array([0.0, 0.0, 0.0]),
                 np.array([1.0, 0.0, 0.0]),
                 np.array([1.0, 0.0, 0.0]),
                 np.array([-1.0, 0.0, 0.0]),
                 np.array([-1.0, 0.0, 0.0])]

    # Define facet optical coefficients
    specCoeff = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    diffCoeff = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

.. important::
    Note that in order to use this module, the facet articulation axes must always be configured regardless of whether
    articulated facets are considered. For all static facets, the articulation axes must be set to zero. Ensure that the
    specified number of articulated facets matches the number of nonzero articulation axes.

.. important::
    The module requires the articulated facet data to be added at the end of the facet data vectors.

#. Populate the module's ``FacetedSRPSpacecraftGeometryData`` structure with the spacecraft facet information using the ``addFacet()`` method::

    for i in range(numFacets)):
        SRPEffector.addFacet(facetAreas[i], specCoeff[i], diffCoeff[i], facetNormals_B[i], locationsPntB_B[i], rotAxes_B[i])

#. Connect the Sun's ephemeris message to the SRP module::

    SRPEffector.sunInMsg.subscribeTo(sunMsg)

#. Add the SRP dynamic effector to the spacecraft::

    scObject.addDynamicEffector(SRPEffector)

   See :ref:`spacecraft` documentation on how to set up a spacecraft object.

#. Finally, add the SRP effector module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, SRPEffector)

.. note::
    See the example script :ref:`scenarioSepMomentumManagement`, which illustrates how to set up a spacecraft with articulated panels for SRP calculation.
