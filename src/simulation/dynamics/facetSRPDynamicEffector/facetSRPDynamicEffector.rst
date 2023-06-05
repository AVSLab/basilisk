Executive Summary
-----------------
This new dynamic effector module uses a static faceted spacecraft model to calculate the force and torque acting on a
spacecraft due to solar radiation pressure (SRP). The force and torque acting on the spacecraft are defined with respect
to the spacecraft body frame origin :math:`B`.

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

Detailed Module Description
---------------------------

Mathematical Modeling
^^^^^^^^^^^^^^^^^^^^^
The spacecraft is represented as a collection of :math:`n` facets with negligible thickness. Each facet is characterized
by an area :math:`A`, a vector normal to its surface :math:`\boldsymbol{\hat{n}}`, a position vector from the
spacecraft body frame origin, point :math:`B` to the facet center of pressure, :math:`\boldsymbol{r}_{F/B}`, and three optical coefficients
representing the interaction of impinging photons with the facet surface. The fraction of specularly reflected,
diffusely scattered, and absorbed photons are represented using the coefficients :math:`\delta, \rho,` and
:math:`\alpha`, respectively.

A faceted force model is used to estimate the SRP force acting on the spacecraft:

.. math::
    \boldsymbol{F}_{\text{SRP}} = \sum_{i = 1}^{n} \boldsymbol{F}_{\text{SRP}_i} = -P(|\boldsymbol{r}_{\text{sc} / \odot }|) \sum_{i = 1}^{n} A_i \cos(\theta_i) \left [ (1 - \delta_i) \boldsymbol{\hat{s}} + 2 \left ( \frac{\rho_i}{3} + \delta_i \cos(\theta_i) \right ) \boldsymbol{\hat{n}}_{i}\right ]

Note that all vector quantities are expressed in the spacecraft principal body-frame components for the SRP force
calculation. :math:`\boldsymbol{\hat{s}}` is the unit direction vector pointing radially towards the Sun from the
spacecraft body-frame origin. This vector is found by subtracting the current spacecraft inertial position from the
Sun position given from the Spice input message:

.. math::
    {}^B \boldsymbol{\hat{s}} = [BN] ( {}^N \boldsymbol{r}_{\odot / N} - {}^N \boldsymbol{r}_{\text{sc} / N})

Note that both the Sun and spacecraft inertial positions are given in inertial frame components. The current spacecraft
attitude, :math:`\sigma_{\mathcal{B} / \mathcal{N}}` is used to find the current Direction Cosine Matrix (DCM) of the
spacecraft body frame relative to the inertial frame, :math:`[BN]`.

:math:`\theta` is defined as the incidence angle between each facet normal vector and the
Sun-direction vector, and :math:`P(|\boldsymbol{r}_{\text{sc}/ \odot\ }|)` is the pressure acting on the spacecraft
scaled by the spacecraft heliocentric distance.

The total torque acting on the spacecraft about point :math:`B` due to SRP is calculated by summing the torque
contributions over all :math:`n` facets:

.. math::
    \boldsymbol{L}_{\text{SRP},B} = \sum_{i = 1}^{n} \boldsymbol{L}_{{\text{SRP},B}_i} = \sum_{i = 1}^{n} \boldsymbol{r}_{F_i/B} \times \boldsymbol{F}_{\text{SRP}_i}

Module Testing
^^^^^^^^^^^^^^
The unit test for this module ensures that the calculated Solar Radiation Pressure (SRP) force and torque acting
on the spacecraft about the body-fixed point B is properly computed for a static spacecraft. The spacecraft
geometry defined in this test consists of a cubic hub and two circular solar arrays. Six square facets represent
the cubic hub and four circular facets describe the solar arrays. The SRP force and torque module values are
checked with values computed in python both initially and at a time halfway through the simulation.

The initial module-computed SRP force value ``SRPDataForce_B`` is checked with the value computed in
python ``forceTest1Val``. The initial module-computed SRP torque value ``SRPDataTorque_B`` is also checked
with the value computed in python ``torqueTest1Val``. Similarly, these values halfway through the simulation
are checked to match.

User Guide
----------
The user configures the spacecraft geometry as a collection of :math:`n` facets, each with an area :math:`A`,
a vector normal to its surface :math:`\boldsymbol{\hat{n}}`, a position vector from the spacecraft center of mass to
the facet center of pressure, :math:`\boldsymbol{r}_{F/c}`, and the three optical coefficients
:math:`\delta, \rho,` and :math:`\alpha`, representing the fraction of specularly reflected, diffusely scattered,
and absorbed photons, respectively.

The following steps are needed to setup a faceted SRP dynamic effector in python using Basilisk.

#. Import the facetSRPDynamicEffector class::

    from Basilisk.simulation import facetSRPDynamicEffector

#. Add the Earth and Sun as gravitational bodies to the simulation::

    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    sun = gravFactory.createSun()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    earthIdx = 0
    sunIdx = 1

    earthStateMsg = messaging.SpicePlanetStateMsgPayload()
    earthStateMsg.PositionVector = [0.0, -149598023 * 1000, 0.0]
    earthStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    earthMsg = messaging.SpicePlanetStateMsg().write(earthStateMsg)
    gravFactory.gravBodies['earth'].planetBodyInMsg.subscribeTo(earthMsg)

    sunStateMsg = messaging.SpicePlanetStateMsgPayload()
    sunStateMsg.PositionVector = [0.0, 0.0, 0.0]
    sunStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunStateMsg)
    gravFactory.gravBodies['sun'].planetBodyInMsg.subscribeTo(sunMsg)

#. Create an instantiation of the dynamic effector::

    newSRP = facetSRPDynamicEffector.FacetSRPDynamicEffector()
    newSRP.ModelTag = "FacetSRP"

#. Define the spacecraft geometry of interest::

    # Define the facet surface areas
    area1 = 1.5*1.5  # [m]
    area2 = np.pi*(0.5*7.5)*(0.5*7.5)  # [m]
    facetAreas = [area1, area1, area1, area1, area1, area1, area2, area2, area2, area2]

    # Define the facet normals in B frame components
    facetNormal1 = np.array([1.0, 0.0, 0.0])
    facetNormal2 = np.array([0.0, 1.0, 0.0])
    facetNormal3 = np.array([-1.0, 0.0, 0.0])
    facetNormal4 = np.array([0.0, -1.0, 0.0])
    facetNormal5 = np.array([0.0, 0.0, 1.0])
    facetNormal6 = np.array([0.0, 0.0, -1.0])
    facetNormal7 = np.array([0.0, 1.0, 0.0])
    facetNormal8 = np.array([0.0, -1.0, 0.0])
    facetNormal9 = np.array([0.0, 1.0, 0.0])
    facetNormal10 = np.array([0.0, -1.0, 0.0])
    normals_B = [facetNormal1, facetNormal2, facetNormal3, facetNormal4, facetNormal5, facetNormal6, facetNormal7, facetNormal8, facetNormal9, facetNormal10]

    # Define the facet center of pressure locations with respect to point B in B frame components
    facetLoc1 = np.array([0.75, 0.0, 0.0])  # [m]
    facetLoc2 = np.array([0.0, 0.75, 0.0])  # [m]
    facetLoc3 = np.array([-0.75, 0.0, 0.0])  # [m]
    facetLoc4 = np.array([0.0, -0.75, 0.0])  # [m]
    facetLoc5 = np.array([0.0, 0.0, 0.75])  # [m]
    facetLoc6 = np.array([0.0, 0.0, -0.75])  # [m]
    facetLoc7 = np.array([4.5, 0.0, 0.75])  # [m]
    facetLoc8 = np.array([4.5, 0.0, 0.75])  # [m]
    facetLoc9 = np.array([-4.5, 0.0, 0.75])  # [m]
    facetLoc10 = np.array([-4.5, 0.0, 0.75])  # [m]
    locationsPntB_B = [facetLoc1, facetLoc2, facetLoc3, facetLoc4, facetLoc5, facetLoc6, facetLoc7, facetLoc8, facetLoc9, facetLoc10]

    # Define the facet optical coefficients
    specCoeff = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    diffCoeff = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

#. Populate the scGeometry structure with the defined facet information::

    for i in range(len(facetAreas)):
        newSRP.addFacet(facetAreas[i], specCoeff[i], diffCoeff[i], normals_B[i], locationsPntB_B[i])

Note that for each added facet, the user is required to set the corresponding area, normal vector, location, and optical coefficients parameters in the scGeometry structure.

#. Connect the Sun's ephemeris message to the SRP module::

    newSRP.sunInMsg.subscribeTo(sunMsg)

#. Add the SRP dynamic effector to the spacecraft::

    scObject.addDynamicEffector(newSRP)

   See :ref:`spacecraft` documentation on how to set up a spacecraft object.

#. Set up the spacecraft orbit::

    oe = orbitalMotion.ClassicElements()
    r_eq = 6371*1000.0  # [m]
    rN = np.array([r_eq+2000.0, -149598023 * 1000, 0.0])  # [m]
    vN = np.array([0.0, 7.90854, 0.0])  # [m/s]
    sig_BN = np.array([0.0, 0.0, 0.0])

#. Initialize the spacecraft states with the initialization variables::

    scObject.hub.r_CN_NInit = rN  # [m] r_CN_N
    scObject.hub.v_CN_NInit = vN  # [m] v_CN_N
    scObject.hub.sigma_BNInit = sig_BN

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, newSRP)

