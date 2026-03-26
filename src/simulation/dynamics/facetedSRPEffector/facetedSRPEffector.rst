Executive Summary
-----------------
The faceted spacecraft solar radiation pressure (SRP) module computes the aggregate force and torque acting on the
spacecraft due to impinging photons from the Sun. The inertial Sun state information must be subscribed
to the module ``SpicePlanetStateMsgPayload`` input message. The module assumes the spacecraft is modeled as a
collection of facets, where the facet geometry information is passed as a vector of ``FacetElementBodyMsgPayload``
input messages to the module. The facet geometry information is required to be provided in the spacecraft body frame.
This SRP module does not make any assumptions regarding whether the facets are rigid or articulate.
The ``facetedSpacecraftModel`` module can be connected upstream and used to transform the facet geometry data from the
facet frames to the spacecraft body frame. This upstream module can also be used to configure articulating facets.
Finally, this SRP module also requires the sunlit area of all facets to be pre-computed and connected to the module
vector of ``ProjectedAreaMsgPayload`` input messages. This information can be passed to the SRP module by connecting
the facet geometry information to the ``facetedSpacecraftProjectedArea`` module upstream.

.. important::
    The total number of facets must be set using ``setNumFacets()`` before connecting the facet input message vectors.

.. important::
    All facet geometry and projected area input message vectors must have lengths equal to ``numFacets``.


Message Connection Descriptions
-------------------------------
The following table lists all module input messages.
The module msg connections are set by the user from Python.
The msg type contains a link to the message structure definition, while the description
provides information on what each message is used for.

.. list-table:: Module I/O Messages
    :widths: 30 25 45
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - sunStateInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - Input message containing the Sun inertial state
    * - facetElementBodyInMsgs
      - :ref:`FacetElementBodyMsgPayload`
      - Input message vector containing facet element data expressed in body-frame components
    * - facetProjectedAreaInMsgs
      - :ref:`ProjectedAreaMsgPayload`
      - Input message vector containing pre-computed per-facet projected areas


Module Functions
----------------
Below is a list of functions this simulation module performs:

    - Reads the Sun inertial position and computes the Sun direction vector in spacecraft body-frame components
    - Reads facet body-frame geometry and optical coefficients for each facet
    - Reads per-facet projected areas for each facet
    - Computes SRP pressure scaled by spacecraft heliocentric distance
    - Computes and sums per-facet SRP force and torque contributions
    - Writes total SRP force and torque to the dynamic effector base-class outputs ``forceExternal_B`` and ``torqueExternalPntB_B``


Module Assumptions and Limitations
----------------------------------
    - The user must call ``setNumFacets()`` before connecting message vectors
    - This module assumes facet normals and center-of-pressure vectors are already expressed in the spacecraft body frame
    - This module expects projected areas to be provided externally; it does not compute projected area internally
    - This module does not read articulation-angle messages directly
    - Facets with non-positive projected area contribute zero SRP force and torque


Test Description and Success Criteria
-------------------------------------
The unit test for this module is located in :ref:`test_facetedSRPEffector`. The test verifies that the module
correctly computes the aggregate SRP force and torque acting on the spacecraft due to impinging photons
from the Sun. The test sets up a simulation with a faceted spacecraft modeled as a cubic hub with two attached
circular solar arrays. Six square facets represent the cubic hub and four circular facets represent the two solar
arrays. The test varies the initial state information of both the spacecraft and the Sun. Specifically, the test
parameterizes:

    - Spacecraft initial inertial position
    - Sun inertial position
    - Spacecraft initial attitude
    - Spacecraft initial angular velocity

The test checks that the computed SRP forces and torques at each timestep match the values output from the module.

User Guide
----------
The following steps are required to set up the ``facetedSRPEffector`` module in Python.

.. note::
    A common setup is to connect ``facetedSpacecraftModel`` upstream to provide body-frame facet geometry and connect
    ``facetedSpacecraftProjectedArea`` upstream to provide per-facet projected areas.

#. Import the required BSK modules::

    from Basilisk.simulation import facetedSRPEffector
    from Basilisk.simulation import facetedSpacecraftProjectedArea
    from Basilisk.simulation import spacecraft

#. Create the spacecraft object and set its initial states::

    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "scObject"
    sc_object.hub.mHub = 750.0  # [kg]
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [1.0]]  # [m]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg m^2]
    sc_object.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]  # [m]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]  # [m/s]
    sc_object.hub.sigma_BNInit = [0.0, 0.0, 0.0]  # [-]
    sc_object.hub.omega_BN_BInit = [0.0, 0.0, 0.0]  # [rad/s]

#. Create the SRP module and set the number of facets::

    faceted_srp_effector = facetedSRPEffector.FacetedSRPEffector()
    faceted_srp_effector.ModelTag = "facetedSRPEffector"
    faceted_srp_effector.setNumFacets(2)

#. Create the sun and connect the Sun inertial state message::

    sun_state_message_data = messaging.SpicePlanetStateMsgPayload()
    sun_state_message_data.PositionVector = [0.0, 0.0, 0.0]  # [m]
    sun_state_message_data.VelocityVector = [0.0, 0.0, 0.0]  # [m/s]
    sun_state_message = messaging.SpicePlanetStateMsg().write(sun_state_message_data)

    grav_factory = simIncludeGravBody.gravBodyFactory()
    sun = grav_factory.createSun()
    sun.isCentralBody = True
    grav_factory.gravBodies['sun'].planetBodyInMsg.subscribeTo(sun_state_message)
    faceted_srp_effector.sunStateInMsg.subscribeTo(sun_state_message)

#. Create a :ref:`FacetElementBodyMsgPayload` facet geometry input message for each facet::

    facet_area_list = [0.5, 1.0]  # [m^2]
    facet_r_CopB_B_list = [np.array([-0.1, 0.1, -0.1]),
                           np.array([0.1, -0.1, -0.1])]  # [m]
    facet_nHat_B_list = [np.array([1.0, 0.0, 0.0]),
                         np.array([0.0, 1.0, 0.0])]
    facet_rotHat_B_list = [np.array([0.0, 1.0, 0.0]),
                           np.array([1.0, 0.0, 0.0])]
    facet_diffuse_coeff_list = [0.1, 0.1]
    facet_specular_coeff_list = [0.9, 0.9]

    facet_1_message_data = messaging.FacetElementBodyMsgPayload(
        area = facet_area_list[0],
        r_CopB_B = facet_r_CopB_B_list[0],
        nHat_B = facet_nHat_B_list[0],
        rotHat_B = facet_rotHat_B_list[0],
        c_diffuse = facet_diffuse_coeff_list[0],
        c_specular = facet_specular_coeff_list[0],
    )
    facet_2_message_data = messaging.FacetElementBodyMsgPayload(
        area = facet_area_list[1],
        r_CopB_B = facet_r_CopB_B_list[1],
        nHat_B = facet_nHat_B_list[1],
        rotHat_B = facet_rotHat_B_list[1],
        c_diffuse = facet_diffuse_coeff_list[1],
        c_specular = facet_specular_coeff_list[1],
    )
    facet_1_message = messaging.FacetElementBodyMsg().write(facet_1_message_data)
    facet_2_message = messaging.FacetElementBodyMsg().write(facet_2_message_data)

#. Subscribe the facet geometry input messages to the SRP module ``facetElementBodyInMsgs`` input message vector::

    faceted_srp_effector.facetElementBodyInMsgs[0].subscribeTo(facet_1_message)
    faceted_srp_effector.facetElementBodyInMsgs[1].subscribeTo(facet_2_message)

#. Create the faceted spacecraft projected area module and set the total number of facets::

    faceted_sc_projected_area = facetedSpacecraftProjectedArea.FacetedSpacecraftProjectedArea()
    faceted_sc_projected_area.ModelTag = "facetedSpacecraftProjectedArea"
    faceted_sc_projected_area.setNumFacets(2)

#. Subscribe the facet geometry input messages to the projected area module ``facetElementBodyInMsgs`` input message vector::

    faceted_sc_projected_area.facetElementBodyInMsgs[0].subscribeTo(facet_1_message)
    faceted_sc_projected_area.facetElementBodyInMsgs[1].subscribeTo(facet_2_message)

#. Subscribe the projected area module ``facetProjectedAreaOutMsgs`` output message vector to the SRP module ``facetProjectedAreaInMsgs`` input message vector::

    faceted_srp_effector.facetProjectedAreaInMsgs[0].subscribeTo(faceted_sc_projected_area.facetProjectedAreaOutMsgs[0])
    faceted_srp_effector.facetProjectedAreaInMsgs[1].subscribeTo(faceted_sc_projected_area.facetProjectedAreaOutMsgs[1])

#. Subscribe other required messages to the projected area module::

    faceted_sc_projected_area.sunStateInMsg.subscribeTo(sun_state_message)
    faceted_sc_projected_area.spacecraftStateInMsg.subscribeTo(sc_object.scStateOutMsg)

#. Add the SRP effector to the spacecraft::

    sc_object.addDynamicEffector(faceted_srp_effector)

#. Add each module to a task::

    test_sim.AddModelToTask(task_name, faceted_sc_projected_area)
    test_sim.AddModelToTask(task_name, sc_object)
    test_sim.AddModelToTask(task_name, faceted_srp_effector)

.. note::
    Add the spacecraft object to the task after the projected area module to ensure the SRP effector has valid input
    message data at the first integration timestep.
