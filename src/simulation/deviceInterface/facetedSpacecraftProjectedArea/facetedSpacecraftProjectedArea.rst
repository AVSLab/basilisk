Executive Summary
-----------------
The faceted spacecraft projected area module computes the per-facet and total projected area of a faceted spacecraft
model using a provided heading direction vector. The module can be configured for three different heading direction
types. The first (most direct) option is to provide a general heading direction using the module
:ref:`BodyHeadingMsgPayload` input message. In this case, the heading vector must be provided relative to and
expressed in the hub body frame B. The second heading configuration option is for the Sun direction heading.
In this case, the facet sunlit areas are computed. This option requires two module input messages
:ref:`SpicePlanetStateMsgPayload` and :ref:`SCStatesMsgPayload` to be configured. The spice input message
must contain the Sun inertial state information, while the spacecraft state message contains the spacecraft hub
inertial state information. The third heading option is to configure the velocity heading for the exposed area to
drag, which requires setting the module :ref:`SCStatesMsgPayload` input message. In this case, the heading vector
is set as the direction of the spacecraft hub's inertial velocity vector. These three configuration options are
summarized below:

1. **Direct General Heading**: provide a general body-frame heading direction using the module
   :ref:`BodyHeadingMsgPayload` input message.
2. **Sun Direction Heading** (sunlit projected area): compute the sun direction heading using two module input messages:
   :ref:`SpicePlanetStateMsgPayload` and :ref:`SCStatesMsgPayload`.
3. **Spacecraft Inertial Velocity Heading** (projected area exposed to drag): compute the spacecraft inertial velocity
    heading using the :ref:`SCStatesMsgPayload` input message.

.. important::
    The total number of facets must be set by the user using the ``setNumFacets()`` setter method before connecting
    the facet input messages.

.. important::
    Exactly one heading configuration must be selected. Configuration of multiple conflicting input messages
    is not permitted.


Message Connection Descriptions
-------------------------------
The following table lists all module input and output messages.
The module msg connections are set by the user from Python.
The msg type contains a link to the message structure definition, while the description
provides information on what each message is used for.

.. list-table:: Module I/O Messages
    :widths: 30 25 45
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - bodyHeadingInMsg
      - :ref:`BodyHeadingMsgPayload`
      - (Optional) Input message for the direct body heading vector
    * - spacecraftStateInMsg
      - :ref:`SCStatesMsgPayload`
      - (Optional) Input message for the spacecraft inertial state. Used to compute either the velocity-based heading or the sun-direction heading
    * - sunStateInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - (Optional) Input message for the Sun inertial state used with ``spacecraftStateInMsg`` to compute a sun-direction heading
    * - facetElementBodyInMsgs
      - :ref:`FacetElementBodyMsgPayload`
      - (Required) Input msg vector containing all facet geometry expressed in the spacecraft hub body frame
    * - facetProjectedAreaOutMsgs
      - :ref:`ProjectedAreaMsgPayload`
      - Output msg vector containing the projected area for each facet
    * - totalProjectedAreaOutMsg
      - :ref:`ProjectedAreaMsgPayload`
      - Output msg containing the total projected area summed over all facets


Module Functions
----------------
Below is a list of functions this module performs:

    - Reads the selected heading source and computes a body-frame heading unit vector
    - Reads the facet geometry for each facet (area and normal vector)
    - Computes the individual projected area for each facet. Negative projected areas are set to zero
    - Computes the total projected area for all facets by summing all individual facet projected areas
    - Writes a single and a vector of :ref:`ProjectedAreaMsgPayload` output messages for the total projected area and per-facet projected areas, respectively


Module Assumptions and Limitations
----------------------------------
    - The user must set the total number of facets using the ``setNumFacets()`` method before connecting facet messages.
    - Exactly one heading configuration must be selected (direct heading, velocity-based heading, or sun-direction heading).
    - The facet normal vectors provided in :ref:`FacetElementBodyMsgPayload` are assumed to be expressed in the spacecraft hub body frame.
    - The projected area for each facet is clamped to be non-negative using ``max(0, cos(theta))``.


Test Description and Success Criteria
-------------------------------------
The unit test for this module is located in :ref:`test_facetedSpacecraftProjectedArea`. The test verifies that the
module correctly computes the per-facet and total projected area of a faceted spacecraft model using a provided
heading direction vector. This test sets up a simulation for each of the three heading configuration options.
All tests set up the faceted spacecraft projected area BSK module identically with two different facets.
The test checks that the computed truth values match the values output from the module.


User Guide
----------
The following steps are required to set up the ``facetedSpacecraftProjectedArea`` module in Python.

#. Import the module::

    from Basilisk.simulation import facetedSpacecraftProjectedArea

#. Create the module and set the total number of facets::

    faceted_sc_projected_area = facetedSpacecraftProjectedArea.FacetedSpacecraftProjectedArea()
    faceted_sc_projected_area.ModelTag = "facetedSpacecraftProjectedArea"
    faceted_sc_projected_area.setNumFacets(2)

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

#. Subscribe the facet geometry input messages to the module ``facetElementBodyInMsgs`` input message vector::

    faceted_sc_projected_area.facetElementBodyInMsgs[0].subscribeTo(facet_1_message)
    faceted_sc_projected_area.facetElementBodyInMsgs[1].subscribeTo(facet_2_message)

#. Select exactly one heading configuration:

   - **Option 1: Direct General Heading** (general heading direction provided in :math:`\mathcal{B}`)::

        body_heading_message_data = messaging.BodyHeadingMsgPayload()
        body_heading_message_data.rHat_XB_B = np.array([1.0, 0.0, 0.0])
        body_heading_message = messaging.BodyHeadingMsg().write(body_heading_message_data)
        faceted_sc_projected_area.bodyHeadingInMsg.subscribeTo(body_heading_message)

   - **Option 2: Sun Direction Heading** (sunlit projected area; requires both :ref:`SCStatesMsgPayload` and :ref:`SpicePlanetStateMsgPayload`)::

        spacecraft_state_message_data = messaging.SCStatesMsgPayload()
        spacecraft_state_message_data.r_BN_N = np.array([-4020338.690396649, 7490566.741852513, 5248299.211589362])  # [m]
        spacecraft_state_message_data.sigma_BN = np.array([0.0, 0.0, 0.0])
        spacecraft_state_message = messaging.SCStatesMsg().write(spacecraft_state_message_data)
        faceted_sc_projected_area.spacecraftStateInMsg.subscribeTo(spacecraft_state_message)

        sun_state_message_data = messaging.SpicePlanetStateMsgPayload()
        sun_state_message_data.PositionVector = np.array([1.0, -10.0, 50.0])  # [m]
        sun_state_message = messaging.SpicePlanetStateMsg().write(sun_state_message_data)
        faceted_sc_projected_area.sunStateInMsg.subscribeTo(sun_state_message)

   - **Option 3: Spacecraft Inertial Velocity Heading** (projected area exposed to drag; requires :ref:`SCStatesMsgPayload`)::

        spacecraft_state_message_data = messaging.SCStatesMsgPayload()
        spacecraft_state_message_data.v_BN_N = np.array([-5199.77710904224, -3436.681645356935, 1041.576797498721])  # [m/s]
        spacecraft_state_message_data.sigma_BN = np.array([0.0, 0.0, 0.0])
        spacecraft_state_message = messaging.SCStatesMsg().write(spacecraft_state_message_data)
        faceted_sc_projected_area.spacecraftStateInMsg.subscribeTo(spacecraft_state_message)

#. Add the module to a task::

    sim.AddModelToTask(task_name, faceted_sc_projected_area)
