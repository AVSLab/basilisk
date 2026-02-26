Executive Summary
-----------------
The faceted spacecraft module maps spacecraft facet geometry from the local facet frames to the spacecraft hub
body frame. The module supports both fixed facets and single-axis articulating facets. For all facets, the facet
geometry information is provided in each local facet frame as a vector of :ref:`FacetElementMsgPayload` input messages.
Additionally, for articulated facets the current facet articulation angles must be provided as a vector
of :ref:`HingedRigidBodyMsgPayload` input messages. After mapping the facet information to the spacecraft body frame,
the transformed facet geometry data is written to the vector of module :ref:`FacetElementBodyMsgPayload` output
messages.

.. important::
    The total number of facets must be set by the user using the ``setNumTotalFacets()`` setter method
    before connecting the facet geometry messages.

.. important::
    The module assumes all articulating facets are added first, in order, to the ``facetElementInMsgs`` vector of
    facet geometry :ref:`FacetElementMsgPayload` input messages. Therefore, the first ``numArticulatedFacets`` entries
    of the module ``facetElementInMsgs`` correspond to the articulating facets. In addition, the facet articulation
    angle :ref:`HingedRigidBodyMsgPayload` input messages must be set using the ``addArticulatedFacet()`` method.


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
    * - articulatedFacetDataInMsgs
      - :ref:`HingedRigidBodyMsgPayload`
      - (Optional) Input msg vector for the articulating facet angles
    * - facetElementInMsgs
      - :ref:`FacetElementMsgPayload`
      - Input msg vector containing all facet geometry data expressed in each local facet frame
    * - facetElementBodyOutMsgs
      - :ref:`FacetElementBodyMsgPayload`
      - Output msg vector containing all facet geometry data expressed in the spacecraft body frame


Module Functions
----------------
Below is a list of functions this simulation module performs:

    - Reads and stores all ``FacetElementMsgPayload`` facet geometry data at reset
    - Computes fixed-facet body-frame geometry once at reset
    - Reads articulating facet angles at each time update
    - Computes body-frame facet geometry for articulating facets at each time update
    - Writes one ``FacetElementBodyMsgPayload`` output message for each facet


Module Assumptions and Limitations
----------------------------------
    - The user must set the total number of facets using the ``setNumTotalFacets()`` method before connecting facet messages.
    - All facet element input messages are expected to be linked and written before initialization.
    - Articulating facets are assumed to have 1 DOF rotation about their defined rotation axes ``rotHat_F``.
    - Articulating facets must be indexed first in ``facetElementInMsgs``.
    - For each articulating facet, a valid articulation angle message must be available at each time update.


Test Description and Success Criteria
-------------------------------------
The unit test for this module is located in :ref:`test_facetedSpacecraftModel`. The test verifies that the module
correctly transforms facet geometry data from the local facet frames to the spacecraft hub body frame. This test
sets up the module with two facets. The first facet articulates while the second is fixed.

The simulation is executed in two chunks. Between chunks, the articulating facet angle message is updated to verify
the module correctly handles facet articulation changes. The test is parameterized over the initial,
intermediate, and final articulating facet angles, and the fixed facet initial angle. The test checks that the facet
geometry data is correctly transformed to the spacecraft hub body frame. The specific variables checked are the facet
center of pressure locations ``r_CopB_B``, the facet normal vectors ``nHat_B``, and the facet articulation
axes ``rotHat_B``. The simulation values are checked to match the computed truth values with an absolute tolerance
of ``1e-12``.


User Guide
----------
The following steps are required to set up the ``facetedSpacecraftModel`` module in Python.

#. Import the module::

    from Basilisk.simulation import facetedSpacecraftModel

#. Create the module and set the total number of facets::

    faceted_sc_model = facetedSpacecraftModel.FacetedSpacecraftModel()
    faceted_sc_model.ModelTag = "facetedSCModel"
    faceted_sc_model.setNumTotalFacets(1)

#. For each articulating facet, create a ``HingedRigidBodyMsgPayload`` message and add it::

    articulated_facet_angle_message_data = messaging.HingedRigidBodyMsgPayload()
    articulated_facet_angle_message_data.theta = 10 * macros.D2R  # [rad]
    articulated_facet_angle_message_data.thetaDot = 0.0  # [rad]
    articulated_facet_angle_message = messaging.HingedRigidBodyMsg().write(articulated_facet_angle_message_data)
    faceted_sc_model.addArticulatedFacet(articulated_facet_angle_message)

#. Create and connect the facet geometry input messages. For example::

    articulated_facet_element_message_data = messaging.FacetElementMsgPayload(
        area = 0.5,
        r_CopF_F = np.array([-0.1, 0.1, -0.1]),
        nHat_F = np.array([1.0, 0.0, 0.0]),
        rotHat_F = np.array([0.0, 1.0, 0.0]),
        dcm_F0B = facet_dcm_F0B,
        r_FB_B = np.array([0.0, 1.0, 0.0]),
        c_diffuse = 0.1,
        c_specular = 0.9,
    )
    articulated_facet_element_message = messaging.FacetElementMsg().write(articulated_facet_element_message_data)
    faceted_sc_model.facetElementInMsgs[0].subscribeTo(articulated_facet_element_message)


#. Add the module to a task::

    sim.AddModelToTask(task_name, faceted_sc_model)

#. Optionally, log each body frame facet output message::

    facet_element_body_data_log = []
    for outMsg in faceted_sc_model.facetElementBodyOutMsgs:
        facet_element_body_data_log.append(outMsg.recorder())
        test_sim.AddModelToTask(task_name, facet_element_body_data_log[-1])
