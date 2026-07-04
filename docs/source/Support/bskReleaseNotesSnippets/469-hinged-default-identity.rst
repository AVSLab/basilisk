- Fixed :ref:`hingedRigidBodyStateEffector` and :ref:`nHingedRigidBodyStateEffector` constructors that
  called ``Eigen``'s static ``Identity()`` factory as a statement and discarded the result, leaving the
  default ``dcm_HB`` (and ``IPntS_S`` for the single hinged effector) uninitialized instead of identity
  (issue #469).
