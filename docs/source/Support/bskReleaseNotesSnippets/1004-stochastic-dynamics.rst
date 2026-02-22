- :beta:`Stochastic Integration`: Added support for stochastic dynamics (dynamics driven by Stochastic Differential Equations).
- :beta:`Stochastic Integration`: Added stochastic dynamics support in core dynamics infrastructure (``DynamicObject``, ``StateData``,
  and ``DynParamManager``), including diffusion propagation APIs and support for shared noise sources.
- :beta:`Stochastic Integration`: Added diffusion dynamics task support to ``MJScene`` via ``AddModelToDiffusionDynamicsTask()`` for
  MuJoCo systems with stochastic state dynamics.
- :beta:`Stochastic Integration`: Added stochastic integrator base classes ``StateVecStochasticIntegrator``.
- :beta:`Stochastic Integration`: Added ``svIntegratorWeakStochasticRungeKutta`` to support stochastic integrator implementations.
- :beta:`Stochastic Integration`: Added :ref:`svStochasticIntegratorMayurama` (Euler-Mayurama), a 1-weak/1-strong stochastic integrator.
- :beta:`Stochastic Integration`: Added two stochastic integrators: :ref:`svStochasticIntegratorW2Ito1` and :ref:`svStochasticIntegratorW2Ito2`. These are 2-weak-order stochastic integrators.
- :beta:`Stochastic Integration`: Added MuJoCo models (:ref:`meanRevertingNoise`, :ref:`stochasticAtmDensity`, and
  :ref:`stochasticDragCoeff`) to model mean-reverting stochastic processes in dynamics-task modules.
- :beta:`Stochastic Integration`: Added :ref:`meanRevertingNoiseStateEffector`, a generic spacecraft state effector that propagates a scalar
  Ornstein-Uhlenbeck mean-reverting stochastic state.
- :beta:`Stochastic Integration`: Added :ref:`scenarioStochasticDrag` and :ref:`scenarioStochasticDragSpacecraft`, which illustrate how to
  use a state driven by stochastic dynamics to model randomly evolving atmospheric density (and thus drag force).
  The former scenario uses MuJoCo dynamics and the latter spacecraft dynamics.
- :beta:`Stochastic Integration`: Added optional density correction support to :ref:`dragDynamicEffector` through
  ``densityCorrectionStateName``, allowing drag density to be computed as
  ``(1 + correction) * neutralDensity`` using a user-selected scalar state.
