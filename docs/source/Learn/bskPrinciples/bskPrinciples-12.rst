.. _bskPrinciples-12:

Advanced: Stochastic Integration
================================

.. warning::

    :beta:`Stochastic Integration` The following feature is a work in progress!
    Extensive validation has not been performed, and APIs are subject to change
    between releases.

What is stochastic integration?
-------------------------------
Most Basilisk ``DynamicObject`` modules propagate a deterministic *ordinary*
differential equation (ODE), :math:`\dot{x} = f(t,x)`, using a classic integrator
such as :ref:`RK4 <svIntegratorRK4>` (see :ref:`bskPrinciples-9`). Stochastic
integration instead propagates a *stochastic* differential equation (SDE), in which
the state is additionally driven by one or more random (Wiener) processes:

.. math::

    dx = f(t,x)\,dt + \sum_{k=1}^{m} g_k(t,x)\,dW_k

Here :math:`f` is the usual **drift** (the deterministic part), each :math:`g_k` is a
**diffusion** column describing how the :math:`k`-th noise source enters the dynamics,
and :math:`dW_k` are independent increments of a Wiener process (Brownian motion), each
distributed as :math:`\mathcal{N}(0,dt)`. This lets a simulation model process noise,
disturbances, actuator/sensor noise, or any physical effect best represented as a
continuous random forcing rather than a fixed function of time.

Because the driving process is not differentiable, an SDE cannot be integrated with an
ODE method: the stochastic integral :math:`\int g\,dW` must be approximated with
schemes built specifically for SDEs. Basilisk provides a family of such integrators
through the ``StateVecStochasticIntegrator`` interface. They are all **explicit** and
**derivative-free** (no user-supplied Jacobian of :math:`f` or :math:`g` is required):
each integrator only evaluates :math:`f` and :math:`g_k` pointwise at internal stage
states and forms affine combinations of them.

.. note::

    Two mathematical conventions exist for the stochastic integral, **Itô** and
    **Stratonovich**, and they generally give *different* solutions for the same
    coefficients. Each Basilisk integrator targets one convention (see the table
    below); make sure your model is written for the convention of the integrator you
    choose.

Convergence order: strong vs. weak
-----------------------------------
Unlike ODE solvers, SDE solvers come with *two* notions of accuracy, and the right one
depends on what you are computing:

- **Strong order** measures pathwise accuracy: how close the *individual simulated
  trajectory* stays to the true trajectory driven by the *same* noise realization. A
  method has strong order :math:`p` if
  :math:`\mathbb{E}\,\lVert x_N - x(t_N)\rVert \le C\,h^{p}`. Use strong order when the
  trajectory itself matters, e.g. filtering, sensitivity to a specific realized
  disturbance, or comparing against a reference path.

- **Weak order** measures accuracy of *statistics*: how close the expectations of
  functions of the state are, :math:`\lvert \mathbb{E}[\phi(x_N)] -
  \mathbb{E}[\phi(x(t_N))]\rvert \le C\,h^{q}`. Use weak order when you only care about
  moments/distributions computed from a Monte-Carlo ensemble (means, covariances,
  hit probabilities), not the individual sample paths.

A weak-order-:math:`q` method can take much larger steps than a strong-order method for
the same statistical accuracy, because it is allowed to replace the exact Gaussian
increments with cheaper discrete random variables that only match the required moments.
Conversely, if you need trajectory-level fidelity, a high *weak* order does not help —
you need high *strong* order.

Supported methods in Basilisk
-----------------------------
All integrators below live in the ``Basilisk.simulation.svIntegrators`` module. The
"Noise" column indicates the diffusion structure each method supports:

- **scalar** — a single noise source (:math:`m = 1`).
- **diagonal** — :math:`m` independent sources, source :math:`k` drives state
  :math:`k` only (:math:`g_k` has one nonzero component).
- **additive** — the diffusion :math:`g_k` does not depend on the state :math:`x`
  (constant or time-only), which the SRA methods exploit for efficiency.
- **non-commutative** — general (non-diagonal) noise where different sources are
  coupled and the diffusion columns do not commute; these methods evaluate the mixed
  iterated stochastic integrals required for weak order 2 in the general case.

.. list-table:: Native stochastic integrators
   :widths: 22 10 10 12 22 24
   :header-rows: 1

   * - Class (``svIntegrators.``)
     - Strong order
     - Weak order
     - Interp.
     - Noise supported
     - Notes / when to use
   * - ``svStochasticIntegratorMayurama``
     - 0.5
     - 1.0
     - Itô
     - scalar, diagonal, non-comm.
     - Euler-Maruyama. Simplest baseline; robust but low order.
   * - ``svStochasticIntegratorEulerHeun``
     - 0.5
     - 1.0
     - Stratonovich
     - scalar, diagonal, non-comm.
     - Stratonovich analogue of Euler-Maruyama.
   * - ``svStochasticIntegratorRKMil``
     - 1.0
     - 1.0
     - Itô
     - scalar, diagonal
     - Derivative-free Milstein; higher strong order than Euler for little extra cost.
   * - ``svStochasticIntegratorSRIW1``
     - 1.5
     - 2.0
     - Itô
     - scalar, diagonal
     - Roessler SRI. High strong order for diagonal/scalar Itô noise.
   * - ``svStochasticIntegratorSOSRI``
     - 1.5
     - 2.0
     - Itô
     - scalar, diagonal
     - Stability-optimized SRI. Recommended general-purpose strong solver for diagonal/scalar noise, and the most robust to stiffness / large steps.
   * - ``svStochasticIntegratorSRA1``
     - 1.5
     - 2.0
     - Itô [#additive]_
     - additive
     - Roessler SRA, specialized (and cheaper) for additive noise.
   * - ``svStochasticIntegratorSOSRA``
     - 1.5
     - 2.0
     - Itô [#additive]_
     - additive
     - Stability-optimized SRA. Recommended for additive noise; most robust to stiffness / large steps.
   * - ``svStochasticIntegratorDRI1``
     - --
     - 2.0
     - Itô
     - scalar, diagonal, non-comm.
     - Debrabant-Roessler. Recommended general-purpose weak solver; handles fully coupled noise.
   * - ``svStochasticIntegratorDRI1NM``
     - --
     - 2.0
     - Itô
     - scalar, diagonal
     - Non-mixing variant of DRI1; cheaper, exact for diagonal noise.
   * - ``svStochasticIntegratorRI1`` / ``RI3`` / ``RI5`` / ``RI6``
     - --
     - 2.0
     - Itô
     - scalar, diagonal, non-comm.
     - Roessler weak-order-2 family; alternative tableaux sharing the DRI1 step.
   * - ``svStochasticIntegratorW2Ito1``
     - --
     - 2.0
     - Itô
     - scalar, diagonal, non-comm.
     - Tang & Xiao weak-order-2; handles coupled noise.
   * - ``svStochasticIntegratorW2Ito2``
     - --
     - 2.0
     - Itô
     - scalar, diagonal, non-comm.
     - Tang & Xiao weak-order-2 (deterministic order 4); shares W2Ito1's step with a larger tableau.
   * - ``svStochasticIntegratorSIEA`` / ``SMEA`` / ``SIEB`` / ``SMEB``
     - --
     - 2.0
     - Itô
     - scalar, diagonal
     - Tocino & Vigo-Aguiar stochastic improved/modified Euler; use only raw Gaussian increments.
   * - ``svStochasticIntegratorRDI1WM``
     - --
     - 1.0
     - Itô
     - scalar, diagonal
     - Simplest weak Roessler method; two drift stages, one diffusion evaluation.
   * - ``svStochasticIntegratorRS1`` / ``RS2``
     - --
     - 2.0
     - Stratonovich
     - scalar, diagonal, non-comm.
     - Roessler-Stratonovich weak-order-2; the Stratonovich counterpart of DRI1/W2Ito1.

.. note::

    A "--" in the strong-order column means the method is a *weak* scheme: it targets
    accuracy of ensemble statistics, not individual trajectories, and should not be used
    when pathwise accuracy is required.

.. [#additive] For additive noise the diffusion does not depend on the state, so the
   Itô–Stratonovich correction term (:math:`\tfrac12 \sum_k g_k\,\partial_x g_k`) vanishes
   and the Itô and Stratonovich solutions coincide. SRA1 and SOSRA are formulated in the
   Itô framework, but for the additive problems they target this label imposes no
   restriction.

How to choose an integrator
---------------------------
Work through the following questions in order.

#. **Itô or Stratonovich?** Match the integrator to the convention your model is written
   for. Most engineering process-noise models are Itô. If your model is Stratonovich, use
   ``EulerHeun`` (strong) or ``RS1``/``RS2`` (weak).

#. **Do you need trajectories (strong) or statistics (weak)?**

   - *Trajectories / pathwise accuracy* → choose by **strong order**: ``SOSRI`` (diagonal
     or scalar noise) or ``SOSRA`` (additive noise) for the best accuracy, ``RKMil`` for a
     lighter strong-order-1.0 option, or ``Mayurama``/``EulerHeun`` as a simple baseline.
   - *Statistics from a Monte-Carlo ensemble* → choose by **weak order**: ``DRI1`` is the
     recommended general-purpose weak-order-2 method (``RS1``/``RS2`` for Stratonovich).
     Weak methods let you take larger steps for the same moment accuracy.

#. **What is your noise structure?**

   - *Additive* (diffusion independent of state) → prefer the SRA methods (``SOSRA``,
     ``SRA1``): they are tailored to and cheaper for this case.
   - *Diagonal / scalar* (each source drives its own state) → all methods apply; the
     diagonal-only weak methods (``DRI1NM``, the ``SIE*``/``SME*`` family, ``RDI1WM``) are
     efficient choices.
   - *Non-commutative / coupled* (general non-diagonal noise) → you must pick a method
     whose "Noise" column lists *non-comm.*: ``Mayurama``/``EulerHeun`` (low order), or the
     weak-order-2 ``DRI1``/``RI*``/``W2Ito1``/``W2Ito2`` (Itô) and ``RS1``/``RS2``
     (Stratonovich). Diagonal-only methods will silently miss the cross-noise coupling terms.

#. **Is the problem stiff, or do you want large steps?** Basilisk's stochastic
   integrators are all *explicit*, so none is unconditionally stable. Among them the
   **stability-optimized** ``SOSRI`` and ``SOSRA`` have the largest stability regions and
   are the most robust when the drift is stiff or you push the step size; prefer them over
   ``SRIW1``/``SRA1`` in that regime. For strongly stiff dynamics, reduce the step size —
   implicit SDE solvers are not yet available in Basilisk.

Using a stochastic integrator
------------------------------
A stochastic integrator is attached to a ``DynamicObject`` exactly like a deterministic
one, with ``setIntegrator()`` (see :ref:`bskPrinciples-9`). The dynamics module must
declare its noise sources (via ``StateData::setNumNoiseSources`` and, for shared sources,
``registerSharedNoiseSource``) and provide the diffusion terms in
``equationsOfMotionDiffusion``; see the ``_UnitTest`` scripts in
``simulation/dynamics/Integrators`` for complete, runnable examples.

.. code-block:: python

    from Basilisk.simulation import svIntegrators

    # Weak-order-2 Itô solver, suitable for coupled (non-commutative) noise.
    integratorObject = svIntegrators.svStochasticIntegratorDRI1(dynObject)
    integratorObject.setRNGSeed(42)          # optional: reproducible noise stream
    dynObject.setIntegrator(integratorObject)

Every integrator draws its Wiener increments from a pluggable noise generator. By default
this is a ``RandomGaussianNoiseGenerator`` seeded via ``setRNGSeed()``. For tests or
reproducible studies you can inject a specific increment sequence with a
``PrescribedGaussianNoiseGenerator`` and ``setNoiseGenerator()``:

.. code-block:: python

    prescribed = svIntegrators.PrescribedGaussianNoiseGenerator()
    for dW in incrementSequence:             # each entry has one value per noise source
        prescribed.pushStep(list(dW))
    integratorObject.setNoiseGenerator(prescribed)

.. note::

    As with deterministic integration, if several ``DynamicObject`` instances are
    synchronized with ``syncDynamicsIntegration()`` the integrator assigned to the
    *primary* object governs the whole synchronized set (see :ref:`bskPrinciples-9`).
