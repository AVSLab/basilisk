**DONE**
- Add J2 to gravity (done commented out)
- Understand what codex did with thruster
	-> Minimal Δv too high with Thrust setting and min. on time
	-> If the min. on time is shorter than the frame rate, the min. on time defaults to the frame rate!
- Change virtual chief back to barycenter
	-> frozen barycenter (t=0) is used for spacecraftReconfiguration, after thet the 'live' barycenter is used for formation keeping!
- Add hillPointing module: https://avslab.github.io/basilisk/Documentation/fswAlgorithms/attGuidance/hillPoint/hillPoint.html
- Implement HELIX formation!
- Understand what codex did with formation flight change
- Change RK integrator for the thruster from RK-4 to RK-4,5 (or even RK-7,8) => Test if the thruster issue still persists if I change the Thrusterfactory to the default physically correct thruster again.

**IN PROGRESS

TO BE DONE
- facetSRPDynamicEffector: https://hanspeterschaub.info/basilisk/Documentation/simulation/dynamics/facetSRPDynamicEffector/facetSRPDynamicEffector.html
- coarseSunSensor: (is this needed, why?) https://avslab.github.io/basilisk/Documentation/simulation/sensors/coarseSunSensor/coarseSunSensor.html#_CPPv4N15CoarseSunSensor10CSSGroupIDE

- Look into:
  'inertialCartFeedback' (Schaub textbook 14.8.2) -> implemented by Will Schwend. (when spacecrafts are in a formation further apart)
  'hillFrameRelativeFeedbackControl' (when spacecrafts are in a 'close' formation)
  => Using these controllers with a spacecraft with one thruster will require either a cascaded controller or a MPC controller.
  -> SC must 'rotate' into the right attitude and fire (within one framerate)
  -> Thus low bandwidth controller!
- Look into: syncIntegrators (look at examples in Constrained Spacecraft Dynamics Simulations)
