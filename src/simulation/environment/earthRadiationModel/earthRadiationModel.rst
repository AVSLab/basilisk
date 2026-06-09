
Executive Summary
-----------------
This module computes the Earth Radiation Pressure (ERP) flux received at the satellite location
using the Knocke et al. (1988) patch model [Knocke1988]_. At each time step the visible Earth disk is
discretised into latitude/longitude zones. Each visible, illuminated patch contributes Lambertian
albedo flux; every visible patch contributes IR flux regardless of
solar illumination [MontenbruckGill2000]_, [Knocke1988]_.

The module outputs total albedo and IR flux magnitudes together with the flux-weighted net force
direction vectors in the inertial frame.  These outputs are consumed by `facetERPDynamicEffector`
to compute force and torque acting on the spacecraft.

Message Connection Descriptions
--------------------------------
The following table lists all the module input and output messages.  The module msg variable names
are set by the user from Python.  The msg type contains a link to the message structure definition,
while the description provides information on what this message is used for.

.. bsk-module-io:: Module I/O Messages
    :caption: Module I/O Messages

    input spacecraftStateInMsg SCStatesMsgPayload
      spacecraft position (inertial frame)
    input planetInMsg SpicePlanetStateMsgPayload
      Earth state and J2000-to-planet-fixed rotation matrix
    input sunPositionInMsg SpicePlanetStateMsgPayload
      Sun position used to determine patch illumination
    output earthRadiationOutMsg EarthRadiationMsgPayload
      total albedo and IR flux with flux-weighted net direction vectors


Detailed Module Description
---------------------------
Earth is modelled as a Lambertian sphere [Knocke1988]_.  Its surface is discretised into
:math:`n_\text{lat} \times n_\text{lon}` equal-width latitude/longitude zones.  For each patch
:math:`i` visible from the satellite (:math:`\cos\theta_\text{sat,i} > 0`), the module computes:

**Albedo flux** — only when the patch is illuminated by the Sun
(:math:`\cos\theta_\text{sun,i} > 0`):

.. math::

   \delta F_{\text{alb},i} = \frac{S_\odot}{\pi}\,A_i\,\cos\theta_{\text{sun},i}\,
                              \cos\theta_{\text{sat},i}\,\frac{dA_i}{d_i^2}

**IR flux** — emitted by every visible patch (Outgoing Longwave Radiation (OLR)):

.. math::

   \delta F_{\text{IR},i} = \frac{F_{\text{OLR}}}{\pi}\,\cos\theta_{\text{sat},i}\,
                             \frac{dA_i}{d_i^2}

where :math:`S_\odot` is the distance-corrected solar flux, :math:`F_{\text{OLR}} \approx 237\,\text{W/m}^2`
is the mean outgoing IR radiation [Knocke1988]_, :math:`A_i` is the patch albedo coefficient, and
:math:`d_i` is the distance from the patch centroid to the satellite.

The total flux and flux-weighted net direction vectors are accumulated over all visible (and, for
albedo, illuminated) patches [Knocke1988]_, [RodriguezSolano2012]_:

.. math::

   \mathbf{d}_{\text{alb}} = \frac{\sum_i \delta F_{\text{alb},i}\,\hat{\mathbf{r}}_i}{
                               \left|\sum_i \delta F_{\text{alb},i}\,\hat{\mathbf{r}}_i\right|}

where :math:`\hat{\mathbf{r}}_i` is the unit vector from patch :math:`i` to the satellite
(inertial frame).  If the total flux is zero the direction vector is set to :math:`[0,0,0]`.

Patch area uses an authalic latitude correction to account for Earth's oblateness
when the WGS-84 equatorial and polar radii are available (default behaviour).

All patch geometry is delegated to :ref:`planetRadiationBase`.

Solar Flux Correction
~~~~~~~~~~~~~~~~~~~~~
The solar irradiance at Earth is corrected for the Sun-Earth distance:

.. math::

   S_\odot = S_0 \left(\frac{\text{AU}}{|\mathbf{r}_\odot - \mathbf{r}_\oplus|}\right)^2,
   \quad S_0 = 1361\,\text{W/m}^2

Module Assumptions and Limitations
------------------------------------
- Earth is treated as a Lambertian sphere (no specular component).
- IR flux uses a constant global mean OLR; no seasonal or geographic variation unless CERES data
  [VielbergKusche2020]_, [Wielicki1996]_ are provided for the albedo channel.
- No penumbra / partial shadow model is applied to individual patches.
- Eclipse modeling is enabled by calling ``setEclipseCase(True)``.  When enabled, the base-class
  ``isPatchEclipsed()`` applies a Knocke penumbra model per patch; albedo contributions from
  eclipsed patches are zeroed out.  IR flux is unaffected by eclipse.

User Guide
----------
Import and instantiate the module:

.. code-block:: python

   from Basilisk.simulation import earthRadiationModel
   erm = earthRadiationModel.EarthRadiationModel()
   erm.ModelTag = "earthRadiation"

Subscribe to input messages:

.. code-block:: python

   erm.spacecraftStateInMsg.subscribeTo(scMsg)
   erm.planetInMsg.subscribeTo(earthMsg)
   erm.sunPositionInMsg.subscribeTo(sunMsg)

Optional: use CERES albedo data instead of a constant Bond albedo:

.. code-block:: python

   from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path
   data = get_path(DataFile.AlbedoData.Earth_ALB_2018_CERES_All_10x10)
   erm.albedoDataPath = str(data.parent)
   erm.albedoDataFile = data.name

Add to the simulation task:

.. code-block:: python

   sim.AddModelToTask(taskName, erm)

The output message `earthRadiationOutMsg` carries :ref:`EarthRadiationMsgPayload`.

References
----------

.. [MontenbruckGill2000] Montenbruck, O., and Gill, E., *Satellite Orbits: Models, Methods and Applications*, Springer, Berlin, 2000. DOI: `10.1007/978-3-642-58351-3 <https://doi.org/10.1007/978-3-642-58351-3>`_

.. [RodriguezSolano2012] Rodriguez-Solano, C. J., Hugentobler, U., Steigenberger, P., and Lutz, S., *Impact of Earth Radiation Pressure on GPS Position Estimates*, Journal of Geodesy, 2012. DOI: `10.1007/s00190-011-0517-4 <https://doi.org/10.1007/s00190-011-0517-4>`_

.. [VielbergKusche2020] Vielberg, K., and Kusche, J., *Extended forward and inverse modeling of radiation pressure accelerations for LEO satellites*, Journal of Geodesy, 2020. DOI: `10.1007/s00190-020-01368-6 <https://doi.org/10.1007/s00190-020-01368-6>`_

.. [Wielicki1996] Wielicki, B. A., Barkstrom, B. R., Harrison, E. F., Lee, R. B., Smith, G. L., and Cooper, J. E., *Clouds and the Earth's Radiant Energy System (CERES): An Earth Observing System Experiment*, Bulletin of the American Meteorological Society, 1996. DOI: `10.1175/1520-0477(1996)077 <https://doi.org/10.1175/1520-0477(1996)077%3C0853:CATERE%3E2.0.CO;2>`__
