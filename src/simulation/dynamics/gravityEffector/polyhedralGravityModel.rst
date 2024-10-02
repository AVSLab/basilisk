
Executive Summary
-----------------
This module encodes the polyhedron gravity model as described in `Werner and Scheeres 1996 <https://doi.org/10.1007/BF00053511>`__. It inherits from the abstract class ``gravityModel`` and computes the gravity acceleration and potential with the constant density polyhedron equations.

The gravity acceleration is evaluated as

.. math::
    :label: eq:acc_poly

    \mathbf{a}^{P}=\frac{\mu}{V}\left(-\sum_{e\in\text{edges}}\mathbf{E}_e\mathbf{r}_eL_e
    +\sum_{f\in\text{faces}}\mathbf{F}_f\mathbf{r}_fw_f\right)

and the gravity potential as

.. math::
    :label: eq:U_poly

    U=\frac{\mu}{2V}\left(\sum_{e\in\text{edges}}\mathbf{r}^T_e\mathbf{E}_e\mathbf{r}_e
    -\sum_{f\in\text{faces}}\mathbf{r}^T_f\mathbf{F}_f\mathbf{r}_f\right)

Module Assumptions and Limitations
----------------------------------
The evaluation point is referred to geographical coordinates with respect to the polyhedron gravity body. In other words, the input position is expressed in the planet centred rotating frame. The output gravity acceleration is also expressed in the previous frame.

The module assumes the polyhedron body has constant density.

The polyhedron shape is defined by triangular faces.

The module requires the standard gravity parameter to be provided by the user. Then, the body volume is internally computed from the polyhedron data to ensure consistency with the shape.

Detailed Module Description
---------------------------
The ``polyhedralGravityModel`` module handles the following behavior:

#. Vertexes positions and faces indexes (corresponding to vertexes) are loaded from a polyhedron shape file.
#. The module initializes and stores all the variables that have no dependency with the evaluation point: edge-facet dyad matrices, facet normals, facet centers and length of edges.
#. By looping over edges and faces at the same time, the gravity acceleration and potential can be computed for an evaluation point.

User Guide
----------

To use the polyhedron gravity model, instantiate the ``simIncludeGravBody.gravBodyFactory()`` and use the corresponding method to create a custom gravity body (Eros in this example). Then, use the gravity body method to attach a polyhedron gravity model and set the path to the polyhedron shape file as input argument.

.. code-block:: python

    mu = 4.46275472004 * 1e5 # Eros standard gravity parameter
    file_poly = 'eros007790.tab'  # Path to a shape file of Eros
    gravFactory = simIncludeGravBody.gravBodyFactory() # Instantiate gravity factory
    erosGravBody = gravFactory.createCustomGravObject('eros_poly', mu=mu) # Create Eros gravity body
    erosGravBody.isCentralBody = True # (Optional) If the body is to be central
    erosGravBody.usePolyhedralGravityModel(file_poly) # Declare the use of a polyhedron model for Eros and set the shape file

For orbital propagation, the list of gravity bodies have to be appended to a spacecraft as

.. code-block:: python

    scObject = spacecraft.Spacecraft() # Creates spacecraft instance
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values())) # Attach list of created gravity bodies to spacecraft


For gravity evaluations, the polyhedron gravity has to be initialized

.. code-block:: python

    erosGravBody.initializeParameters() # Initializes polyhedron module internal variables
    pos = [40*1e3, 30*1e3, 10*1e3] # Evaluation point
    acc = erosGravBody.computeField(pos) # Gravity evaluation

Supported polyhedron shape files
--------------------------------
Polyhedron shape files contain vertexes coordinates and the vertexes indexes that form each triangular face. The vertexes coordinates are assumed to be expressed in kilometers. The following file formats .tab, .obj and .txt are supported as inputs:

The .tab files do not have a header and have 4 columns where the first one denotes if the row refers to a vertex (v) coordinates or the face (f) composed of three vertexes

.. _glPixelSketchTab:
.. figure:: /../../src/simulation/dynamics/gravityEffector/_Documentation/Figures/poly_tab_example.jpg
    :align: center
    :width: 400px

    Figure 1: Example of .tab polyhedron shape file

The .obj files content is equivalent to the .tab extension but they admit comments (#) and separate the vertexes and faces content with an empty line

.. _glPixelSketchObj:
.. figure:: /../../src/simulation/dynamics/gravityEffector/_Documentation/Figures/poly_obj_example.jpg
    :align: center
    :width: 400px

    Figure 2: Example of .obj polyhedron shape file

The .txt file has a first line as a header where the number of vertexes (first column) and faces (second column) are indicated. Then, it has 3 columns where vertexes coordinates and subsequently the faces correspondence with vertexes are provided

.. _glPixelSketchTxt:
.. figure:: /../../src/simulation/dynamics/gravityEffector/_Documentation/Figures/poly_txt_example.jpg
    :align: center
    :width: 400px

    Figure 3: Example of .txt polyhedron shape file

Additional file formats could be added to the function ``loadPolyFromFileToList(fileName: str)`` under ``gravCoeffOpps.py``.
