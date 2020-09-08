.. toctree::
   :hidden:

Black Lion
----------

Black Lion is a distributed communication architecture for integration of independent mission models/components into
a single simulation run (with Basilisk being one of its potential components).
For further details on Black Lion the reader is referred to
`this paper <https://hanspeterschaub.info/Papers/ColsMargenet2018a.pdf>`_.

``launchBskBL`` File
~~~~~~~~~~~~~~~~~~~~

Run this file to bootstrap a Black Lion simulation that includes three components: a Basilisk-Dynamics simulation,
a Basilisk-FSW simulation and a listener. These components are explained in the next descriptions.


``bskNode`` Directory
~~~~~~~~~~~~~~~~~~~~~

This directory contains Basilisk models for FSW and Dynamics and its corresponding nodes. A Basilisk node is simply
a file in which a Basilisk simulation is instantiated as a Black Lion component --therefore, a node file is the
interface between tha Basilisk and Black Lion tools. In addition, a Delegate and Router APIs are included
(``bskWorkerProcess.py`` is the Delegate and ``bskRouter.py`` is the Router. For further details on the
functionality of the Delegate and Router APIs, the reader is referred to the paper mentioned above.


``listenerNode`` Directory
~~~~~~~~~~~~~~~~~~~~~~~~~~
This directorary contains another component/model that can be included in a Black Lion run. This component simply
subscribes to messages that are available from other nodes in the simulation (e.g. a Basilisk node).

