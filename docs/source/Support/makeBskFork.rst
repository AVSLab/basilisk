
.. _makeBskFork:

Forking from the Repository and Making a Pull Request
=====================================================

.. note::

   While the Basilisk repository can be pulled as a compressed folder and then compiled to run spacecraft simulation, this approach makes it more challenging to pull in code updates that are pushed to the Basilisk repository.  The following instructions illustrate how to make a fork of the Basilisk repository.  General instructions on how to do a fork on `bitbucket.org <https://bitbucket.org>`__ can be found `here <https://confluence.atlassian.com/bitbucket/forking-a-repository-221449527.html>`__.

#. If needed, create your own bitbucket.org account

#. Use a browser to go to the Basilisk Bitbucket Repository

#. Click on the Plus sign in order to Fork the Basilisk Repository into your own account as illustrated in the followign screen capture:

   .. image:: /_images/static/fork-1.jpg
      :align: center
      :scale: 50 %


#. Use the `Pulling/Clonning documentation` for cloning the forked repository into a preferred Git client (`Sourcetree <https://www.sourcetreeapp.com>`__, `GitKraken <https://www.gitkraken.com>`__ etc.)

#. In order to have updates from Basilisk repository click Add Remote in the Git Client:

   .. image:: /_images/static/fork-2.jpg
      :align: center
      :scale: 50 %

#. Use Gitflow to start new feature, and work in your own Branch

#. Push commits to origin (forked repository) and rebase your branch onto develop when necessary

#. If the branch is ready to be reviewed, make sure there is a special Branch created for you in the Basilisk Repository to make a Pull Request to

#. Make the pull request to the Branch in the Upstream (original Basilisk Repo) from `bitbucket.org <https://bitbucket.org>`__.
   Go to Branches section and create the pull request for your branch.

#. If everything works, your branch will be merged into the Branch created for you in Basilisk Repository

#. If there are revisions to make, create patch files for each commit you have and send them as separate files