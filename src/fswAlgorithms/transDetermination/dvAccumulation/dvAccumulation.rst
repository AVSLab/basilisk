
This module reads in a message with an array of accelerometer measurements and integrates them to determine an accumulated :math:`\Delta\mathbf{v}` value.

On reset the net :math:`\Delta\mathbf{v}` is set to zero.  The output navigation message contains the latest measurements time tag and the total :math:`\Delta\mathbf{v}`. More information on can be found in the
:download:`PDF Description </../../src/fswAlgorithms/transDetermination/dvAccumulation/_Documentation/Basilisk-dvAccumulation-2019-03-28.pdf>`.

