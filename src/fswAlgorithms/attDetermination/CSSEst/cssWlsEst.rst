
A weighted least-squares minimum-norm algorithm is used to estimate the body-relative sun heading using a cluster of coarse sun sensors.  Using two successive sun heading evaluation the module also computes the inertial angular velocity vector.  As rotations about the sun-heading vector are not observable, this angular velocity vector only contains body rates orthogonal to this sun heading vector.  More information on can be found in the
:download:`PDF Description </../../src/fswAlgorithms/attDetermination/CSSEst/_Documentation/Basilisk-cssWlsEst-20180429.pdf>`

