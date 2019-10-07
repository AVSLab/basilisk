## \defgroup fswAlgorithms fswAlgorithm Modules
# @brief Folder with Basilisk Flight Software Algorithm Modules using ANSI-C or C++.
# The collection provide a self-study guide
# to learn how to setup Basilisk astrodynamics simulations.  The chapters provide groupings of
# common sub-topics.
## @{
#   \defgroup 1-fswTemplateFolder   _fswTemplateFolder
#   @brief Contains a FSW template module
#   @{
#       \defgroup fswModuleTemplate
#   @}
#
#   \defgroup attControl attControl
#   @brief Contains modules that related to attitude control solutions.
#   @{
#       \defgroup lowPassFilterTorqueCommand lowPassFilterTorqueCommand
#       \defgroup MRP_Feedback MRP_Feedback
#       \defgroup MRP_PD MRP_PD
#       \defgroup MRP_Steering MRP_Steering
#       \defgroup PRV_Steering PRV_Steering
#       \defgroup rateServoFullNonlinear rateServoFullNonlinear
#       \defgroup thrMomentumManagement thrMomentumManagement
#   @}
#
#   \defgroup attDetermination attDetermination
#   @brief Contains modules that related to attitude determination solutions.
#   @{
#       \defgroup cssWlsEst cssWlsEst
#       \defgroup inertialUKF inertialUKF
#       \defgroup okeefeEKF okeefeEKF
#       \defgroup sunlineEKF sunlineEKF
#       \defgroup sunlineEphem sunlineEphem
#       \defgroup sunlineSEKF sunlineSEKF
#       \defgroup sunlineSuKF sunlineSuKF
#       \defgroup sunlineUKF sunlineUKF
#       \defgroup ukfUtilities ukfUtilities
#       \defgroup headingSuKF headingSuKF
#   @}
#
#   \defgroup attGuidance attGuidance
#   @brief Contains modules that related to attitude guidance solutions.
#   @{
#       \defgroup attTrackingError attTrackingError
#       \defgroup celestialTwoBodyPoint celestialTwoBodyPoint
#       \defgroup eulerRotation eulerRotation
#       \defgroup hillPoint hillPoint
#       \defgroup inertial3D inertial3D
#       \defgroup inertial3DSpin inertial3DSpin
#       \defgroup mrpRotation mrpRotation
#       \defgroup rasterManager rasterManager
#       \defgroup simpleDeadband simpleDeadband
#       \defgroup sunSafePoint sunSafePoint
#       \defgroup opNavPoint opNavPoint
#       \defgroup velocityPoint velocityPoint
#   @}
#
#   \defgroup dvGuidance dvGuidance
#   @brief Contains modules that related to orbital trajectory changes.
#   @{
#       \defgroup dvAttGuidance dvAttGuidance
#       \defgroup dvExecuteGuidance dvExecuteGuidance
#   @}
#
#   \defgroup effectorInterfaces effectorInterfaces
#   @brief Contains modules that interface with attitude and translational effectors.
#   @{
#       \defgroup _GeneralModuleFilesEffector _GeneralModuleFiles
#       @{
#           \defgroup thrustGroupData thrustGroupData
#       @}
#       \defgroup errorConversion errorConversion
#       @{
#       \defgroup sunSafeACS sunSafeACS
#       \defgroup dvAttEffect dvAttEffect
#       @}
#       \defgroup dvAttEffect dvAttEffect
#       \defgroup rwMotorTorque rwMotorTorque
#       \defgroup rwMotorVoltage rwMotorVoltage
#       \defgroup rwNullSpace rwNullSpace
#       \defgroup thrFiringRemainder thrFiringRemainder
#       \defgroup thrFiringSchmitt thrFiringSchmitt
#       \defgroup thrForceMapping thrForceMapping
#       \defgroup thrMomentumDumping thrMomentumDumping
#       \defgroup thrusterRWDesat thrusterRWDesat
#   @}
#
#   \defgroup formationFlying formationFlying
#   @brief Contains modules that relate to spacecraft formation flying.
#   @{
#       \defgroup spacecraftPointing spacecraftPointing
#   @}
#
#   \defgroup rwConfigData rwConfigData
#   \defgroup messaging messaging
#
#   \defgroup sensorInterfaces sensorInterfaces
#   @brief Contains modules that provide sensor interfaces.
#   @{
#       \defgroup cssComm cssComm
#       \defgroup imuComm imuComm
#       \defgroup rateMsgConverter rateMsgConverter
#       \defgroup stComm stComm
#   @}
#
#   \defgroup transDetermination transDetermination
#   @brief Contains modules that relate to orbital solutions.
#   @{
#       \defgroup _GeneralModuleFilesTransDet _GeneralModuleFiles
#       @{
#           \defgroup ephemerisUtilities ephemerisUtilities
#       @}
#       \defgroup chebyPosEphem chebyPosEphem
#       \defgroup dvAccumulation dvAccumulation
#       \defgroup ephemDifference ephemDifference
#       \defgroup ephemNavConverter ephemNavConverter
#       \defgroup navAggregate navAggregate
#       \defgroup oeStateEphem oeStateEphem
#   @}
#
#   \defgroup imageProcessing imageProcessing
#   @brief Contains modules that relate to image processing modules and visual navigation.
#   @{
#       \defgroup houghCircles houghCircles
#       \defgroup pixelLineConverter pixelLineConverter
#       \defgroup limbFinding limbFinding
#       \defgroup horizonOpNav horizonOpNav
#   @}
#   \defgroup opticalNavigation opticalNavigation
#   @brief Contains modules that relate to optical navigation.
#   @{
#       \defgroup _GeneralModuleFiles _GeneralModuleFiles
#       @{
#           \defgroup ukfUtilities ukfUtilities
#       @}
#       \defgroup relativeODuKF relativeODuKF
#       \defgroup PixelLineBiasUKF PixelLineBiasUKF
#   @}
#
#   \defgroup vehicleConfigData vehicleConfigData
#
## @}

