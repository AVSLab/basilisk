
#ifndef SpicePlanetState_H
#define SpicePlanetState_H

/*! \addtogroup SimModelGroup
 * @{
 */

#define MAX_BODY_NAME_LENGTH 64

//! The SPICE planet statate structure is the struct used to ouput planetary body states to the messaging system
typedef struct {
    double J2000Current;        //!< s Time of validity for the planet state
    double PositionVector[3];   //!< m True position of the planet for the time
    double VelocityVector[3];   //!< m/s True velocity of the planet for the time
    double J20002Pfix[3][3];    //!< (-) Orientation matrix of planet-fixed relative to inertial
    int computeOrient;         //!< (-) Flag indicating whether the reference should be computed
    char PlanetName[MAX_BODY_NAME_LENGTH];        //!< -- Name of the planet for the state
}SpicePlanetState;

#endif
