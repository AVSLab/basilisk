/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */


#ifndef FACET_DRAG_DYNAMIC_EFFECTOR_H
#define FACET_DRAG_DYNAMIC_EFFECTOR_H

#include <Eigen/Dense>
#include <vector>
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/AtmoPropsMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/bskLogging.h"





/*! @brief spacecraft geometry data */
typedef struct {
  std::vector<double> facetAreas;                   //!< vector of facet areas
  std::vector<double> facetCoeffs;                  //!< vector of facet coefficients
  std::vector<Eigen::Vector3d> facetNormals_B;      //!< vector of facet normals
  std::vector<Eigen::Vector3d> facetLocations_B;    //!< vector of facet locations
}SpacecraftGeometryData;


/*! @brief faceted atmospheric drag dynamic effector */
class FacetDragDynamicEffector: public SysModel, public DynamicEffector {
public:


    FacetDragDynamicEffector();
    ~FacetDragDynamicEffector();
    void linkInStates(DynParamManager& states);
    void computeForceTorque(double integTime, double timeStep);
    void Reset(uint64_t CurrentSimNanos);               //!< class method
    void UpdateState(uint64_t CurrentSimNanos);
    void WriteOutputMessages(uint64_t CurrentClock);
    bool ReadInputs();
    void addFacet(double area, double dragCoeff, Eigen::Vector3d B_normal_hat, Eigen::Vector3d B_location);

private:

    void plateDrag();
    void updateDragDir();
public:
    uint64_t numFacets;                             //!< number of facets
    ReadFunctor<AtmoPropsMsgPayload> atmoDensInMsg; //!< atmospheric density input message
    StateData *hubSigma;                            //!< -- Hub/Inertial attitude represented by MRP
    StateData *hubVelocity;                         //!< m/s Hub inertial velocity vector
    Eigen::Vector3d v_B;                            //!< m/s local variable to hold the inertial velocity
    Eigen::Vector3d v_hat_B;                        //!< class variable
    BSKLogger bskLogger;                            //!< -- BSK Logging

private:
    AtmoPropsMsgPayload atmoInData;
    SpacecraftGeometryData scGeometry;              //!< -- Struct to hold spacecraft facet data

};

#endif 
