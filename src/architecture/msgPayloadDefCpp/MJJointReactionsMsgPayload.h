/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef MJ_JOINT_REACTIONS_MESSAGE_H
#define MJ_JOINT_REACTIONS_MESSAGE_H

#include <vector>

/*! @brief Structure used by the messaging system to communicate details about the spacecraft system including the reaction forces and torques acting on the joints.*/
typedef struct
//@cond DOXYGEN_IGNORE
MJJointReactionsMsgPayload
//@endcond

{
    std::vector<int> jointTreeIdx;                    //!< [-] list of the kinematic tree index for each joint
    std::vector<int> jointParentBodyIdx;              //!< [-] list of the parent body index for each joint
    std::vector<int> jointTypes;                      //!< [-] list of the joint types (0:free,1:ball,2:slide,3:hinge) in the system
    std::vector<int> jointDOFStart;                   //!< [-] list of the starting DOF index for each joint
    std::vector<double> biasForces;                   //!< [-] bias reaction forces and torques on the joint DOFs in the system
    std::vector<double> passiveForces;                //!< [-] passive reaction forces and torques on the joint DOFs in the system
    std::vector<double> constraintForces;             //!< [-] constraint reaction forces and torques on the joint DOFs in the system
    std::vector<double> appliedForces;                //!< [-] applied reaction forces and torques on the joint DOFs in the system
    std::vector<double> actuatorForces;               //!< [-] actuator reaction forces and torques on the joint DOFs in the system
} MJJointReactionsMsgPayload;

#endif
