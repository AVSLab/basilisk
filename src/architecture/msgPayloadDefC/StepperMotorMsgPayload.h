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

#ifndef stepperMotorSimMsg_h
#define stepperMotorSimMsg_h

/*! @brief Structure containing stepper motor state information */
typedef struct {
    double theta;        //!< [rad] Current motor angle
    double thetaDot;     //!< [rad/s] Current motor angle rate
    double thetaDDot;    //!< [rad/s^2] Current motor angular acceleration
    int stepsCommanded;  //!< Current number of commanded motor steps
    int stepCount;       //!< Current motor step count (number of steps taken)
}StepperMotorMsgPayload;

#endif /* stepperMotorSimMsg_h */
