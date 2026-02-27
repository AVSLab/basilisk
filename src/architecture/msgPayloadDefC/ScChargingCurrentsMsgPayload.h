/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#ifndef SC_CHARGING_CURRENTS_MESSAGE_H
#define SC_CHARGING_CURRENTS_MESSAGE_H

/*! @brief Structure holding individual current components used in spacecraftChargingEquilibrium. */
typedef struct {
    double electronCurrent;               //!< [A] ambient plasma electron current
    double ionCurrent;                    //!< [A] ambient plasma ion current
    double seeElectronCurrent;            //!< [A] secondary electron emission from ambient electrons
    double seeIonCurrent;                 //!< [A] secondary electron emission from ambient ions
    double backscatteringCurrent;         //!< [A] ambient electron backscattering current
    double photoelectricCurrent;          //!< [A] photoelectric current
    double nonBeamCurrent;                //!< [A] sum of all non-beam current components
    double beamCurrent;                   //!< [A] primary beam current term (servicer emit or target receive)
    double beamSeeCurrent;                //!< [A] beam-induced secondary electron emission current
    double beamBackscatteringCurrent;     //!< [A] beam-induced backscattering current
    double totalCurrent;                  //!< [A] total current used in equilibrium solve
} ScChargingCurrentsMsgPayload;

#endif
