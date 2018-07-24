/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _SysModel_HH_
#define _SysModel_HH_

#include <string>
#include <stdint.h>

class SysModel
{
    
public:
    SysModel();
    SysModel(const SysModel &obj);
    virtual ~SysModel();
    virtual void SelfInit();
    virtual void CrossInit();
    virtual void IntegratedInit();
    virtual void UpdateState(uint64_t CurrentSimNanos);
    virtual void Reset(uint64_t CurrentSimNanos);
    
public:
    std::string ModelTag;        //!< -- name for the algorithm to base off of
    uint64_t CallCounts=0;       //!< -- Counts on the model being called
    uint32_t RNGSeed;            //!< -- Giving everyone a random seed for ease of MC
    uint64_t moduleID;           //!< -- Module ID for this module
};


#endif /* _SYS_MODEL_H_ */
