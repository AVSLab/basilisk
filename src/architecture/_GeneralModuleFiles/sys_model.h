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

#include <architecture/utilities/bskLogging.h>
#include <string>
#include <stdint.h>

/*! @brief Simulation System Model Class */
class SysModel
{
public:
    SysModel();

    /**
     * @brief Copy constructor for SysModel.
     *
     * This constructor initializes a new SysModel instance by copying data
     * from another SysModel instance.
     *
     * @param obj The SysModel object to copy data from.
     */
    SysModel(const SysModel &obj);

    virtual ~SysModel(){};

    /** Initializes the module, create messages */
    virtual void SelfInit(){};

    /** ??? */
    virtual void IntegratedInit(){};

    /** Reads incoming messages, performs module actions, writes output messages */
    virtual void UpdateState(uint64_t CurrentSimNanos){};

    /** Called at simulation initialization, resets module to specified time */
    virtual void Reset(uint64_t CurrentSimNanos){};

    std::string ModelTag = "";     //!< Basilisk module tag name
    uint64_t CallCounts = 0;       //!< Counts on the model being called
    uint32_t RNGSeed = 0x1badcad1; //!< Giving everyone a random seed for ease of MC
    int64_t moduleID;              //!< Module ID for this module  (handed out by module_id_generator)
};


#endif /* _SYS_MODEL_H_ */
