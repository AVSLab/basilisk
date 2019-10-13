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

#ifndef _FSW_MODULE_TEMPLATE_FSW_MSG_H_
#define _FSW_MODULE_TEMPLATE_FSW_MSG_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/fswModuleTemplateFswMsg.h"



/*! \defgroup fswModuleTemplate
  @brief This is a template module for FSW algorithms in ANSI-C.

 # Module Purpose
 ## Executive Summary
    Provide a brief introduction to purpose and intent of this module.

 ## Module Assumptions and Limitations
    This section should describe the assumptions used in formulating the mathematical model and how those assumptions limit the usefulness of the module.

 ## Message Connection Descriptions
    Input Message | Msg Type | Description
    --------------|----------|-------------
    dataInMsgName|FswModuleTemplateFswMsg | Input message description.  Note here if this message is optional, and what the default behavior is if this message is not provided.

    Output Message | Msg Type | Description
    --------------|----------|-------------
    dataOutMsgName |  FswModuleTemplateFswMsg | Output message description.


 # Detailed Module Description
    Provide a brief introduction to the material being discussed in this report.  For example, include what the motivation is, maybe provide a supportive figure such as shown below, reference earlier work if needed in a literature review web links. Describe the module including mathematics, implementation, etc.

 ## Equations
    Equations can be provided with LaTeX as well.  For example, the code

    `\f$ a = b^{2} \f$`

    produces this equation

    \f$ a = b^{2} \f$

    If the module description requires extensive math discussion, this can be TeX'd up using the technical note template inside the `_Documentation` folder. To include the PDF file in the `doxygen` HTML documentation the file path must be included in the `docs/DoxyData` file.  The path to the BSK technical note must be included in the `HTML_EXTRA_FILES` tag.  Then, link to this [PDF](Basilisk-MODULENAME.pdf) using the code
    ~~~~~~~~~
    [PDF](Basilisk-MODULENAME.pdf)
    ~~~~~~~~~
    The PDF technical should only be used as a last resort effort if the math is simply too complex and long to include in the `doxygen` documentation.  Another option is to link to a web site, conference paper, journal paper, book or thesis document that discussed the mathematical developments used.

 ## Citations
    If you want to cite other papers or text, provide a web link to a paper.  For example,

    `[The link text](http://example.net/)`

    creates [The link text](http://example.net/).

 ## Figures
    To include figures, you must copy the web compatible image (jpg, png, svg) to the `docs/Images/doc` folder, preferably into a module specific sub-folder.  This keeps the modules images grouped within this sub-folder. For example, you can use code such as

    `\image html Images/doc/fswTemplateModule/Fig1.jpg "Sample Figure Illustration" width=500px`

    to generate the image such as

    \image html Images/doc/fswTemplateModule/Fig1.jpg "Sample Figure Illustration" width=500px

    Note that with pixelated figures providing a width value that is at least half that of the actual images the figure will look good even on a high resolution display.

 ## Tables
    The standard Doxygen table formating can be used to generate tables.  More info on tables is found [here](http://www.doxygen.nl/manual/markdown.html#md_tables).  For example, the code

    ~~~~~~~
    Animal | Description | Price ($)
    -------|-------------|----------
    Gnat   | per gram | 13.65
    Gnu    | stuffed  | 92.50
    Emu    | stuffed  | 33.33
    Armadillo | frozen  |  8.99
    ~~~~~~~

    generates this table

    Animal | Description | Price ($)
    -------|-------------|----------
    Gnat   | per gram | 13.65
    Gnu    | stuffed  | 92.50
    Emu    | stuffed  | 33.33
    Armadillo | frozen  |  8.99

 # User Guide
    This section contains information directed specifically to users. It contains clear descriptions of what inputs are needed and what effect they have. It should also help the user be able to use the model for the first time.

    Add sample code as needed.  For example, to specify that the module variables `dummy` and `dumVector` must be setup first, you can include python formatted code using:
    ~~~~~~~{.py}
    moduleConfig.dummy = 1
    moduleConfig.dumVector = [1., 2., 3.]
    ~~~~~~~

    In the user guide, provide sub-sections as need to help explain how to use this module, list what variables must be set, discuss variables that might have default values if not specified by the user, etc.
  @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double dummy;                                   //!< [units] sample module variable declaration
    double dumVector[3];                            //!< [units] sample vector variable

    /* declare module IO interfaces */
    char dataOutMsgName[MAX_STAT_MSG_LENGTH];       //!< The name of the output message
    int32_t dataOutMsgID;                           //!< ID for the outgoing message
    char dataInMsgName[MAX_STAT_MSG_LENGTH];        //!< The name of the Input message
    int32_t dataInMsgID;                            //!< ID for the incoming message

    double  inputVector[3];                         //!< [units]  vector description
}fswModuleTemplateConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_fswModuleTemplate(fswModuleTemplateConfig *configData, int64_t moduleID);
    void CrossInit_fswModuleTemplate(fswModuleTemplateConfig *configData, int64_t moduleID);
    void Update_fswModuleTemplate(fswModuleTemplateConfig *configData, uint64_t callTime, int64_t moduleID);
    void Reset_fswModuleTemplate(fswModuleTemplateConfig *configData, uint64_t callTime, int64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
