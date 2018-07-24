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
%module navAggregate
%{
   #include "navAggregate.h"
%}

%include "swig_conly_data.i"
%constant void Update_aggregateNav(void*, uint64_t, uint64_t);
%ignore Update_aggregateNav;
%constant void SelfInit_aggregateNav(void*, uint64_t);
%ignore SelfInit_aggregateNav;
%constant void CrossInit_aggregateNav(void*, uint64_t);
%ignore CrossInit_aggregateNav;
STRUCTASLIST(AggregateAttInput)
STRUCTASLIST(AggregateTransInput)
GEN_SIZEOF(AggregateAttInput);
GEN_SIZEOF(AggregateTransInput);
GEN_SIZEOF(NavAggregateData);
%include "navAggregate.h"
%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

