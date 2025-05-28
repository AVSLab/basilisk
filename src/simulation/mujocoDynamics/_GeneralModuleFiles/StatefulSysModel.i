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
%module cStatefulSysModel
%{
   #include "StatefulSysModel.h"
%}

%include "architecture/utilities/bskLogging.h"
%import "architecture/_GeneralModuleFiles/sys_model.i"
%import "simulation/dynamics/_GeneralModuleFiles/dynParamManager.i"

// We don't need to construct the DynParamRegisterer on the Python side
%ignore DynParamRegisterer::DynParamRegisterer;

// Current limitation of SWIG for complex templated types like
// std::pair<const StateData*, size_t>, we need to declare these manually
%traits_swigtype(StateData);
%fragment(SWIG_Traits_frag(StateData));

%extend DynParamRegisterer {
   // SWIG doesnt like const StateData& so we have to use const StateData* and convert
   void registerSharedNoiseSource(std::vector<std::pair<const StateData*, size_t>> list) {
      // Convert from pointer pairs to reference pairs
      std::vector<std::pair<const StateData&, size_t>> refList;
      refList.reserve(list.size());
      for (const auto& p : list) {
         refList.emplace_back(*p.first, p.second);
      }
      $self->registerSharedNoiseSource(refList);
   }
}

%ignore DynParamRegisterer::registerSharedNoiseSource(std::vector<std::pair<const StateData&, size_t>>);

%include "StatefulSysModel.h"
