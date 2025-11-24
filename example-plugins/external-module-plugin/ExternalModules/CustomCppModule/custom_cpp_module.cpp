#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

#include <Basilisk/architecture/_GeneralModuleFiles/sys_model.h>
#include <Basilisk/architecture/messaging/messaging.h>

#include <bsk/sdk.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace {

struct CustomPluginMsgPayload
{
    std::array<double, 3> dataVector{};
};

class CustomCppModule : public SysModel
{
  public:
    CustomCppModule()
      : output_writer_(dataOutMsg.addAuthor())
      , input_reader_(input_channel_.addSubscriber())
      , input_writer_(input_channel_.addAuthor())
    {
        this->ModelTag = "CustomCppModule";
    }

    void Reset(uint64_t /*current_sim_nanos*/) override
    {
        reset_called_ = true;
        update_called_ = false;
        steps_ = 0;
        last_input_ = {};
        last_output_ = {};
        last_update_nanos_ = 0;
    }

    void UpdateState(uint64_t current_sim_nanos) override
    {
        update_called_ = true;
        ++steps_;

        if (input_reader_.isLinked() && input_reader_.isWritten()) {
            last_input_ = input_reader_();
        }

        last_output_ = last_input_;
        last_output_.dataVector[0] += static_cast<double>(steps_);
        last_output_.dataVector[2] = static_cast<double>(current_sim_nanos) * 1e-9;

        output_writer_(&last_output_, this->moduleID, current_sim_nanos);
        last_update_nanos_ = current_sim_nanos;
    }

    void set_input_payload(CustomPluginMsgPayload payload)
    {
        // WriteFunctor wants a non-const pointer; payload is local, so OK.
        input_writer_(&payload, this->moduleID, last_update_nanos_);
    }

    CustomPluginMsgPayload last_input() const { return last_input_; }
    CustomPluginMsgPayload last_output() const { return last_output_; }
    uint64_t last_update_nanos() const { return last_update_nanos_; }
    bool reset_called() const { return reset_called_; }
    bool update_called() const { return update_called_; }

  private:
    Message<CustomPluginMsgPayload> dataOutMsg;
    Message<CustomPluginMsgPayload> input_channel_;

    WriteFunctor<CustomPluginMsgPayload> output_writer_;
    ReadFunctor<CustomPluginMsgPayload> input_reader_;
    WriteFunctor<CustomPluginMsgPayload> input_writer_;

    CustomPluginMsgPayload last_input_{};
    CustomPluginMsgPayload last_output_{};

    uint64_t last_update_nanos_ = 0;
    bool reset_called_ = false;
    bool update_called_ = false;
    int steps_ = 0;
};

} // namespace

PYBIND11_MODULE(_custom_cpp, m)
{
    namespace py = pybind11;

    py::class_<CustomPluginMsgPayload>(m, "CustomPluginMsgPayload")
      .def(py::init<>())
      .def(py::init([](const std::vector<double>& values) {
          CustomPluginMsgPayload payload;
          for (std::size_t i = 0; i < std::min(values.size(), payload.dataVector.size()); ++i) {
              payload.dataVector[i] = values[i];
          }
          return payload;
      }))
      .def_readwrite("dataVector", &CustomPluginMsgPayload::dataVector);

    py::class_<CustomCppModule>(m, "CustomCppModule")
      .def(py::init<>())
      .def("Reset", &CustomCppModule::Reset, py::arg("current_sim_nanos"))
      .def("UpdateState", &CustomCppModule::UpdateState, py::arg("current_sim_nanos"))
      .def("set_input_payload", &CustomCppModule::set_input_payload, py::arg("payload"))
      .def_property_readonly("last_input", &CustomCppModule::last_input)
      .def_property_readonly("last_output", &CustomCppModule::last_output)
      .def_property_readonly("last_update_nanos", &CustomCppModule::last_update_nanos)
      .def_property_readonly("reset_called", &CustomCppModule::reset_called)
      .def_property_readonly("update_called", &CustomCppModule::update_called);

    m.def("create_factory", []() { return bsk::plugin::make_factory<CustomCppModule>(); });
}
