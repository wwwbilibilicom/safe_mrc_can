#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <safemrc/safemrc.hpp>

namespace nb = nanobind;
using namespace safemrc;
using namespace safemrc::mrc;

NB_MODULE(safemrc_can, m) {
    m.doc() = "SafeMRC CAN driver Python bindings";

    nb::enum_<MRCMode>(m, "MRCMode")
        .value("FREE", MRCMode::FREE)
        .value("FIX_LIMIT", MRCMode::FIX_LIMIT)
        .value("ADAPTATION", MRCMode::ADAPTATION)
        .value("DEBUG", MRCMode::DEBUG)
        .value("MRC_RESET", MRCMode::MRC_RESET)
        .value("ZERO", MRCMode::ZERO)
        .value("REFRESH", MRCMode::REFRESH)
        .export_values();

    nb::class_<MRCCommand>(m, "MRCCommand")
        .def(nb::init<>())
        .def("__init__",
             [](MRCCommand* cmd, MRCMode mode, int32_t current) {
                 new (cmd) MRCCommand{mode, current};
             },
             nb::arg("mode"), nb::arg("des_coil_current") = 0)
        .def_rw("mode", &MRCCommand::mode)
        .def_rw("des_coil_current", &MRCCommand::des_coil_current);

    nb::class_<MRCFeedback>(m, "MRCFeedback")
        .def(nb::init<>())
        .def_rw("mode", &MRCFeedback::mode)
        .def_rw("collision", &MRCFeedback::collision)
        .def_rw("encoder_value", &MRCFeedback::encoder_value)
        .def_rw("present_current", &MRCFeedback::present_current)
        .def_prop_ro("position_rad", &MRCFeedback::position_rad)
        .def_prop_ro("current_amp", &MRCFeedback::current_amp);

    nb::class_<MRCState>(m, "MRCState")
        .def_prop_ro("device_id", &MRCState::device_id)
        .def_prop_ro("mode", &MRCState::mode)
        .def_prop_ro("collision", &MRCState::collision)
        .def_prop_ro("position_rad", &MRCState::position_rad)
        .def_prop_ro("current_amp", &MRCState::current_amp)
        .def("is_stale",
             [](const MRCState& s, int timeout_ms) {
                 return s.is_stale(std::chrono::milliseconds(timeout_ms));
             },
             nb::arg("timeout_ms") = 5)
        .def("feedback", &MRCState::feedback);

    nb::exception<can::CANSocketException>(m, "CANSocketException");

    nb::class_<SafeMRC>(m, "SafeMRC")
        .def(nb::init<const std::string&, bool>(),
             nb::arg("interface"), nb::arg("use_fd") = false)
        .def("add_device", &SafeMRC::add_device, nb::arg("device_id"))
        .def("remove_device", &SafeMRC::remove_device, nb::arg("device_id"))
        .def("send_command", &SafeMRC::send_command,
             nb::arg("device_id"), nb::arg("cmd"))
        .def("send_command_all", &SafeMRC::send_command_all, nb::arg("cmd"))
        .def("set_free", &SafeMRC::set_free, nb::arg("device_id"))
        .def("set_free_all", &SafeMRC::set_free_all)
        .def("set_fix_limit", &SafeMRC::set_fix_limit,
             nb::arg("device_id"), nb::arg("current_amp"))
        .def("set_adaptation", &SafeMRC::set_adaptation,
             nb::arg("device_id"), nb::arg("current_amp"))
        .def("reset_collision", &SafeMRC::reset_collision, nb::arg("device_id"))
        .def("set_zero", &SafeMRC::set_zero, nb::arg("device_id"))
        .def("device_state", &SafeMRC::device_state,
             nb::arg("device_id"), nb::rv_policy::reference)
        .def("device_ids", &SafeMRC::device_ids)
        .def("device_count", &SafeMRC::device_count)
        .def("start", &SafeMRC::start)
        .def("stop", &SafeMRC::stop)
        .def("is_running", &SafeMRC::is_running);
}
