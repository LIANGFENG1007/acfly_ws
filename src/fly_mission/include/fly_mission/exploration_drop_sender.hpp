#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include "fly_mission/arduino_serial.hpp"

namespace fly_mission {
// One batch per ordinal per mission. No serial reopen or blocking wait in flight.
class ExplorationDropSender {
public:
    struct Result { unsigned index; std::uint64_t target; bool success; std::string detail; };
    void reset(ArduinoSerial& serial) { cancel(serial); results_ = {}; completed_ = 0; }
    void cancel(ArduinoSerial& serial) {
        if (pending_) { serial.cancel_pending(); finish(false, "mission interrupted"); }
    }
    void request(unsigned index, std::uint64_t target, double now, ArduinoSerial& serial, int repeats) {
        if (!index || index > 3 || !target || pending_ || index != completed_ + 1) return;
        pending_ = Result{index, target, false, ""}; started_ = now;
        const std::string command = index == 1 ? "DIANCI" : index == 2 ? "DIANCI2" : "DIANCI3";
        // Previous unrelated traffic must not be mistaken for a successful batch.
        if (!serial.ok()) finish(false, "serial port is not open");
        else if (serial.pending() || !serial.queue(command, repeats)) finish(false, "serial queue unavailable");
    }
    // Check the deadline before touching the fd: a delayed timer must not
    // transmit an expired batch and then incorrectly report success.
    void pump(double now, ArduinoSerial& serial) {
        poll(now, serial);
        serial.pump();
        poll(now, serial);
    }
    void poll(double now, ArduinoSerial& serial) {
        if (!pending_) return;
        if (!serial.ok()) { serial.cancel_pending(); finish(false, "serial write failed"); }
        else if (!serial.pending()) finish(true, "all five lines written to serial driver (no device ACK)");
        else if (now - started_ >= 1.0) {
            serial.cancel_pending(); finish(false, "serial write timed out; unsent bytes discarded");
        }
    }
    std::optional<Result> result(unsigned index, std::uint64_t target) const {
        if (!index || index > 3 || !results_[index-1]) return {};
        if (results_[index-1]->target != target)
            return Result{index, target, false, "package ordinal already consumed for another target; not retransmitted"};
        return results_[index-1];
    }
    bool pending() const { return pending_.has_value(); }
private:
    void finish(bool ok, const std::string& detail) {
        pending_->success = ok; pending_->detail = detail;
        completed_ = pending_->index; results_[completed_-1] = pending_; pending_.reset();
    }
    std::array<std::optional<Result>, 3> results_{};
    std::optional<Result> pending_;
    unsigned completed_ = 0;
    double started_ = 0;
};
} // namespace fly_mission
