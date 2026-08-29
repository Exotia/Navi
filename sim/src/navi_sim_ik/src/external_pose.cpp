#include "navi_sim_ik/external_pose.hpp"

#include <string>

namespace navi_sim_ik
{

std::string localization_state(const std::string & status_json)
{
  const std::string key = "\"state\"";
  const auto key_at = status_json.find(key);
  if (key_at == std::string::npos) {
    return "";
  }
  const auto colon = status_json.find(':', key_at + key.size());
  if (colon == std::string::npos) {
    return "";
  }
  // The value must be a quoted string belonging to THIS key. Without the
  // end bound, {"state": 3, "source": "zed_vio"} would answer "zed_vio" -
  // a state that looks unrecognised rather than absent, which is a
  // different and more confusing failure.
  const auto value_end = status_json.find_first_of(",}", colon + 1);
  const auto open = status_json.find('"', colon + 1);
  if (open == std::string::npos ||
    (value_end != std::string::npos && open > value_end))
  {
    return "";
  }
  const auto close = status_json.find('"', open + 1);
  if (close == std::string::npos) {
    return "";
  }
  return status_json.substr(open + 1, close - open - 1);
}

ExternalPoseGate::ExternalPoseGate(double max_rate_hz)
: min_interval_(max_rate_hz > 0.0 ? 1.0 / max_rate_hz : 0.0)
{
}

void ExternalPoseGate::set_state(const std::string & state)
{
  state_ = state;
}

bool ExternalPoseGate::accept(double now_seconds)
{
  if (!ok()) {
    return false;
  }
  // ever_applied_ rather than a sentinel time: a steady clock starts
  // wherever it starts, so comparing an unset last_applied_ of 0.0 against
  // it would either pass by luck or block for as long as the machine has
  // been up.
  if (ever_applied_ && now_seconds - last_applied_ < min_interval_) {
    return false;
  }
  last_applied_ = now_seconds;
  ever_applied_ = true;
  return true;
}

}  // namespace navi_sim_ik
