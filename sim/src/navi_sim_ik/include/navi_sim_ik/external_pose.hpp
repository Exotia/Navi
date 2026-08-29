#ifndef NAVI_SIM_IK__EXTERNAL_POSE_HPP_
#define NAVI_SIM_IK__EXTERNAL_POSE_HPP_

#include <string>

namespace navi_sim_ik
{

/// The "state" field of a /localization/status payload, or "" if the payload
/// does not carry one as a string.
///
/// Scanned rather than parsed. The alternative is a JSON dependency in this
/// package for one string field out of a document this project's own
/// localization_status node writes with json.dumps - and a parser that
/// throws on a malformed payload would need the same "anything unreadable is
/// not OK" fallback this returns directly.
std::string localization_state(const std::string & status_json);

/// Decides whether a pose from outside may be written into the simulation.
///
/// Two rules, both of them the spec's:
///  - the model holds still whenever /localization/status is not OK, because
///    a rover drawn where it was 40 seconds ago, moving, is worse than one
///    that visibly stops;
///  - poses are applied at no more than max_rate_hz, because each one costs
///    a service call and a write on Gazebo's physics thread.
///
/// Deliberately knows nothing about ROS or Gazebo: everything it decides is
/// decidable from a state string and a clock reading, which is what lets it
/// be tested exhaustively without a node.
class ExternalPoseGate
{
public:
  explicit ExternalPoseGate(double max_rate_hz = 30.0);

  /// The latest state from /localization/status. "" means none has arrived.
  void set_state(const std::string & state);
  const std::string & state() const {return state_;}
  bool ok() const {return state_ == "OK";}

  /// True if a pose seen at now_seconds may be applied, and records that it
  /// was. A refusal does not move the rate window: if it did, a publisher
  /// faster than max_rate_hz would starve the gate forever.
  bool accept(double now_seconds);

private:
  double min_interval_;
  std::string state_;
  double last_applied_{0.0};
  bool ever_applied_{false};
};

}  // namespace navi_sim_ik

#endif  // NAVI_SIM_IK__EXTERNAL_POSE_HPP_
