#!/usr/bin/env bash
# PreToolUse: block publishing to /manual_twist (drives the physical rover)
# and any edit under the vendored, read-only IK sources.
input=$(cat)
tool=$(printf '%s' "$input" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("tool_name",""))' 2>/dev/null)
case "$tool" in
  Bash)
    cmd=$(printf '%s' "$input" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("tool_input",{}).get("command",""))' 2>/dev/null)
    if printf '%s' "$cmd" | grep -qE 'topic pub[^|]*/manual_twist'; then
      echo "BLOCKED: /manual_twist drives the physical rover. Use a scratch topic (--twist-topic /sim_test_twist) or a throwaway domain." >&2; exit 2; fi ;;
  Edit|Write|MultiEdit)
    path=$(printf '%s' "$input" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("tool_input",{}).get("file_path",""))' 2>/dev/null)
    case "$path" in *sim/src/navi_sim_ik/vendor/*) echo "BLOCKED: sim/src/navi_sim_ik/vendor/ is read-only (vendored Simulink/IK sources)." >&2; exit 2 ;; esac ;;
esac
exit 0
