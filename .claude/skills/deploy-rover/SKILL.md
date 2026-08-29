---
name: deploy-rover
description: Deploy the current tree to the Orin (build + remote tests), restart the rover stack cleanly, and verify pose/tile rates. User-invoked only.
disable-model-invocation: true
---

# /deploy-rover

Runs, in order, and reports each step's result:

1. `./deploy_rover.sh --test` from the repo root (rsync + remote colcon + remote pytest + gate harness; ~3–5 min). Stop on failure.
2. Stop the running stack on the Orin — process names are the kernel's 15-char comm field, so these exact names:
   `ssh star@a_navi 'pkill -x start_navi.sh; pkill -x component_conta; pkill -x localization_st; pkill -x elevation_mappe; pkill -x video_sender; sleep 6; pgrep -x component_conta >/dev/null && pkill -9 -x component_conta'`
   Never use `pkill -f` (its pattern can match your own ssh command line).
3. Start: `ssh star@a_navi 'cd ~/navi && nohup ./start_navi.sh > /tmp/start_navi.log 2>&1 < /dev/null &'`
4. Verify (Orin env: `source /opt/ros/humble/setup.bash && source ~/workspaces/isaac_ros-dev/install/setup.bash && source ~/navi/install/local_setup.bash`): wait up to 100 s for `ros2 topic hz /localization/pose` (expect ≈15 Hz), then `hz` of `/localization/map_tile` and `/localization/obstacle_tile` for 12 s, and `ros2 topic echo --once /localization/map_status`.
5. Report: deploy test counts, pose-up time, the three rates, and `tail -5 /tmp/start_navi.log` if anything failed. Leave the stack running.

Power mode reminder: `cat /var/lib/nvpmodel/status` should say `pmode:0001` (25 W); mode IDs on this Orin: 0=15 W, 1=25 W, 2=MAXN, 3=7 W.
