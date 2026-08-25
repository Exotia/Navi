# Ground Station — Manual Verification

Automated tests (`pytest`) cover all app logic with fake ROS objects. This
step is the one real check that needs an actual rosbridge server, which
isn't available in the dev sandbox this plan was built in.

## Steps (run on/near the Jetson, once ROS2 Humble + rosbridge_suite are set up)

1. On the Jetson: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
2. In another terminal on the Jetson: publish a test Twist repeatedly, e.g.
   `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4, y: -0.05}, angular: {z: 0.1}}"`
3. On the ground station machine: `python -m ground_station.main --host <jetson-ip>`
4. Confirm:
   - Header shows "ROSBRIDGE: CONNECTED" within a couple seconds.
   - The Drive card shows live vx/vy/wz values matching step 2 and a
     nonzero Hz.
   - System Nodes panel lists at least `/rosbridge_websocket`.
   - Clicking "view details" opens the Drive detail page with the same
     live values and a scrolling raw-message log; "back to dashboard"
     returns to the dashboard.
5. Stop the `ros2 topic pub` command and confirm the Drive card's Hz
   reading drops to "0 Hz (no data)" within about a second (the staleness
   check firing once no new samples have entered the rate window), and
   that the Drive detail page's `/cmd_vel` readout shows the same
   stale/no-data indication.
