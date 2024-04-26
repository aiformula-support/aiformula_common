## Launchers
This package manages launch files each node\
**Note:**  Topic name and Frame_id are maneged YAML file

| Topic Name | `/config/topic_lint.yaml` |
| ---- | ---- |
| Frame id | `/config/frame_id_list.yaml` |

### Launch Files
* socket_can_bridge.launch.py\
**Topic**\
  input_can_data: "/aiformula_sensing/vehicle_info"\
  output_can_data: "/aiformula_control/motor_controller/reference_signal"

* joy.launch.py\
**Topic:**\
  game_pad: "/aiformula_control/joy_node/joy"

* teleop.launch.py\
**Topic**\
  game_pad: "/aiformula_control/joy_node/joy"

* vectornav.launch.py\
**Topic**\
  game_pad: "/aiformula_control/game_pad/cmd_vel"\
  handle_controller: "/aiformula_control/handle_controller/cmd_vel"


* zedx_camera.launch.py\
**Topic:**\
  left_image: /aiformula_sensing/zed_node/left_image/undistorted\
  right_image: /aiformula_sensing/zed_node/right_image/undistorted\
  imu: /aiformula_sensing/zed_node/imu