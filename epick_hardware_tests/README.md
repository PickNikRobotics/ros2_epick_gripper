# Hardware tests

This package contains commands to manually test the gripper functionality.

`activate`

Connect to the gripper and activate it.

`get_status`

Connect to the gripper, activate it and get the gripper status.

`set_relative_pressure`

Connect to the gripper, activate it and set the gripper pressure in kPa.

- pressure = 0kPa means -100Kpa below atmospheric pressure for the gripper to hold the object at maximum power.

- pressure = 100kPa means atmospheric pressure, for the gripper to passively hold the object.

- pressure > 100kPa means above atmospheric pressure for the gripper to actively release the object.
