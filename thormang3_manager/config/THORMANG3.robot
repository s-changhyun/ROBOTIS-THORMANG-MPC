[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/DXLBUS0 | 2000000  | r_arm_sh_p1
/dev/DXLBUS1 | 2000000  | l_arm_sh_p1
/dev/DXLBUS2 | 2000000  | r_leg_hip_y
/dev/DXLBUS3 | 2000000  | l_leg_hip_y

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME     | BULK READ ITEMS
dynamixel | /dev/DXLBUS0 | 1   | H54-100-S500-R | 2.0      | r_arm_sh_p1  | present_position, present_voltage
dynamixel | /dev/DXLBUS1 | 2   | H54-100-S500-R | 2.0      | l_arm_sh_p1  | present_position, present_voltage
dynamixel | /dev/DXLBUS0 | 3   | H54-100-S500-R | 2.0      | r_arm_sh_r   | present_position, present_voltage
dynamixel | /dev/DXLBUS1 | 4   | H54-100-S500-R | 2.0      | l_arm_sh_r   | present_position, present_voltage
dynamixel | /dev/DXLBUS0 | 5   | H54-100-S500-R | 2.0      | r_arm_sh_p2  | present_position, present_voltage
dynamixel | /dev/DXLBUS1 | 6   | H54-100-S500-R | 2.0      | l_arm_sh_p2  | present_position, present_voltage
dynamixel | /dev/DXLBUS0 | 7   | H54-100-S500-R | 2.0      | r_arm_el_y   | present_position, present_voltage
dynamixel | /dev/DXLBUS1 | 8   | H54-100-S500-R | 2.0      | l_arm_el_y   | present_position, present_voltage
dynamixel | /dev/DXLBUS0 | 9   | H42-20-S300-R  | 2.0      | r_arm_wr_r   | present_position, present_voltage
dynamixel | /dev/DXLBUS1 | 10  | H42-20-S300-R  | 2.0      | l_arm_wr_r   | present_position, present_voltage
dynamixel | /dev/DXLBUS0 | 11  | H42-20-S300-R  | 2.0      | r_arm_wr_y   | present_position, present_voltage
dynamixel | /dev/DXLBUS1 | 12  | H42-20-S300-R  | 2.0      | l_arm_wr_y   | present_position, present_voltage
dynamixel | /dev/DXLBUS0 | 13  | H42-20-S300-R  | 2.0      | r_arm_wr_p   | present_position, present_voltage
dynamixel | /dev/DXLBUS1 | 14  | H42-20-S300-R  | 2.0      | l_arm_wr_p   | present_position, present_voltage
dynamixel | /dev/DXLBUS2 | 15  | H54-100-S500-R | 2.0      | r_leg_hip_y  | present_position, present_voltage
dynamixel | /dev/DXLBUS3 | 16  | H54-100-S500-R | 2.0      | l_leg_hip_y  | present_position, present_voltage
dynamixel | /dev/DXLBUS2 | 17  | H54-200-S500-R | 2.0      | r_leg_hip_r  | present_position, present_voltage
dynamixel | /dev/DXLBUS3 | 18  | H54-200-S500-R | 2.0      | l_leg_hip_r  | present_position, present_voltage
dynamixel | /dev/DXLBUS2 | 19  | H54-200-B500-R | 2.0      | r_leg_hip_p  | present_position, present_voltage
dynamixel | /dev/DXLBUS3 | 20  | H54-200-B500-R | 2.0      | l_leg_hip_p  | present_position, present_voltage
dynamixel | /dev/DXLBUS2 | 21  | H54-200-S500-R | 2.0      | r_leg_kn_p   | present_position, present_voltage
dynamixel | /dev/DXLBUS3 | 22  | H54-200-S500-R | 2.0      | l_leg_kn_p   | present_position, present_voltage
dynamixel | /dev/DXLBUS2 | 23  | H54-200-B500-R | 2.0      | r_leg_an_p   | present_position, present_voltage
dynamixel | /dev/DXLBUS3 | 24  | H54-200-B500-R | 2.0      | l_leg_an_p   | present_position, present_voltage
dynamixel | /dev/DXLBUS2 | 25  | H54-200-S500-R | 2.0      | r_leg_an_r   | present_position, present_voltage
dynamixel | /dev/DXLBUS3 | 26  | H54-200-S500-R | 2.0      | l_leg_an_r   | present_position, present_voltage
dynamixel | /dev/DXLBUS0 | 27  | H54-100-S500-R | 2.0      | torso_y      | present_position, present_voltage
dynamixel | /dev/DXLBUS1 | 28  | H42-20-S300-R  | 2.0      | head_y       | present_position, present_voltage
dynamixel | /dev/DXLBUS1 | 29  | H42-20-S300-R  | 2.0      | head_p       | present_position, present_voltage
dynamixel | /dev/DXLBUS1 | 30  | GRIPPER        | 2.0      | l_arm_grip   | present_position, present_voltage
dynamixel | /dev/DXLBUS0 | 31  | GRIPPER        | 2.0      | r_arm_grip   | present_position, present_voltage
