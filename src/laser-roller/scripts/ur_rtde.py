"""
This script includes implementation for robot control 
functions for UR robots using the ur_rtde package
"""
import rtde_control, rtde_receive, rtde_io
import rospy
from robot_base import Robot


class UrRtde(Robot):
    def __init__(self, robot_ip, read_only=False):
        Robot.__init__(self)
        self.robot_ip = robot_ip
        self.read_only = read_only
        # RTDEReceiveInterface is read-only and never conflicts with EtherNet/IP/MODBUS
        self.robot_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        if not read_only:
            # These use input registers and will conflict if EtherNet/IP/MODBUS is active
            self.robot_c = rtde_control.RTDEControlInterface(robot_ip)
            self.robot_io = None # rtde_io.RTDEIOInterface(robot_ip)  # Temporarily disabled to avoid register conflict
        else:
            self.robot_c = None
            self.robot_io = None
        
    def move_TCP(self, pose_vec, vel, acc, slow=False):
        if not self.robot_c.moveL((pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]), vel, acc): #TODO: Debug wait
            self.robot_c.reuploadScript()
            rospy.sleep(1)
            if slow:
                self.robot_c.moveL((pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]), vel/5, acc/5)
            else:
                self.robot_c.moveL((pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]), vel, acc)

    
    def move_TCP_compound(self, pose_vec_list, vel, acc, blend=0.1, slow=False):
        pose_list = []
        print(pose_vec_list)
        for pose_vec in pose_vec_list:
            pose_list.append([pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5], vel, acc, blend])
        print(pose_list)
        self.robot_c.moveL(pose_list, vel, acc) #TODO: Debug wait
    
    def speed_command(self, twist_vec, acc):
        self.robot_c.speedL(twist_vec, acc, 0)

    def get_pose(self):
        return self.robot_r.getActualTCPPose()

    def get_vel(self):
       return  self.robot_r.getActualTCPSpeed()
                
    def get_wrench(self):
        return self.robot_r.getActualTCPForce()

    def setio(self, pin, value):
        self.robot_io = rtde_io.RTDEIOInterface(self.robot_ip) #TODO: investigate this
        self.robot_io.setio(pin, value)

    def get_analog_input(self):
        return self.robot_r.getStandardAnalogInput1()

    def disconnect(self):
        """Cleanly stop and disconnect all RTDE interfaces."""
        try:
            self.robot_c.stopScript()
        except Exception:
            pass
        try:
            self.robot_c.disconnect()
        except Exception:
            pass
        try:
            self.robot_r.disconnect()
        except Exception:
            pass
        try:
            self.robot_io.disconnect()
        except Exception:
            pass