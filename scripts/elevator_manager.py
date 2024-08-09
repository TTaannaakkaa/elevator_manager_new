#!/usr/bin/env python3

import rospy
import subprocess
import os
import signal
from std_msgs.msg import Bool

class LaunchManager:
    def __init__(self):
        self.process = None
        self.flag_up = False
        self.flag_down = False

    def start_command(self, command):
        self.process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
        rospy.logwarn(f"Command '{command}' started with PID {self.process.pid}.")

    def stop_command(self):
        if self.process:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            rospy.logwarn("Launch file stopped.")
            self.process = None
        else:
            rospy.logwarn("No launch file to stop.")

    def upflag_callback(self, upflag):
            rospy.loginfo("I heard: %s", upflag.data)
            self.flag_up = upflag.data

    def up_flag_listener(self):
        rospy.Subscriber('up_flag', Bool, self.upflag_callback)

    def downflag_callback(self, upflag):
            rospy.loginfo("I heard: %s", upflag.data)
            self.flag_down = upflag.data

    def down_flag_listener(self):
        rospy.Subscriber('down_flag', Bool, self.downflag_callback)

if __name__ == '__main__':
    rospy.init_node('launch_manager')
    manager = LaunchManager()
    command_1f_first = rospy.get_param('~command_1f_first', '/default/path/to/your/launchfile.launch')
    command_1f_second = rospy.get_param('~command_1f_second', '/default/path/to/your/launchfile.launch')
    command_2f = rospy.get_param('~command_2f', '/default/path/to/your/launchfile.launch')

    manager.start_command(command_1f_first)
    manager.up_flag_listener()
    manager.down_flag_listener()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if manager.flag_up:
            manager.stop_command()
            manager.start_command(command_2f)
            manager.flag_up = False

        if manager.flag_down:
            manager.stop_command()
            manager.start_command(command_1f_second)
            manager.flag_down = False
        rate.sleep()

    rospy.spin()
