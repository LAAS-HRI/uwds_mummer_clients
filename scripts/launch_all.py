#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import subprocess
import time
from threading import Thread
from signal import signal, SIGINT
from sys import exit

class Launcher(object):
    def __init__(self, waiting_time=2.0):
        self.server_thread = Thread(target=self.launch_server, args=[None])
        self.server_thread.start()
        rospy.loginfo("Underworlds server started, waiting to launch the pipeline...")
        time.sleep(waiting_time)
        self.pipeline_thread = Thread(target=self.launch_pipeline, args=[None])
        self.pipeline_thread.start()
        rospy.loginfo("Underworlds pipeline started, waiting to load the environment...")
        time.sleep(waiting_time)
        self.env_thread = Thread(target=self.launch_env, args=[None])
        self.env_thread.start()
        rospy.loginfo("Environment loaded, waiting to launch the navigation...")
        time.sleep(waiting_time)
        self.nav_thread = Thread(target=self.launch_nav, args=[None])
        self.nav_thread.start()
        signal(SIGINT, self.kill_them_all)
        self.run()

    def launch_server(self, arg):
        self.p1 = subprocess.Popen(['xterm', '-e', 'roslaunch uwds uwds_server.launch'])

    def launch_pipeline(self, arg):
        self.p2 = subprocess.Popen(['xterm', '-e', 'roslaunch uwds_mummer_clients mummer_pipeline.launch'])

    def launch_env(self, arg):
        self.p3 = subprocess.Popen(['xterm', '-e', 'roslaunch uwds_mummer_clients ideapark_provider.launch'])

    def launch_nav(self, arg):
        self.p4 = subprocess.Popen(['xterm', '-e', 'roslaunch uwds_mummer_clients navigation.launch'])

    def kill_them_all(self, signal_received, frame):
        rospy.loginfo("\n\rSIGINT or CTRL-C detected. Killing threads...")
        self.p1.send_signal(SIGINT)
        self.p2.send_signal(SIGINT)
        self.p3.send_signal(SIGINT)
        self.p4.send_signal(SIGINT)
        self.server_thread.join()
        self.pipeline_thread.join()
        self.env_thread.join()
        self.nav_thread.join()
        rospy.loginfo("Threads killed, exiting properly.")
        exit(0)

    def run(self):
        while True:
            time.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node("Underworlds launcher")
    launcher = Launcher()
