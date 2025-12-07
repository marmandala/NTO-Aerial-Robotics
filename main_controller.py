#!/usr/bin/env python3
import subprocess
import rospy
import signal
import sys
import os

rospy.init_node("main_controller", anonymous=True)

rospy.set_param("/mission_command", "idle")

cwd = os.getcwd()
web_dir = os.path.join(cwd, "web")

rospy.loginfo("Launching subsystems...")

# Запуск процессов
# Vision
vision_proc = subprocess.Popen(["python3", "vision_pipeline.py"])
# Flight Control
flight_proc = subprocess.Popen(["python3", "flight_control.py"])
# Web Server
web_proc = subprocess.Popen(["python3", "-m", "http.server", "8000", "--directory", web_dir])

rospy.loginfo("All systems GO. Open http://localhost:8000")

def shutdown_handler(signum, frame):
    rospy.loginfo("Shutting down...")
    vision_proc.terminate()
    flight_proc.terminate()
    web_proc.terminate()
    rospy.signal_shutdown("Main controller exit")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown_handler)
signal.signal(signal.SIGTERM, shutdown_handler)

rospy.spin()