"""
 # @ Author: Kenneth Simon
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2022-06-16 01:26:11
 # @ Modified time: 2022-06-16 02:02:25
 # @ Description: A script to control simulation and screen recording. Use with tmux and SimpleScreenRecorder.
 """


import pyautogui
import time
import os

if __name__ == "__main__":
    # takeoff
    uav_num = 3
    os.system("""tmux send -t dmpc_uav_ad_hoc:main.4 "C-c" ENTER""")
    os.system(
        f"""tmux send -t dmpc_uav_ad_hoc:main.4 "rosrun dmpc_uav_ad_hoc swarm_controller_switch.py on _uav_num:=${uav_num}" ENTER"""
    )
    print("start recording")
    pyautogui.hotkey("ctrl", "r")  # start record
    start_time = time.time()
    time.sleep(3)
    os.system("""tmux send -t dmpc_uav_ad_hoc:main.7 "C-c" ENTER""")
    os.system(
        f"""tmux send -t dmpc_uav_ad_hoc:main.7 "rosrun dmpc_uav_ad_hoc simu_time_ctrl.py _simu_time:=300 _uav_num:=${uav_num}" ENTER"""
    )
    input("Press Enter to pause recording: ")
    pyautogui.hotkey("ctrl", "r")  # pause record
    print(f"Recorded: {time.time() - start_time}")
