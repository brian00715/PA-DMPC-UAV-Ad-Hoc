"""
 # @ Author: Your name
 # @ Create Time: 2022-05-27 14:04:34
 # @ Modified by: Your name
 # @ Modified time: 2022-06-03 00:27:37
 # @ Description: conduct simulation experiment automatically
 """


import os
import socket
import subprocess
import sys
import time
from threading import Thread

from tqdm import tqdm

# experiment settings
exp_idx = 8
simu_time = 300
uav_nums = [i for i in range(3, 10, 1)]
# uav_nums = [3, 8, 9]
controllers = [
    "nmpc",
    # "px4"
]
schedulers = [
    "both",
    # "only_local",
]
fix_nmpc_param = False

tcp_recv_msg = ""
global_stop_flag = False


def tcp_recv_cb(client_socket: socket.socket):
    global tcp_recv_msg, global_stop_flag
    try:
        while not global_stop_flag:
            tcp_recv_msg = str(client_socket.recv(1024), encoding="utf-8")
            if tcp_recv_msg != "":
                # print(f"[{time.time():<20}] received: {tcp_recv_msg} from {client_socket.getpeername()}")
                client_socket.send(bytes(tcp_recv_msg, encoding="utf-8"))
            time.sleep(0.01)
        print(f"global_stop_flag: {global_stop_flag}")
    except Exception as e:
        print(f"{e}")
    finally:
        client_socket.close()


def tcp_server(host, port):
    global tcp_recv_msg, global_stop_flag
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    print(f"[{time.time():<20}] TCP server listening on {host}:{port}")

    while not global_stop_flag:
        try:
            client_socket, client_addr = server_socket.accept()
            clinet_recv_thread = Thread(target=tcp_recv_cb, args=(client_socket,))
            clinet_recv_thread.setDaemon(True)
            clinet_recv_thread.start()
            time.sleep(0.1)
        except Exception as e:
            client_socket.join()
            server_socket.close()
            break
    print(f"global_stop_flag: {global_stop_flag}")


if __name__ == "__main__":
    time_per_uav = 2.5

    # use socket to receive stop signal from subprocess, which can prevent killing subprocess before it really finished
    server_thread = Thread(target=tcp_server, args=("localhost", 10086))
    server_thread.setDaemon(True)
    server_thread.start()

    for i, controller in enumerate(controllers):
        for scheduler in schedulers:
            for uav_num in uav_nums:
                cmd = [
                    "./scripts/run_all.sh",
                    "-u",
                    str(uav_num),
                    "-t",
                    str(simu_time),
                    "-c",
                    controller,
                    "-r",
                ]
                if scheduler == "both":
                    cmd.extend(["-g", "-l"])
                    cmd.extend(["-o", sys.path[0] + f"/../data/perf_anaysis/exp{exp_idx}/both"])
                if scheduler == "only_local":
                    cmd.extend(["-l"])
                    cmd.extend(["-o", sys.path[0] + f"/../data/perf_anaysis/exp{exp_idx}/only_local"])
                if fix_nmpc_param:
                    cmd.extend(["-f"])
                print(f"{'controller':<25}: {controller}")
                print(f"{'uav_num':<25}: {uav_num}")
                print(f"{'simu_time':<25}: {simu_time}")
                start_time = time.time()
                process = subprocess.Popen(cmd)
                realtime_factor = 0.0875 * uav_num + 0.9125
                timeout_limit = (time_per_uav * uav_num + simu_time + 15) * realtime_factor
                simu_stop_flag = False
                pbar = tqdm(total=timeout_limit, desc="simu progess", unit="s")
                tcp_recv_msg = ""
                try:
                    while True:
                        elapsed_time = time.time() - start_time
                        pbar.update(int(elapsed_time) - pbar.n)
                        if tcp_recv_msg == "1":
                            simu_stop_flag = True
                            print(f"[{time.time():<20}] received stop signal.")
                        if elapsed_time > timeout_limit or simu_stop_flag:
                            subprocess.call(["./scripts/kill_all.sh"])
                            process.terminate()
                            break
                        time.sleep(1)
                    pbar.close()
                    print(f"[{time.time():<20}] simulation finished.")
                    print("-" * 50)
                    time.sleep(1)
                except KeyboardInterrupt as e:
                    print(f"{e}")
                    subprocess.call(["./scripts/kill_all.sh"])
                    process.terminate()
                    os.system("tmux kill-session -t dmpc_uav_ad_hoc")
                    exit(0)
    global_stop_flag = True
    exit(0)
