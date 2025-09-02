# coding: utf8

import argparse
import socket
import os
import time
import libmaster_board_sdk_pywrap as mbs


def listener_script(name_interface):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_host = "127.0.0.1"
    udp_port = 5005

    cpt = 0
    dt = 0.001

    print("-- Start of listener script --")
    print(f"Sending IMU data to {udp_host}:{udp_port}")

    os.nice(-20)
    robot_if = mbs.MasterBoardInterface(name_interface, True)
    robot_if.Init()

    last = time.perf_counter()

    while True:
        if (time.perf_counter() - last) > dt:
            last = time.perf_counter()
            cpt += 1
            robot_if.ParseSensorData()

            try:
                acc_x = robot_if.imu_data_accelerometer(0)
                acc_y = robot_if.imu_data_accelerometer(1)
                acc_z = robot_if.imu_data_accelerometer(2)

                gyr_x = robot_if.imu_data_gyroscope(0)
                gyr_y = robot_if.imu_data_gyroscope(1)
                gyr_z = robot_if.imu_data_gyroscope(2)

                roll = robot_if.imu_data_attitude(0)
                pitch = robot_if.imu_data_attitude(1)
                yaw = robot_if.imu_data_attitude(2)
                
                imu_data_str_o = f"O {roll:.3f} {pitch:.3f} {yaw:.3f}"
                imu_data_str_a = f"A {acc_x:.3f} {acc_y:.3f} {acc_z:.3f}"
                imu_data_str_g = f"G {gyr_x:.3f} {gyr_y:.3f} {gyr_z:.3f}"

                data_to_send = "{} {} {}".format(imu_data_str_o, imu_data_str_a, imu_data_str_g)
                udp_socket.sendto(data_to_send.encode(), (udp_host, udp_port))

            except Exception as e:
                print(f"Error sending IMU data: {e}")

            if cpt % 100 == 0:
                print("Session ID : {}".format(robot_if.GetSessionId()))
                robot_if.PrintIMU()
                try:
                    print(f"Sending: {imu_data_str}")
                except:
                    pass


def main():
    parser = argparse.ArgumentParser(description='Listener for masterboard sensor packets in python.')
    parser.add_argument('-i',
                        '--interface',
                        required=True,
                        help='Name of the interface (use ifconfig in a terminal), for instance "enp1s0"')

    listener_script(parser.parse_args().interface)


if __name__ == "__main__":
    main()