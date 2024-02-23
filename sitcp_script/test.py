#!/usr/bin/python
# This is test.py file
# author: zhj@ihep.ac.cn
# 2019-06-18 created

from time import sleep
import time
import sys
import os
import socket
current_path = os.path.realpath(__file__)
directory_path = os.path.dirname(current_path)
sys.path.insert(0, directory_path+"/lib")
import rbcp
import sysmon
import reg
import spi
import i2c

# import interface

TEST_REG    = 1
TEST_SYSMON = 0
TEST_TCP_TX = 1

#################################################################
# register test
if TEST_REG:
    board_reg = reg.reg()
    board_reg.read_info()
    # board_reg.read_sitcp_reg()

#################################################################
# sysmon test
if TEST_SYSMON:
    sysmon = sysmon.sysmon()
    sysmon.print_status()
    print("")

if TEST_TCP_TX:
    reg = reg.reg()
    reg.set_sitcp_keep_alive()
    reg.set_sitcp_fast_retrains()
    reg.set_sitcp_nagle()
    reg.set_sitcp_window_scaling()
    reg.set_sitcp_retransmission_time(250)

    test_duration = 20 # seconds
    tx_rate = 100 # unit: 100 Mbps
    clear_buffer = 0
    check_data_fast = 1 # Note that it's fast but it check the package head and tail only 
    check_one_by_one = 0 # Note that it is very slow, the max speed is about 150Mbps
    print_error = 1

    num_of_data = int(test_duration*tx_rate*100_000_000/8)
    blk_size = 1024

    reg.set_tcp_test_data_gen(0)
    reg.set_tcp_test_tx_rate(tx_rate)
    reg.set_tcp_test_blk_size(blk_size)
    reg.set_tcp_test_num_of_data(num_of_data)

    # Set socket timeout option
    timeout = 5
    socket.setdefaulttimeout(timeout)

    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_ip = "192.168.10.16"
    server_port = 24
    tcp_socket.connect((server_ip, server_port))

    error_cnt = 0
    rxlength = -1
    if clear_buffer:     # clear buffer
        try:
            while (rxlength):
                recv_data = tcp_socket.recv(1460)
                rxlength = len(recv_data)
        except socket.timeout:
            print("Recv buffer cleared")

    print("Start tcp tx test")
    rxlength = 0
    recv_data = []
    start = time.time()
    reg.set_tcp_test_data_gen(1)
    try:
        while (rxlength < num_of_data):
            recv_data = tcp_socket.recv(1460)
            data_len = len(recv_data)
            # check data
            if rxlength != 0:
                if((pre_data + 1) != recv_data[0]):
                    if (pre_data==0xff)&(recv_data[0]==0x1):
                        pass
                    else:
                        error_cnt += 2
                        if print_error:
                            print("boundry error: 0x%x, 0x%x"%(pre_data, recv_data[0]))
            if check_one_by_one:
                for i in range(data_len-1):
                    if(recv_data[i]+1 != recv_data[i+1]):
                        if (recv_data[i] == 0xff) & (recv_data[i+1] == 0x1):
                            pass
                        else:
                            error_cnt += 1
                            if print_error:
                                print("error: 0x%x, 0x%x"%(recv_data[i], recv_data[i+1]))
            elif check_data_fast:
                cycles = data_len%255 - 1
                last_data = recv_data[0]+cycles
                if last_data > 0xFF:
                    last_data += 1
                if (last_data&0xFF != recv_data[-1]):
                    print(".", end = "")
                    error_cnt += 1
                    if print_error:
                        for i in range(data_len-1):
                            if(recv_data[i]+1 != recv_data[i+1]):
                                if (recv_data[i] == 0xff) & (recv_data[i+1] == 0x1):
                                    pass
                                else:
                                    print("error: 0x%x, 0x%x"%(recv_data[i], recv_data[i+1]))
            rxlength += data_len
            # print(rxlength)
            pre_data=recv_data[-1]
        stop = time.time()
        run_time = stop - start
    except socket.timeout:
        stop = time.time()
        run_time = stop - start - timeout
        print("Timeout end")
    finally:
        tcp_socket.close()
        reg.set_tcp_test_data_gen(0)

    print("\nData length: %d bytes"%rxlength)
    print("Duration: %f s"%run_time)
    print("Speed: %.2f Mbps"%(8*rxlength/run_time/1000_000))
    if check_data_fast|check_one_by_one:
        print("Package error count: %d"%(error_cnt/2))
