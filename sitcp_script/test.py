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
import spi
import i2c

# import interface

TEST_REG    = 1
TEST_SYSMON = 1
TEST_TCP_TX = 1

REG_SYN_INFO = 0x0
REG_SYN_VER = 0x4
REG_FPGA_DNA = 0x5
REG_TCP_MODE = 0xd
REG_TCP_TEST_TX_RATE = 0xe
REG_TCP_TEST_NUM_OF_DATA = 0xf
REG_TCP_TEST_DATA_GEN = 0x17
REG_TCP_TEST_WORD_LEN = 0x18
REG_TCP_TEST_SELECT_SEQ = 0x19
REG_TCP_TEST_SEQ_PATTERN = 0x1a
REG_TCP_TEST_BLK_SIZE = 0x1e
REG_TCP_TEST_INS_ERROR = 0x21

def read_info(): # the date of compiling
    temp = reg.read(REG_SYN_INFO, 4)
    # print(temp)
    hour  = hex(temp[0]).lstrip("0x")
    day   = hex(temp[1]).lstrip("0x")
    month = hex(temp[2]).lstrip("0x")
    year  = "20" + hex(temp[3]).lstrip("0x")
    print("Compiling date: ", year, "-", month, "-", day, ",", hour, ":00")
    temp = reg.read(REG_SYN_VER, 1)
    print("Firmware version: ", temp[0])

def read_fpga_dna(): # FPGA DNA
    return reg.read(REG_FPGA_DNA, 8)

def set_tcp_loopback_mode():
    reg.write(REG_TCP_MODE, bytes([0x1]))

def set_tcp_test_mode():
    reg.write(REG_TCP_MODE, bytes([0x2]))

def set_tcp_normal_mode():
    reg.write(REG_TCP_MODE, bytes([0x0]))

def set_tcp_test_tx_rate(speed_in_100Mbps):
    speed_in_100Mbps &= 0xFF
    reg.write(REG_TCP_TEST_TX_RATE, bytes([speed_in_100Mbps]))

def set_tcp_test_num_of_data(num):
    if num > 0xFFFF_FFFF_FFFF_FFFF:
        print("Max length is 0xFFFFFFFFFFFFFFFF")
        num = 0xFFFF_FFFF_FFFF_FFFF
    bytes_num = num.to_bytes(8, byteorder='big')
    # print(bytes_num)
    reg.write(REG_TCP_TEST_NUM_OF_DATA, bytes_num)

def set_tcp_test_data_gen(enable):
    if(enable):
        reg.write(REG_TCP_TEST_DATA_GEN, bytes([0x1]))
    else:
        reg.write(REG_TCP_TEST_DATA_GEN, bytes([0x0]))

def set_tcp_test_word_len(len=8):
    len = (len-1)&0x7
    reg.write(REG_TCP_TEST_WORD_LEN, bytes([0x1]))

def set_tcp_test_select_seq(enable):
    if(enable):
        reg.write(REG_TCP_TEST_SELECT_SEQ, bytes([0x1]))
    else:
        reg.write(REG_TCP_TEST_SELECT_SEQ, bytes([0x0]))

def set_tcp_test_seq_pattern(pattern):
    pattern &= 0xFFFFFFFF
    bytes_num = pattern.to_bytes(4, byteorder='big')
    # print(bytes_num)
    reg.write(REG_TCP_TEST_SEQ_PATTERN, bytes_num)

def set_tcp_test_blk_size(size):
    if size > 0xFF_FFFF:
        print("Max size is 0xFFFFFF")
        size = 0xFFF_FFFF
    bytes_num = size.to_bytes(3, byteorder='big')
    # print(bytes_num)
    reg.write(REG_TCP_TEST_BLK_SIZE, bytes_num)

def set_tcp_test_ins_error():
    reg.write(REG_TCP_TEST_INS_ERROR, bytes([1]))

#################################################################
# register test
if TEST_REG:
    reg = rbcp.Rbcp()
    read_info()
    # shift_led()

#################################################################
# sysmon test
if TEST_SYSMON:
    sysmon = sysmon.sysmon()
    sysmon.print_status()
    print("")

if TEST_TCP_TX:
    num_of_data = 100_000_000
    blk_size = 1024
    tx_rate = 5 # unit: 100 Mbps

    set_tcp_test_data_gen(0)
    set_tcp_test_tx_rate(tx_rate)
    set_tcp_test_blk_size(blk_size)
    set_tcp_test_num_of_data(num_of_data)

    check_data = 1
    print_error = 1
    clear_buffer = 0

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
    set_tcp_test_data_gen(1)
    try:
        while (rxlength < num_of_data):
            recv_data = tcp_socket.recv(1460)
            data_len = len(recv_data)
            # check data
            if check_data:
                cycles = data_len%255 -1
                last_data = recv_data[0]+cycles
                if last_data > 0xFF:
                    last_data += 1
                if (last_data&0xFF != recv_data[-1]):
                    print(".", end = "")
                    error_cnt += 1
                    if print_error:
                        for i in range(data_len-1):
                            if(recv_data[i]+1 != recv_data[i+1]):
                                if (recv_data[i] == 0xff) & (recv_data[i+1] != 0x1):
                                    print("0x%x, 0x%x"%(recv_data[i], recv_data[i+1]))
            rxlength += data_len
            # print(rxlength)
        stop = time.time()
        run_time = stop - start
    except socket.timeout:
        stop = time.time()
        run_time = stop - start - timeout
        print("Timeout end")
    finally:
        tcp_socket.close()
        set_tcp_test_data_gen(0)

    print("\nData length: %d bytes"%rxlength)
    print("Duration: %f s"%run_time)
    print("Speed: %.2f Mbps"%(8*rxlength/run_time/1000_000))
    if check_data:
        print("Package error count: %d"%error_cnt)
