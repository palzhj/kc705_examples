#!/usr/bin/python
# This is sitcp_config.py file
# author: zhj@ihep.ac.cn
# 2024-02-22 created
import lib
from lib import rbcp

# Reg address
REG_SYN_INFO                = 0x0
REG_SYN_VER                 = 0x4
REG_FPGA_DNA                = 0x5
REG_TCP_MODE                = 0xd
REG_TCP_TEST_TX_RATE        = 0xe
REG_TCP_TEST_NUM_OF_DATA    = 0xf
REG_TCP_TEST_DATA_GEN       = 0x17
REG_TCP_TEST_WORD_LEN       = 0x18
REG_TCP_TEST_SELECT_SEQ     = 0x19
REG_TCP_TEST_SEQ_PATTERN    = 0x1a
REG_TCP_TEST_BLK_SIZE       = 0x1e
REG_TCP_TEST_INS_ERROR      = 0x21

REG_SITCP                   = 0xFFFFFF00

REG_SITCP_CONTROL           = 0xFFFFFF10
REG_SITCP_RETRANSMISSION_TIME = 0xFFFFFF2E

# REG_SITCP_CONTROL
CTRL_RESET          = 0x80
CTRL_WINDOW_SCALING = 0x10
CTRL_KEEP_ALIVE     = 0x04
CTRL_FAST_RETRAINS  = 0x02
CTRL_NAGLE          = 0x01

class reg(object):
    def __init__(self, device_ip="192.168.10.16", udp_port=4660):
        self._rbcp = rbcp.Rbcp(device_ip=device_ip, udp_port=udp_port)

    def read_info(self): # the date of compiling
        temp = self._rbcp.read(REG_SYN_INFO, 4)
        # print(temp)
        hour  = hex(temp[0]).lstrip("0x")
        day   = hex(temp[1]).lstrip("0x")
        month = hex(temp[2]).lstrip("0x")
        year  = "20" + hex(temp[3]).lstrip("0x")
        print("Compiling date: ", year, "-", month, "-", day, ",", hour, ":00")
        temp = self._rbcp.read(REG_SYN_VER, 1)
        print("Firmware version: ", temp[0])

    def read_fpga_dna(self): # FPGA DNA
        return self._rbcp.read(REG_FPGA_DNA, 8)

    def set_tcp_loopback_mode(self):
        self._rbcp.write(REG_TCP_MODE, bytes([0x1]))

    def set_tcp_test_mode(self):
        self._rbcp.write(REG_TCP_MODE, bytes([0x2]))

    def set_tcp_normal_mode(self):
        self._rbcp.write(REG_TCP_MODE, bytes([0x0]))

    def set_tcp_test_tx_rate(self, speed_in_100Mbps):
        speed_in_100Mbps &= 0xFF
        self._rbcp.write(REG_TCP_TEST_TX_RATE, bytes([speed_in_100Mbps]))

    def set_tcp_test_num_of_data(self, num):
        if num > 0xFFFF_FFFF_FFFF_FFFF:
            print("Max length is 0xFFFFFFFFFFFFFFFF")
            num = 0xFFFF_FFFF_FFFF_FFFF
        bytes_num = num.to_bytes(8, byteorder='big')
        # print(bytes_num)
        self._rbcp.write(REG_TCP_TEST_NUM_OF_DATA, bytes_num)

    def set_tcp_test_data_gen(self, enable):
        if(enable):
            self._rbcp.write(REG_TCP_TEST_DATA_GEN, bytes([0x1]))
        else:
            self._rbcp.write(REG_TCP_TEST_DATA_GEN, bytes([0x0]))

    def set_tcp_test_word_len(self, len=8):
        len = (len-1)&0x7
        self._rbcp.write(REG_TCP_TEST_WORD_LEN, bytes([0x1]))

    def set_tcp_test_select_seq(self, enable):
        if(enable):
            self._rbcp.write(REG_TCP_TEST_SELECT_SEQ, bytes([0x1]))
        else:
            self._rbcp.write(REG_TCP_TEST_SELECT_SEQ, bytes([0x0]))

    def set_tcp_test_seq_pattern(self, pattern):
        pattern &= 0xFFFFFFFF
        bytes_num = pattern.to_bytes(4, byteorder='big')
        # print(bytes_num)
        self._rbcp.write(REG_TCP_TEST_SEQ_PATTERN, bytes_num)

    def set_tcp_test_blk_size(self, size):
        if size > 0xFF_FFFF:
            print("Max size is 0xFFFFFF")
            size = 0xFFF_FFFF
        bytes_num = size.to_bytes(3, byteorder='big')
        # print(bytes_num)
        self._rbcp.write(REG_TCP_TEST_BLK_SIZE, bytes_num)

    def set_tcp_test_ins_error(self):
        self._rbcp.write(REG_TCP_TEST_INS_ERROR, bytes([1]))

    # sitcp config
    def read_sitcp_reg(self): # the date of compiling
        temp = self._rbcp.read(REG_SITCP, 0x42)
        for i in range(0x42):
            print("@addr0x%02x: 0x%02x"%(i,temp[i]))

    def set_sitcp_retransmission_time(self, msec):
        self._rbcp.write(REG_SITCP_RETRANSMISSION_TIME,  bytes([(msec >> 8)&0xFF]))
        self._rbcp.write(REG_SITCP_RETRANSMISSION_TIME+1,  bytes([msec&0xFF]))

    def set_sitcp_keep_alive(self, enable=True):
        temp = self._rbcp.read(REG_SITCP_CONTROL, 1)[0]
        if(enable):
            self._rbcp.write(REG_SITCP_CONTROL, bytes([temp|CTRL_KEEP_ALIVE]))
        else:
            self._rbcp.write(REG_SITCP_CONTROL, bytes([temp&~CTRL_KEEP_ALIVE]))

    def set_sitcp_fast_retrains(self, enable=True):
        temp = self._rbcp.read(REG_SITCP_CONTROL, 1)[0]
        if(enable):
            self._rbcp.write(REG_SITCP_CONTROL, bytes([temp|CTRL_FAST_RETRAINS]))
        else:
            self._rbcp.write(REG_SITCP_CONTROL, bytes([temp&~CTRL_FAST_RETRAINS]))

    def set_sitcp_nagle(self, enable=True):
        temp = self._rbcp.read(REG_SITCP_CONTROL, 1)[0]
        if(enable):
            self._rbcp.write(REG_SITCP_CONTROL, bytes([temp|CTRL_NAGLE]))
        else:
            self._rbcp.write(REG_SITCP_CONTROL, bytes([temp&~CTRL_NAGLE]))

    def set_sitcp_window_scaling(self, enable=True):
        # for sitcpxg only
        if(0x58 == self._rbcp.read(REG_SITCP+8, 1)[0]):
            temp = self._rbcp.read(REG_SITCP_CONTROL, 1)[0]
            if(enable):
                self._rbcp.write(REG_SITCP_CONTROL, bytes([temp|CTRL_WINDOW_SCALING]))
            else:
                self._rbcp.write(REG_SITCP_CONTROL, bytes([temp&~CTRL_WINDOW_SCALING]))