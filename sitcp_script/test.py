#!/usr/bin/python
# This is test.py file
# author: zhj@ihep.ac.cn
# 2019-06-18 created

from time import sleep
import sys
import os
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

# def shift_led():
#     reg.write(LED_ADDR,'\x80')
#     for i in range(8):
#         b = reg.read(LED_ADDR, 1)
#         b[0] = b[0] >> 1
#         if b[0] == 0:
#             b[0] = 0x80
#         reg.write(LED_ADDR , b)
#         sleep(0.5)

def read_info(): # the date of compiling
    temp = reg.read(0, 4)
    # print(temp)
    hour  = hex(temp[0]).lstrip("0x")
    day   = hex(temp[1]).lstrip("0x")
    month = hex(temp[2]).lstrip("0x")
    year  = "20" + hex(temp[3]).lstrip("0x")
    print("Compiling date: ", year, "-", month, "-", day, ",", hour, ":00")
    temp = reg.read(4, 1)
    print("Firmware version: ", temp[0])

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

