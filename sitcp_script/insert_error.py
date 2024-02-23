#!/usr/bin/python
# This is test.py file
# author: zhj@ihep.ac.cn
# 2024-02-23 created

current_path = os.path.realpath(__file__)
directory_path = os.path.dirname(current_path)
sys.path.insert(0, directory_path+"/lib")
import reg

reg = reg.reg()
reg.set_tcp_test_ins_error()
