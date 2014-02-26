#!   /usr/bin/env python
#    coding: utf8

# Copyright CERN, 2013
# Author: Matthieu Cattin <matthieu.cattin@cern.ch>
# Licence: GPL v2 or later.
# Website: http://www.ohwr.org

# Import system modules
import sys
import time
import os
import logging

# Import common modules
from vv_pts import *

# Import specific modules
from cvora_class import *
from numpy import *
from pylab import *

sys.path.append('.')


def main (default_directory='.'):

    # Constants declaration
    LUN = 0

    # create logger
    logger = logging.getLogger('CVORA logger')
    #logger.setLevel(logging.DEBUG)
    logger.setLevel(logging.INFO)

    # create console handler
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.DEBUG)
    ch_formatter = logging.Formatter('[%(levelname)s] %(message)s')
    ch.setFormatter(ch_formatter)
    logger.addHandler(ch)

    # Create cvorb object
    cvora = Cvora(LUN, logger)
    cvora.vv_open()

    print("\n\n==================================================")
    print("CVORA gateware test:\n")

    fw_ver = cvora.get_firmware_version()
    print("VHDL version : %2.2f"%(fw_ver))

    print("Module LUN: %d"%(LUN))

    print("\n--------------------------------------------------")
    raw_input("Connect CTRV 1kHz outptut to CLOCK.\nConnect CTRV outputs to START, STOP, RESET, BUP and BDOWN.\nConnect a scope to front panel analog outputs 1 and 2.\nPress [ENTER]")


    mode = 'CNT2X16'

    ##################################################
    print("\n--------------------------------------------------")
    print("Testing %s mode"%(mode))
    cvora.module_disable()
    cvora.channel_disable_all()
    cvora.set_mode(mode)
    cvora.set_irq_vector(0x86+LUN)
    cvora.module_enable()
    cvora.irq_enable()
    #cvora.sw_reset()
    #cvora.sw_start()
    time.sleep(2)
    #cvora.sw_stop()
    cvora.irq_wait()
    print("Interrupt occured.")
    data_32 = cvora.ram_read()

    data = []
    for d in data_32:
        data.append(d & 0xFFFF)
        data.append((d >> 16) & 0xFFFF)

    ylimit = max(data)
    data_cnt1 = data[0::2]
    data_cnt2 = data[1::2]
    sample = arange(len(data)/2)
    clf()
    plot(sample, data_cnt1, 'b', label='Counter 1')
    plot(sample, data_cnt2, 'r', label='Counter 2')
    ylim(-(ylimit/10.0), ylimit+(ylimit/10.0))
    xlim(0, len(sample))
    grid(which='both')
    legend()
    draw()
    show()

    cvora.vv_close()


if __name__ == '__main__' :
    main()
