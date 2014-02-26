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
    raw_input("Connect cable to CVORB .\nConnect CTRV 1kHz outptut to CLOCK.\nConnect a scope to front panel analog outputs 1 and 2.\nPress [ENTER]")


    ##################################################
    mode = 'RTM_CVORB'

    for ch in range(32):
        print("\n--------------------------------------------------")
        print("Testing %s mode: RTM channel %d"%(mode, ch+1))
        raw_input("Plug cable to RTM channel %d.\nPress [ENTER]"%(ch+1))
        cvora.module_disable()
        cvora.channel_disable_all()
        cvora.set_mode(mode)
        cvora.set_irq_vector(0x86+LUN)
        cvora.channel_enable(ch)
        cvora.channel_select(ch)
        cvora.module_enable()
        # cvora.irq_enable()
        cvora.sw_reset()
        cvora.sw_start()
        time.sleep(1)
        cvora.sw_stop()
        # cvora.irq_wait()
        # print("Interrupt occured.")
        data_16 = cvora.channel_read(ch)
        label = 'Channel %d'%ch
        cvora.show_wfm(data_16, label)


    cvora.vv_close()


if __name__ == '__main__' :
    main()
