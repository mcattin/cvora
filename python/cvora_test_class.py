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

    print("Module LUN: %d"%(LUN))

    print("\n\n==================================================")
    print("Module methods:\n")

    fw_ver = cvora.get_firmware_version()
    print("VHDL version : %2.2f"%(fw_ver))

    status = cvora.get_status()
    in_pol = cvora.get_in_pol()
    cvora.set_in_pol('negative')
    cvora.set_in_pol('positive')
    mode = cvora.get_mode()
    chan_en = cvora.get_channel_enable()
    cvora.module_disable()
    cvora.channel_enable(0)
    chan_en = cvora.get_channel_enable()
    cvora.channel_disable(0)
    cvora.module_enable()
    chan_en = cvora.get_channel_enable()
    clk_freq = cvora.get_clk_freq()
    cvora.channel_select(3)
    chan_sel = cvora.get_channel_select()


    cvora.vv_close()


if __name__ == '__main__' :
    main()
