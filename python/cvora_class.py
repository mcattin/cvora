#!   /usr/bin/env	python
#    coding: utf8

# Copyright CERN, 2013
# Author: Matthieu Cattin <matthieu.cattin@cern.ch>
# Licence: GPL v2 or later.

import sys
import logging
from numpy import *
from pylab import *

from vv_pts import *

class CvoraException(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return ("[CVORA] %s" %(self.msg))

class Cvora(VME):

    csr_reg        = 0x00
    ram_wr_ptr_reg = 0x04
    mode_reg       = 0x08
    chan_en_reg    = 0x0C
    clk_freq_reg   = 0x10
    chan_sel_reg   = 0x14
    cvorb_reg      = 0x18

    ram_start    = 0x20
    ram_size     = 0x80000-0x20

    mode_enum = {'FP_OP16_SCI'        :0x1,
                 'FP_CU16_SCI'        :0x2,
                 'CNT32'              :0x3,
                 'RTM_PARALLEL'       :0x4,
                 'FP_OP32_SCI'        :0x5,
                 'FP_CU32_SCI'        :0x6,
                 'RTM_SCI'            :0x7,
                 'FP_OP16_CVORB'      :0x9,
                 'FP_CU16_CVORB'      :0xA,
                 'CNT2X16'            :0xB,
                 'RTM_PARALLEL_STROBE':0xC,
                 'FP_OP32_CVORB'      :0xD,
                 'FP_CU32_CVORB'      :0xE,
                 'RTM_CVORB'          :0xF}

    in_pol_enum = {'positive':1,
                   'negative':0}

    enable_enum = {'disabled':0,
                   'enabled':1}


    # Cvora class initialisation
    def __init__(self, lun, logger):
        super(Cvora, self).__init__(lun)
        self.logger = logger

    # Set a field without changing the rest of the register
    def set_reg_field(self, addr, mask, offset, value):
        reg = self.vv_read(addr)
        self.logger.debug("[set_reg_field] read :0x%x @ 0x%x", reg, addr)
        reg &= ~(mask << offset)
        reg |= (mask & value) << offset
        self.logger.debug("[set_reg_field] write:0x%x @ 0x%x", reg, addr)
        self.vv_write(addr, reg)

    # Get a field without the rest of the register
    def get_reg_field(self, addr, mask, offset):
        reg = self.vv_read(addr)
        self.logger.debug("[get_reg_field] read :0x%x @ 0x%x", reg, addr)
        reg = mask & (reg >> offset)
        self.logger.debug("[get_reg_field] field:0x%x @ 0x%x", reg, addr)
        return reg

    # Get a hex number slice
    def get_field(self, value, mask, offset):
        field = mask & (value >> offset)
        return field

    def get_key(self, dic, search_value):
        for key, value in dic.iteritems():
            if value == search_value:
                return key
        raise CvoraException("[get_key] Value not found in dictionary")

    # Converts two's complement hex to signed
    def hex2signed(self, value, nb_bit = 16):
        if(value & (0x1 << (nb_bit-1))):
            mask = 0
            for i in range(nb_bit):
                mask += 1 << i
            return -((~value & mask) + 1)
        else:
            return value

    # Converts digital value to volts
    def digital2volt(self, value, full_scale, nb_bit):
        return float(value) * float(full_scale)/2**nb_bit

    # Converts volts to digital value
    def volt2digital_twoscomp(self, value, full_scale, nb_bit):
        if(value > (2**nb_bit)/2 - 1):
            value = (2**nb_bit)/2 - 1
        if(value < -((2**nb_bit)/2)):
            value = -((2**nb_bit)/2)
        digital = (value) * 2**nb_bit/full_scale
        # print('volt2digital: %2.9f > %2.9f')%(value,digital)
        return int(digital)

    # Converts volts to digital value with half full range offset
    def volt2digital_offbin(self, value, full_scale, nb_bit):
        digital = (value + full_scale/2) * 2**nb_bit/full_scale
        if(digital > 2**nb_bit - 1):
            digital = 2**nb_bit - 1
        if(digital < 0):
            digital = 0
        # print('volt2digital: %2.9f > %2.9f')%(value,digital)
        return int(digital)

    # Converts volts to digital value
    def volt2digital_offbin_unipolar(self, value, full_scale, nb_bit):
        digital = value * 2**nb_bit/full_scale
        if(digital > 2**nb_bit - 1):
            digital = 2**nb_bit - 1
        if(digital < 0):
            digital = 0
        # print('volt2digital: %2.9f > %2.9f')%(value,digital)
        return int(digital)


    ################################################################################
    # Module methods
    ################################################################################

    # Reset board
    def reset(self):
        pass

    # Returns the firmware version in float
    def get_firmware_version(self):
        reg = self.vv_read(self.csr_reg)
        fw_ver = (reg >> 16) & 0xFFFF
        ver_maj = ((fw_ver >> 12) & 0xF) * 10 + ((fw_ver >> 8) & 0xF)
        ver_min = ((fw_ver >> 4) & 0xF) * 0.1 + (fw_ver & 0xF) * 0.01
        self.logger.info("[get_firmware_version] Firmware version=%2.2f", (ver_maj+ver_min))
        return ver_maj + ver_min

    # Get status
    def get_status(self):
        reg = self.vv_read(self.csr_reg)
        self.logger.info("[get_status] Status=0x%x"%(reg))
        return reg

    # Get input pulse polarity
    def get_in_pol(self):
        pol = self.get_reg_field(self.csr_reg, 0x1, 0)
        self.logger.info("[get_in_pol] Input polarity: %d"%(pol))
        return pol

    # Set input pulse polarity
    def set_in_pol(self, in_pol):
        if not(in_pol in self.in_pol_enum):
            raise CvoraException("[set_in_pol] Unknown input polarity.")
        self.set_reg_field(self.csr_reg, 0x1, 0, self.in_pol_enum[in_pol])
        pol = self.get_reg_field(self.csr_reg, 0x1, 0)
        self.logger.info("[set_in_pol] Input polarity: %d"%(pol))

    # Enable module
    def module_enable(self):
        self.set_reg_field(self.csr_reg, 0x1, 1, 1)
        self.logger.info("[module_enable] MODULE ENABLE")

    # Disable module
    def module_disable(self):
        self.set_reg_field(self.csr_reg, 0x1, 1, 0)
        self.logger.info("[module_disable] MODULE DISABLE")

    # get module status
    def get_module_status(self):
        status = self.get_reg_field(self.csr_reg, 0x1, 1)
        self.logger.info("[get_module_status] Module status: %d"%(status))
        return status

    # Enable interrupt
    def irq_enable(self):
        self.set_reg_field(self.csr_reg, 0x1, 2, 1)
        self.logger.info("[irq_enable] INTERRUPT ENABLE")

    # Disable interrupts
    def irq_disable(self):
        self.set_reg_field(self.csr_reg, 0x1, 2, 0)
        self.logger.info("[irq_disable] INTERRUPT DISABLE")

    # Wait interrupt (interrupt is generated by a stop pulse)
    def irq_wait(self):
        self.vv_irqwait()

    # Get interrupt vector
    def get_irq_vector(self):
        irq_vec = self.get_reg_field(self.csr_reg, 0xFF, 8)
        self.logger.info("[get_irq_vector] Interrupt vector: 0x%04x"%(irq_vec))
        return irq_vec

    # Set interrupt vector
    def set_irq_vector(self, vec):
        self.set_reg_field(self.csr_reg, 0xFF, 8, vec)
        self.logger.info("[set_irq_vector] Interrupt vector: 0x%04x"%(vec))

    # Software start
    def sw_start(self):
        self.set_reg_field(self.csr_reg, 0x1, 3, 1)
        self.logger.info("[sw_start] SOFTWARE START")

    # Software stop
    def sw_stop(self):
        self.set_reg_field(self.csr_reg, 0x1, 4, 1)
        self.logger.info("[sw_stop] SOFTWARE STOP")

    # Software reset
    def sw_reset(self):
        self.set_reg_field(self.csr_reg, 0x1, 5, 1)
        self.logger.info("[sw_reset] SOFTWARE RESET")

    # Get acquisition status
    def is_acq(self):
        acq_stat = self.get_reg_field(self.csr_reg, 0x1, 5)
        self.logger.info("[is_acq] Acquisition in progress: %d"%(acq_stat))
        if (acq_stat):
            return True
        else:
            return False

    # Get up/down 32-bit counter status
    def is_cnt32_overflow(self):
        cnt_ovf = self.get_reg_field(self.csr_reg, 0x1, 6)
        self.logger.info("[is_cnt32_overflow] 32-bit up/down counter overflow: %d"%(cnt_ovf))
        if (cnt_ovf):
            return True
        else:
            return False

    # Get ram write pointer status
    def is_ram_overflow(self):
        ram_ovf = self.get_reg_field(self.csr_reg, 0x1, 7)
        self.logger.info("[is_ram_overflow] RAM write pointer overflow: %d"%(ram_ovf))
        if (ram_ovf):
            return True
        else:
            return False

    # Set mode
    def set_mode(self, mode):
        if not(mode in self.mode_enum):
            raise CvoraException("[set_mode] Unknown mode.")
        self.set_reg_field(self.mode_reg, 0xF, 0, self.mode_enum[mode])
        self.logger.info("[set_mode] Mode: %s"%(mode))

    # Get mode
    def get_mode(self):
        mode_v = self.get_reg_field(self.mode_reg, 0xF, 0)
        for k, v in self.mode_enum.items():
            #print "key: %s, val: 0x%04x, mode_reg: 0x%04x"%(k,v,mode_v)
            if v == mode_v:
                mode = k
                self.logger.info("[get_mode] Mode: %s"%(mode))
                return mode

    # Get enable channels register
    def get_channel_enable(self):
        chan_en = self.vv_read(self.chan_en_reg)
        self.logger.info("[channel_enable] Channel enable register: 0x%08x"%(chan_en))
        return chan_en

    # Enable channel
    def channel_enable(self, ch):
        if(self.get_module_status() == 1):
            raise CvoraException("[channel_enable] Module MUST be disabled to enable a channel!")
        self.set_reg_field(self.chan_en_reg, 0x1, ch, 1)

    # Disable channel
    def channel_disable(self, ch):
        if(self.get_module_status() == 1):
            raise CvoraException("[channel_disable] Module MUST be disabled to disable a channel!")
        self.set_reg_field(self.chan_en_reg, 0x1, ch, 0)

    # Disable all channels
    def channel_disable_all(self):
        self.vv_write(self.chan_en_reg, 0)

    # Check if channel is enabled
    def is_channel_enabled(self, ch):
        ch_en = self.get_channel_enable()
        if ((ch_en >> ch) & 0x1):
            return True
        else:
            return False

    # Get clock frequency
    def get_clk_freq(self):
        clk_freq = self.vv_read(self.clk_freq_reg)
        self.logger.info("[get_clk_freq] Clock frequency: %d Hz"%(clk_freq))
        return clk_freq

    # Select channel to output on DAC
    def channel_select(self, ch):
        self.set_reg_field(self.chan_sel_reg, 0x1F, 0, ch)
        self.logger.info("[channel_select] Channel %d output to DAC"%(ch))

    # Get channel select register
    def get_channel_select(self):
        chan_sel = self.get_reg_field(self.chan_sel_reg, 0x1F, 0)
        self.logger.info("[get_channel_select] Channel %d output to DAC"%(chan_sel))
        return chan_sel

    # Set CVORB pulse width threshold
    def set_cvorb_threshold(self, thres):
        self.set_reg_field(self.cvorb_reg, 0xFF, 0, thres)
        self.logger.info("[set_cvorb_threshold] CVORB pulse width threshold: %d"%(thres))

    # Get CVORB pulse width threshold
    def get_cvorb_threshold(self):
        thres = self.get_reg_field(self.cvorb_reg, 0xFF, 0)
        self.logger.info("[get_cvorb_threshold] CVORB pulse width threshold: %d"%(thres))
        return thres

    # Get measured CVORB pulse width
    def get_cvorb_pulse_width(self):
        width_1 = self.get_reg_field(self.cvorb_reg, 0xFF, 24)
        width_2 = self.get_reg_field(self.cvorb_reg, 0xFF, 16)
        self.logger.info("[get_cvorb_pulse_width] Measured CVORB pulse width 1: 0x%02x (%d)"%(width_1, width_1))
        self.logger.info("[get_cvorb_pulse_width] Measured CVORB pulse width 2: 0x%02x (%d)"%(width_2, width_2))
        return width_1, width_2

    # Read RAM
    def ram_read(self):
        ram_wr_ptr = self.vv_read(self.ram_wr_ptr_reg)
        self.logger.info("[ram_read] RAM write pointer: 0x%08x"%(ram_wr_ptr))
        if (ram_wr_ptr == 0x20):
            raise CvoraException("[ram_read] Nothing to read, the RAM is empty!")
        ram_data = []
        for ram_addr in range(0x20, ram_wr_ptr, 0x4):
            ram_data.append(self.vv_read(ram_addr))
            #self.logger.info("[ram_read] RAM data: 0x%08x @ 0x%05x"%(ram_data[-1],ram_addr))
        self.logger.info("[ram_read] RAM data length: %d"%(len(ram_data)))
        return ram_data

    # Read channel data
    def channel_read(self, ch):
        if not(self.is_channel_enabled(ch)):
            raise CvoraException("[channel_read] Requested channel (%d) is not enabled!"%(ch))
        ch_en = self.get_channel_enable()
        data_32 = self.ram_read()
        data = []
        for d in data_32:
            data.append(d & 0xFFFF)
            data.append((d >> 16) & 0xFFFF)
        ch_data = []
        pairs = 0
        ch_index = 0
        for ch_pair in range(0, 32, 2):
            # At least one of the channel pair is enabled
            if ((ch_en >> ch_pair) & 0x3):
                if (ch_pair == ch):
                    ch_index = 2*pairs
                elif (ch_pair+1 == ch):
                    ch_index = (2*pairs)+1
                pairs += 1
        self.logger.info("[channel_read] pairs   : %d"%(pairs))
        self.logger.info("[channel_read] ch_index: %d"%(ch_index))
        ch_data = data[ch_index::(2*pairs)]
        ch_data = [i-0x8000 for i in ch_data]
        return ch_data

    ################################################################################
    # Helper methods
    ################################################################################

    def show_wfm(self, wfm_data, wfm_label):
        ylimit = max(wfm_data)
        sample = arange(len(wfm_data))
        clf()
        plot(sample, wfm_data, 'b', label=wfm_label)
        ylim(-ylimit-(ylimit/10.0), ylimit+(ylimit/10.0))
        xlim(0, len(sample))
        grid(which='both')
        legend()
        draw()
        show()

    def get_data_16(self, data_32, word, offset=0x8000):
        data = []
        for d in data_32:
            data.append(d & 0xFFFF)
            data.append((d >> 16) & 0xFFFF)
        data = [i-offset for i in data]
        if word == 'LOW':
            return data[0::2]
        elif word == 'HIGH':
            return data[1::2]
        else:
            raise CvoraException("[get_data_16] Unknown data word: %s"%(word))

    def get_lun(self):
        ask = ""
        while (not(ask.isdigit())) :
            print "-------------------------------------------------------------"
            ask = raw_input("LUN? : ")
            print " "
        lun = int(ask)
        print("Selected LUN: %d"%(lun))
        return lun

