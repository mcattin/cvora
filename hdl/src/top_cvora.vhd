--------------------------------------------------------------------------------
-- CERN (BE-CO-HT)
-- CVORA top level entity
--------------------------------------------------------------------------------
--
-- unit name: top_cvora
--
-- author: Philippe Nouchi
--         Matthieu Cattin (matthieu.cattin@cern.ch)
--
-- date: 25-04-2006
--
-- description: Top level module for the CVORA board.
--
-- dependencies:
--
--------------------------------------------------------------------------------
-- GNU LESSER GENERAL PUBLIC LICENSE
--------------------------------------------------------------------------------
-- This source file is free software; you can redistribute it and/or modify it
-- under the terms of the GNU Lesser General Public License as published by the
-- Free Software Foundation; either version 2.1 of the License, or (at your
-- option) any later version. This source is distributed in the hope that it
-- will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
-- of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
-- See the GNU Lesser General Public License for more details. You should have
-- received a copy of the GNU Lesser General Public License along with this
-- source; if not, download it from http://www.gnu.org/licenses/lgpl-2.1.html
--------------------------------------------------------------------------------
-- changes history: see git log
--------------------------------------------------------------------------------
-- last changes:
--   23.10.2013 mcattin  New major version 2.0
--                       Code clean-up, re-structuring, external inputs sync,
--                       unifying data sampling for all modes.
--                       Add 2x 16-bit up counter mode.
--                       Add CVORB protocol decoding.
--                       Add parallel mode with strobe on 32nd bit.
----------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.bus_int_pkg.all;
use work.message_pkg.all;
use work.cvora_pkg.all;
use work.vme_pkg.all;
use work.gencores_pkg.all;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
library UNISIM;
use UNISIM.VComponents.all;

entity top_cvora is
  generic(
    g_GATEWARE_VER      : std_logic_vector(15 downto 0) := X"0200";
    g_LED_PULSE_WIDTH   : natural                       := 8000000;  -- in sys_clk cycles = 320 ms
    g_DEFAULT_INPUT_POL : std_logic                     := '1';      -- 0 = negative logic, 1 = positive logic
    g_DEFAULT_IRQ_VECT  : std_logic_vector(7 downto 0)  := x"86"     -- 134 decimal
    );

  port (
    sys_rst_n_a_i    : in  std_logic;                       -- Active low asynchronous power-on reset (from TLC7733)
    sys_clk_i        : in  std_logic;                       -- 40MHz system clock
    fp_bup_i         : in  std_logic;                       -- "BUP" front panel input (pulled-up)
    fp_bdown_i       : in  std_logic;                       -- "BDOWN" front panel input (pulled-up)
    fp_clock_i       : in  std_logic;                       -- "CLOCK" front panel input (pulled-up)
    fp_reset_i       : in  std_logic;                       -- "RESET" front panel input (pulled-up)
    fp_strobe_i      : in  std_logic;                       -- "STROBE" front panel input (pulled-up)
    fp_start_i       : in  std_logic;                       -- "START" front panel input (pulled-up)
    fp_stop_i        : in  std_logic;                       -- "STOP" front panel input (pulled-up)
    rtm_data_i       : in  std_logic_vector (31 downto 0);  -- Data inputs from VME P2 connector
    mode_select_sw_i : in  std_logic_vector(2 downto 0);    -- Jumper for mode selection
    rs232_rx_i       : in  std_logic;                       -- RS232 Rx interface for LCD ascii display
    rs232_tx_o       : out std_logic;                       -- RS232 Tx interface for LCD ascii display
    test_o           : out std_logic_vector (3 downto 0);   -- Test points
    fp_led_o         : out std_logic_vector (7 downto 0);   -- Front panel LEDs
    dac1_data_o      : out std_logic_vector (15 downto 0);  -- Data to DAC, analog output 1 (AD669)
    dac2_data_o      : out std_logic_vector (15 downto 0);  -- Data to DAC, analog output 2 (AD669)
    dac1_load_o      : out std_logic;                       -- Load data to DAC, analog output 1 (AD669)
    dac2_load_o      : out std_logic;                       -- Load data to DAC, analog output 1 (AD669)
    fp_data_optic1_i : in  std_logic;                       -- Front panel optical input 1
    fp_data_optic2_i : in  std_logic;                       -- Front panel optical input 2
    fp_data_cu1_i    : in  std_logic;                       -- Front panel copper input 1
    fp_data_cu2_i    : in  std_logic;                       -- Front panel optical input 2
--  StrobeOut        : out std_logic;                       -- Not connected on PCB!
--  SData1Led        : out std_logic;                       -- Not connected on PCB!
--  SData2Led        : out std_logic;                       -- Not connected on PCB!

    -- External RAM IDT71V35761 (Same as CTRV) (66 pins)
    ram_add_o      : out   std_logic_vector(16 downto 0);  -- Address Inputs
    ram_data_b     : inout std_logic_vector(31 downto 0);  -- Synchronous data input/output pins
    ram_data_par_o : out   std_logic_vector(3 downto 0);   -- Data parity (not used)
--  RamZZ      : out   std_logic;                      -- Sleep Mode
    ram_oe_n_o     : out   std_logic;                      -- Asynchronous output enable
    ram_lbo_n_o    : out   std_logic;                      -- Linear Burst Order
    ram_gw_n_o     : out   std_logic;                      -- Synchronous global write enable
    ram_ce_n_o     : out   std_logic;                      -- Synchronous chip enable
--  RamCS0     : out std_logic;                        -- Synchronous active HIGH chip select
--  RamCS1N    : out std_logic;                        -- Synchronous active LOW chip select
    ram_wr_n_o     : out   std_logic;                      -- Synchronous byte write enable
    ram_bw_n_o     : out   std_logic_vector(4 downto 1);   -- Individual Write Enables
    ram_adv_n_o    : out   std_logic;                      -- Burst Address Advance
    ram_adsp_n_o   : out   std_logic;                      -- Address Status (Processor)
    ram_adsc_n_o   : out   std_logic;                      -- Address Status (Cache Controller)
    ram_clk_o      : out   std_logic;                      -- RAM clock

    -- VME (79 pins)
    module_addr_i : in std_logic_vector (5 downto 0);  -- VME board address from switch (bits[23:18]), address range = 0x80000 per card

    vme_rst_i       : in    std_logic;
    vme_addr_i      : in    std_logic_vector (23 downto 1);
    vme_am_i        : in    std_logic_vector (5 downto 0);
    vme_as_n_i      : in    std_logic;
    vme_buf_oe_n_o  : out   std_logic;
    vme_data_b      : inout std_logic_vector (31 downto 0);
    vme_dir_o       : out   std_logic;
    vme_ds0_n_i     : in    std_logic;
    vme_ds1_n_i     : in    std_logic;
    vme_dtack_n_o   : out   std_logic;
    vme_iack_n_i    : in    std_logic;
    vme_iackin_n_i  : in    std_logic;
    vme_iackout_n_o : out   std_logic;
    vme_lword_n_i   : in    std_logic;
    vme_write_n_i   : in    std_logic;
    vme_intreq2_n_o : out   std_logic;  -- only Interrupt level 2 is used in DPRAM
    vme_iack_mux    : out   std_logic   -- Interrupt daisy chain mux (connects iackin to iackout when interrupts are disabled)
    );
end top_cvora;


architecture Behavioral of top_cvora is


  ------------------------------------------------------------------------------
  -- Clock, reset and local pps
  signal sys_clk   : std_logic;
  signal sys_rst_n : std_logic;
  signal pps_cnt   : unsigned(26 downto 0);
  signal pps_p     : std_logic;

  ------------------------------------------------------------------------------
  -- VME
  signal module_am                 : std_logic_vector(4 downto 0);
  signal module_addr               : std_logic_vector(4 downto 0);
  signal IRQStatusIDReg            : std_logic_vector(31 downto 0);
  signal irq_vector                : std_logic_vector(7 downto 0) := g_DEFAULT_IRQ_VECT;
  signal IntProcessed, UserIntReqN : std_logic;
  signal vme_intreq_n              : std_logic_vector (7 downto 1);
  signal irq_en                    : std_logic                    := '0';
  signal OpFinishedOut             : std_logic;

  ------------------------------------------------------------------------------
  -- Bus Interface Signals
  signal intRead     : std_logic;       -- Interface Read Signal
  signal intWrite    : std_logic;       -- Interface Write Signal
  signal dataFromInt : IntDataType;     -- Data From interface
  signal intAdd      : IntAddrOutType;  -- Address From interface
  signal opDone      : std_logic;       -- Operation Done, Read or Write Finished
  signal dataToInt   : IntDataType;     -- Data going from Control to the Interface
  -- Registers  Signals
  signal contToRegs  : contToRegsType;  -- Data going from Control to the Registers
                                        -- This consists of Data + Write Enable Siganal
  signal regsToCont  : RegsToContType;  -- Data Array From the Registers to the Control
  -- Memory Signals
  signal contToMem   : ContToMemType;   -- Data going from Control to the Registers
                                        -- This consists of Data + Enable + Read + Write
  signal memToCont   : MemToContType;   -- Data Array  From the Registers to the Control
                                        -- Data + Done

  ------------------------------------------------------------------------------
  -- Registers
  signal csr_reg_wren            : std_logic;
  signal mode_reg_wren           : std_logic;
  signal channel_en_reg_wren     : std_logic;
  signal clk_freq_reg_wren       : std_logic;
  signal channel_select_reg_wren : std_logic;
  signal channel_select_reg      : std_logic_vector(4 downto 0);
  signal channel_select          : integer;
  signal cvorb_reg_wren          : std_logic;
  signal mode                    : std_logic_vector(3 downto 0);
  signal channel_en              : std_logic_vector(31 downto 0);
  signal cvorb_pulse_width_thres : std_logic_vector(7 downto 0);
  signal cvorb_meas_pulse_width1 : std_logic_vector(7 downto 0);
  signal cvorb_meas_pulse_width2 : std_logic_vector(7 downto 0);
  signal cnt_overflow            : std_logic;

  ------------------------------------------------------------------------------
  -- parallel mode
  signal parallel_data : std_logic_vector(31 downto 0);
  signal rtm_data_d    : std_logic_vector(31 downto 0);
  signal rtm_data_d2   : std_logic_vector(31 downto 0);

  ------------------------------------------------------------------------------
  -- 32-bit up/down counter (b-train)
  signal ud_cnt_value    : std_logic_vector(31 downto 0);
  signal ud_cnt_en       : std_logic;
  signal ud_cnt_overflow : std_logic;
  signal ud_cnt_valid    : std_logic;

  ------------------------------------------------------------------------------
  -- 16-bit up counters
  signal up_cnt_en        : std_logic;
  signal up_cnt1_value    : std_logic_vector(15 downto 0);
  signal up_cnt1_overflow : std_logic;
  signal up_cnt1_valid    : std_logic;
  signal up_cnt2_value    : std_logic_vector(15 downto 0);
  signal up_cnt2_overflow : std_logic;
  signal up_cnt2_valid    : std_logic;

  ------------------------------------------------------------------------------
  -- Front panel serial decoders
  signal fp_sci_en           : std_logic;
  signal fp_data1_sci_serial : std_logic;
  signal fp_data1_sci        : std_logic_vector(15 downto 0);
  signal fp_data1_sci_valid  : std_logic;
  signal fp_data2_sci_serial : std_logic;
  signal fp_data2_sci        : std_logic_vector(15 downto 0);
  signal fp_data2_sci_valid  : std_logic;

  signal fp_cvorb_en           : std_logic;
  signal fp_data1_cvorb_serial : std_logic;
  signal fp_data1_cvorb        : std_logic_vector(15 downto 0);
  signal fp_data1_cvorb_valid  : std_logic;
  signal fp_data2_cvorb_serial : std_logic;
  signal fp_data2_cvorb        : std_logic_vector(15 downto 0);
  signal fp_data2_cvorb_valid  : std_logic;
  signal cvorb_zero_test       : std_logic;
  signal cvorb_one_test        : std_logic;
  signal cvorb_strobe_test     : std_logic;

  ------------------------------------------------------------------------------
  -- RTM serial decoders
  signal rtm_data_clk_p     : std_logic;
  signal rtm_data           : rtm_data_array_t;
  signal rtm_data_valid     : std_logic_vector(31 downto 0);
  signal rtm_reset_ram_addr : std_logic;
  signal rtm_ram_data       : std_logic_vector(31 downto 0);
  signal rtm_ram_data_valid : std_logic;
  signal rtm_ram_addr       : std_logic_vector(RAM_ADDR_LENGTH-1 downto 0);
  signal rtm_ram_overflow   : std_logic;

  ------------------------------------------------------------------------------
  -- CVORB protocol pulse width measurement
  signal rtm_cvorb_meas_pulse_width : cvorb_pulse_width_array_t;
  signal fp_cvorb_meas_pulse_width1 : std_logic_vector(7 downto 0);
  signal fp_cvorb_meas_pulse_width2 : std_logic_vector(7 downto 0);

  ------------------------------------------------------------------------------
  -- RS232 LCD display
  signal message_to_send : message_array;

  ------------------------------------------------------------------------------
  -- LEDs
  signal fp_led : std_logic_vector(7 downto 0);

  ------------------------------------------------------------------------------
  -- Inter-acquisition time measurement
  signal inter_acq_time_cnt : unsigned(15 downto 0);
  signal inter_acq_time     : std_logic_vector(15 downto 0);
  signal inter_acq_time_bcd : BCD_vector_TYPE(4 downto 0);

  ------------------------------------------------------------------------------
  -- Front panel input sync and edge detect
  signal fp_clock    : std_logic;
  signal fp_start    : std_logic;
  signal fp_stop     : std_logic;
  signal fp_bup      : std_logic;
  signal fp_bdown    : std_logic;
  signal fp_strobe   : std_logic;
  signal fp_reset    : std_logic;
  signal fp_clock_p  : std_logic;
  signal fp_start_p  : std_logic;
  signal fp_stop_p   : std_logic;
  signal fp_bup_p    : std_logic;
  signal fp_bdown_p  : std_logic;
  signal fp_strobe_p : std_logic;
  signal fp_reset_p  : std_logic;
  signal sw_start_p  : std_logic;
  signal sw_stop_p   : std_logic;
  signal sw_reset_p  : std_logic;
  signal start_p     : std_logic;
  signal stop_p      : std_logic;
  signal reset_p     : std_logic;
  signal start_p_d   : std_logic_vector(3 downto 0);
  signal clock_p_d   : std_logic_vector(2 downto 0);
  signal strobe_p_d  : std_logic_vector(4 downto 0);

  ------------------------------------------------------------------------------
  -- DAC
  signal dac1_load_p : std_logic;
  signal dac2_load_p : std_logic;

  ------------------------------------------------------------------------------
  -- RAM manager
  signal ram_rd            : std_logic;
  signal ram_rd_addr       : std_logic_vector(RAM_ADDR_LENGTH-1 downto 0);
  signal ram_rd_data       : std_logic_vector(31 downto 0);
  signal ram_rd_data_valid : std_logic;

  signal ram_wr_data     : std_logic_vector(31 downto 0);
  signal ram_wr          : std_logic;
  signal ram_wr_done     : std_logic;
  signal ram_wr_addr_cnt : unsigned(RAM_ADDR_LENGTH-1 downto 0);
  signal ram_wr_addr     : std_logic_vector(RAM_ADDR_LENGTH-1 downto 0);
  signal ram_wr_overflow : std_logic;

  ------------------------------------------------------------------------------
  -- Input clock frequency counter
  signal clock_freq         : std_logic_vector(31 downto 0);
  signal clock_freq_t       : std_logic_vector(31 downto 0);
  signal clock_freq_valid   : std_logic;
  signal clock_freq_valid_d : std_logic;
  signal clock_freq_valid_p : std_logic;
  signal clock_freq_wd      : std_logic;
  signal clock_freq_bcd     : BCD_vector_TYPE(7 downto 0);

  ------------------------------------------------------------------------------
  -- Signal for data acquisition
  signal data_acq_en    : std_logic;
  signal input_polarity : std_logic;
  signal module_en      : std_logic;


begin


  ------------------------------------------------------------------------------
  -- System clock and reset
  ------------------------------------------------------------------------------
  cmp_sys_clk_buf : IBUFG
    port map(
      I => sys_clk_i,
      O => sys_clk);

  sys_rst_n <= sys_rst_n_a_i;


  ------------------------------------------------------------------------------
  -- Local PPS generation
  ------------------------------------------------------------------------------
  process(sys_clk)
  begin
    if rising_edge(sys_clk) then
      if pps_cnt = to_unsigned(39999999, pps_cnt'length) then
        pps_cnt <= (others => '0');
        pps_p   <= '1';
      else
        pps_cnt <= pps_cnt + 1;
        pps_p   <= '0';
      end if;
    end if;
  end process;


  ------------------------------------------------------------------------------
  -- Front panel inputs
  ------------------------------------------------------------------------------

  -- Front panel input polarity
  --   0 -> negative logic
  --   1 -> positive logic (default)
  fp_clock  <= fp_clock_i  when input_polarity = '1' else not fp_clock_i;
  fp_start  <= fp_start_i  when input_polarity = '1' else not fp_start_i;
  fp_stop   <= fp_stop_i   when input_polarity = '1' else not fp_stop_i;
  fp_bup    <= fp_bup_i    when input_polarity = '1' else not fp_bup_i;
  fp_bdown  <= fp_bdown_i  when input_polarity = '1' else not fp_bdown_i;
  fp_strobe <= fp_strobe_i when input_polarity = '1' else not fp_strobe_i;
  fp_reset  <= fp_reset_i  when input_polarity = '1' else not fp_reset_i;

  -- Detects rising edge on front panel inputs
  cmp_clock_sync : gc_sync_ffs
    port map(
      clk_i    => sys_clk,
      rst_n_i  => sys_rst_n,
      data_i   => fp_clock,
      synced_o => open,
      npulse_o => open,
      ppulse_o => fp_clock_p);

  cmp_start_sync : gc_sync_ffs
    port map(
      clk_i    => sys_clk,
      rst_n_i  => sys_rst_n,
      data_i   => fp_start,
      synced_o => open,
      npulse_o => open,
      ppulse_o => fp_start_p);

  cmp_stop_sync : gc_sync_ffs
    port map(
      clk_i    => sys_clk,
      rst_n_i  => sys_rst_n,
      data_i   => fp_stop,
      synced_o => open,
      npulse_o => open,
      ppulse_o => fp_stop_p);

  cmp_bup_sync : gc_sync_ffs
    port map(
      clk_i    => sys_clk,
      rst_n_i  => sys_rst_n,
      data_i   => fp_bup,
      synced_o => open,
      npulse_o => open,
      ppulse_o => fp_bup_p);

  cmp_bdown_sync : gc_sync_ffs
    port map(
      clk_i    => sys_clk,
      rst_n_i  => sys_rst_n,
      data_i   => fp_bdown,
      synced_o => open,
      npulse_o => open,
      ppulse_o => fp_bdown_p);

  cmp_strobe_sync : gc_sync_ffs
    port map(
      clk_i    => sys_clk,
      rst_n_i  => sys_rst_n,
      data_i   => fp_strobe,
      synced_o => open,
      npulse_o => open,
      ppulse_o => fp_strobe_p);

  cmp_reset_sync : gc_sync_ffs
    port map(
      clk_i    => sys_clk,
      rst_n_i  => sys_rst_n,
      data_i   => fp_reset,
      synced_o => open,
      npulse_o => open,
      ppulse_o => fp_reset_p);

  -- Combining front panel input pulse with software pulses
  start_p <= fp_start_p or sw_start_p;
  stop_p  <= fp_stop_p or sw_stop_p;
  reset_p <= fp_reset_p or sw_reset_p;

  -- Pulses delay
  p_start_delay : process (sys_clk, sys_rst_n)
  begin
    if sys_rst_n = '0' then
      start_p_d <= (others => '0');
    elsif rising_edge(sys_clk) then
      start_p_d <= start_p_d(start_p_d'left-1 downto 0) & start_p;
    end if;
  end process p_start_delay;

  p_strobe_delay : process (sys_clk, sys_rst_n)
  begin
    if sys_rst_n = '0' then
      strobe_p_d <= (others => '0');
    elsif rising_edge(sys_clk) then
      strobe_p_d <= strobe_p_d(strobe_p_d'left-1 downto 0) & fp_strobe_p;
    end if;
  end process p_strobe_delay;

  p_clock_delay : process (sys_clk, sys_rst_n)
  begin
    if sys_rst_n = '0' then
      clock_p_d <= (others => '0');
    elsif rising_edge(sys_clk) then
      clock_p_d <= clock_p_d(clock_p_d'left-1 downto 0) & fp_clock_p;
    end if;
  end process p_clock_delay;


  ------------------------------------------------------------------------------
  -- VME interface
  ------------------------------------------------------------------------------
  cmp_vme_interface : Vme_intfce
    generic map(
      AddrWidth        => 24,
      BaseAddrWidth    => 5,
      DataWidth        => 32,
      DirSamePolarity  => '0',
      UnalignDataWidth => 8,
      InterruptEn      => '1')
    port map(
      clk              => sys_clk,
      ResetNA          => sys_rst_n,
      VmeAddrA         => vme_addr_i,
      VmeAsNA          => vme_as_n_i ,
      VmeDs1NA         => vme_ds1_n_i,
      VmeDs0NA         => vme_ds0_n_i,
      VmeData          => vme_data_b,
      VmeDataUnAlign   => open,
      VmeDir           => vme_dir_o,
      VmeDirFloat      => open,
      VmeBufOeN        => vme_buf_oe_n_o,
      VmeWriteNA       => vme_write_n_i,
      VmeLwordNA       => vme_lword_n_i,
      VmeIackNA        => vme_iack_n_i,
      IackOutNA        => vme_iackout_n_o,
      IackInNA         => vme_iackin_n_i,
      VmeIntReqN       => vme_intreq_n,
      VmeDtackN        => vme_dtack_n_o,
      ModuleAddr       => module_addr,
      AddrMem          => IntAdd,
      VmeAmA           => module_am,
      ReadMem          => IntRead,
      WriteMem         => IntWrite,
      DataFromMemValid => opDone,
      DataFromMem      => dataToInt,
      DataToMem        => dataFromInt,
      IntProcessed     => IntProcessed,
      UserIntReqN      => UserIntReqN,
      UserBlocks       => '0',
      OpFinishedOut    => OpFinishedOut,
      IRQLevelReg      => "010",        -- Only level 2 is connected on the board
      IRQStatusIDReg   => IRQStatusIDReg,
      VmeState         => open);


  module_am       <= vme_am_i(5 downto 3) & vme_am_i(1 downto 0);
  module_addr     <= not(module_addr_i(4 downto 0));
  vme_intreq2_n_o <= vme_intreq_n(2);

  cmp_bus_interface_controller : BusIntControl
    port map(
      Clk   => sys_clk,
      RstNA => sys_rst_n,

      -- Interface
      IntRead     => IntRead,           -- Interface Read Signal
      IntWrite    => IntWrite,          -- Interface Write Signal
      DataFromInt => DataFromInt,       -- Data From interface
      IntAdd      => intAdd,            -- Address From interface

      OpDone    => opDone,              -- Operation Done, Read or Write Finished
      DataToInt => dataToInt,           -- Data going from Control to the Interface

      -- Registers
      contToRegs => contToRegs,         -- Data going from Control to the Registers
                                        -- This consists of Data + Write Enable Signal
      RegsToCont => regsToCont,         -- Data Array From the Registers to the Control

      -- Memory
      ContToMem => contToMem,           -- Data going from Control to the Registers
      -- This consists of Data + Enable + Read + Write
      MemToCont => memToCont            -- Data Array  From the Registers to the Control
                                        -- Data + Done
      );

  -- An interrupt is issued on the "STOP" input pulse
  p_irq : process(sys_rst_n, sys_clk)
  begin
    if sys_rst_n = '0' then
      UserIntReqN <= '1';
    elsif rising_edge(sys_clk) then
      if irq_en = '1' then
        if IntProcessed = '1' then
          UserIntReqN <= '1';
        elsif stop_p = '1' then
          UserIntReqN <= '0';
        end if;
      else UserIntReqN <= '1';
      end if;
    end if;
  end process p_irq;

  vme_iack_mux <= not irq_en;


  ------------------------------------------------------------------------------
  -- Registers
  ------------------------------------------------------------------------------

  -- Control and status register
  --   [0]     | rw | input pulse polarity, 0=negative, 1=positive
  --   [1]     | rw | enable/disable the module
  --   [2]     | rw | enable/disable IRQ
  --   [3]     | wo | soft start (read as 0)
  --   [4]     | wo | soft stop (read as 0)
  --   [5]     | rw | wr = soft reset, rd = acquisition in progress (between a start and a stop input)
  --   [6]     | ro | counter overflow
  --   [7]     | ro | RAM overflow
  --   [15:8]  | rw | irq vector (delfaut 0x86)
  --   [31:16] | rw | gateware version number (4 digits BCD, e.g. 0x0142=v1.42)

  csr_reg_wren <= contToRegs.Sel(CSR_REG_P) and contToRegs.Wr;

  p_csr_reg : process(sys_clk, sys_rst_n)
  begin
    if sys_rst_n = '0' then
      input_polarity <= g_DEFAULT_INPUT_POL;
      module_en      <= '0';
      irq_en         <= '0';
      sw_start_p     <= '0';
      sw_stop_p      <= '0';
      sw_reset_p     <= '0';
      irq_vector     <= (others => '0');
    elsif rising_edge(sys_clk) then
      if csr_reg_wren = '1' then
        input_polarity <= contToRegs.Data(0);
        module_en      <= contToRegs.Data(1);
        irq_en         <= contToRegs.Data(2);
        sw_start_p     <= contToRegs.Data(3);
        sw_stop_p      <= contToRegs.Data(4);
        sw_reset_p     <= contToRegs.Data(5);
        irq_vector     <= contToRegs.Data(15 downto 8);
      else
        sw_start_p <= '0';
        sw_stop_p  <= '0';
        sw_reset_p <= '0';
      end if;
    end if;
  end process p_csr_reg;
  IRQStatusIDReg <= x"000000" & irq_vector;

  regsToCont(CSR_REG_P) <= g_GATEWARE_VER &   -- [31:16]
                           irq_vector &       -- [15:8]
                           ram_wr_overflow &  -- [7]
                           cnt_overflow &     -- [6]
                           data_acq_en &      -- [5]
                           "00" &             -- [4:3]
                           irq_en &           -- [2]
                           module_en &        -- [1]
                           input_polarity;    -- [0]


  -- Memory pointer register
  --   [31:19] | ro | unused (read as 0)
  --   [18:0]  | ro | memory pointer (byte address)
  regsToCont(MEM_PTR_REG_P) <= x"000" & "0" & std_logic_vector(ram_wr_addr) & "00";


  -- Mode register
  --   [32:8] | ro | 0x43564f = "CVO"
  --   [7:4]  | ro | 0x0
  --   [3:0]  | rw | mode
  --                 0x0 = reserved
  --                 0x1 = front panel optical input 1 only (16-bit serial, SCI protocol)
  --                 0x2 = front panel copper input 1 only (16-bit serial, SCI protocol)
  --                 0x3 = 32-bit up/down counter (btrain)
  --                 0x4 = parallel rtm input (32-bit)
  --                 0x5 = front panel optical input 1 and 2 (2x 16-bit serial, SCI protocol)
  --                 0x6 = front panel copper input 1 and 2 (2x 16-bit serial, SCI protocol)
  --                 0x7 = rtm copper inputs (32x 16-bit serial, SCI protocol)
  --                 0x8 = reserved
  --                 0x9 = front panel optical input 1 only (16-bit serial, CVORB protocol)
  --                 0xA = front panel copper input 1 only (16-bit serial, CVORB protocol)
  --                 0xB = 2x 16-bit up counter
  --                 0xC = parallel rtm input (31-bit) with strobe on 32nd bit
  --                 0xD = front panel optical input 1 and 2 (2x 16-bit serial, CVORB protocol)
  --                 0xE = front panel copper input 1 and 2 (2x 16-bit serial, CVORB protocol)
  --                 0xF = rtm copper inputs (32x 16-bit serial, CVORB protocol)

  mode_reg_wren <= contToRegs.Sel(MODE_REG_P) and contToRegs.Wr;

  p_mode_reg : process(sys_clk, sys_rst_n)
  begin
    if sys_rst_n = '0' then
      mode <= (others => '0');
    elsif rising_edge(sys_clk) then
      -- the module MUST be disable to change the mode!
      if (mode_reg_wren = '1' and module_en = '0') then
        mode <= contToRegs.Data(3 downto 0);
      end if;
    end if;
  end process p_mode_reg;

  regsToCont(MODE_REG_P) <= x"43564f" & x"0" & mode;


  -- Channel enable register
  --   ONLY used in 32x serial rtm inputs mode
  --   [31:0] | wr | rtm serial channel enable mask
  channel_en_reg_wren <= contToRegs.Sel(CHAN_EN_REG_P) and contToRegs.Wr;

  process(sys_clk, sys_rst_n)
  begin
    if sys_rst_n = '0' then
      channel_en <= (others => '0');
    elsif rising_edge(sys_clk) then
      -- the module MUST NOT be acquiring to change the channel enable mask
      if channel_en_reg_wren = '1' and data_acq_en = '0' then
        channel_en <= contToRegs.Data;
      end if;
    end if;
  end process;
  regsToCont(CHAN_EN_REG_P) <= channel_en;


  -- Input clock frequency register
  --   [31:0] | ro | front panel input clock frequency
  regsToCont(CLK_FREQ_REG_P) <= clock_freq;


  -- DAC select register
  --   ONLY unsed in 32x serial rtm input mode
  --   [4:0]  | rw | selects rtm serial inputs to be reproduced on the front panel DAC
  --   [31:5] | ro | unused (read as 0)
  channel_select_reg_wren <= contToRegs.Sel(CHAN_SEL_REG_P) and contToRegs.Wr;

  process(sys_clk, sys_rst_n)
  begin
    if sys_rst_n = '0' then
      channel_select_reg <= (others => '0');
    elsif rising_edge(sys_clk) then
      if channel_select_reg_wren = '1' then
        -- Takes only even number -> always used in pair (channel_select, channel_select+1)
        channel_select_reg <= contToRegs.Data(4 downto 1) & '0';
      end if;
    end if;
  end process;
  regsToCont(CHAN_SEL_REG_P) <= x"000000" & "000" & channel_select_reg;

  -- RTM channel selection
  channel_select <= to_integer(unsigned(channel_select_reg));



  -- CVORB serial decoder setting
  --   [7:0]   | rw | pulse width threshold
  --   [15:8]  | ro | measured pulse width (from cvorb serial stream)
  --   [31:16] | ro | unused (read as 0)
  cvorb_reg_wren <= contToRegs.Sel(CVORB_REG_P) and contToRegs.Wr;

  process(sys_clk, sys_rst_n)
  begin
    if sys_rst_n = '0' then
      cvorb_pulse_width_thres <= std_logic_vector(to_unsigned(8, cvorb_pulse_width_thres'length));
    elsif rising_edge(sys_clk) then
      if cvorb_reg_wren = '1' then
        cvorb_pulse_width_thres <= contToRegs.Data(7 downto 0);
      end if;
    end if;
  end process;
  regsToCont(CVORB_REG_P) <= cvorb_meas_pulse_width1 & cvorb_meas_pulse_width2 & x"00" & cvorb_pulse_width_thres;

  cvorb_meas_pulse_width1 <= rtm_cvorb_meas_pulse_width(channel_select)   when (mode = c_RTM_CVORB_M) else fp_cvorb_meas_pulse_width1;
  cvorb_meas_pulse_width2 <= rtm_cvorb_meas_pulse_width(channel_select+1) when (mode = c_RTM_CVORB_M) else fp_cvorb_meas_pulse_width2;


  ------------------------------------------------------------------------------
  -- Data acquisition enable
  ------------------------------------------------------------------------------
  process(sys_clk, sys_rst_n)
  begin
    if sys_rst_n = '0' then
      data_acq_en <= '0';
    elsif rising_edge(sys_clk) then
      if start_p = '1' and module_en = '1' then
        data_acq_en <= '1';
      elsif stop_p = '1' or module_en = '0' then
        data_acq_en <= '0';
      end if;
    end if;
  end process;


  ------------------------------------------------------------------------------
  -- Acquisition clock frequency meter
  ------------------------------------------------------------------------------
  cmp_clk_freq_meter : gc_frequency_meter
    generic map(
      g_with_internal_timebase => false,
      g_clk_sys_freq           => 40000000,
      g_counter_bits           => 32)
    port map(
      clk_sys_i    => sys_clk,
      clk_in_i     => fp_clock_p,
      rst_n_i      => sys_rst_n,
      pps_p1_i     => pps_p,
      freq_o       => clock_freq_t,
      freq_valid_o => clock_freq_valid);

  p_clk_freq_reg : process (sys_rst_n, sys_clk)
  begin
    if sys_rst_n = '0' then
      clock_freq         <= (others => '0');
      clock_freq_wd      <= '0';
      clock_freq_valid_d <= '0';
    elsif rising_edge(sys_clk) then
      clock_freq_valid_d <= clock_freq_valid;

      if clock_freq_valid_p = '1' then
        clock_freq    <= clock_freq_t;
        clock_freq_wd <= '0';
      elsif pps_p = '1' then
        if clock_freq_wd = '1' then
          clock_freq <= (others => '0');
        else
          clock_freq_wd <= '1';
        end if;
      end if;
    end if;
  end process p_clk_freq_reg;

  clock_freq_valid_p <= clock_freq_valid and not(clock_freq_valid_d);


  ------------------------------------------------------------------------------
  -- Inter-acquisition time measurement
  --   counts input clock pulses between a stop pulse and a start pulse
  ------------------------------------------------------------------------------
  p_inter_acq_time : process(sys_rst_n, sys_clk)
  begin
    if sys_rst_n = '0' then
      inter_acq_time_cnt <= (others => '0');
      inter_acq_time     <= (others => '0');
    elsif rising_edge(sys_clk) then
      if (data_acq_en = '0') then
        if (fp_clock_p = '1') then
          inter_acq_time_cnt <= inter_acq_time_cnt + 1;
        else
          inter_acq_time <= std_logic_vector(inter_acq_time_cnt);
        end if;
      else
        inter_acq_time_cnt <= (others => '0');
      end if;
    end if;
  end process p_inter_acq_time;


  ------------------------------------------------------------------------------
  -- Parallel mode
  ------------------------------------------------------------------------------
  p_parallel_sampling : process (sys_rst_n, sys_clk)
  begin
    if sys_rst_n = '0' then
      parallel_data <= (others => '0');
      rtm_data_d    <= (others => '0');
      rtm_data_d2   <= (others => '0');
    elsif rising_edge(sys_clk) then
      rtm_data_d  <= rtm_data_i;
      rtm_data_d2 <= rtm_data_d;

      if mode = c_RTM_PARALLEL_M then
        -- Samples parallel data input 50ns after STROBE rising edge
        if strobe_p_d(2) = '1' then
          parallel_data <= rtm_data_d2;
        end if;
      elsif mode = c_RTM_PARALLEL_STROBE_M then
        if rtm_data_d2(31) = '1' then
          parallel_data <= '0' & rtm_data_d2(30 downto 0);
        end if;
      end if;
    end if;
  end process p_parallel_sampling;


  ------------------------------------------------------------------------------
  -- 32-bit up/down counter mode (b-train)
  ------------------------------------------------------------------------------
  ud_cnt_en <= data_acq_en when mode = c_CNT32_M else '0';

  cmp_ud_cnt : up_down_counter
    generic map(
      g_width => 32)
    port map(
      rst_n_i    => sys_rst_n,
      clk_i      => sys_clk,
      clear_i    => reset_p,
      up_i       => fp_bup_p,
      down_i     => fp_bdown_p,
      enable_i   => ud_cnt_en,
      overflow_o => ud_cnt_overflow,
      value_o    => ud_cnt_value,
      valid_o    => ud_cnt_valid);


  ------------------------------------------------------------------------------
  -- 2x 16-bit up counter mode
  --   BUP   input increments counter 1
  --   BDOWN input increments counter 2
  ------------------------------------------------------------------------------
  up_cnt_en <= data_acq_en when mode = c_CNT2X16_M else '0';

  cmp_up_cnt1 : up_down_counter
    generic map(
      g_width => 16)
    port map(
      rst_n_i    => sys_rst_n,
      clk_i      => sys_clk,
      clear_i    => reset_p,
      up_i       => fp_bup_p,
      down_i     => '0',
      enable_i   => up_cnt_en,
      overflow_o => up_cnt1_overflow,
      value_o    => up_cnt1_value,
      valid_o    => up_cnt1_valid);

  cmp_up_cnt2 : up_down_counter
    generic map(
      g_width => 16)
    port map(
      rst_n_i    => sys_rst_n,
      clk_i      => sys_clk,
      clear_i    => reset_p,
      up_i       => fp_bdown_p,
      down_i     => '0',
      enable_i   => up_cnt_en,
      overflow_o => up_cnt2_overflow,
      value_o    => up_cnt2_value,
      valid_o    => up_cnt2_valid);

  cnt_overflow <= ud_cnt_overflow when mode = c_CNT32_M else
                  (up_cnt1_overflow or up_cnt2_overflow) when mode = c_CNT2X16_M else
                  '0';


  ------------------------------------------------------------------------------
  -- RTM serial decoders
  ------------------------------------------------------------------------------

  rtm_data_clk_p <= fp_clock_p and data_acq_en;

  -- RESET ignored during acquisition
  rtm_reset_ram_addr <= reset_p when data_acq_en = '0' else '0';

  cmp_rtm_serial_manager : rtm_serial_manager
    port map(
      rst_n_i                   => sys_rst_n,
      clk_i                     => sys_clk,
      rtm_data_i                => rtm_data_i,
      rtm_data_o                => rtm_data,
      rtm_data_valid_o          => rtm_data_valid,
      mode_i                    => mode,
      channel_en_i              => channel_en,
      data_clk_i                => rtm_data_clk_p,
      cvorb_pulse_width_thres_i => cvorb_pulse_width_thres,
      cvorb_meas_pulse_width_o  => rtm_cvorb_meas_pulse_width,
      ram_data_o                => rtm_ram_data,
      ram_data_valid_o          => rtm_ram_data_valid,
      ram_addr_o                => rtm_ram_addr,
      ram_overflow_o            => rtm_ram_overflow,
      ram_data_written_i        => ram_wr_done,
      reset_ram_addr_i          => rtm_reset_ram_addr);


  ------------------------------------------------------------------------------
  -- Front panel serial decoders, SCI protocol
  ------------------------------------------------------------------------------
  fp_sci_en <= '1' when
               (mode = c_FP_OP16_SCI_M) or
               (mode = c_FP_OP32_SCI_M) or
               (mode = c_FP_CU16_SCI_M) or
               (mode = c_FP_CU32_SCI_M) else '0';

  -- Selects between optical or copper input
  fp_data1_sci_serial <= fp_data_optic1_i when (mode = c_FP_OP16_SCI_M) or (mode = c_FP_OP32_SCI_M) else
                         not(fp_data_cu1_i) when (mode = c_FP_CU16_SCI_M) or (mode = c_FP_CU32_SCI_M) else
                         '0';

  fp_data2_sci_serial <= fp_data_optic2_i when (mode = c_FP_OP16_SCI_M) or (mode = c_FP_OP32_SCI_M) else
                         not(fp_data_cu2_i) when (mode = c_FP_CU16_SCI_M) or (mode = c_FP_CU32_SCI_M) else
                         '0';

  -- Decoders
  cmp_fp_sci_decoder1 : sci_decoder
    port map(
      rst_n_i      => sys_rst_n,
      clk_i        => sys_clk,
      enable_i     => fp_sci_en,
      data_i       => fp_data1_sci_serial,
      data_o       => fp_data1_sci,
      data_valid_o => fp_data1_sci_valid);

  cmp_fp_sci_decoder2 : sci_decoder
    port map(
      rst_n_i      => sys_rst_n,
      clk_i        => sys_clk,
      enable_i     => fp_sci_en,
      data_i       => fp_data2_sci_serial,
      data_o       => fp_data2_sci,
      data_valid_o => fp_data2_sci_valid);


  ------------------------------------------------------------------------------
  -- Front panel serial decoders, CVORB protocol
  ------------------------------------------------------------------------------
  fp_cvorb_en <= '1' when
                 (mode = c_FP_OP16_CVORB_M) or
                 (mode = c_FP_OP32_CVORB_M) or
                 (mode = c_FP_CU16_CVORB_M) or
                 (mode = c_FP_CU32_CVORB_M) else '0';

  -- Selects between optical or copper inputs
  fp_data1_cvorb_serial <= fp_data_optic1_i when (mode = c_FP_OP16_CVORB_M) or (mode = c_FP_OP32_CVORB_M) else
                           not(fp_data_cu1_i) when (mode = c_FP_CU16_CVORB_M) or (mode = c_FP_CU32_CVORB_M) else
                           '0';

  fp_data2_cvorb_serial <= fp_data_optic2_i when (mode = c_FP_OP16_CVORB_M) or (mode = c_FP_OP32_CVORB_M) else
                           not(fp_data_cu2_i) when (mode = c_FP_CU16_CVORB_M) or (mode = c_FP_CU32_CVORB_M) else
                           '0';

  -- Decoders
  cmp_fp_cvorb_decoder1 : cvorb_decoder
    port map(
      rst_n_i             => sys_rst_n,
      clk_i               => sys_clk,
      enable_i            => fp_cvorb_en,
      data_i              => fp_data1_cvorb_serial,
      zero_test_o         => cvorb_zero_test,
      one_test_o          => cvorb_one_test,
      strobe_test_o       => cvorb_strobe_test,
      pulse_width_thres_i => cvorb_pulse_width_thres,
      pulse_width_o       => fp_cvorb_meas_pulse_width1,
      data_o              => fp_data1_cvorb,
      data_valid_o        => fp_data1_cvorb_valid);

  cmp_fp_cvorb_decoder2 : cvorb_decoder
    port map(
      rst_n_i             => sys_rst_n,
      clk_i               => sys_clk,
      enable_i            => fp_cvorb_en,
      data_i              => fp_data2_cvorb_serial,
      zero_test_o         => open,
      one_test_o          => open,
      strobe_test_o       => open,
      pulse_width_thres_i => cvorb_pulse_width_thres,
      pulse_width_o       => fp_cvorb_meas_pulse_width2,
      data_o              => fp_data2_cvorb,
      data_valid_o        => fp_data2_cvorb_valid);


  ------------------------------------------------------------------------------
  -- Dual port RAM manager
  --   port 1: read from VME
  --   port 2: write from data sources
  ------------------------------------------------------------------------------

  -- RAM read from VME
  ram_rd <= ContToMem.Rd and ContToMem.SelectedPos(EXT_RAM_P);

  p_ram_read : process (sys_rst_n, sys_clk)
  begin
    if sys_rst_n = '0' then
      ram_rd_addr <= (others => '0');
    elsif rising_edge(sys_clk) then
      if ram_rd = '1' then
        -- VME provides byte address, RAM manager takes 32-bit word address
        ram_rd_addr <= ContToMem.Add(18 downto 2);
      end if;
    end if;
  end process p_ram_read;

  MemToCont(EXT_RAM_P).RdDone <= ram_rd_data_valid;
  MemToCont(EXT_RAM_P).Data   <= ram_rd_data;

  -- Sample data from sources and write to RAM
  p_ram_write_data : process (sys_rst_n, sys_clk)
  begin
    if sys_rst_n = '0' then
      ram_wr_data <= (others => '0');
    elsif rising_edge(sys_clk) then
      -- priority on data_acq_en as the rtm_serial_manager might still be writing data to RAM after the STOP
      if (mode = c_RTM_SCI_M or mode = c_RTM_CVORB_M) then
        ram_wr_data <= rtm_ram_data;
      elsif (data_acq_en = '1' and fp_clock_p = '1') then
        if mode = c_CNT32_M then
          ram_wr_data <= ud_cnt_value;
        elsif mode = c_CNT2X16_M then
          ram_wr_data <= up_cnt2_value & up_cnt1_value;
        elsif (mode = c_RTM_PARALLEL_M or mode = c_RTM_PARALLEL_STROBE_M) then
          ram_wr_data <= parallel_data;
        elsif (mode = c_FP_OP16_SCI_M or mode = c_FP_CU16_SCI_M) then
          ram_wr_data <= x"0000" & fp_data1_sci;
        elsif (mode = c_FP_OP16_CVORB_M or mode = c_FP_CU16_CVORB_M) then
          ram_wr_data <= x"0000" & fp_data1_cvorb;
        elsif (mode = c_FP_OP32_SCI_M or mode = c_FP_CU32_SCI_M) then
          ram_wr_data <= fp_data2_sci & fp_data1_sci;
        elsif (mode = c_FP_OP32_CVORB_M or mode = c_FP_CU32_CVORB_M) then
          ram_wr_data <= fp_data2_cvorb & fp_data1_cvorb;
        else
          -- if no valid mode is selected, write zeros to RAM
          ram_wr_data <= (others => '0');
        end if;
      end if;
    end if;
  end process p_ram_write_data;

  p_ram_write_addr : process (sys_rst_n, sys_clk)
  begin
    if sys_rst_n = '0' then
      ram_wr_addr_cnt <= (others => '0');
      ram_wr_overflow <= '0';
    elsif rising_edge(sys_clk) then
      if (mode = c_RTM_SCI_M) or (mode = c_RTM_CVORB_M) then
        ram_wr_addr_cnt <= unsigned(rtm_ram_addr);
        ram_wr_overflow <= rtm_ram_overflow;
      elsif data_acq_en = '1' then
        if ram_wr_addr_cnt = EXTRAMFULL then
          ram_wr_overflow <= '1';
        elsif clock_p_d(2) = '1' then
          ram_wr_addr_cnt <= ram_wr_addr_cnt + EXTRAM_BUF_ONE;
        end if;
      elsif (start_p = '1') or (reset_p = '1') then
        ram_wr_addr_cnt <= MEMEMPTY;
        ram_wr_overflow <= '0';
      end if;
    end if;
  end process p_ram_write_addr;

  ram_wr_addr <= std_logic_vector(ram_wr_addr_cnt);

  p_ram_write : process (sys_rst_n, sys_clk)
  begin
    if sys_rst_n = '0' then
      ram_wr <= '0';
    elsif rising_edge(sys_clk) then
      if (data_acq_en = '1') and (ram_wr_overflow = '0') then
        if (mode = c_RTM_SCI_M) or (mode = c_RTM_CVORB_M) then
          ram_wr <= rtm_ram_data_valid;
        else
          ram_wr <= clock_p_d(1);
        end if;
      else
        ram_wr <= '0';
      end if;
    end if;
  end process p_ram_write;

  -- RAM manager
  cmp_ram_manager : RAMManager
    port map(
      RstN => sys_rst_n,
      Clk  => sys_clk,

      DataFromHistory        => ram_wr_data,
      AddrFromHistory        => ram_wr_addr,
      WriteFromHistory       => ram_wr,
      DataFromHistoryWritten => ram_wr_done,

      AddrFromCont    => ram_rd_addr,
      ReadFromCont    => ram_rd,
      DataToCont      => ram_rd_data,
      DataToContValid => ram_rd_data_valid,

      RAMAddr  => ram_add_o,
      RAMData  => ram_data_b,
      RAMOEN   => ram_oe_n_o,
      RAMGWN   => ram_gw_n_o,
      RAMADSCN => ram_adsc_n_o,
      RAMCEN   => ram_ce_n_o,
      RAMCS0   => open,
      RAMCS1N  => open,
      RAMBWEN  => ram_wr_n_o,
      RAMBWN   => ram_bw_n_o,
      RAMADVN  => ram_adv_n_o,
      RAMLBON  => ram_lbo_n_o,
      RAMZZ    => open,
      RAMADSPN => ram_adsp_n_o);

  ram_clk_o      <= not sys_clk;
  ram_data_par_o <= (others => 'Z');


  ------------------------------------------------------------------------------
  -- DAC managment
  ------------------------------------------------------------------------------
  dac1_load_p <= strobe_p_d(4) when mode = c_RTM_PARALLEL_M else
                 rtm_data_d2(31)                when mode = c_RTM_PARALLEL_STROBE_M               else
                 ud_cnt_valid                   when mode = c_CNT32_M                             else
                 up_cnt1_valid                  when mode = c_CNT2X16_M                           else
                 rtm_data_valid(channel_select) when (mode = c_RTM_SCI_M or mode = c_RTM_CVORB_M) else
                 fp_data1_sci_valid when (mode = c_FP_OP16_SCI_M or
                                          mode = c_FP_CU16_SCI_M or
                                          mode = c_FP_OP32_SCI_M or
                                          mode = c_FP_CU32_SCI_M) else
                 fp_data1_cvorb_valid when (mode = c_FP_OP16_CVORB_M or
                                            mode = c_FP_CU16_CVORB_M or
                                            mode = c_FP_OP32_CVORB_M or
                                            mode = c_FP_CU32_CVORB_M) else
                 '0';

  cmp_dac1_load : dac_load_delay
    port map(
      rst_n_i   => sys_rst_n,
      clk_i     => sys_clk,
      pulse_i   => dac1_load_p,
      d_pulse_o => dac1_load_o);

  dac2_load_p <= strobe_p_d(4) when mode = c_RTM_PARALLEL_M else
                 rtm_data_d2(31)                  when mode = c_RTM_PARALLEL_STROBE_M               else
                 ud_cnt_valid                     when mode = c_CNT32_M                             else
                 up_cnt2_valid                    when mode = c_CNT2X16_M                           else
                 rtm_data_valid(channel_select+1) when (mode = c_RTM_SCI_M or mode = c_RTM_CVORB_M) else
                 fp_data2_sci_valid when (mode = c_FP_OP16_SCI_M or
                                          mode = c_FP_CU16_SCI_M or
                                          mode = c_FP_OP32_SCI_M or
                                          mode = c_FP_CU32_SCI_M) else
                 fp_data2_cvorb_valid when (mode = c_FP_OP16_CVORB_M or
                                            mode = c_FP_CU16_CVORB_M or
                                            mode = c_FP_OP32_CVORB_M or
                                            mode = c_FP_CU32_CVORB_M) else
                 '0';

  cmp_dac2_load : dac_load_delay
    port map(
      rst_n_i   => sys_rst_n,
      clk_i     => sys_clk,
      pulse_i   => dac2_load_p,
      d_pulse_o => dac2_load_o);

  -- DAC data selection mux
  p_dac_data_select : process (sys_rst_n, sys_clk)
  begin
    if sys_rst_n = '0' then
      dac1_data_o <= (others => '0');
      dac2_data_o <= (others => '0');
    elsif rising_edge(sys_clk) then
      if (ud_cnt_valid = '1') and (mode = c_CNT32_M) then
        dac1_data_o <= ud_cnt_value(15 downto 0);
        dac2_data_o <= ud_cnt_value(31 downto 16);
      elsif mode = c_CNT2X16_M then
        if up_cnt1_valid = '1' then
          dac1_data_o <= up_cnt1_value(15 downto 0);
        end if;
        if up_cnt2_valid = '1' then
          dac2_data_o <= up_cnt2_value(15 downto 0);
        end if;
      elsif (strobe_p_d(3) = '1') and (mode = c_RTM_PARALLEL_M) then
        dac1_data_o <= parallel_data(15 downto 0);
        dac2_data_o <= parallel_data(31 downto 16);
      elsif mode = c_RTM_PARALLEL_STROBE_M then
        dac1_data_o <= parallel_data(15 downto 0);
        dac2_data_o <= parallel_data(31 downto 16);
      elsif (mode = c_RTM_SCI_M or mode = c_RTM_CVORB_M) then
        if rtm_data_valid(channel_select) = '1' then
          dac1_data_o <= rtm_data(channel_select);
        end if;
        if rtm_data_valid(channel_select+1) = '1' then
          dac2_data_o <= rtm_data(channel_select+1);
        end if;
      else
        if (mode = c_FP_OP16_SCI_M or mode = c_FP_CU16_SCI_M or mode = c_FP_OP32_SCI_M or mode = c_FP_CU32_SCI_M) then
          if fp_data1_sci_valid = '1' then
            dac1_data_o <= fp_data1_sci;
          end if;
          if fp_data2_sci_valid = '1' then
            dac2_data_o <= fp_data2_sci;
          end if;
        elsif (mode = c_FP_OP16_CVORB_M or mode = c_FP_CU16_CVORB_M or mode = c_FP_OP32_CVORB_M or mode = c_FP_CU32_CVORB_M) then
          if fp_data1_cvorb_valid = '1' then
            dac1_data_o <= fp_data1_cvorb;
          end if;
          if fp_data2_cvorb_valid = '1' then
            dac2_data_o <= fp_data2_cvorb;
          end if;
        end if;
      end if;
    end if;
  end process p_dac_data_select;


  ------------------------------------------------------------------------------
  -- Front panel LEDs
  ------------------------------------------------------------------------------

  fp_led_o <= fp_led;

  -- SPARE
  fp_led(0) <= ram_wr_overflow or cnt_overflow;

  -- INT ENABLE
  fp_led(1) <= irq_en;

  -- STOP
  cmp_monostable_stop_led : gc_extend_pulse
    generic map (
      g_width => g_LED_PULSE_WIDTH)
    port map (
      clk_i      => sys_clk,
      rst_n_i    => sys_rst_n,
      pulse_i    => stop_p,
      extended_o => fp_led(2));

  -- START
  cmp_monostable_start_led : gc_extend_pulse
    generic map (
      g_width => g_LED_PULSE_WIDTH)
    port map (
      clk_i      => sys_clk,
      rst_n_i    => sys_rst_n,
      pulse_i    => start_p,
      extended_o => fp_led(3));

  -- READ
  cmp_monoastable_read_led : gc_extend_pulse
    generic map (
      g_width => g_LED_PULSE_WIDTH)
    port map (
      clk_i      => sys_clk,
      rst_n_i    => sys_rst_n,
      pulse_i    => ram_rd,
      extended_o => fp_led(4));

  -- WRITE
  cmp_monostable_write_led : gc_extend_pulse
    generic map (
      g_width => g_LED_PULSE_WIDTH)
    port map (
      clk_i      => sys_clk,
      rst_n_i    => sys_rst_n,
      pulse_i    => ram_wr,
      extended_o => fp_led(5));

  -- ACQ ENABLE
  fp_led(6) <= data_acq_en;

  -- EXT DISABLE
  fp_led(7) <= not module_en;


  ------------------------------------------------------------------------------
  -- RS232 LCD display
  ------------------------------------------------------------------------------

  -- Input clock frequency conversion to BCD
  cmp_clk_freq_slv2bcd : SB_to_BCD
    generic map(32, 8)
    port map(
      Clock      => sys_clk,
      Reset      => sys_rst_n,
      SB_Ready   => pps_p,
      SB_Number  => clock_freq,
      BCD_Vector => clock_freq_bcd,
      BCD_Ready  => open);

  -- Inter-acquisition time conversion to BCD
  cmp_inter_acq_time : SB_to_BCD
    generic map(16, 5)
    port map(
      Clock      => sys_clk,
      Reset      => sys_rst_n,
      SB_Ready   => start_p_d(2),
      SB_Number  => inter_acq_time,
      BCD_Vector => inter_acq_time_bcd,
      BCD_Ready  => open);

  -- "CVORA Ver: nn.nn"
  message_to_send(1) <= (
    LETTER_C, LETTER_V, LETTER_O, LETTER_R, LETTER_A,
    space, LETTER_V, LETTER_emin, LETTER_rmin, semicolon,
    space, X"3" & g_GATEWARE_VER(15 downto 12), X"3" & g_GATEWARE_VER(11 downto 8),
    dot, X"3" & g_GATEWARE_VER(7 downto 4), X"3" & g_GATEWARE_VER(3 downto 0),
    Carriage_Return, Line_Feed);

  -- "  BA: 0xnn0000  "
  message_to_send(2) <= (
    space, space,
    LETTER_B, LETTER_A, semicolon, space, LETTER_0, LETTER_xmin,  -- BASE Address BA: 0x
    "0011" & module_addr(4 downto 1), "0011" & module_addr(0) & "000",
    LETTER_0, LETTER_0, LETTER_0, LETTER_0, space, space,
    Carriage_Return, Line_feed);

  -- ""
  message_to_send(3) <=
    c_FP_OP16_SCI_LINE         when mode = c_FP_OP16_SCI_M         else
    c_FP_CU16_SCI_LINE         when mode = c_FP_CU16_SCI_M         else
    c_CNT32_LINE               when mode = c_CNT32_M               else
    c_RTM_PARALLEL_LINE        when mode = c_RTM_PARALLEL_M        else
    c_FP_OP32_SCI_LINE         when mode = c_FP_OP32_SCI_M         else
    c_FP_CU32_SCI_LINE         when mode = c_FP_CU32_SCI_M         else
    c_RTM_SCI_LINE             when mode = c_RTM_SCI_M             else
    c_FP_OP16_CVORB_LINE       when mode = c_FP_OP16_CVORB_M       else
    c_FP_CU16_CVORB_LINE       when mode = c_FP_CU16_CVORB_M       else
    c_CNT2X16_LINE             when mode = c_CNT2X16_M             else
    c_RTM_PARALLEL_STROBE_LINE when mode = c_RTM_PARALLEL_STROBE_M else
    c_FP_OP32_CVORB_LINE       when mode = c_FP_OP32_CVORB_M       else
    c_FP_CU32_CVORB_LINE       when mode = c_FP_CU32_CVORB_M       else
    c_RTM_CVORB_LINE           when mode = c_RTM_CVORB_M           else
    c_RESERVED_LINE;

  -- "F1: nnnnn.nnnkHz"
  message_to_send(4) <= (
    LETTER_F, letter_1, semicolon,
    space, "0011" & clock_freq_bcd(7), "0011" & clock_freq_bcd(6), "0011" & clock_freq_bcd(5),
    "0011" & clock_freq_bcd(4), "0011" & clock_freq_bcd(3), dot,
    "0011" & clock_freq_bcd(2), "0011" & clock_freq_bcd(1),
    "0011" & clock_freq_bcd(0), letter_kmin, LETTER_H, letter_zmin,
    Carriage_Return, Line_feed);

  -- "ACQ: nnnnn x Clk"
  message_to_send(5) <= (
    LETTER_A, LETTER_C, LETTER_Q, semicolon,
    space, "0011" & inter_acq_time_bcd(4), "0011" & inter_acq_time_bcd(3),
    "0011" & inter_acq_time_bcd(2), "0011" & inter_acq_time_bcd(1), "0011" & inter_acq_time_bcd(0),
    space, Letter_xmin, space, LETTER_C, Letter_lmin, Letter_kmin,
    Carriage_Return, Line_feed);

  -- Sends ascii message to RS232 LCD display
  cmp_message : message
    port map(
      rst         => sys_rst_n,
      clk         => sys_clk,
      rs232_start => pps_p,
      message     => message_to_send,
      message_env => open,
      rs232out    => rs232_tx_o
      );


  ------------------------------------------------------------------------------
  -- Test outputs
  ------------------------------------------------------------------------------
  test_o(3) <= start_p_d(2);            -- TP7
  test_o(2) <= data_acq_en;             -- TP8
  test_o(0) <= fp_clock_p;              -- TP9
  test_o(1) <= stop_p;                  -- TP10



end Behavioral;
