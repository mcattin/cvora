--------------------------------------------------------------------------------
-- CERN (BE-CO-HT)
-- Package for CVORA project
--------------------------------------------------------------------------------
--
-- unit name: cvora_pkg
--
-- author: Matthieu Cattin (matthieu.cattin@cern.ch)
--
-- date: 11-11-2013
--
-- description: Types, subtypes, constants, components and functions definitions.
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
-- last changes: see git log.
----------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.bus_int_pkg.all;
use work.vme_pkg.all;
use work.message_pkg.all;


package cvora_pkg is


--******************* Constants *********************
  constant c_NB_RTM_CHAN : natural := 32;

  -- Modes
  constant c_RESERVED1_M           : std_logic_vector(3 downto 0) := "0000";
  constant c_FP_OP16_SCI_M         : std_logic_vector(3 downto 0) := "0001";
  constant c_FP_CU16_SCI_M         : std_logic_vector(3 downto 0) := "0010";
  constant c_CNT32_M               : std_logic_vector(3 downto 0) := "0011";
  constant c_RTM_PARALLEL_M        : std_logic_vector(3 downto 0) := "0100";
  constant c_FP_OP32_SCI_M         : std_logic_vector(3 downto 0) := "0101";
  constant c_FP_CU32_SCI_M         : std_logic_vector(3 downto 0) := "0110";
  constant c_RTM_SCI_M             : std_logic_vector(3 downto 0) := "0111";
  constant c_RESERVED2_M           : std_logic_vector(3 downto 0) := "1000";
  constant c_FP_OP16_CVORB_M       : std_logic_vector(3 downto 0) := "1001";
  constant c_FP_CU16_CVORB_M       : std_logic_vector(3 downto 0) := "1010";
  constant c_CNT2X16_M             : std_logic_vector(3 downto 0) := "1011";
  constant c_RTM_PARALLEL_STROBE_M : std_logic_vector(3 downto 0) := "1100";
  constant c_FP_OP32_CVORB_M       : std_logic_vector(3 downto 0) := "1101";
  constant c_FP_CU32_CVORB_M       : std_logic_vector(3 downto 0) := "1110";
  constant c_RTM_CVORB_M           : std_logic_vector(3 downto 0) := "1111";


  constant RAM_ADDR_LENGTH : integer                                := 17;                              -- Internal Ram address used (number of bits).
  constant EXTRAM_BUF_ONE  : unsigned(RAM_ADDR_LENGTH - 1 downto 0) := (0      => '1', others => '0');
  constant MEMEMPTY        : unsigned(RAM_ADDR_LENGTH - 1 downto 0) := (3      => '1', others => '0');  -- memory begin at 8
  constant EXTRAMFULL      : unsigned(RAM_ADDR_LENGTH - 1 downto 0) := (others => '1');
  constant QUARTZFREQ      : integer                                := 40;                              -- Define here the FREQUENCE of your clock in MHz


--******************* Types *********************
  type rtm_data_array_t is array (0 to c_NB_RTM_CHAN-1) of std_logic_vector(15 downto 0);
  type cvorb_pulse_width_array_t is array (0 to c_NB_RTM_CHAN-1) of std_logic_vector(7 downto 0);


--******************* Components *********************
  component BUFG
    port (
      I : in  std_logic;
      O : out std_logic);
  end component;

  component RS232_Rx
    generic (
      NUMBER_OF_BIT : integer := 8;     -- without start and stop bits
      QUARTZFREQ    : integer := 40;
      BAUD_RATE     : integer := 128000);
    port (
      Clock  : in  std_logic;
      Reset  : in  std_logic;
      Serial : in  std_logic;
      dOut   : out std_logic_vector(NUMBER_OF_BIT - 1 downto 0);
      dReady : out std_logic);
  end component RS232_Rx;

  component dac_load_delay
    port (
      rst_n_i   : in  std_logic;
      clk_i     : in  std_logic;
      pulse_i   : in  std_logic;
      d_pulse_o : out std_logic);
  end component dac_load_delay;

  component Vme_intfce
    generic (
      AddrWidth        : integer   := 24;
      BaseAddrWidth    : integer   := 8;
      DataWidth        : integer   := 32;
      UnalignDataWidth : integer   := 8;
      DirSamePolarity  : std_logic := '0';
      InterruptEn      : std_logic := '0');
    port (
      ResetNA        : in    std_logic;
      Clk            : in    std_logic;
      VmeAddrA       : in    std_logic_vector(AddrWidth-1 downto 1);
      VmeAsNA        : in    std_logic;
      VmeDs1NA       : in    std_logic;
      VmeDs0NA       : in    std_logic;
      VmeData        : inout std_logic_vector(DataWidth-1 downto 0);
      VmeDataUnAlign : inout std_logic_vector(UnalignDataWidth-1 downto 0);
      VmeDir         : out   std_logic;
      VmeBufOeN      : out   std_logic;
      VmeWriteNA     : in    std_logic;
      VmeLwordNA     : in    std_logic;
      VmeIackNA      : in    std_logic;
      IackOutNA      : out   std_logic;
      IackInNA       : in    std_logic;
      VmeIntReqN     : out   std_logic_vector (7 downto 1);
      vmeDtackN      : out   std_logic;
      ModuleAddr     : in    std_logic_vector(BaseAddrWidth-1 downto 0);
      VmeAmA         : in    std_logic_vector(4 downto 0);

      AddrMem          : out std_logic_vector(AddrWidth-BaseAddrWidth-1 downto 0);
      ReadMem          : out std_logic;
      WriteMem         : out std_logic;
      DataFromMemValid : in  std_logic;
      DataFromMem      : in  std_logic_vector(DataWidth-1 downto 0);
      DataToMem        : out std_logic_vector(DataWidth-1 downto 0);
      IntProcessed     : out std_logic;
      UserIntReqN      : in  std_logic;
      UserBlocks       : in  std_logic;
      VmeDirFloat      : out std_logic;
      OpFinishedOut    : out std_logic;
      IRQLevelReg      : in  std_logic_vector (3 downto 1);
      IRQStatusIDReg   : in  std_logic_vector (DataWidth-1 downto 0);
      VmeState         : out std_logic_vector (3 downto 0));
  end component;

  component BusIntControl
    port (
      Clk         : in  std_logic;
      RstNA       : in  std_logic;
      IntRead     : in  std_logic;
      IntWrite    : in  std_logic;
      DataFromInt : in  IntDataType;
      IntAdd      : in  IntAddrOutType;
      OpDone      : out std_logic;
      DataToInt   : out IntDataType;
      ContToRegs  : out ContToRegsType;
      RegsToCont  : in  RegsToContType;
      ContToMem   : out ContToMemType;
      MemToCont   : in  MemToContType);
  end component BusIntControl;

  component message
    port (
      rst         : in  std_logic;
      clk         : in  std_logic;
      rs232_start : in  std_logic;
      message     : in  message_array;
      message_env : out std_logic;
      rs232out    : out std_logic);
  end component;

  component SB_to_BCD
    generic(
      SB_Bits     : natural := 32;      -- default for the instantiation,
      BCD_Numbers : natural := 8);
    port (
      Clock      : in  std_logic;
      Reset      : in  std_logic;
      SB_Number  : in  std_logic_vector ((SB_Bits-1) downto 0);
      SB_Ready   : in  std_logic;
      BCD_Vector : out BCD_vector_TYPE ((BCD_Numbers - 1) downto 0);
      BCD_Ready  : out std_logic);
  end component;

  component up_down_counter
    generic (
      g_width : integer := 32);
    port (
      rst_n_i    : in  std_logic;
      clk_i      : in  std_logic;
      clear_i    : in  std_logic;
      up_i       : in  std_logic;
      down_i     : in  std_logic;
      enable_i   : in  std_logic;
      overflow_o : out std_logic;
      value_o    : out std_logic_vector(g_width - 1 downto 0);
      valid_o    : out std_logic);
  end component up_down_counter;

  component RAMManager
    port (
      RstN                   : in    std_logic;
      Clk                    : in    std_logic;
      DataFromHistory        : in    std_logic_vector(31 downto 0);
      AddrFromHistory        : in    std_logic_vector(16 downto 0);
      WriteFromHistory       : in    std_logic;
      DataFromHistoryWritten : out   std_logic;
      AddrFromCont           : in    std_logic_vector(16 downto 0);
      ReadFromCont           : in    std_logic;
      DataToCont             : out   std_logic_vector(31 downto 0);
      DataToContValid        : out   std_logic;
      RAMAddr                : out   std_logic_vector(16 downto 0);
      RAMData                : inout std_logic_vector(31 downto 0);
      RAMOEN                 : out   std_logic;
      RAMGWN                 : out   std_logic;
      RAMADSCN               : out   std_logic;
      RAMCEN                 : out   std_logic;
      RAMCS0                 : out   std_logic;
      RAMCS1N                : out   std_logic;
      RAMBWEN                : out   std_logic;
      RAMBWN                 : out   std_logic_vector(4 downto 1);
      RAMADVN                : out   std_logic;
      RAMLBON                : out   std_logic;
      RAMZZ                  : out   std_logic;
      RAMADSPN               : out   std_logic);
  end component RAMManager;

  component cvorb_decoder
    port (
      rst_n_i             : in  std_logic;
      clk_i               : in  std_logic;
      enable_i            : in  std_logic;
      data_i              : in  std_logic;
      zero_test_o         : out std_logic;
      one_test_o          : out std_logic;
      strobe_test_o       : out std_logic;
      pulse_width_thres_i : in  std_logic_vector(7 downto 0);
      pulse_width_o       : out std_logic_vector(7 downto 0);
      data_o              : out std_logic_vector(15 downto 0);
      data_valid_o        : out std_logic);
  end component cvorb_decoder;

  component sci_decoder
    port (
      rst_n_i      : in  std_logic;
      clk_i        : in  std_logic;
      enable_i     : in  std_logic;
      data_i       : in  std_logic;
      data_o       : out std_logic_vector(15 downto 0);
      data_valid_o : out std_logic
      );
  end component sci_decoder;

  component rtm_serial_manager
    port (
      rst_n_i                   : in  std_logic;
      clk_i                     : in  std_logic;
      rtm_data_i                : in  std_logic_vector(31 downto 0);
      rtm_data_o                : out rtm_data_array_t;
      rtm_data_valid_o          : out std_logic_vector(31 downto 0);
      mode_i                    : in  std_logic_vector(3 downto 0);
      channel_en_i              : in  std_logic_vector(31 downto 0);
      data_clk_i                : in  std_logic;
      cvorb_pulse_width_thres_i : in  std_logic_vector(7 downto 0);
      cvorb_meas_pulse_width_o  : out cvorb_pulse_width_array_t;
      ram_data_o                : out std_logic_vector(31 downto 0);
      ram_data_valid_o          : out std_logic;
      ram_addr_o                : out std_logic_vector(RAM_ADDR_LENGTH-1 downto 0);
      ram_overflow_o            : out std_logic;
      ram_data_written_i        : in  std_logic;
      reset_ram_addr_i          : in  std_logic
      );
  end component rtm_serial_manager;


end cvora_pkg;


package body cvora_pkg is

end package body cvora_pkg;
