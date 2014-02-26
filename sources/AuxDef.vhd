--      Package File AuDef.vhd
--
--      Purpose: This package defines supplemental types, subtypes,
--               constants, and functions


library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;
use work.IntContPackage.all;
use work.vme.all;
use work.message_package.all;


package AuxDef is


  constant RESET_ACTIVE : std_logic := '0';

  constant NB_RTM_CHANNELS : natural := 32;

  -- Modes
  constant RESERVED1_M     : std_logic_vector(3 downto 0) := "0000";
  constant FP_OP16_SCI_M   : std_logic_vector(3 downto 0) := "0001";
  constant FP_CU16_SCI_M   : std_logic_vector(3 downto 0) := "0010";
  constant CNT32_M         : std_logic_vector(3 downto 0) := "0011";
  constant RTM_PARALLEL_M  : std_logic_vector(3 downto 0) := "0100";
  constant FP_OP32_SCI_M   : std_logic_vector(3 downto 0) := "0101";
  constant FP_CU32_SCI_M   : std_logic_vector(3 downto 0) := "0110";
  constant RTM_SCI_M       : std_logic_vector(3 downto 0) := "0111";
  constant RESERVED2_M     : std_logic_vector(3 downto 0) := "1000";
  constant FP_OP16_CVORB_M : std_logic_vector(3 downto 0) := "1001";
  constant FP_CU16_CVORB_M : std_logic_vector(3 downto 0) := "1010";
  constant CNT2X16_M       : std_logic_vector(3 downto 0) := "1011";
  constant RESERVED3_M     : std_logic_vector(3 downto 0) := "1100";
  constant FP_OP32_CVORB_M : std_logic_vector(3 downto 0) := "1101";
  constant FP_CU32_CVORB_M : std_logic_vector(3 downto 0) := "1110";
  constant RTM_CVORB_M     : std_logic_vector(3 downto 0) := "1111";


  constant PARAMODE     : std_logic_vector(2 downto 0) := "000";
  constant PARAMODE2BIT : std_logic_vector(1 downto 0) := "00";
  constant MODEQUATRE   : std_logic_vector(2 downto 0) := "100";
  constant OP16MODE     : std_logic_vector(2 downto 0) := "001";
  constant OP32MODE     : std_logic_vector(2 downto 0) := "101";
  constant CU16MODE     : std_logic_vector(2 downto 0) := "010";
  constant CU32MODE     : std_logic_vector(2 downto 0) := "110";
  constant BTRAINMODE   : std_logic_vector(2 downto 0) := "011";
  constant P2SERIALMODE : std_logic_vector(2 downto 0) := "111";



  constant RAM_ADDR_LENGTH   : integer                                := 17;                              -- Internal Ram address used (number of bits).
  constant EXTRAM_BUF_ONE    : unsigned(RAM_ADDR_LENGTH - 1 downto 0) := (0      => '1', others => '0');
  constant MEMEMPTY          : unsigned(RAM_ADDR_LENGTH - 1 downto 0) := (3      => '1', others => '0');  -- memory begin at 8
  constant EXTRAMFULL        : unsigned(RAM_ADDR_LENGTH - 1 downto 0) := (others => '1');
  constant QUARTZFREQ        : integer                                := 40;                              -- Define here the FREQUENCE of your clock in MHz
  constant NUMBER_OF_CHANNEL : integer                                := 32;                              -- must always be even



  type rtm_data_array_t is array (0 to NB_RTM_CHANNELS-1) of std_logic_vector(15 downto 0);



  type    array8by8 is array(0 to 7) of std_logic_vector(7 downto 0);
  type    array16by8 is array(0 to 15) of std_logic_vector(7 downto 0);
  type    P2Array is array(1 to NUMBER_OF_CHANNEL) of std_logic_vector(15 downto 0);
  type    P2DataReadyArray is array(1 to NUMBER_OF_CHANNEL) of std_logic;
  subtype P2DataBufferType is std_logic_vector((NUMBER_OF_CHANNEL * 16) - 1 downto 0);
  subtype LWordType is std_logic_vector(31 downto 0);
  subtype UtcScMsType is std_logic_vector(9 downto 0);     -- Sc Millisecond (UTC)
  subtype WordType is std_logic_vector(15 downto 0);
  subtype UtcScTimeType is std_logic_vector(13 downto 0);  -- Sc Tick Counter Format

  -- This funtions fills of zeros the input register giving a 32 bits output
  function Extend32StdLogicVector(S      : std_logic_vector) return std_logic_vector;
  -- This funtions fills of zeros the input register giving a 16 bits output
  function Extend16StdLogicVector(S      : std_logic_vector) return std_logic_vector;
  -- This funtions fills of zeros the input register giving a n bits output
  function ExtendStdLogicVector(signal S : std_logic_vector; constant n : integer) return std_logic_vector;

--****************************************************
--******************* Components *********************
--****************************************************
  component ROC
    port (
      O : out std_logic);
  end component ROC;

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

  component P2SerialManagerSTM is
    port (
      rst              : in  std_logic;
      clk              : in  std_logic;
      Mode             : in  std_logic_vector (2 downto 0);
      DataIn           : in  std_logic_vector(31 downto 0);
      DataInClk        :     std_logic;
      ChannelReg       : in  std_logic_vector(31 downto 0);
      DataWritten      : in  std_logic;  -- the Ram is ready to accept another write
      ClearMem         : in  std_logic;
      WriteRam         : out std_logic;
      WriteAdd         : out std_logic_vector(MEM_ADDRESS_LENGTH - 1 downto 0);
      P2RamAddOverflow : out std_logic;
      DatatoRAM        : out std_logic_vector (31 downto 0));
  end component P2SerialManagerSTM;

  component Serial_Receiver
    port (
      rst           : in  std_logic;
      clk           : in  std_logic;
      mode          : in  std_logic;
      SDataIn1      : in  std_logic;
      SDataIn2      : in  std_logic;
      DataOut       : out std_logic_vector (31 downto 0);
      DataReady     : out std_logic;
      ChannelReady1 : out std_logic;
      ChannelReady2 : out std_logic);
  end component Serial_Receiver;

  component P2Serial_Receiver
    port (
      rst       : in  std_logic;
      clk       : in  std_logic;
      Enable    : in  std_logic;
      SDataIn   : in  std_logic;
      DataOut   : out std_logic_vector (15 downto 0);
      DataReady : out std_logic);
  end component P2Serial_Receiver;

  component LoadDacDelay
    port (
      clk        : in  std_logic;
      inputpulse : in  std_logic;
      output     : out std_logic);
  end component LoadDacDelay;

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

  component VmeIntfceWrapped
    generic (
      AddrWidth        : integer   := 24;
      BaseAddrWidth    : integer   := 8;
      DataWidth        : integer   := 32;
      DirSamePolarity  : std_logic := '0';
      UnalignDataWidth : integer   := 8;
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
      VmeDirTri      : out   std_logic;
      VmeBufOeN      : out   std_logic;
      VmeWriteNA     : in    std_logic;
      VmeLwordNA     : in    std_logic;
      VmeIackNA      : in    std_logic;
      IackOutNA      : out   std_logic;
      IackInNA       : in    std_logic;
      VmeIntReqN     : out   std_logic_vector (7 downto 1);
      vmeDtackNTri   : out   std_logic;
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
      OpFinishedOut    : out std_logic;
      IRQLevelReg      : in  std_logic_vector (3 downto 1);
      IRQStatusIDReg   : in  std_logic_vector (DataWidth-1 downto 0);
      VmeState         : out std_logic_vector (3 downto 0));
  end component VmeIntfceWrapped;

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

  component SimpleFrequencyMeter
    generic (
      QUARTZFREQ : integer := 40;       --en Megahertz
      ACQTIME    : integer := 1000);    --en millisecondesecondes
    port(
      rst                 : in  std_logic;
      clk                 : in  std_logic;
      inclk               : in  std_logic;
      FreqCounterReady    : out std_logic;
      FrequencyCounterReg : out std_logic_vector(31 downto 0));
  end component;

  component Monostable
    generic(
      DURATION              : natural   := 255;
      DEFAULT_PULSEPOLARITY : std_logic := '1');  -- Determine polarity of the inputs
    -- '0' is from TG8 (negative logic), '1' if inputs come from new timing modules
    port (
      clk        : in  std_logic;
      inputpulse : in  std_logic;                 -- A clock wide pulse
      output     : out std_logic);
  end component Monostable;

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
      pulse_width_thres_i : in  std_logic_vector(7 downto 0);
      pulse_width_o       : out std_logic_vector(7 downto 0);
      data_o              : out std_logic_vector(15 downto 0);
      data_valid_o        : out std_logic);
  end component sci_decoder;

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
      rst_n_i            : in  std_logic;
      clk_i              : in  std_logic;
      rtm_data_i         : in  std_logic_vector(31 downto 0);
      mode_i             : in  std_logic;
      channel_en_i       : in  std_logic_vector(31 downto 0);
      data_clk_i         : in  std_logic;
      ram_data_o         : out std_logic_vector(31 downto 0);
      ram_data_valid_o   : out std_logic;
      ram_addr_o         : out std_logic_vector(RAM_ADDR_LENGTH-1 downto 0);
      ram_overflow_o     : out std_logic;
      ram_data_written_i : in  std_logic;
      reset_ram_addr_i   : in  std_logic
      );
  end component rtm_serial_manager;


end AuxDef;


package body auxdef is


  function Extend32StdLogicVector(S : std_logic_vector) return std_logic_vector is
    variable SL : std_logic_vector(31 downto 0);
  begin
    for I in 0 to 31 loop
      if I > (S'left - S'right) then
        SL(I) := '0';
      else
        SL(I) := S(I + S'right);
      end if;
    end loop;
    return SL;
  end Extend32StdLogicVector;

  function Extend16StdLogicVector(S : std_logic_vector) return std_logic_vector is
    variable SL : std_logic_vector(15 downto 0);
  begin
    for I in 0 to 15 loop
      if I > (S'left - S'right) then
        SL(I) := '0';
      else
        SL(I) := S(I + S'right);
      end if;
    end loop;
    return SL;
  end Extend16StdLogicVector;

  function ExtendStdLogicVector(signal S : std_logic_vector; constant n : integer) return std_logic_vector is
    variable SL : std_logic_vector(n - 1 downto 0);
  begin
    for I in SL'range loop
      if I > S'left then
        SL(I) := '0';
      else
        SL(I) := S(I);
      end if;
    end loop;
    return SL;
  end ExtendStdLogicVector;

end package body auxdef;
