----------------------------------------------------------------------------------
-- Company:  CERN
-- Engineer: Philippe Nouchi
--
-- Create Date:    10:30:24 04/25/2006
-- Design Name:    cvora
-- Module Name:    cvora_top - Behavioral
-- Project Name:   cvora
-- Target Devices: Spartan 3
-- Description:    Top level module for the cvora card FPGA
--
-- Dependencies:
--
-- Revision:
-- Revision 0.01 - File Created
--
-- Version 0.02 P.N. le 16-05-2006 VME access in 32 bits data
-- Version 0.03 P.N. le 17-05-2006 Progress on the Btrain Up/Down counter and RS232 Display
-- Version 0.04 P.N. le 20-06-2006 Add Serial Data input and output for the DAC
-- Version 0.05 P.N. le 20-06-2006 Construct cadence composant, so all IOs must be used
-- Version 0.06 P.N. le 21-06-2006 Add the internal RAM
-- Version 0.07 P.N. le 22-06-2006 Reduce Memory --> 16 kLword
-- Version 0.08 P.N. le 22-06-2006 Add External Ram pins
-- Version 0.09 P.N. le 17-10-2006 Add Monostable for leds (300ms)
-- Version 0.10 P.N. le 18-10-2006 Add Base address display (line 2)
-- Version 0.11 P.N. le 19-10-2006 Works on leds and data acquisition signals
-- Version 0.12 P.N. le 23-10-2006 Add input pulses polarity ('0' is from a TG8)
-- Version 0.13 P.N. le 26-10-2006 Add logic for IRQ vector
-- Version 0.14 P.N. le 30-10-2006 Start works on serial inputs
-- Version 1.00 P.N. le 16-11-2006 First compilation with final ucf file
-- Version 1.01 P.N. le 30-01-2007 First compilation with ise9
-- Version 1.02 P.N. le 01-02-2007 Works on monostable part
-- Version 1.03 P.N. le 02-02-2007 Works on polarity of the inputs
-- Version 1.04 P.N. le 05-02-2007 Works on DAC outputs
-- Version 1.05 P.N. le 05-02-2007 Works on Optical fiber inputs and Btrain Counter
-- Version 1.10 P.N. le 06-02-2007 Compilation for the Flash
-- Version 1.11 P.N. le 05-03-2007 Inverse ModuleAddr
-- Version 1.12 P.N. le 06-03-2007 Works on memory
-- Version 1.13 P.N. le 07-03-2007 Compilation with distributed RAM only
-- Version 1.14 P.N. le 07-03-2007 Back to BlockRam + work on the Paralleles inputs
-- Version 1.15 P.N. le 08-03-2007 Value of the address pointer returned in byte (x4)
-- Version 1.16 P.N. le 09-03-2007 Analog outputs is independant of the 16/32 bit mode (serial only)
-- Version 1.17 P.N. le 12-03-2007 Utilisation of the External RAM (idt71v35761)
-- Version 1.18 P.N. le 12-03-2007 Remove the Internal RAM
-- Version 1.19 P.N. le 13-03-2007 modification for the external Ram
-- Version 1.20 P.N. le 13-03-2007 Creation of the LoadDACDelay component
-- Version 1.21 P.N. le 14-03-2007 Works on Analog output
-- Version 1.22 P.N. le 21-03-2007 Modification to use all the external memory 512 kLword
-- Version 1.23 P.N. le 21-03-2007 In serail mode write only the memory on a rising edge of external clock
-- Version 1.24 P.N. le 27-03-2007 Change register mode + change mapping (asked by JMN)
--                                 Add a Software start and stop
-- Version 1.25 P.N. le 28-03-2007 Change memory pointer offset (0x20 to 0x7fffc) and add P2 serial inputs
--                                 in place of paralleles inputs
--                                 Protection againt mask change during acquisition (must be stopped to change the mask)
-- Version 1.26 P.N. le 29-03-2007 Works on mode P2Serial - creation of the component P2SerialManagerSTM for this mode
-- Version 1.27 P.N. le 03-04-2007 Modification of the P2SerialManager component
-- Version 1.28 P.N. le 04-04-2007 Modification in the STM of the P2Serial component and valid all 32 inputs (MASKCHANNEL)
-- Version 1.29 P.N. le 04-04-2007 Add the signal P2MemBusy to be sure no read from VME bus occurs when there is a write in the External memory
-- Version 1.30 P.N. le 05-04-2007 Mods on  serial acquisition - the startpulse don't clear the memory - remove signal P2MemBusy
-- Version 1.31 P.N. le 05-04-2007 Need a time out if one of the rear inputs are not cabled
-- Version 1.32 P.N. le 10-04-2007 Remove of this annoying RAMPar warning during compilation
--                                 and mods in P2SerialManager (see version 1.3 of the component)
-- Version 1.33 P.N. le 18-04-2007 Add a frequency Counter to monitor the external input clock
-- Version 1.34 P.N. le 19-04-2007 Modification of the time acquisition of the frequencymeter
-- Version 1.35 P.N. le 20-04-2007 Modification of external clock rising edge detection for the frequency meter
--                                 The analog outputs show also the inputs one and two of the rear panel in P2SERIALMODE
-- Version 1.36 P.N. le 20-04-2007 Inverse P2 inputs one and two for analog reconstruction in P2SERIALMODE
-- Version 1.37 P.N. le 23-04-2007 Bug in offset address of the memory: fixed
-- Version 1.38 P.N. le 24-04-2007 Bug to read the frequency register: fixed
-- Version 1.39 P.N. le 24-04-2007 Add a spare register
-- Version 1.40 P.N. le 25-04-2007 start Frequency meter with local PPS rising_edge (localPPSRE signal)
-- Version 1.41 P.N. le 26-04-2007 Write a complete new and simple Frequency Meter generic component
-- Version 1.42 P.N. le 26-04-2007 MPPR Compilation and cosmetic change
-- Version 1.43 P.N. le 22-10-2007 Default Pulse change for '0' and some minor change when the board receive command in SOURCEREG
-- Version 1.44 P.N. le 23-10-2007 Wide the Stop and start edge detection
-- Version 1.45 P.N. le 23-10-2007 Add a display line with time between stop and start pulses
-- Version 1.46 P.N. le 23-10-2007 modify the display process  of the line5 line
-- Version 1.47 P.N. le 24-10-2007 Remove unused signals
-- Version 1.48 P.N. le 07-02-2008 Bug found in the parallel mode by JM Nonglaton - the first address memory was not written
-- Version 1.49 P.N. le 08-02-2008 Add Dac Register to monitor P2 Inputs
-- Version 1.50 P.N. le 11-02-2008 Work on memory pointer in the others mode than P2Serial
-- Version 1.51 P.N. le 14-02-2008 Mods in BTRAIN Counter component
-- Version 1.52 P.N. le 19-02-2008 In BTRAIN Counter mode the memory is written only when an external clock pulse arise.
-- Version 1.53 P.N. le 20-02-2008 JMN found a bug in the BTRAIN Mode, works again on this one...
--                                 The memory was not written in a good timing.
--         1.54 MC      23.10.2013 Code clean-up, no functionnal changes.
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;
use work.IntContPackage.all;
use work.message_package.all;
use work.auxdef.all;
use work.vme.all;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
library UNISIM;
use UNISIM.VComponents.all;

entity topdpram is
  generic(
    HARDVERSION           : std_logic_vector(15 downto 0) := X"0154";
    MONOSTABLE_DURATION   : natural                       := 8000000;     -- multiply by clk period - only for leds
    DEFAULT_PULSEPOLARITY : std_logic                     := '0';         -- Determine polarity of the inputs
                                                                          -- '0' is from TG8 (negative logic), '1' if inputs come from new timing modules
    COUNTER_WIDTH         : integer                       := 32;
    DEFAULT_IRQVECTOR     : std_logic_vector(7 downto 0)  := x"86";       -- 134 decimal
    MASKCHANNEL           : std_logic_vector(31 downto 0) := x"ffffffff"  -- all 32 inputs on rear panel
    );

  port (
    rst                 : in  std_logic;                       -- Power-on reset (from TLC7733)
    Clk                 : in  std_logic;                       -- 40MHz clock
    BTrainUp            : in  std_logic;                       -- "BUP" front panel input (pulled-up)
    BTrainDown          : in  std_logic;                       -- "BDOWN" front panel input (pulled-up)
    ExternalClk         : in  std_logic;                       -- "CLOCK" front panel input (pulled-up)
    ExternalReset       : in  std_logic;                       -- "RESET" front panel input (pulled-up)
    Strobe              : in  std_logic;                       -- "STROBE" front panel input (pulled-up)
    Start               : in  std_logic;                       -- "START" front panel input (pulled-up)
    Stop                : in  std_logic;                       -- "STOP" front panel input (pulled-up)
    PDataIn             : in  std_logic_vector (31 downto 0);  -- Data inputs from VME P2 connector
    ModeSelect          : in  std_logic_vector(2 downto 0);    -- Jumper for mode selection
    RS232In             : in  std_logic;                       -- RS232 Rx interface for LCD display
    Rs232Out            : out std_logic;                       -- RS232 Tx interface for LCD display
    Xtest               : out std_logic_vector (3 downto 0);   -- Test points
    LedOut              : out std_logic_vector (7 downto 0);   -- Front panel LEDs
    ParalleleDataOutOne : out std_logic_vector (15 downto 0);  -- Data to DAC, analog output 1 (AD669)
    ParalleleDataOutTwo : out std_logic_vector (15 downto 0);  -- Data to DAC, analog output 2 (AD669)
    ldacOne             : out std_logic;                       -- Load data to DAC, analog output 1 (AD669)
    ldacTwo             : out std_logic;                       -- Load data to DAC, analog output 1 (AD669)
    SData1InOF          : in  std_logic;                       -- Front panel optical input 1
    SData1InCU          : in  std_logic;                       -- Front panel copper input 1
    SData2InOF          : in  std_logic;                       -- Front panel optical input 2
    SData2InCU          : in  std_logic;                       -- Front panel optical input 2
    StrobeOut           : out std_logic;                       -- Not connected on PCB!
    SData1Led           : out std_logic;                       -- Not connected on PCB!
    SData2Led           : out std_logic;                       -- Not connected on PCB!

    -- External RAM IDT71V35761 (Same as CTRV) (66 pins)
    RamAdd     : out   std_logic_vector(16 downto 0);  -- Address Inputs
    RamData    : inout std_logic_vector(31 downto 0);  -- Synchronous data input/output pins
    RamDataPar : out   std_logic_vector(3 downto 0);   -- Data parity (not used)
--  RamZZ      : out   std_logic;                      -- Sleep Mode
    RamOEN     : out   std_logic;                      -- Asynchronous output enable
    RamLBON    : out   std_logic;                      -- Linear Burst Order
    RamGWN     : out   std_logic;                      -- Synchronous global write enable
    RamCEN     : out   std_logic;                      -- Synchronous chip enable
--  RamCS0     : out std_logic;                        -- Synchronous active HIGH chip select
--  RamCS1N    : out std_logic;                        -- Synchronous active LOW chip select
    RamWRN     : out   std_logic;                      -- Synchronous byte write enable
    RamBWN     : out   std_logic_vector(4 downto 1);   -- Individual Write Enables
    RamADVN    : out   std_logic;                      -- Burst Address Advance
    RamADSPN   : out   std_logic;                      -- Address Status (Processor)
    RamADSCN   : out   std_logic;                      -- Address Status (Cache Controller)
    XORamClk40 : out   std_logic;                      -- RAM clock

    -- VME (79 pins)
    VmeReset     : in    std_logic;
    ModuleAddr   : in    std_logic_vector (5 downto 0);  -- VME board address from switch (bits[23:18]), address range =  0x80000 per card
    VmeAddrA     : in    std_logic_vector (23 downto 1);
    VmeAmA       : in    std_logic_vector (5 downto 0);
    VmeAsNA      : in    std_logic;
    VmeBufOeN    : out   std_logic;
    VmeData      : inout std_logic_vector (31 downto 0);
    VmeDir       : out   std_logic;
    VmeDs0NA     : in    std_logic;
    VmeDs1NA     : in    std_logic;
    VmeDtackN    : out   std_logic;
    VmeIackNA    : in    std_logic;
    VmeIackInNA  : in    std_logic;
    VmeIackOutNA : out   std_logic;
    VmeLwordNA   : in    std_logic;
    VmeWriteNA   : in    std_logic;
    VmeIntReqN2  : out   std_logic;                      -- only Interrupt level 2 is used in DPRAM
    XMuxCtrl     : out   std_logic                       -- Interrupt daisy chain mux (connects iackin to iackout when interrupts are disabled)
    );
end topdpram;


architecture Behavioral of topdpram is


----------------------------------------------
-- VME
----------------------------------------------
  signal VmeDataUnAlign            : std_logic_vector(7 downto 0);
  signal moduleAM                  : std_logic_vector(4 downto 0);
  signal iModuleAddr               : std_logic_vector(4 downto 0);
  signal IRQStatusIDReg            : std_logic_vector(31 downto 0);
  signal IrqVector                 : std_logic_vector(7 downto 0) := DEFAULT_IRQVECTOR;
  signal IntProcessed, UserIntReqN : std_logic;
  signal VmeIntReqN                : std_logic_vector (7 downto 1);
  signal EnableInt                 : std_logic                    := '0';
  signal OpFinishedOut             : std_logic;

----------------------------------------------
-- Bus Interface Signals
----------------------------------------------
  signal intRead          : std_logic;       -- Interface Read Signal
  signal intWrite         : std_logic;       -- Interface Write Signal
  signal dataFromInt      : IntDataType;     -- Data From interface
  signal intAdd           : IntAddrOutType;  -- Address From interface
  signal opDone           : std_logic;       -- Operation Done, Read or Write Finished
  signal dataToInt        : IntDataType;     -- Data going from Control to the Interface
  -- Registers  Signals
  signal contToRegs       : contToRegsType;  -- Data going from Control to the Registers
                                             -- This consists of Data + Write Enable Siganal
  signal regsToCont       : RegsToContType;  -- Data Array From the Registers to the Control
  -- Memory Signals
  signal contToMem        : ContToMemType;   -- Data going from Control to the Registers
                                             -- This consists of Data + Enable + Read + Write
  signal memToCont        : MemToContType;   -- Data Array  From the Registers to the Control
                                             -- Data + Done
----------------------------------------------
-- Btrain Up/Down counter Signals
----------------------------------------------
  signal iUDEnable        : std_logic;
  signal LoadValue        : std_logic_vector(COUNTER_WIDTH - 1 downto 0);
  signal UDInstantValue   : std_logic_vector(COUNTER_WIDTH - 1 downto 0);
  signal UDTerminalCount  : std_logic;
  signal UDTerminalCountB : std_logic := '0';
  signal rstA, rstRE      : std_logic := '0';
  signal CounterOverflow  : std_logic;

----------------------------------------------
-- Signal for the display part
----------------------------------------------
  signal message_to_send    : message_array;
  signal localPPS           : std_logic;
  signal igap_counter_value : unsigned(31 downto 0);
  signal en1M               : std_logic;
  signal iledout            : std_logic_vector(7 downto 0);
  signal AcqCounter         : unsigned(15 downto 0)         := (others => '0');
  signal AcquisitionTime    : std_logic_vector(15 downto 0) := (others => '0');
  signal BCD_acq_vec        : BCD_vector_TYPE (4 downto 0);
  signal StartSBAcq         : std_logic;

----------------------------------------------
-- Signal for the External Ram
----------------------------------------------
  signal ExtWriteAdd             : unsigned(MEM_ADDRESS_LENGTH - 1 downto 0)         := MEMEMPTY;
  signal iExtWriteAdd            : std_logic_vector(MEM_ADDRESS_LENGTH - 1 downto 0) := std_logic_vector(MEMEMPTY);
  signal ExtAddRd                : std_logic_vector(18 downto 0);
  signal DataFromHistoryWritten  : std_logic;
  signal DataToContValidRM       : std_logic;
  signal ReadExtRam, WriteExtRam : std_logic                                         := '0';
  signal ExtRdAdd                : std_logic_vector(MEM_ADDRESS_LENGTH - 1 downto 0);
  signal DataToContRM            : std_logic_vector(31 downto 0);
  signal CurrentRecordForRam     : std_logic_vector(31 downto 0)                     := (others => '0');
  signal ExtRamAddOverflow       : std_logic;
  signal ExtrdDel                : std_logic_vector(1 downto 0)                      := (others => '0');  -- Delay to read the External Ram
  signal PdataInValid            : std_logic;
  signal StrobeB, StrobeC        : std_logic                                         := '0';
  signal StrobeRE                : std_logic;
  signal StrobeReg               : std_logic_vector(5 downto 0)                      := (others => '0');

----------------------------------------------
-- Signal for the Monostables
----------------------------------------------
  signal iStart, iStop, iExternalReset, iExternalClk : std_logic;
  signal StartB, StopB, StartC, StopC, StartD, StopD : std_logic := '0';
  signal StartPulse, StopPulse                       : std_logic;
  signal iLoadDAC1                                   : std_logic;
  signal iLoadDAC2                                   : std_logic;

----------------------------------------------
-- Signal for Data Acquisition
----------------------------------------------
  signal AcqEnable                                  : std_logic                     := '0';
  signal PulsePolarity                              : std_logic                     := DEFAULT_PULSEPOLARITY;
  signal SourceWrEn, ModeWrEn                       : std_logic;
  signal SourceWrEnB                                : std_logic                     := '0';
  signal EnableAccess                               : std_logic                     := '1';  -- the module is enable by default
  signal mode, UsedMode                             : std_logic_vector(2 downto 0)  := P2SERIALMODE;
  signal ModeValidFromVME                           : std_logic                     := '0';
  signal ExternalClkB, ExternalClkC, ExternalClkD   : std_logic                     := '0';
  signal iSDataIn1, iSDataIn2                       : std_logic;
  signal PDataFromSerialIn                          : std_logic_vector (31 downto 0);
  signal DataReadyFromSerial, DataReadyFromSerialB  : std_logic;
  signal ChannelReady1, ChannelReady2               : std_logic;
  signal SerialMode                                 : std_logic;
  signal iParalleleDataOutOne, iParalleleDataOutTwo : std_logic_vector(15 downto 0) := (others => '0');
  signal SerialDataReg                              : std_logic_vector(31 downto 0) := (others => '0');
  signal SoftStart, SoftStop, SoftArm               : std_logic                     := '0';

----------------------------------------------
-- Signals for P2 serial inputs
----------------------------------------------
  signal ChannelReg       : std_logic_vector(31 downto 0) := (others => '0');
  signal DacReg, iDacReg  : std_logic_vector(31 downto 0) := (0      => '1', others => '0');
  signal ChannelWr        : std_logic;
  signal DacWr            : std_logic;
  signal P2WriteRam       : std_logic;
  signal P2WriteAdd       : std_logic_vector(MEM_ADDRESS_LENGTH - 1 downto 0);
  signal P2DatatoRAM      : std_logic_vector(31 downto 0);
  signal P2RamAddOverflow : std_logic;
  signal P2DataInClk      : std_logic;
  signal P2ClearMem       : std_logic;
  signal FirstBitatOne    : natural range 0 to 31;

----------------------------------------------
-- Signal for the Frequency Counter
----------------------------------------------
  signal FrequencyCounter1 : std_logic_vector(31 downto 0);
  signal BCD_fq1_vec       : BCD_vector_TYPE (7 downto 0);
  signal ExtClkRE          : std_logic;
  signal FreqCounterReady  : std_logic;
  signal iFCounter         : std_logic_vector(31 downto 0);


begin


-- Polarity of the inputs
  iStart <= start when DEFAULT_PULSEPOLARITY = '1' else                  -- there is a pullup on the start input
            not start;                                                   -- to work always as TTL positive input
  iStop <= stop when DEFAULT_PULSEPOLARITY = '1' else                    -- same
           not stop;
  iExternalReset <= ExternalReset when DEFAULT_PULSEPOLARITY = '1' else  -- same
                    not ExternalReset;
  iExternalClk <= ExternalClk when DEFAULT_PULSEPOLARITY = '1' else      -- there is a pullup on the start input
                  not ExternalClk;                                       -- to work always as TTL positive input
-- now need to detect the rising edge
  process(clk, iStart, iStop)
  begin
    if rising_edge(clk) then
      StartB <= iStart;
      StartC <= StartB;
      StartD <= StartC;
      StopB  <= iStop;
      StopC  <= StopB;
      StopD  <= StopC;
    end if;
  end process;
  StartPulse <= iStart and not StartD;
  StopPulse  <= iStop and not StopD;
  StartSBAcq <= StartC and not StartD;

-- The source register is a control register in write and a status register in read
  SourceWrEn <= contToRegs.Sel(SOURCEREGP) and contToRegs.Wr;
-- bit 0 is the pulse polarity of the inputs
-- bit 1 enable/disable the module
-- bit 2 enable/disable IRQ
-- bit 4-3 not used
-- bit 5 is set during acquisition (between a start and a stop input) - Read only
-- bit 6 is set if there is a Counter oberflow (btrain mode) - read only
-- bit 7 is set if the RAM is overflowed - read only
-- bit 15 downto 8 is the IRQ vector default: 134 decimal, 86 hexa - read write
-- bit 31 downto 16 is the version number (read only)

-- i will try to delay the Write enable to be sure the data is well written
  process (clk, SourceWrEn)
  begin
    if rising_edge(clk) then
      SourceWrEnB <= SourceWrEn;
    end if;
  end process;
  process(clk, SourceWrEnB)
  begin
    if falling_edge(clk) then
      if SourceWrEnB = '1' then
        PulsePolarity <= contToRegs.Data(0);
        EnableAccess  <= contToRegs.Data(1);
        EnableInt     <= contToRegs.Data(2);
        SoftStart     <= contToRegs.Data(3);
        SoftStop      <= contToRegs.Data(4);
        SoftArm       <= contToRegs.Data(5);
        irqVector     <= contToRegs.Data(15 downto 8);
      else
        SoftStart <= '0';
        SoftStop  <= '0';
        SoftArm   <= '0';
      end if;
    end if;
  end process;
  IRQStatusIDReg <= x"000000" & irqVector;

  ModeWrEn <= contToRegs.Sel(MODEREGP) and contToRegs.Wr;
-- bit 2-0 are the mode of the Mode Register :
-- mode 000 --> parallele inputs
-- mode 001 --> Opticals inputs 16 bits - SDataIn2 is ignored
-- mode 101 --> Opticals inputs 32 bits
-- mode 010 --> Copper Inputs 16 bits
-- mode 110 --> Copper Inputs 32 bits
-- mode 011 --> Btrain counter
-- mode 111 --> P2 Serial Inputs
  process(rst, clk, ModeWrEn)
  begin
    if falling_edge(clk) then
      if ModeWrEn = '1' then
        mode             <= contToRegs.Data(2 downto 0);
        ModeValidFromVME <= '1';
      end if;
    end if;
  end process;
  UsedMode <= ModeSelect when ModeValidFromVME = '0' else mode;
-- Data Acuisition ----
  process(rst, clk, StartPulse, StopPulse, PulsePolarity, EnableAccess, SoftStart, SoftStop)
  begin
    if rising_edge(clk) then
      if (StartPulse = '1' or SoftStart = '1') and EnableAccess = '1' then
        AcqEnable <= '1';
      elsif StopPulse = '1' or SoftStop = '1' or EnableAccess = '0' then
        AcqEnable <= '0';
      end if;
    end if;
  end process;
  regsToCont(SOURCEREGP) <= HARDVERSION & IrqVector
                            & ExtRamAddOverflow & CounterOverflow & AcqEnable & "00" & EnableInt & EnableAccess & PulsePolarity;
  regsToCont(MODEREGP) <= x"43564f" & "00000" & UsedMode;  -- 3 lettres ascii CVO + mode

--*******************************
--*        VME Interface        *
--*******************************
  UBusIntControl : BusIntControl
    port map(
      Clk   => clk,
      RstNA => Rst,

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

  moduleAM    <= VmeAmA(5 downto 3) & VmeAmA(1 downto 0);
  iModuleAddr <= not(ModuleAddr(4 downto 0));

  UVmeInt : Vme_intfce
    generic map(
      AddrWidth        => 24,
      BaseAddrWidth    => 5,
      DataWidth        => 32,
      DirSamePolarity  => '0',
      UnalignDataWidth => 8,
      InterruptEn      => '1')
    port map(
      clk              => clk,
      ResetNA          => Rst,
      VmeAddrA         => VmeAddrA,
      VmeAsNA          => VmeAsNA ,
      VmeDs1NA         => VmeDs1NA,
      VmeDs0NA         => VmeDs0NA ,
      VmeData          => VmeData,
      VmeDataUnAlign   => VmeDataUnAlign,
      VmeDir           => VmeDir,
      VmeDirFloat      => open,
      VmeBufOeN        => VmeBufOeN,
      VmeWriteNA       => VmeWriteNA,
      VmeLwordNA       => VmeLwordNA,
      VmeIackNA        => VmeIackNA,
      IackOutNA        => VmeIackOutNA,
      IackInNA         => VmeIackInNA,
      VmeIntReqN       => VmeIntReqN,
      VmeDtackN        => VmeDtackN ,
      ModuleAddr       => iModuleAddr,
      AddrMem          => IntAdd,
      VmeAmA           => moduleAM,
      ReadMem          => IntRead,
      WriteMem         => IntWrite,
      DataFromMemValid => opDone,
      DataFromMem      => dataToInt,
      DataToMem        => dataFromInt,
      IntProcessed     => IntProcessed,
      UserIntReqN      => UserIntReqN,
      UserBlocks       => '0',
      OpFinishedOut    => OpFinishedOut,
      IRQLevelReg      => "010",        --IRQLevelReg, Only level2 is cabled on this card
      IRQStatusIDReg   => IRQStatusIDReg,
      VmeState         => open);


  VmeIntReqN2 <= VmeIntReqN(2);

  INT_REQ_PROC : process(rst, clk, IntProcessed, stopPulse, EnableInt)
  begin
    if rst = RESET_ACTIVE then
      UserIntReqN <= '1';
    elsif rising_edge(clk) then
      if EnableInt = '1' then
        if IntProcessed = '1' then
          UserIntReqN <= '1';
        elsif stopPulse = '1' then
          UserIntReqN <= '0';
        end if;
      else UserIntReqN <= '1';
      end if;
    end if;
  end process;

-- **************************************
-- * Btrain Up-Down Counter
-- **************************************
-- need a rising_edge of the External Reset
  process(clk, iExternalReset)
  begin
    if rising_edge(clk) then
      rstA <= iExternalReset;
    end if;
  end process;
  rstRE <= (iExternalReset and not rstA) or SoftArm;

  iUDEnable <= AcqEnable when UsedMode = BTRAINMODE else '0';
  loadValue <= (others => '0');

  Btrain_UD1 : up_down_counter          -- provisoire the component must be changed
    generic map(
      COUNTER_WIDTH => COUNTER_WIDTH)
    port map (
      rst             => rstRE,
      clk             => Clk,
      clk1            => BTrainUp,
      clk2            => BTrainDown,
      CounterEnable   => iUDEnable,
      loadValue       => loadValue,     -- must be defined
      CounterOverflow => CounterOverflow,
      InstantValue    => UDInstantValue,
      TerminalCount   => UDTerminalCount);

-- *******************************
-- * Serial Data Inputs
-- *******************************
-- bit 2-0 are the mode of the Mode Register :
--      mode 000 0 --> reserved but parallele inputs for the moment
--      mode 100 4 --> parallele inputs
--      mode 001 1 --> Opticals inputs 16 bits - SDataIn2 is ignored
--      mode 101 5 --> Opticals inputs 32 bits
--      mode 010 2 --> Copper Inputs 16 bits
--      mode 110 6 --> Copper Inputs 32 bits
--      mode 011 3 --> Btrain counter
--      mode 111 7 --> P2 Serial Inputs

-- found the two first bits at one in the Dac Register DacReg
-- shift register to find the first bit at one in the register
  process(clk, DacWr)
--variable count : natural range 0 to 31 := 0;
    variable intdacvalue : positive range 1 to 31 := 1;
--variable TestBit : std_ulogic := '0';
  begin
    if rising_edge(clk) then
      FirstBitatOne <= intdacvalue - 1;
      if DacWr = '1' then
        if DacReg(4 downto 0) = "00000" then
          intdacvalue := 1;
        else
          intdacvalue := to_integer(unsigned(DacReg(4 downto 0)));
        end if;
      end if;
    end if;
  end process;


  process(UsedMode)
  begin
    if UsedMode = P2SERIALMODE then
      iSDataIn1 <= PDataIn(FirstBitatOne);
      iSDataIn2 <= PDataIn(FirstBitatOne + 1);
    elsif UsedMode(1 downto 0) = "01" then
      iSDataIn1 <= SData1InOF;
      iSDataIn2 <= SData2InOF;
    elsif UsedMode(1 downto 0) = "10" then
      iSDataIn1 <= not SData1InCu;
      iSDataIn2 <= not SData2InCu;
    else
      iSDataIn1 <= '0';
      iSDataIn2 <= '0';
    end if;
  end process;

  SerialMode <= UsedMode(2);            -- if 0 => mode 16 bits (iSDataIn2 ignored) else mode 32 bits

  Serial_Receiver1 : Serial_Receiver
    port map(
      rst           => rst,
      clk           => clk,
      mode          => SerialMode,
      SDataIn1      => iSDataIn1,
      SDataIn2      => iSDataIn2,
      DataOut       => PDataFromSerialIn,
      DataReady     => DataReadyFromSerial,
      ChannelReady1 => ChannelReady1,
      ChannelReady2 => ChannelReady2
      );

--*******************************
--*****  Channel Register  ******
--*******************************
  ChannelWr <= contToRegs.Sel(CHANNELREGP) and contToRegs.Wr;

  process(clk, ChannelWr, AcqEnable)
  begin
    if falling_edge(clk) then
      if ChannelWr = '1' and AcqEnable = '0' then
        ChannelReg <= contToRegs.Data and MASKCHANNEL;
      end if;
    end if;
  end process;
  regsToCont(CHANNELREGP) <= ChannelReg;

--*******************************
--*****  DAC Register  **********
--*******************************
  DacWr <= contToRegs.Sel(DACREGP) and contToRegs.Wr;

  process(clk, DacWr)
  begin
    if falling_edge(clk) then
      if DacWr = '1' then
        DacReg <= contToRegs.Data;
      end if;
    end if;
  end process;
  regsToCont(DACREGP) <= DacReg;

--******************************
-- Frequency Counter
--******************************
  FREQCOUNTER1 : SimpleFrequencyMeter
    generic map(
      QUARTZFREQ => 40,
      ACQTIME    => 1000)
    port map(
      rst                 => rst,
      clk                 => clk,
      inclk               => ExtClkRE,
      FreqCounterReady    => FreqCounterReady,
      FrequencyCounterReg => FrequencyCounter1);
  iFCounter <= FrequencyCounter1;

  convertor_for_FreqCounter1 : SB_to_BCD
    generic map(32, 8)
    port map(
      Clock      => clk,
      Reset      => rst,
      SB_Ready   => FreqCounterReady,
      SB_Number  => iFCounter,          --FrequencyCounter1,
      BCD_Vector => BCD_fq1_vec,
      BCD_Ready  => open
      );

--*************************************
--*****  Frequency Meter Register *****
--*************************************
  regsToCont(FREQREGP) <= iFCounter;
--regsToCont(SPAREREGP) <= x"ffff" & iFCounter(15 downto 0);

--******************************
-- Counter between a stop and a start
--******************************
  process(clk, ExtClkRE, AcqEnable)
  begin
    if rising_edge(clk) then
      if AcqEnable = '0' then                               -- count External clock pulses between a stop and a start
        if ExtClkRE = '1' then
          AcqCounter <= AcqCounter + 1;
        else
          AcquisitionTime <= std_logic_vector(AcqCounter);  -- store value between incrment
        end if;
      else
        AcqCounter <= (others => '0');                      -- clear the counter
      end if;
    end if;
  end process;

  convertor_for_AcqTime : SB_to_BCD
    generic map(16, 5)
    port map(
      Clock      => clk,
      Reset      => rst,
      SB_Ready   => StartSBAcq,
      SB_Number  => AcquisitionTime,    --FrequencyCounter1,
      BCD_Vector => BCD_acq_vec,
      BCD_Ready  => open
      );

-- *******************************
-- * RS232 Display
-- *******************************
-- first make a pulse per seconde to refresh the display
-- need a 1 MHZ Clock enable
  clk1Mhz : process(rst, clk)
    variable divider40 : natural range 0 to 39;
  begin
    if Rst = RESET_ACTIVE then
      divider40 := 0;
      en1M      <= '0';
    elsif rising_edge(Clk) then
      if divider40 = 39 then
        divider40 := 0;
        en1M      <= '1';
      else
        divider40 := divider40 + 1;
        en1M      <= '0';
      end if;
    end if;
  end process;
  process(rst, clk, en1M)
  begin
    if rst = RESET_ACTIVE then
      localPPS           <= '0';
      igap_counter_value <= (others => '0');
    elsif falling_edge(clk) then
      if en1M = '1' then
        if igap_counter_value = x"0F423E" then  --stop at 999999 us
          igap_counter_value <= (others => '0');
          localPPS           <= '1';
        else
          igap_counter_value <= igap_counter_value + 1;
          localPPS           <= '0';
        end if;
      end if;
    end if;
  end process;

-- construct message array to be sent
--BinLedLoop1: for I in 0 to 15 generate
--      BinSIN1(I) <= x"31" when ChannelReg(I) = '1' else x"30";
--end generate;
--
--BinLedLoop2: for I in 0 to 15 generate
--      BinSIN2(I) <= x"31" when ChannelReg(I+16) = '1' else x"30";
--end generate;

  message_to_send(1) <= (LETTER_C, LETTER_V, LETTER_O, LETTER_R, LETTER_A,  -- CVORA was DPRam
                         space, LETTER_V, LETTER_emin, LETTER_rmin, semicolon,
                         space, X"3" & HARDVERSION(15 downto 12), X"3" & HARDVERSION(11 downto 8),
                         dot, X"3" & HARDVERSION(7 downto 4), X"3" & HARDVERSION(3 downto 0),
                         Carriage_Return, Line_Feed);                       --info line n# 1 to display

  message_to_send(2) <= (space, space,
                         LETTER_B, LETTER_A, semicolon, space, LETTER_0, LETTER_xmin,  -- BASE Address BA: 0x
                         "0011" & iModuleAddr(4 downto 1), "0011" & iModuleAddr(0) & "000",
                         LETTER_0, LETTER_0, LETTER_0, LETTER_0, space, space,
                         Carriage_Return, Line_feed);

  message_to_send(3) <=
    MODEPARA32 when UsedMode(1 downto 0) = PARAMODE2BIT else
    MODEOPT16  when UsedMode = OP16MODE                 else
    MODEOPT32  when UsedMode = OP32MODE                 else
    MODECU16   when UsedMode = CU16MODE                 else
    MODECU32   when UsedMode = CU32MODE                 else
    MODEBTRAIN when UsedMode = BTRAINMODE               else
    MODEP2SERIAL;

  message_to_send(4) <= (
    LETTER_F, letter_1, semicolon,      -- F: "FrequencyCounter"
    space, "0011" & BCD_fq1_vec(7), "0011" & BCD_fq1_vec(6), "0011" & BCD_fq1_vec(5),
    "0011" & BCD_fq1_vec(4), "0011" & BCD_fq1_vec(3), dot,
    "0011" & BCD_fq1_vec(2), "0011" & BCD_fq1_vec(1),
    "0011" & BCD_fq1_vec(0), letter_K, letter_hmin, letter_zmin,
    Carriage_Return, Line_feed);

  message_to_send(5) <= (
--                                        BinSin1(15), BinSin1(14), BinSin1(13), BinSin1(12), BinSin1(11), BinSin1(10), BinSin1(9), BinSin1(8),
--                                        BinSin1(7), BinSin1(6), BinSin1(5), BinSin1(4), BinSin1(3), BinSin1(2), BinSin1(1), BinSin1(0),
    LETTER_A, LETTER_C, LETTER_Q, semicolon,
    space, "0011" & BCD_acq_vec(4), "0011" & BCD_acq_vec(3),
    "0011" & BCD_acq_vec(2), "0011" & BCD_acq_vec(1), "0011" & BCD_acq_vec(0),
    space, Letter_xmin, space, LETTER_C, Letter_lmin, Letter_kmin,
    Carriage_Return, Line_feed);
--
--
--message_to_send(6) <= (
--                                        BinSin2(15), BinSin2(14), BinSin2(13), BinSin2(12), BinSin2(11), BinSin2(10), BinSin2(9), BinSin2(8),
--                                        BinSin2(7), BinSin2(6), BinSin2(5), BinSin2(4), BinSin2(3), BinSin2(2), BinSin2(1), BinSin2(0),
--                 Carriage_Return, Line_feed);



  MESSAG1 : message
    port map(
      rst         => rst,
      clk         => clk,
      rs232_start => localPPS,
      message     => message_to_send,
      message_env => open,
      rs232out    => rs232out
      );

--*******************************
--*****   External RAM     ******
--*******************************
  process(Rst, clk, iExternalClk, StrobeRE)
  begin
    if rising_edge(clk) then
      StrobeReg    <= StrobeReg(StrobeReg'left - 1 downto 0) & (StrobeRE or ExtClkRE);  -- Valid data after 100ns
      StrobeB      <= Strobe;
      StrobeC      <= StrobeB;
      ExternalClkB <= iExternalClk;
      ExternalClkC <= ExternalClkB;
      ExternalClkD <= ExternalClkC;
    end if;
  end process;
  ExtClkRE             <= ExternalClkB and not(ExternalClkC);
  StrobeRE             <= StrobeB and not StrobeC;                                      -- Rising edge of stobe input
  PdataInValid         <= ExternalClkB and not(ExternalClkC);                           -- 1rst front of the input pulse clock
  DataReadyFromSerialB <= ExternalClkC and not(ExternalClkD);                           -- 1rst front of the input pulse clock
  process(clk, PdataInValid, SerialMode)
  begin
    if rising_edge(clk) then
      if PdataInValid = '1' then
        if SerialMode = '0' then
          SerialDataReg <= x"0000" & PDataFromSerialIn(15 downto 0);
        else SerialDataReg <= PDataFromSerialIn;
        end if;
      end if;
    end if;
  end process;

  process(clk, AcqEnable, StrobeReg, PulsePolarity, UsedMode, ExtRamAddOverflow)
  begin
    if rising_edge(clk) then
      if AcqEnable = '1' and ExtRamAddOverflow = '0' then
        if UsedMode = P2SERIALMODE then
          WriteExtRam <= P2WriteRam;
        else
          WriteExtRam <= StrobeReg(4);  -- or DataReadyFromSerialB;
        end if;
      else WriteExtRam <= '0';
      end if;
    end if;
  end process;

  P2DataInClk <= PdataInValid and AcqEnable;
-- the startpulse don't clear the memory
  P2ClearMem  <= RstRE when AcqEnable = '0' else '0';  -- the memory can't be cleared during acquisition
-- Write the memory depending the UsedMode
  P2SerialManager : P2SerialManagerSTM
    port map(
      rst              => rst,
      clk              => clk,
      Mode             => UsedMode,
      DataIn           => PDataIn,
      DataInClk        => P2DataInClk,
      ChannelReg       => ChannelReg,
      DataWritten      => DataFromHistoryWritten,      -- the Ram is ready to accept another write
      ClearMem         => P2ClearMem,
      WriteRam         => P2WriteRam,
      WriteAdd         => P2WriteAdd,
      P2RamAddOverflow => P2RamAddOverflow,
      DatatoRAM        => P2DatatoRAM
      );

  process(rst, clk, PDataIn, mode, UsedMode, StartPulse, AcqEnable, rstRE, StrobeReg)
    variable counter : integer := 0;
    variable offset  : integer := 0;
  begin
    if rising_edge(clk) then
      if UsedMode = P2SERIALMODE then
        CurrentRecordForRam <= P2DatatoRAM;
        ExtWriteAdd         <= unsigned(P2WriteAdd);
        ExtRamAddOverflow   <= P2RamAddOverflow;
      elsif AcqEnable = '1' then                                                   -- a startpulse has no action during acquisition
        if ExtWriteAdd = EXTRAMFULL then
          ExtRamAddOverflow <= '1';                                                -- The ram is full
        elsif ExtClkRE = '1' and UsedMode = BTRAINMODE then
          CurrentRecordForRam <= UDInstantValue;
        elsif StrobeReg(2) = '1' and UsedMode(1 downto 0) = PARAMODE2BIT then
          CurrentRecordForRam <= PDataIn;
        elsif StrobeReg(2) = '1' and UsedMode /= BTRAINMODE then                   -- an external clock has occured
          CurrentRecordForRam <= SerialDataReg;
        elsif StrobeReg(StrobeReg'left) = '1' then
          ExtWriteAdd <= ExtWriteAdd + EXTRAM_BUF_ONE;
        end if;
      elsif StartPulse = '1' or rstRE = '1' then                                   -- reset the address counter
        ExtWriteAdd       <= MEMEMPTY;
        ExtRamAddOverflow <= '0';
      end if;
    end if;
  end process;
  regsToCont(READADDREGP) <= x"000" & "0" & std_logic_vector(ExtWriteAdd) & "00";  --Read the address counter register in byte
-- read the External RAM from VME
  ReadExtRam              <= ContToMem.Rd and ContToMem.SelectedPos(EXTRAMP);
  ExtAddRd                <= ContToMem.Add;  --(ContToMem.Add'left downto 0); -- NO OFFSET in DPRam card

  process(Rst, clk, ReadExtRam, ExtAddRd, DataToContValidRM)
  begin
    if rising_edge (clk) then
      if ReadExtRam = '1' then
        ExtRdAdd <= ExtAddRd(18 downto 2);  -- all memory used
      end if;
      MemToCont(EXTRAMP).Data <= DataToContRM;

      ExtrdDel(0)                      <= DataToContValidRM;
      ExtrdDel(ExtrdDel'left downto 1) <= ExtrdDel(ExtrdDel'left - 1 downto 0);
    end if;
  end process;
  MemToCont(EXTRAMP).RdDone <= ExtrdDel(ExtrdDel'left);


  URAMManager : RAMManager
    port map(
      RstN => rst,
      Clk  => clk,

      -- Interface with History block
      DataFromHistory        => CurrentRecordForRam,
      AddrFromHistory        => iExtWriteAdd,
      WriteFromHistory       => WriteExtRam,
      DataFromHistoryWritten => DataFromHistoryWritten,
      -- Interface with Bus Control block
      AddrFromCont           => ExtRdAdd,
      ReadFromCont           => Readextram,
      DataToCont             => DataToContRM,
      DataToContValid        => DataToContValidRM,
      -- Interface with IDT 71V35761 RAM chip
      RAMAddr                => RAMAdd,
      RAMData                => RAMData,
      RAMOEN                 => RAMOEN,
      RAMGWN                 => RAMGWN,
      RAMADSCN               => RAMADSCN,

      -- Other IDT 71V35761 that we don't really use
      RAMCEN   => RAMCEN,
      RAMCS0   => open,
      RAMCS1N  => open,
      RAMBWEN  => RAMWRN,
      RAMBWN   => RAMBWN,
      RAMADVN  => RAMADVN,
      --         RAMPar => iRamDataPar,   -- to be left floating
      RAMLBON  => RAMLBON,
      RAMZZ    => open,
      RAMADSPN => RAMADSPN);

  XORamClk40   <= not clk;
  iExtWriteAdd <= std_logic_vector(ExtWriteAdd);

--*******************************
--*****Monostables for leds******
--*******************************
  MONOSTABLE1 : Monostable              -- led stop
    generic map(
      DURATION              => MONOSTABLE_DURATION,
      DEFAULT_PULSEPOLARITY => DEFAULT_PULSEPOLARITY)
    port map(
      clk        => clk,
      inputpulse => StopPulse,          -- A clock wide pulse
      output     => iledout(2));
  MONOSTABLE2 : Monostable              -- led start
    generic map(
      DURATION              => MONOSTABLE_DURATION,
      DEFAULT_PULSEPOLARITY => DEFAULT_PULSEPOLARITY)
    port map(
      clk        => clk,
      inputpulse => StartPulse,         -- A clock wide pulse
      output     => iledout(3));
  MONOSTABLE3 : Monostable              -- led read
    generic map(
      DURATION              => MONOSTABLE_DURATION,
      DEFAULT_PULSEPOLARITY => DEFAULT_PULSEPOLARITY)
    port map(
      clk        => clk,
      inputpulse => ReadExtRam,         -- A clock wide pulse
      output     => iledout(4));
  MONOSTABLE4 : Monostable              -- led write
    generic map(
      DURATION              => MONOSTABLE_DURATION,
      DEFAULT_PULSEPOLARITY => DEFAULT_PULSEPOLARITY)
    port map(
      clk        => clk,
      inputpulse => WriteExtRam,        -- A clock wide pulse
      output     => iledout(5));

--*******************************
--*****  Load of the DACs  ******
--*******************************
-- ldacone must be 45 ns minimum duration and start after 100ns
  iLoadDAC1 <= ChannelReady1 when UsedMode(1 downto 0) = "01" or UsedMode = "10" or UsedMode = P2SERIALMODE else
               UDTerminalCount when UsedMode = BTRAINMODE               else
               StrobeReg(4)    when UsedMode(1 downto 0) = PARAMODE2BIT else
               '0';

  LoadDAC1 : LoadDacDelay
    port map(
      clk        => clk,
      inputpulse => iLoadDAC1,          -- A clock wide pulse
      output     => ldacOne);

  iLoadDAC2 <= ChannelReady2 when UsedMode(1 downto 0) = "01" or UsedMode = "10" or UsedMode = P2SERIALMODE else
               UDTerminalCount when UsedMode = BTRAINMODE               else
               StrobeReg(4)    when UsedMode(1 downto 0) = PARAMODE2BIT else
               '0';

  LoadDAC2 : LoadDacDelay
    port map(
      clk        => clk,
      inputpulse => iLoadDAC2,          -- A clock wide pulse
      output     => ldacTwo);

--pipeline outputs
  process (clk, PDataFromSerialIn, UsedMode, PdataIn, ChannelReady1, ChannelReady2, UDTerminalCount,
           UDInstantValue, StrobeReg, P2WriteRam, P2DatatoRam)
  begin
    if rising_edge(clk) then
      if UDTerminalCount = '1' and UsedMode = BTRAINMODE then                -- Btrain Counter
        iParalleleDataOutOne <= UDInstantValue(15 downto 0);
        iParalleleDataOutTwo <= UDInstantValue(15 downto 0);                 -- same as analog out1 for the moment to test the outputs
      elsif StrobeReg(2) = '1' and UsedMode(1 downto 0) = PARAMODE2BIT then  -- mode parallele
        iParalleleDataOutOne <= PdataIn(15 downto 0);
        iParalleleDataOutTwo <= PdataIn(31 downto 16);
      else
        if ChannelReady1 = '1' then
          iParalleleDataOutOne <= PDataFromSerialIn(15 downto 0);
        end if;
        if ChannelReady2 = '1' then
          iParalleleDataOutTwo <= PDataFromSerialIn(31 downto 16);
        end if;
      end if;
    end if;
  end process;
  ParalleleDataOutOne <= iParalleleDataOutOne;
  ParalleleDataOutTwo <= iParalleleDataOutTwo;
  iledOut(0)          <= ExtRamAddOverflow or CounterOverflow;
  iledOut(1)          <= EnableInt;
  iledOut(6)          <= AcqEnable;
  iledOut(7)          <= not EnableAccess;
  ledOut              <= iledOut;
  SData1Led           <= iSDataIn1;
  SData2Led           <= iSDataIn2;
  StrobeOut           <= Strobe;
  XMuxCtrl            <= not EnableInt;

  Xtest(3) <= iLoadDAC1;
  Xtest(2) <= DataFromHistoryWritten;
  Xtest(0) <= UDTerminalCount;
  Xtest(1) <= RS232In or VmeamA(2) or vmeReset or ModuleAddr(5);

  RamDataPar <= (others => 'Z');


end Behavioral;

