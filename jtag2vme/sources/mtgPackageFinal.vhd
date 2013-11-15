------------------------------------------
------------------------------------------
-- Date        : Wed Apr 10 15:07:29 2002
--
-- Author      : David Dominguez
--
-- Company     : CERN
--
-- Description : MTG constants and parameters
--
------------------------------------------
------------------------------------------
library ieee;
use ieee.std_logic_1164.all;


package  mtg  is


-- Note that each of the 2 VME interfaces inside the CTG have different memory sizes mapped into them
-- Also, when the DSC is changing the Prog Counter, the latest refers to 64-bit words, whereas the 
-- memory is organised in 32bit-words. This obliges to define different generics refering to the memory size

------------------------
--  START   VME INTERFACE
--
------------------------

     -- Address Modifier to be used in the module

 -- ModuleAm :std_logic_vector (4 downto 0):= "11110" is generated 
 -- automaticaly in the VME interface according to the AddressLength
 -- generic parameter. Two moduleAm are generated: for word by word
 -- data transfer (ModuleAmNormal) or block data transfer(ModuleAmBlock)


-- line AM(2) from VME bus is not taken into account
-- So moduleAm(4): AM (5) from VME bus
--    moduleAm(3): AM(4) from VME
--    moduleAm(2): AM(3) from VME
--    moduleAm(1): AM(1) from VME
--    moduleAm(0): AM(0) from VME

------------------------
--  END   VME INTERFACE
--
------------------------


------------------------
--  START JTAG REGISTER
--
------------------------

constant JTAGAddr :std_logic_vector (7 downto 1):= "0000000";


------------------------
--  END JTAG REGISTER
--
------------------------



------------------------
--  START  MAIN LOGIC REGISTERS DECLARATION
--
------------------------

subtype MaxIntType is integer range 0 to 127;
subtype AddrType is integer range 0 to 16#1FFFFF#;

--Addresses of the Registers to be implemented (A20 downto 0)
-- NOTE: A0 is already taken into account: last 2 bits from these addresses will be ignored
-- inside the FPGA (32-bits data words are used, instead of 8-bits words (VME spec for addressing).
-- All addresses are divided by 2 because bit A0 is not transmitted in the VME bus, but implied
-- in Ds0N,etc. I.E.: for the IENR Register:
--       The software (DSC task/driver) will address 16#00008#/2 BUT
--       The FPGA will always receive and work with 16#000004# (A20 downto A1)

-- The whole adress map of the CTG is divided into 3 different spaces (SRAM chips, Slices, BlockRam):
-- Address: FirstRegAddr to FirstBlkRamRegAddr -1   : Registers contained in Slices (inside the FPGA)
-- Address:FirstBlkRamRegAddr to LastBlkRamRegAddr  : REgisters inside a blockRam (inside the FPGA)
-- Address:LastBlkRamRegAddr+1 to LastRegAddr       : Registers contained in Slices (inside the FPGA)
-- Address:FirstRAMAddr to LastRAMAddr              : SRAM (chip outside FPGA)

constant FirstRegAddr       :AddrType := 16#000000#/2;
constant FirstBlkRamRegAddr :AddrType := 16#00000C#/2;
constant LastBlkRamRegAddr  :AddrType := 16#000188#/2;   --The last register actually implemented, not the address of the highest position in the blkRAM
constant LastRegAddr        :AddrType := 16#0FFFFC#/2;   --The highest address available for registers (even if it is not used at the moment)
                                                         -- to differenciate registers and SRAM
constant FirstRAMAddr       :AddrType := 16#100000#/2;
constant LastRAMAddr        :AddrType := 16#1FFFFC#/2;

constant NumSliceRegs       : MaxIntType:=17;
constant NumBlkRamRegs      : MaxIntType:=96;

constant DataRegLength :MaxIntType :=31;
constant AddressBits: MaxIntType :=18;
   -- This is the number of bits contained in the address bus that connects the VME
   -- interface to the memory chips/embeded memory (SRAM,DPM,...)
   --I.E.: 1st version of CTG: use A23, base address:3 bit, Addr(20):selects memory/register
   -- so AddressBit=23 -3 -1= 18 bits

constant BlkRegAddrbits: MaxIntType :=7;  -- this defines the width of the Addr bus entering the
--blkRAM that contain the BlockRamRegisters. If it changes, the primitive will probably
-- change as well (RAMB4_S16_s16). A value of 7 stands for 8 bits width:(7 downto 0)


--The following array is to connect all registers outputs to the VME interface (interface automation in the top design)
-- and to create the Slice registers (distributed flipflops)
type DataSliceRegArray is array (0 to NumSliceRegs -1) of std_logic_vector(DataRegLength downto 0);

 -- The following is a general purpose array
type signalSliceRegArray is array (0 to NumSliceRegs -1) of std_logic;


type RegType is
record
   Data    : std_logic_vector (DataRegLength downto 0);
   Address : AddrType;
   clkenab : std_logic;
   ResetNA : std_logic;
end record;

type CountRegType is
record
	DataIn      : std_logic_vector (DataRegLength downto 0);
	DataOut 	: std_logic_vector (DataRegLength downto 0);
	Load		: std_logic;
	clk  		: std_logic;
	clkenab	: std_logic;
	ResetN	: std_logic;   --synchronous Reset
end record;

  
type RegArrayType is array (0 to NumSliceRegs -1) of RegType;


		--Interrupt Source Register
constant ISCRAdd   :AddrType  := 16#00000#/2;
		--Alarms Register
constant ALMRAdd     :AddrType := 16#00004#/2;
		--Interrupt Enable Register
constant IENRAdd   :AddrType  := 16#00008#/2;
		--Interrupt Time Registers (32-bits in miliseconds [UTCR1]

constant ITIME0Add :AddrType  := 16#0000C#/2;
constant ITIME1Add :AddrType  := 16#00010#/2;
constant ITIME2Add :AddrType  := 16#00014#/2;
constant ITIME3Add :AddrType  := 16#00018#/2;
constant ITIME4Add :AddrType  := 16#0001C#/2;
constant ITIME5Add :AddrType  := 16#00020#/2;
constant ITIME6Add :AddrType  := 16#00024#/2;
constant ITIME7Add :AddrType  := 16#00028#/2;
constant ITIME8Add :AddrType  := 16#0002C#/2;
constant ITIME9Add :AddrType  := 16#00030#/2;
constant ITIME10Add :AddrType := 16#00034#/2;
constant ITIME11Add :AddrType := 16#00038#/2;
constant ITIME12Add :AddrType := 16#0003C#/2;
constant ITIME13Add :AddrType := 16#00040#/2;
constant ITIME14Add :AddrType := 16#00044#/2;
constant ITIME15Add :AddrType := 16#00048#/2;
constant ITIME16Add :AddrType := 16#0004C#/2;
constant ITIME17Add :AddrType := 16#00050#/2;
constant ITIME18Add :AddrType := 16#00054#/2;
constant ITIME19Add :AddrType := 16#00058#/2;
constant ITIME20Add :AddrType := 16#0005C#/2;
constant ITIME21Add :AddrType := 16#00060#/2;
constant ITIME22Add :AddrType := 16#00064#/2;
constant ITIME23Add :AddrType := 16#00068#/2;
constant ITIME24Add :AddrType := 16#0006c#/2;
constant ITIME25Add :AddrType := 16#00070#/2;
constant ITIME26Add :AddrType := 16#00074#/2;
constant ITIME27Add :AddrType := 16#00078#/2;
constant ITIME28Add :AddrType := 16#0007C#/2;
constant ITIME29Add :AddrType := 16#00080#/2;
constant ITIME30Add :AddrType := 16#00084#/2;
constant ITIME31Add :AddrType := 16#00088#/2;

     -- External Event Instruction Registers (opcode, Argument)

constant EEIR0OpAdd :AddrType  := 16#0008C#/2;
constant EEIR1OpAdd :AddrType  := 16#00090#/2;
constant EEIR2OpAdd :AddrType  := 16#00094#/2;
constant EEIR3OpAdd :AddrType  := 16#00098#/2;
constant EEIR4OpAdd :AddrType  := 16#0009C#/2;
constant EEIR5OpAdd :AddrType  := 16#000A0#/2;
constant EEIR6OpAdd :AddrType  := 16#000A4#/2;
constant EEIR7OpAdd :AddrType  := 16#000A8#/2;
constant EEIR8OpAdd :AddrType  := 16#000AC#/2;
constant EEIR9OpAdd :AddrType  := 16#000B0#/2;
constant EEIR10OpAdd :AddrType := 16#000B4#/2;
constant EEIR11OpAdd :AddrType  := 16#000B8#/2;
constant EEIR12OpAdd :AddrType  := 16#000BC#/2;
constant EEIR13OpAdd :AddrType  := 16#000C0#/2;
constant EEIR14OpAdd :AddrType  := 16#000C4#/2;
constant EEIR15OpAdd :AddrType  := 16#000C8#/2;
constant EEIR16OpAdd :AddrType  := 16#000CC#/2;
constant EEIR17OpAdd :AddrType  := 16#000D0#/2;
constant EEIR18OpAdd :AddrType  := 16#000D4#/2;
constant EEIR19OpAdd :AddrType  := 16#000D8#/2;
constant EEIR20OpAdd :AddrType  := 16#000DC#/2;
constant EEIR21OpAdd :AddrType  := 16#000E0#/2;
constant EEIR22OpAdd :AddrType  := 16#000E4#/2;
constant EEIR23OpAdd :AddrType  := 16#000E8#/2;
constant EEIR24OpAdd :AddrType  := 16#000EC#/2;
constant EEIR25OpAdd :AddrType  := 16#000F0#/2;
constant EEIR26OpAdd  :AddrType := 16#000F4#/2;
constant EEIR27OpAdd :AddrType  := 16#000F8#/2;
constant EEIR28OpAdd :AddrType  := 16#000FC#/2;
constant EEIR29OpAdd :AddrType  := 16#00100#/2;
constant EEIR30OpAdd :AddrType  := 16#00104#/2;
constant EEIR31OpAdd :AddrType  := 16#00108#/2;

constant EEIR0ArgAdd :AddrType :=  16#0010C#/2;
constant EEIR1ArgAdd :AddrType :=  16#00110#/2;
constant EEIR2ArgAdd :AddrType :=  16#00114#/2;
constant EEIR3ArgAdd :AddrType :=  16#00118#/2;
constant EEIR4ArgAdd :AddrType :=  16#0011C#/2;
constant EEIR5ArgAdd :AddrType :=  16#00120#/2;
constant EEIR6ArgAdd :AddrType :=  16#00124#/2;
constant EEIR7ArgAdd :AddrType :=  16#00128#/2;
constant EEIR8ArgAdd :AddrType :=  16#0012C#/2;
constant EEIR9ArgAdd :AddrType :=  16#00130#/2;
constant EEIR10ArgAdd :AddrType := 16#00134#/2;
constant EEIR11ArgAdd :AddrType := 16#00138#/2;
constant EEIR12ArgAdd :AddrType := 16#0013C#/2;
constant EEIR13ArgAdd :AddrType := 16#00140#/2;
constant EEIR14ArgAdd :AddrType := 16#00144#/2;
constant EEIR15ArgAdd :AddrType := 16#00148#/2;
constant EEIR16ArgAdd :AddrType := 16#0014C#/2;
constant EEIR17ArgAdd :AddrType := 16#00150#/2;
constant EEIR18ArgAdd :AddrType := 16#00154#/2;
constant EEIR19ArgAdd :AddrType := 16#00158#/2;
constant EEIR20ArgAdd :AddrType := 16#0015C#/2;
constant EEIR21ArgAdd :AddrType := 16#00160#/2;
constant EEIR22ArgAdd :AddrType := 16#00164#/2;
constant EEIR23ArgAdd :AddrType := 16#00168#/2;
constant EEIR24ArgAdd :AddrType := 16#0016C#/2;
constant EEIR25ArgAdd :AddrType := 16#00170#/2;
constant EEIR26ArgAdd :AddrType := 16#00174#/2;
constant EEIR27ArgAdd :AddrType := 16#00178#/2;
constant EEIR28ArgAdd :AddrType := 16#0017C#/2;
constant EEIR29ArgAdd :AddrType := 16#00180#/2;
constant EEIR30ArgAdd :AddrType := 16#00184#/2;
constant EEIR31ArgAdd :AddrType := 16#00188#/2;


     --External Event Enable Mask
constant EXNRAdd    :AddrType   := 16#0018C#/2;

		--Module Control Value Register
constant MCVRAdd     :AddrType := 16#00190#/2;

		--Module Control Register
constant MCRAdd      :AddrType := 16#00194#/2;

		--Module SetUp Register (moduleID, IntLevel, StatusId,…)
constant MSURAdd        :AddrType  := 16#00198#/2;  

		--Module Runtime Register (Realtime mask)
constant MRTRAdd     :AddrType := 16#0019C#/2;

		--Module Status Register
constant MSRAdd      :AddrType := 16#001A0#/2;

     --Manchester Ticks Number Register
constant MTNRAdd        :AddrType := 16#001A4#/2; 

		--Output Delay Register
constant ODRAdd      :AddrType := 16#001A8#/2; 
		--Sync Period Register
constant SPRAdd      :AddrType := 16#001AC#/2; 
		--Universal Time Register (counter)
constant UTCR1Add    :AddrType := 16#001B0#/2;	
constant UTCR2Add    :AddrType := 16#001B4#/2;  
		--Time Since last SYNC Register (counter)
constant TSYNCAdd    :AddrType := 16#001B8#/2;
		--Milisecond Modulo Register(in either BCD or Binary format) (counter)
constant MSMRAdd     :AddrType := 16#001BC#/2;
		--Program Counter Register for event table (counter)
constant EPCRAdd     :AddrType := 16#001C0#/2;	




----The following code creates an indexed array with all the registers addresses.

type SliceAddrArray is array (0 to NumSliceRegs-1) of AddrType;
constant SliceRegAddrArray  :SliceAddrArray :=
 (ISCRAdd   , IENRAdd , EXNRAdd,
 MSURAdd , MRTRAdd, UTCR1Add,
 UTCR2Add ,TSYNCAdd,
 MSMRAdd,  SPRAdd, EPCRAdd,
 MCRAdd, MCVRAdd,  MSRAdd,
 ALMRAdd, ODRAdd,  MTNRAdd );




------------------------
--  END MAIN LOGIC REGISTERS DECLARATION
--
------------------------




------------------------
--  Start ControlMachine LOGIC DECLARATION
--
------------------------

--constant NumOfExternalEvents :MaxIntType :=32;
--type EEIRArray is array (0 to NumOfExternalEvents -1) of std_logic_vector (DataRegLength downto 0);




type CmdStateType is (Init, FrameOutS, GoTo, HaltS, ResetLogic, Run,
                              SelfTest, SetUTC, SingleStepS, SetMSUR, WaitCmd,SetEEOpS,SetEEArgS,
                      SetOutDelayS, SetMTNRS, SetSPRS );


constant CmdLength :MaxIntType :=6;  --(number of bits -1) composing the command for the ControlStateMAchine


-- Commands for the Control Machine  (DO NOT CHANGE THE CODIFICATION FOR SETEEOPCMD AND SETEEARGCMD
-- SINCE SOME SIGNALS ARE HARDWIRED TO THEESE BITS FOR optimizing speed and logic. Ex: seteeOp15Cmd="0001111"
-- "XX01111" codifies the number of the register to be loaded(in this case: 15)


constant SetEEOp0Cmd         :std_logic_vector (CmdLength downto 0)  :="0000000";
constant SetEEOp1Cmd         :std_logic_vector (CmdLength downto 0)  :="0000001";
constant SetEEOp2Cmd         :std_logic_vector (CmdLength downto 0)  :="0000010";
constant SetEEOp3Cmd         :std_logic_vector (CmdLength downto 0)  :="0000011";
constant SetEEOp4Cmd         :std_logic_vector (CmdLength downto 0)  :="0000100";
constant SetEEOp5Cmd         :std_logic_vector (CmdLength downto 0)  :="0000101";
constant SetEEOp6Cmd         :std_logic_vector (CmdLength downto 0)  :="0000110";
constant SetEEOp7Cmd         :std_logic_vector (CmdLength downto 0)  :="0000111";
constant SetEEOp8Cmd         :std_logic_vector (CmdLength downto 0)  :="0001000";
constant SetEEOp9Cmd         :std_logic_vector (CmdLength downto 0)  :="0001001";
constant SetEEOp10Cmd         :std_logic_vector (CmdLength downto 0) :="0001010";
constant SetEEOp11Cmd         :std_logic_vector (CmdLength downto 0) :="0001011";
constant SetEEOp12Cmd         :std_logic_vector (CmdLength downto 0) :="0001100";
constant SetEEOp13Cmd         :std_logic_vector (CmdLength downto 0) :="0001101";
constant SetEEOp14Cmd         :std_logic_vector (CmdLength downto 0) :="0001110";
constant SetEEOp15Cmd         :std_logic_vector (CmdLength downto 0) :="0001111";
constant SetEEOp16Cmd         :std_logic_vector (CmdLength downto 0)  :="0010000";
constant SetEEOp17Cmd         :std_logic_vector (CmdLength downto 0)  :="0010001";
constant SetEEOp18Cmd         :std_logic_vector (CmdLength downto 0)  :="0010010";
constant SetEEOp19Cmd         :std_logic_vector (CmdLength downto 0)  :="0010011";
constant SetEEOp20Cmd         :std_logic_vector (CmdLength downto 0)  :="0010100";
constant SetEEOp21Cmd         :std_logic_vector (CmdLength downto 0)  :="0010101";
constant SetEEOp23Cmd         :std_logic_vector (CmdLength downto 0)  :="0010111";
constant SetEEOp22Cmd         :std_logic_vector (CmdLength downto 0)  :="0010110";
constant SetEEOp24Cmd         :std_logic_vector (CmdLength downto 0)  :="0011000";
constant SetEEOp25Cmd         :std_logic_vector (CmdLength downto 0)  :="0011001";
constant SetEEOp26Cmd         :std_logic_vector (CmdLength downto 0) :="0011010";
constant SetEEOp27Cmd         :std_logic_vector (CmdLength downto 0) :="0011011";
constant SetEEOp28Cmd         :std_logic_vector (CmdLength downto 0) :="0011100";
constant SetEEOp29Cmd         :std_logic_vector (CmdLength downto 0) :="0011101";
constant SetEEOp30Cmd         :std_logic_vector (CmdLength downto 0) :="0011110";
constant SetEEOp31Cmd         :std_logic_vector (CmdLength downto 0) :="0011111";

constant SetEEArg0Cmd         :std_logic_vector (CmdLength downto 0) :="0100000";
constant SetEEArg1Cmd         :std_logic_vector (CmdLength downto 0) :="0100001";
constant SetEEArg2Cmd         :std_logic_vector (CmdLength downto 0) :="0100010";
constant SetEEArg3Cmd         :std_logic_vector (CmdLength downto 0) :="0100011";
constant SetEEArg4Cmd         :std_logic_vector (CmdLength downto 0) :="0100100";
constant SetEEArg5Cmd         :std_logic_vector (CmdLength downto 0) :="0100101";
constant SetEEArg6Cmd         :std_logic_vector (CmdLength downto 0) :="0100110";
constant SetEEArg7Cmd         :std_logic_vector (CmdLength downto 0) :="0100111";
constant SetEEArg8Cmd         :std_logic_vector (CmdLength downto 0) :="0101000";
constant SetEEArg9Cmd         :std_logic_vector (CmdLength downto 0) :="0101001";
constant SetEEArg10Cmd         :std_logic_vector (CmdLength downto 0):="0101010";
constant SetEEArg11Cmd         :std_logic_vector (CmdLength downto 0):="0101011";
constant SetEEArg12Cmd         :std_logic_vector (CmdLength downto 0):="0101100";
constant SetEEArg13Cmd         :std_logic_vector (CmdLength downto 0):="0101101";
constant SetEEArg14Cmd         :std_logic_vector (CmdLength downto 0):="0101110";
constant SetEEArg15Cmd         :std_logic_vector (CmdLength downto 0):="0101111";
constant SetEEArg16Cmd         :std_logic_vector (CmdLength downto 0) :="0110000";
constant SetEEArg17Cmd         :std_logic_vector (CmdLength downto 0) :="0110001";
constant SetEEArg18Cmd         :std_logic_vector (CmdLength downto 0) :="0110010";
constant SetEEArg19Cmd         :std_logic_vector (CmdLength downto 0) :="0110011";
constant SetEEArg20Cmd         :std_logic_vector (CmdLength downto 0) :="0110100";
constant SetEEArg21Cmd         :std_logic_vector (CmdLength downto 0) :="0110101";
constant SetEEArg22Cmd         :std_logic_vector (CmdLength downto 0) :="0110110";
constant SetEEArg23Cmd         :std_logic_vector (CmdLength downto 0) :="0110111";
constant SetEEArg24Cmd         :std_logic_vector (CmdLength downto 0) :="0111000";
constant SetEEArg25Cmd         :std_logic_vector (CmdLength downto 0) :="0111001";
constant SetEEArg26Cmd         :std_logic_vector (CmdLength downto 0):="0111010";
constant SetEEArg27Cmd         :std_logic_vector (CmdLength downto 0):="0111011";
constant SetEEArg28Cmd         :std_logic_vector (CmdLength downto 0):="0111100";
constant SetEEArg29Cmd         :std_logic_vector (CmdLength downto 0):="0111101";
constant SetEEArg30Cmd         :std_logic_vector (CmdLength downto 0):="0111110";
constant SetEEArg31Cmd         :std_logic_vector (CmdLength downto 0):="0111111";

constant SetUTCLowcmd       :std_logic_vector (CmdLength  downto 0)  :="1000000";
constant SetUTCHiCmd        :std_logic_vector (CmdLength  downto 0)  :="1000001";
constant ResetCmd           :std_logic_vector (CmdLength downto 0)   :="1000010";
constant TestCmd            :std_logic_vector (CmdLength downto 0)   :="1000011";
constant HaltCmd            :std_logic_vector (CmdLength downto 0)   :="1000100";
constant GoToCmd            :std_logic_vector (CmdLength downto 0)   :="1000101";
constant StepCmd            :std_logic_vector (CmdLength downto 0)   :="1000110";
constant FrameEnaCmd        :std_logic_vector (CmdLength downto 0)   :="1000111";
                             -- Arg(0)='0':Disab; Arg(0)='1':Enab 
constant SetMSURCmd         :std_logic_vector (CmdLength downto 0)   :="1001000";
constant RunCmd             :std_logic_vector (CmdLength downto 0)   :="1001001";
constant SetOutDelayCmd     :std_logic_vector (CmdLength downto 0)   :="1001010";
constant SetIENRCmd         :std_logic_vector (CmdLength downto 0)   :="1001011";
constant SetMTNRCmd         :std_logic_vector (CmdLength downto 0)   :="1001100";
constant SetSpRCmd          :std_logic_vector (CmdLength downto 0)   :="1001101";

------------------------
--  End ControlMachine LOGIC DECLARATION
--
------------------------

end;

package body  mtg is


end mtg;