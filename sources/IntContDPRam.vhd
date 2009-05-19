--	Package File Template
--
--	Purpose: This package defines supplemental types, subtypes, 
--		 constants, and functions for the core BIC module


library IEEE;
use IEEE.STD_LOGIC_1164.all;
use work.vme.all;

package IntContPackage is
  
-- in auxdef --  constant RESET_ACTIVE : std_logic := '0';

  constant  ADDTOP : integer := 16#7FFFF#;  --
  constant  NUMREGS : integer := 6;  -- Number of registers !!!!!
  constant  NUMRAM : integer := 1;    -- Number of RAMs!!!!!

  
  constant FIRSTREGPOS : integer := 2;
  constant FIRSTRAMPOS : integer := NUMREGS + 2;
  
  constant RAMOPPOS : integer := 0;
  constant REGOPPOS : integer := 1;
  
  type KINDOFMUXType is (USEMUXZ, USEMUXCOMB);
  constant KINDOFMUX : KINDOFMUXType := USEMUXCOMB;

  subtype IntDataType is std_logic_vector(31 downto 0);
--  subtype IntAddrOutType is std_logic_vector(SlaveAddrOutType'left - 1 downto 0);  

 subtype MemIntWrAddrType is std_logic_vector(SlaveAddrOutType'left downto 0); -- 18 downto 0
 subtype MemIntRdAddrType is std_logic_vector(SlaveAddrOutType'left -1 downto 0); --17 downto 0

  type MuxDataArrType is array (0 to NUMREGS + NUMRAM -1) of IntDataType;

  subtype MuxSelType is std_logic_vector(NUMREGS + NUMRAM -1 downto 0);  
  
  subtype RegPType is integer range 0 to  NUMREGS -1;
  subtype RegAType is integer range 0 to ADDTOP;

  subtype RamPType is integer range 0 to  NUMRAM -1;
  subtype RamAType is integer range 0 to ADDTOP;
  
  subtype SelectedPosType is std_logic_vector(NUMREGS + NUMRAM + 1 downto 0);
  subtype SelectedRegPosType is std_logic_vector(NUMREGS - 1 downto 0);
  subtype SelectedRamPosType is std_logic_vector(NUMRAM - 1 downto 0);
  
  type SelRamDataType is array (integer range 0 to NUMRAM - 1) of IntDataType;
 
  type ContToRegsType is
  record
  		Data : IntDataType;
		Sel : SelectedRegPosType;
		Wr : std_logic;
		Rd : std_logic;
--		WrPos : SelectedRegPosType;
  end record;
  
  type RegsToContType is array (integer range 0 to  NUMREGS - 1) of IntDataType;

  type ContToMemType is
  record
  		Data : IntDataType;
		Add  : IntAddrOutType;
		SelectedPos : SelectedRamPosType;
		Wr : std_logic;
		Rd : std_logic;
  end record;

  type MemToContCellType is
  record
		Data : IntDataType;
		RdDone : std_logic;
  end record;

  type MemToContType is array (integer range 0 to NUMRAM - 1) of MemToContCellType;


  type AddRwRecord is
  record
		Add : RegAType;
		rwt : boolean;
  end record;
		
  type ADDMAPPINGType is array (integer range 0 to NUMREGS - 1) of AddRwRecord;
--*****************************************
--VME Address Map
--*****************************************
-- The source register is a control register in write and a status register in read
-- bit 0 is the pulse polarity of the inputs
-- bit 1 enable/disable the module
-- bit 2 enable/disable IRQ
-- bit 4-3 are not used
-- bit 5 is set during acquisition (between a start and a stop input) - Read only
-- bit 6 is set if there is a Counter oberflow (btrain mode) - read only
-- bit 7 is set if the RAM is overflowed - read only
-- bit 15 downto 8 is the IRQ vector default: 134 decimal, 86 hexa - read write
-- bit 31 downto 16 is the version number (read only)
constant SOURCEREGARW : RegAType:= 16#00000#;
constant READADDREGRO : RegAType:= 16#00004#; --memory pointer
-- bit 2-0 are the mode :	mode 000 --> parallele inputs
--	of the Mode Register :	mode 001 --> Opticals inputs 16 bits - SDataIn2 is ignored
--									mode 101 --> Opticals inputs 32 bits
--									mode 010 --> Copper Inputs 16 bits
--									mode 110 --> Copper Inputs 32 bits
--									mode 011 --> Btrain counter
--									mode 111 --> P2 Serial Inputs
constant MODEREGARW : RegAType:= 16#00008#;
constant CHANNELREGARW : RegAType:= 16#0000C#;
constant FREQREGARW : RegAType:= 16#00010#;
constant DACREGARW : RegAType:= 16#00014#;
--constant SPAREREGARW : RegAType:= 16#00018#;
--constant HBRAMP : RamPType := 0;
--constant HBRAMAL : RamAType := 16#10000#;
--constant HBRAMAH : RamAType := 16#1FFFF#;
constant EXTRAMP : RamPType := 0;
constant EXTRAMAL : RamAType := 16#00020#; 
constant EXTRAMAH : RamAType := 16#7FFFF#;
constant MEMOFFSET : RamAType := 16#00008#; --long word offset 
--********************************************
--********************************************
--Vme Registers position
--********************************************
--********************************************
--The Bus interface will provide an Array of Data Accesed lines plus
--a Read and Write line.
--To selecte the desired data, the registers distributed all along the
--cvora are maped to an Table of Read Registers.
--
--The following list of constants indicates the position of every register
--into the Array of Data Accesed Lines and the Table of Read Registers.
--********************************************
--********************************************
constant SOURCEREGP : RegPType := 0;
constant READADDREGP : RegPType := 1;
constant MODEREGP : RegPType := 2;
constant CHANNELREGP : RegPType := 3;
constant FREQREGP : RegPType := 4;
constant DACREGP: RegPType := 5;
 
--constant SPAREREGP : RegPType := 6;

end IntContPackage;
