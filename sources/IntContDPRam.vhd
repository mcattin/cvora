--      Package File Template
--
--      Purpose: This package defines supplemental types, subtypes,
--               constants, and functions for the core BIC module


library ieee;
use ieee.std_logic_1164.all;

use work.vme_pkg.all;


package bus_int_pkg is


  constant ADDTOP  : integer := 16#7FFFF#;  --
  constant NUMREGS : integer := 7;          -- Number of registers !!!!!
  constant NUMRAM  : integer := 1;          -- Number of RAMs!!!!!

  constant FIRSTREGPOS : integer := 2;
  constant FIRSTRAMPOS : integer := NUMREGS + 2;

  constant RAMOPPOS : integer := 0;
  constant REGOPPOS : integer := 1;

  type     KINDOFMUXType is (USEMUXZ, USEMUXCOMB);
  constant KINDOFMUX : KINDOFMUXType := USEMUXCOMB;

  subtype IntDataType is std_logic_vector(31 downto 0);
--  subtype IntAddrOutType is std_logic_vector(SlaveAddrOutType'left - 1 downto 0);

  subtype MemIntWrAddrType is std_logic_vector(SlaveAddrOutType'left downto 0);     -- 18 downto 0
  subtype MemIntRdAddrType is std_logic_vector(SlaveAddrOutType'left -1 downto 0);  --17 downto 0

  type MuxDataArrType is array (0 to NUMREGS + NUMRAM -1) of IntDataType;

  subtype MuxSelType is std_logic_vector(NUMREGS + NUMRAM -1 downto 0);

  subtype RegPType is integer range 0 to NUMREGS -1;
  subtype RegAType is integer range 0 to ADDTOP;

  subtype RamPType is integer range 0 to NUMRAM -1;
  subtype RamAType is integer range 0 to ADDTOP;

  subtype SelectedPosType is std_logic_vector(NUMREGS + NUMRAM + 1 downto 0);
  subtype SelectedRegPosType is std_logic_vector(NUMREGS - 1 downto 0);
  subtype SelectedRamPosType is std_logic_vector(NUMRAM - 1 downto 0);

  type SelRamDataType is array (integer range 0 to NUMRAM - 1) of IntDataType;

  type ContToRegsType is
  record
    Data : IntDataType;
    Sel  : SelectedRegPosType;
    Wr   : std_logic;
    Rd   : std_logic;
--              WrPos : SelectedRegPosType;
  end record;

  type RegsToContType is array (integer range 0 to NUMREGS - 1) of IntDataType;

  type ContToMemType is
  record
    Data        : IntDataType;
    Add         : IntAddrOutType;
    SelectedPos : SelectedRamPosType;
    Wr          : std_logic;
    Rd          : std_logic;
  end record;

  type MemToContCellType is
  record
    Data   : IntDataType;
    RdDone : std_logic;
  end record;

  type MemToContType is array (integer range 0 to NUMRAM - 1) of MemToContCellType;

  type AddRwRecord is
  record
    Add : RegAType;
    rwt : boolean;
  end record;

  type     ADDMAPPINGType is array (integer range 0 to NUMREGS - 1) of AddRwRecord;

--*****************************************
--VME Address Map
--*****************************************
  constant CSR_REG_ADDR      : RegAType := 16#00000#;
  constant MEM_PTR_REG_ADDR  : RegAType := 16#00004#;
  constant MODE_REG_ADDR     : RegAType := 16#00008#;
  constant CHAN_EN_REG_ADDR  : RegAType := 16#0000C#;
  constant CLK_FREQ_REG_ADDR : RegAType := 16#00010#;
  constant CHAN_SEL_REG_ADDR : RegAType := 16#00014#;
  constant CVORB_REG_ADDR    : RegAType := 16#00018#;

  constant EXT_RAM_P     : RamPType := 0;
  constant EXT_RAM_ADDRL : RamAType := 16#00020#;
  constant EXT_RAM_ADDRH : RamAType := 16#7FFFF#;
  constant MEMOFFSET     : RamAType := 16#00008#;  --long word offset

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
  constant CSR_REG_P      : RegPType := 0;
  constant MEM_PTR_REG_P  : RegPType := 1;
  constant MODE_REG_P     : RegPType := 2;
  constant CHAN_EN_REG_P  : RegPType := 3;
  constant CLK_FREQ_REG_P : RegPType := 4;
  constant CHAN_SEL_REG_P : RegPType := 5;
  constant CVORB_REG_P    : RegPType := 6;


end bus_int_pkg;
