------------------------------------------
------------------------------------------
-- Date        : Mon 07 October 2002
--
-- Author      : David Dominguez
--
-- Company     : 
--
-- Description : This design implements the whole FPGA#1 (CTG board).
--               This design provides a VME interface and a Register
--               used for JTAG dialog. The JTAG protocol must be implemented
--               in software in the the VME CPU.This design provides
--               a mechanism to program other FPGAs in the same board
--               via the VME bus and using standard VME read/write cycles  
--
------------------------------------------
------------------------------------------
-- Version 1.1 P.Nouchi le 05-05-2006 Add Xreset input and Change LVTTL by LVCMOS33

library ieee;
use ieee.STD_LOGIC_1164.all;
use ieee.STD_LOGIC_ARITH.all;
use ieee.STD_LOGIC_UNSIGNED.all;

entity  FPGA1  is
     generic ( AddrWidth  : integer :=16;
	BaseAddrWidth   : integer :=8;
	DataWidth       : integer :=16);

  port(

        Clk : in std_logic;
        VmeAddrA : in std_logic_vector(AddrWidth-1 downto 1 );
        VmeAsNA : in std_logic;
        VmeDs1NA : in std_logic;
        VmeDs0NA : in std_logic;
        VmeData  : inout std_logic_vector(7 downto 0 );
        VmeDir : out std_logic;
        VmeBufOeN : out std_logic;
        VmeWriteNA : in std_logic;
        VmeLwordNA : in std_logic;
        VmeIackNA : in std_logic;
--        VMEResetNA  :in std_logic;
        VmeDtackN : out std_logic;
        ModuleAddr : in std_logic_vector(BaseAddrWidth -1 downto 0 );
        VmeAmA : in std_logic_vector(5 downto 0 );
        ResetNA : in std_logic;

 	     test :out std_logic_vector (5 downto 0);
		  Xreset : in std_logic;

	   	x2TDI, x2TMS, x2TCK :out std_logic;
		TDOFlashA : in std_logic

       );
end;


 architecture simple of FPGA1 is


component Vme_intfce is


 generic ( AddrWidth  : integer :=16;
	BaseAddrWidth   : integer :=8;
	DataWidth       : integer :=8;
    DirSamePolarity  : std_logic :='0';
    UnalignDataWidth : integer := 8;
    InterruptEn      : std_logic :='1'
 );

  port (
        ResetNA       : in std_logic;
        Clk           : in std_logic;
        VmeAddrA      : in std_logic_vector(AddrWidth-1 downto 1 );
        VmeAsNA       : in std_logic;
        VmeDs1NA      : in std_logic;
        VmeDs0NA      : in std_logic;
        VmeData       : inout std_logic_vector(DataWidth-1 downto 0 );
        VmeDataUnAlign: inout std_logic_vector(UnalignDataWidth-1 downto 0); 
        VmeDir        : out std_logic;
        VmeDirFloat   : out std_logic;
        VmeBufOeN     : out std_logic;
        VmeWriteNA    : in std_logic;
        VmeLwordNA    : in std_logic;
        VmeIackNA     : in std_logic;
        IackOutNA     : out std_logic;
        IackInNA      : in std_logic;
        VmeIntReqN    : out std_logic_vector (7 downto 1);
        vmeDtackN     : out std_logic;
        ModuleAddr    : in std_logic_vector(BaseAddrWidth-1 downto 0 );
        VmeAmA        : in std_logic_vector(4 downto 0 );

        AddrMem       : out std_logic_vector(AddrWidth-BaseAddrWidth-1 downto 0 );
        ReadMem       : out std_logic;
        WriteMem      : out std_logic;
        DataFromMemValid : in std_logic;
        DataFromMem   : in std_logic_vector(DataWidth-1 downto 0 );
        DataToMem     : out std_logic_vector(DataWidth-1 downto 0 );
        IntProcessed  : out std_logic;
        UserIntReqN   : in std_logic;
        UserBlocks    : in std_logic;
        OpFinishedOut : out std_logic;
	  IRQLevelReg   : in std_logic_vector (3 downto 1);
	  IRQStatusIDReg: in std_logic_vector (DataWidth-1 downto 0);

---------------------------------------- debugging
        VmeState      : out std_logic_vector (3 downto 0)

--This signal is intended for debugging purposes and tells
-- the state of the VME interface.
----------------------------------------

				);  
end component Vme_intfce;


component JTAG_Register
  generic ( AddrWidth: integer :=16;
	   BaseAddrWidth: integer :=8;
	    DataWidth: integer :=16 );

   port(
	Clk: in std_logic;
	ResetNA: in std_logic;
	DataToMem: in std_logic_vector(DataWidth-1 downto 0);
	Write: in std_logic;
	Addr: in std_logic_vector((AddrWidth-BaseAddrWidth-1) downto 1);
	Read: in std_logic;
	TDOFlashA: in std_logic;

	DataFromMem: out std_logic_vector (DataWidth-1 downto 0);
	DataValid: out std_logic;
	TDI, TMS, TCK :out std_logic

	);
end component;



signal	  Addr       : std_logic_vector(AddrWidth-BaseAddrWidth-1 downto 0  );
signal        ReadMem    : std_logic;
signal        WriteMem   :  std_logic;
signal        DataFromMemValid :  std_logic;
signal        DataFromMem, iDataFromMem :  std_logic_vector(DataWidth-1 downto 0 );
signal        DataToMem   :  std_logic_vector(DataWidth-1 downto 0 );
signal	  one, zero   : std_logic;
signal 	  IRQLevelReg_in:  std_logic_vector (3 downto 1);
signal	  IRQStatusIDReg_in:  std_logic_vector (DataWidth-1 downto 0);
signal	  UserIntReqN  : std_logic;
signal        UserBlocks   :  std_logic;
signal        InterruptEn  : std_logic;
signal        IntProcessed :  std_logic;

signal        counter: std_logic_vector (7 downto 0);

signal        VmeDataWidthMatch: std_logic_vector (DataWidth-1 downto 0);
signal        IntfceVmeAmA     : std_logic_vector (4 downto 0);
signal        iModuleAddr      :std_logic_vector (BaseAddrWidth-1 downto 0 );
signal 		iVmeDir, iVmeBufOeN: std_logic;


begin

 VmeData <= VmeDataWidthMatch (7 downto 0);
 IntfceVmeAmA <= VmeAmA(5) & VmeAmA(4) & VmeAmA(3) & VmeAmA(1) & VmeAmA(0);
 iModuleAddr <=not (ModuleAddr);

   VMEInt: Vme_intfce
 generic map (
        AddrWidth =>16,
        BaseAddrWidth =>8,
        DataWidth =>16,
        DirSamePolarity =>'0',
        UnAlignDataWidth =>8,
        InterruptEn =>'0' )

port map(
        Clk	        =>	clk,
        VmeAddrA       => VmeAddrA,
        VmeAsNA        => VmeAsNA,
        VmeDs1NA       => VmeDs1NA,
        VmeDs0NA       => VmeDs0NA,
        VmeData        => VmeDataWidthMatch,
        VmeDataUnAlign => open,
        VmeDir         => iVmeDir,
	   VmeDirFloat    => open,
        VmeBufOeN      => iVmeBufOeN,
        VmeWriteNA     => VmeWriteNA,
        VmeLwordNA     => VmeLwordNA,
        VmeIackNA      => VmeIackNA,
        IackOutNA      => open,
        IackInNA       => one,
        VmeIntReqN     => open,
        VmeDtackN      => VmeDtackN,
        ModuleAddr     => iModuleAddr,
        AddrMem        => Addr,
        VmeAmA         => IntfceVmeAmA,
        ReadMem        => ReadMem,
        WriteMem       => WriteMem,
        DataFromMemValid=> DataFromMemValid,
        DataFromMem     => DataFromMem,
        DataToMem       => DataToMem,
        IntProcessed    => IntProcessed,
        ResetNA         => ResetNA,
        UserIntReqN     => one,
        UserBlocks      => zero,
        OpFinishedOut   => open,
		VmeState        =>open,
        IRQLevelReg     => IRQLevelReg_in,
        IRQStatusIDReg  => IRQStatusIDReg_in

      );


Reg1:JTAG_Register
 port map (
 	Clk		=> Clk,
	ResetNA	=> ResetNA,
	DataToMem	=> DataToMem,
	Write		=> WriteMem,
	Addr		=> Addr(AddrWidth-baseAddrWidth-1 downto 1),
	Read		=> ReadMem,
	TDOFlashA	=> TDOFlashA,

	DataFromMem	=> iDataFromMem,
	DataValid	=> DataFromMemValid,
	TDI		=> x2TDI,
	TMS		=> x2TMS,
	TCK 		=> x2TCK);
DataFromMem(6 downto 0) <= iDataFromMem(6 downto 0);
DataFromMem(8 downto 7) <= "10";
DataFromMem(15 downto 9) <= (others => '0');

zero <='0';
one <='1';
IRQLevelReg_in <= "011";

IDStat: if DataWidth > 8 generate
     		IRQStatusIDReg_in(IRQStatusIDReg_in'left downto 8) <= (others =>'1');
end generate IdStat;

test (5 downto 2) <=iModuleAddr(7 downto 4);
test(0) <= Xreset;
test(1) <= '0';
VmeDir <= iVmeDir;
VmeBufOeN <= iVmeBufOeN;
IRQStatusIDReg_in(7 downto 0) <= "10101110"; -- ox"AFFE"
end ;


