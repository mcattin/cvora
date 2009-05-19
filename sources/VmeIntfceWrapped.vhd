----------------------------------------------------
--  
-- Unit  Name  :  VmeIntfceWrapped
--
-- Date        : Wed Apr 10 15:07:29 2002
--
-- Author      : David Dominguez
--
-- Company     : CERN
--
--  
------------------------------------------------------

library ieee;
use ieee.STD_LOGIC_1164.all;
use ieee.STD_LOGIC_ARITH.all;
use ieee.STD_LOGIC_UNSIGNED.all;



entity VmeIntfceWrapped is

 generic ( AddrWidth     : integer :=24;
	BaseAddrWidth      : integer :=8;
	DataWidth          : integer :=32;
      UnalignDataWidth   : integer := 8;
      DirSamePolarity     :  std_logic :='0';
      InterruptEn         : std_logic :='0'

       );
  port (
        Clk           :in std_logic;
        VmeAddrA      :in std_logic_vector(AddrWidth-1 downto 1 );
        VmeAsNA       :in std_logic;
        VmeDs1NA      :in std_logic;
        VmeDs0NA      :in std_logic;
        VmeData       :inout std_logic_vector(DataWidth-1 downto 0 );
        VmeDataUnAlign:inout std_logic_vector(UnalignDataWidth-1 downto 0); 
        VmeDirTri     :out std_logic;
        VmeBufOeN     :out std_logic;
        VmeWriteNA    :in std_logic;
        VmeLwordNA    :in std_logic;
        VmeIackNA     :in std_logic;
        IackOutNA     :out std_logic;
        IackInNA      :in std_logic;
        VmeIntReqN    :out std_logic_vector (7 downto 1);
        VmeDtackNTri  :out std_logic;
        ModuleAddr    :in std_logic_vector(BaseAddrWidth-1 downto 0 );
        VmeAmA        :in std_logic_vector(4 downto 0 );

        AddrMem       :out std_logic_vector(AddrWidth-BaseAddrWidth-1 downto 0 );--here (15 downto 0);
        ReadMem       :out std_logic;
        WriteMem      :out std_logic;
        DataFromMemValid : in std_logic;
        DataFromMem   :in std_logic_vector(DataWidth-1 downto 0 );
        DataToMem     :out std_logic_vector(DataWidth-1 downto 0 );
        IntProcessed  :out std_logic;
        ResetNA       :in std_logic;
        UserIntReqN   :in std_logic;
        UserBlocks    :in std_logic;
        OpFinishedOut :out std_logic;
--------------------------------------  
----------------
        VmeState      :out std_logic_vector (3 downto 0);
----------------
--------------------------------------
	  IRQLevelReg   :in std_logic_vector (3 downto 1);
	  IRQStatusIDReg: in std_logic_vector (DataWidth-1 downto 0)
      );


end VmeIntfceWrapped;

architecture simple of VmeIntfceWrapped is


component Vme_intfce

 generic ( AddrWidth  : integer :=24;
	BaseAddrWidth   : integer :=8;
	DataWidth       : integer :=32;
      UnalignDataWidth: integer :=8;
      DirSamePolarity  : std_logic :='0';
      InterruptEn      : std_logic :='0'
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
        VmeDirFloat   : out std_logic;
        OpFinishedOut : out std_logic;
	  IRQLevelReg   : in std_logic_vector (3 downto 1);
	  IRQStatusIDReg: in std_logic_vector (DataWidth-1 downto 0);

---------------------------------------- debugging
        VmeState      : out std_logic_vector (3 downto 0)

--This signal is intended for debugging purposes and tells
-- the state of the VME interface.
----------------------------------------

				);  
  
end component;

component VmeWrap 
 generic (
     DirSamePolarity: std_logic :='0');

  port (
     ResetNA       : in std_logic;
     Clk           : in std_logic;
     VmeDirTri     : out std_logic;
     VmeDtackN     : out std_logic;
     IntfceDir     : in std_logic;
     VmeDirFloat   : in std_logic;
     IntfceDtackN  : in std_logic
        );
end component;


signal        VmeDir      :std_logic;
signal        VmeDtackN   :std_logic;
signal        VmeDirFloat :std_logic;


begin



c1: Vme_intfce 

   generic map(AddrWidth       =>AddrWidth ,
               BaseAddrWidth   =>BaseAddrWidth ,
               DataWidth       =>DataWidth ,
               UnalignDataWidth =>UnalignDataWidth,
               DirSamePolarity  =>DirSamePolarity,
               InterruptEn      =>InterruptEn
 )


   port map (
             Clk      	=>	       Clk	  ,
        VmeAddrA   	=>	  VmeAddrA	  ,
        VmeAsNA    	=>	  VmeAsNA	  ,
        VmeDs1NA   	=>	  VmeDs1NA	  ,
        VmeDs0NA    	=>	  VmeDs0NA	  ,
        VmeData     	=>	  VmeData	  ,
        VmeDataUnAlign  =>      VmeDataUnAlign,
        VmeDir     	=>	  VmeDir	  ,
        VmeBufOeN  	=>	  VmeBufOeN	  ,
        VmeWriteNA  	=>	  VmeWriteNA  ,
        VmeLwordNA  	=>	  VmeLwordNA  ,
        VmeIackNA  	=>	  VmeIackNA	  ,
        IackOutNA  	=>	  IackOutNA	  ,
        IackInNA   	=>	  IackInNA	  ,
        VmeIntReqN  	=>	  VmeIntReqN  ,
        VmeDtackN  	=>	  VmeDtackN,
        ModuleAddr 	=>	  ModuleAddr  ,
        AddrMem       	=>	  AddrMem  	  ,
        VmeAmA     	=>	  VmeAmA	  ,
        ReadMem    	=>	  ReadMem	  ,
        WriteMem   	=>	  WriteMem	  ,
        DataFromMemValid  =>	  DataFromMemValid  ,
        DataFromMem  	=>	  DataFromMem ,
        DataToMem  	=>	  DataToMem	  ,
        IntProcessed  	=>	  IntProcessed ,
        ResetNA    	=>	  ResetNA	  ,
        UserIntReqN  	=>	  UserIntReqN ,
        UserBlocks  	=>	  UserBlocks ,
        VmeDirFloat     =>      VmeDirFloat,
        OpFinishedOut   =>      open       ,
---------------------------------------------------debugging
        VmeState         => vmeState,
------------------------------------------------------------
	  IRQLevelReg	=>  IRQLevelReg,
	  IRQStatusIDReg	=> IRQStatusIDReg	
	  );


c3: VmeWrap 
   generic map ( 
    DirSamePolarity =>DirSamePolarity )

   port map(
     ResetNA      =>  ResetNA,     
     Clk          =>  Clk,
     VmeDirTri    => VmeDirTri,
     VmeDtackN    => VmeDtackNTri,
     IntfceDir    => VmeDir,
     VmeDirFloat  => VmeDirFloat,
     IntfceDtackN => VmeDtackN
   );

end simple;	
	  
