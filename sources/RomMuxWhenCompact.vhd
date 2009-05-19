--==============================================================--
--Design Units : BIC Control and Statistics
--Size:		
--Speed:		  
--File Name: 	 RomMuxWhen.vhd
--
--Purpose: This entity generates an addres maping between the 
--				bus and the registers and memory.
--				It also multiplexes the registers to the bus interface.

--
--Note:	  This model was written and developed in ISE 4.2.
--				The core of the case when has been generated using
--				the tcl script creaArrayConst2.do
--
--Limitations:	
--
--Errors:
--
--Libraries: 
--
--Dependancies:
--
--Author: Pablo Antonio Alvarez Sanchez
--		  	 European Organisation for Nuclear Research
--		 	 SL SPS/LHC -- Control -- Timing Division
--		 	 CERN, Geneva, Switzerland,  CH-1211
--		 	 Building 864 Room 1 - A24
--
-- Mod by P. Nouchi to adapt this module to the specific BIC-core module mapping addresses
--
--Simulator:               ModelSim XE 5.5e_p1
--==============================================================--
--Revision List
--Version Author Date		 Changes
--
--1.0		 PAAS	  30.09.2002 Added comments, tested with the
--										rest of the design
--2.0		 PN     13.02.2003 For BIC-core module 
--2.1		 PN     13.02.2003 For CVORA module 
--==============================================================--


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

use IEEE.std_logic_unsigned.Conv_Integer;
use work.IntContPackage.all;
use work.vme.all;

entity ROMMUX is
    Port ( 
			IntAdd : in IntAddrOutType;
			DoingOpC : in std_logic;
			DoneRam : in SelectedRamPosType; 
			DataArray : in MuxDataArrType;
			Sel : out SelectedPosType;
			Done : out std_logic;
			DataOut : out IntDataType
);
end ROMMUX;

architecture CaseWhen of ROMMUX is

begin

process(IntAdd, DoingOpC, DataArray, DoneRam)
variable vAdd : integer; -- range 0 to 
begin
vAdd := Conv_Integer(IntAdd);
case vAdd is
--Registers
when  SOURCEREGARW  =>  
 		 Sel <= ( SOURCEREGP + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
 		 DataOut <= DataArray( SOURCEREGP);
 		 Done <= DoingOpC;
when  READADDREGRO  =>  
 		 Sel <= ( READADDREGP + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
 		 DataOut <= DataArray( READADDREGP);
 		 Done <= DoingOpC;
when  MODEREGARW  =>  
 		 Sel <= ( MODEREGP + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
 		 DataOut <= DataArray( MODEREGP);
 		 Done <= DoingOpC;
when  CHANNELREGARW  =>  
 		 Sel <= ( CHANNELREGP + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
 		 DataOut <= DataArray( CHANNELREGP);
 		 Done <= DoingOpC;
when  FREQREGARW  =>  
 		 Sel <= ( FREQREGP + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
 		 DataOut <= DataArray( FREQREGP);
 		 Done <= DoingOpC;
when  DACREGARW  =>  
 		 Sel <= ( DACREGP + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
 		 DataOut <= DataArray( DACREGP);
 		 Done <= DoingOpC;
--when  SPAREREGARW  =>  
-- 		 Sel <= ( SPAREREGP + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
-- 		 DataOut <= DataArray( SPAREREGP);
-- 		 Done <= DoingOpC;
-- Memory
--when  HBRAMAL to HBRAMAH  =>  
-- 		 Sel <= ( HBRAMP + FIRSTRAMPOS => '1', RAMOPPOS => '1', others => '0');
----		 Sel(0) <= '1';   -- because only one ram !!!
-- 		 DataOut <= DataArray(HBRAMP + FIRSTRAMPOS - FIRSTREGPOS);
-- 		 Done <= DoneRam(HBRAMP );
when  EXTRAMAL to EXTRAMAH  =>  
 		 Sel <= ( EXTRAMP + FIRSTRAMPOS => '1', RAMOPPOS => '1', others => '0');
--		 Sel(0) <= '1';   -- because only one ram !!!
 		 DataOut <= DataArray(EXTRAMP + FIRSTRAMPOS - FIRSTREGPOS);
 		 Done <= DoneRam(EXTRAMP );


when  others => 
			Sel <= ( RAMOPPOS => '0', REGOPPOS => '0', others => '0');
			DataOut <= (others => '-');
			Done <= '0';

end case;
end process;
end CaseWhen;

