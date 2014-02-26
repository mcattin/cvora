
----------------------------------------------------
-- Date          : Wed Apr 10 15:07:29 2002
--
-- Author        : David Dominguez
--
-- Company       : CERN
-- 
--  Library Name :  prueba
--  Unit    Name :  JTAG_Register
--  Unit    Type :  Text Unit
--  
--  Unit description:
-- Provides a register to connect to the JTAG lines of a FLASH EPROM/any external device, and a single address decoder allowing to
-- read/write into it.
-- A special protocol (data format) in the data to be written must be used. see below.
--
-- Interface details:
--	*Processor/bus side: Read/Write, DataValid and data: 1 clock cycle asserted means 1 operation to be done/ 1 data valid (block transfer oriented).
--	*JTAG side: The register's bits are hardwired to the JTAG pins of the entended FLASH/any device to be programed.
--		 All these signals (Tck, Tms, Tdi and Tdo) are registered.
--
-- Protocol details:
-- 	*Data format for writing into the register (only the Less Significant Byte of the whole data word is taken into account):
--					  Bit7| Bit6 | Bit5 |Bit4 |Bit3|Bit2 |Bit1 |Bit0
--					 Tck# | Tms# | Tdi# | XXX |Tck | Tms | Tdi | Tdo
--
--		Tck#,Tms#, Tdi# are the Tck,Tms and Tdi signals inverted. XXX: not used.
--		Tdo is an imput to the design, so when a write from the bus side is performed into the register,
--		this bit is never overwritten.
--
--	*When the processor/bus side tries to write, the data format seen above must be respected, otherwise, no write
--	 operation will be executed and the register (bits 3 downto 1) will remain unchanged.
--	*After a successfull write into the register, the redundant bits (bit 7 downto 4) will be set to '1' by the internal
--		logic, in order to provide a mechanisme for an acknowledgement, should the need arise.
--	*If a wrong data format is used when writing into the reg., bits 7 downto 4 will be set to '0' by the internal logic,
--	providing a mechanism to detect undesired accesses to the register.
--    
------------------------------------------------------

library ieee;
use ieee.STD_LOGIC_1164.all;
use ieee.STD_LOGIC_ARITH.all;
use ieee.STD_LOGIC_UNSIGNED.all;

use work.mtg.all;

entity  JTAG_Register  is

  generic ( AddrWidth: integer :=16;
	   BaseAddrWidth: integer :=8;
	    DataWidth: integer :=8 );
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

end;
architecture  simple  of  JTAG_Register  is

signal JTAGReg: std_logic_vector (DataWidth-1 downto 0);

begin

clocking: process (ResetNA, Clk)
begin
if ResetNA='0' then
  DataValid <='0';
  JTAGReg (DataWidth-1 downto 0)<= (others =>'0'); 
  DataFromMem <= (others =>'0');
elsif rising_edge(Clk) then

  JTAGReg(0) <= TDOFlashA;

  if Write ='1' and Addr=JTAGAddr then

    if (DataToMem (7 downto 5) = not (DataToMem (3 downto 1))) then
	JTAGReg (DataWidth-1 downto 4) <= (others =>'1');
        JTAGReg (3 downto 1) <= DataToMem(3 downto 1);
    else
        JTAGReg (DataWidth-1 downto 4) <= (others =>'0');
    end if;

    DataValid <='0';
    DataFromMem <= (others=>'0');
  elsif Read ='1' and Addr=JTAGAddr then

    DataFromMem <= JTAGReg;
    DataValid <='1';
  else
    DataFromMem <=(others =>'0');
    DataValid <='0';
  end if;
end if;   
  
end process clocking;

--Assignements
 TCK <=JTAGReg(3);
 TMS <= JTAGReg(2);
 TDI <= JTAGReg(1);


end;

