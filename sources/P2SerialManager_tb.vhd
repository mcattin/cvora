
--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   17:01:45 04/02/2007
-- Design Name:   P2SerialManagerSTM
-- Module Name:   C:/FPGADesigns/cvora/sources/P2SerialManager_tb.vhd
-- Project Name:  cvora
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: P2SerialManagerSTM
--
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends 
-- that these types always be used for the top-level I/O of a design in order 
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.all;
USE ieee.numeric_std.ALL;
use work.auxdef.all;

ENTITY P2SerialManager_tb_vhd IS
END P2SerialManager_tb_vhd;

ARCHITECTURE behavior OF P2SerialManager_tb_vhd IS 

	-- Component Declaration for the Unit Under Test (UUT)
	COMPONENT P2SerialManagerSTM
	PORT(
		rst : IN std_logic;
		clk : IN std_logic;
		Mode : IN std_logic_vector(2 downto 0);
		DataIn : IN std_logic_vector(31 downto 0);
		DataInClk : IN std_logic;
		ChannelReg : IN std_logic_vector(31 downto 0);
		DataWritten : IN std_logic;
		ClearMem : IN std_logic;          
		WriteRam : OUT std_logic;
		WriteAdd : OUT std_logic_vector(16 downto 0);
		P2RamAddOverflow : OUT std_logic;
--		P2MemBusy : out std_logic;
		DatatoRAM : OUT std_logic_vector(31 downto 0)
		);
	END COMPONENT;

	--Inputs
	SIGNAL rst :  std_logic := '1';
	SIGNAL clk :  std_logic := '0';
	SIGNAL DataInClk :  std_logic := '0';
	SIGNAL DataWritten :  std_logic := '0';
	SIGNAL ClearMem :  std_logic := '0';
	SIGNAL Mode :  std_logic_vector(2 downto 0) := (others=>'1');
	SIGNAL DataIn :  std_logic_vector(31 downto 0) := (others=>'0');
	SIGNAL ChannelReg :  std_logic_vector(31 downto 0) := (others=>'1');

	--Outputs
	SIGNAL WriteRam :  std_logic;
	SIGNAL WriteAdd :  std_logic_vector(16 downto 0);
	SIGNAL P2RamAddOverflow :  std_logic;
	SIGNAL DatatoRAM :  std_logic_vector(31 downto 0);

BEGIN

	-- Instantiate the Unit Under Test (UUT)
	uut: P2SerialManagerSTM PORT MAP(
		rst => rst,
		clk => clk,
		Mode => Mode,
		DataIn => DataIn,
		DataInClk => DataInClk,
		ChannelReg => ChannelReg,
		DataWritten => DataWritten,
		ClearMem => ClearMem,
		WriteRam => WriteRam,
		WriteAdd => WriteAdd,
		P2RamAddOverflow => P2RamAddOverflow,
		DatatoRAM => DatatoRAM
	);

mode <= P2SERIALMODE;
ChannelReg <= (4=>'1',3 => '1', 2 => '1', 1 => '1', 0 => '1', others => '0');
--clearmem <= '0';

pCLK: process
begin
clk <= '0';
wait for 12 ns;
clk <= '1';
wait for 13 ns;
end process;

--preset: process
--begin
--rst <= '0';
--ClearMem <= '1';
--wait for 100 ns;
--rst <= '1';
--wait for 100 ns;
--ClearMem <= '0';
--wait;
--end process;

pMS: process
begin
--DataInClk <= '0';
wait for 100005 ns;
DataInClk <= '1';
wait for 25 ns;
DataInClk <= '0';
wait for 200 ns;
datawritten <= '1';
wait for 25 ns;
datawritten <= '0';
wait for 200 ns;

wait for 200 ns;
datawritten <= '1';
wait for 25 ns;
datawritten <= '0';
wait for 200 ns;
datawritten <= '1';
wait for 25 ns;
datawritten <= '0';
wait for 812 ns;
datawritten <= '1';
wait for 25 ns;
datawritten <= '0';

wait for 1200 ns;
Clearmem <= '1';
wait for 25 ns;
Clearmem <= '0';

end process;



END;
