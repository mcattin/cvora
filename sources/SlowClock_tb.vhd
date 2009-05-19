
--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   10:39:39 10/17/2006
-- Design Name:   SlowClock
-- Module Name:   C:/FPGADesigns/dpramcard/sources/SlowClock_tb.vhd
-- Project Name:  dpramcard
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: SlowClock
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

ENTITY SlowClock_tb_vhd IS
END SlowClock_tb_vhd;

ARCHITECTURE behavior OF SlowClock_tb_vhd IS 

	-- Component Declaration for the Unit Under Test (UUT)
	COMPONENT SlowClock
	PORT(
		clk : IN std_logic;
		en1M : IN std_logic;          
		SlowClockOut : OUT std_logic
		);
	END COMPONENT;

	--Inputs
	SIGNAL clk :  std_logic;
	SIGNAL en1M :  std_logic;
--	signal rst : std_logic;
	--Outputs
	SIGNAL SlowClockOut :  std_logic;

BEGIN

	-- Instantiate the Unit Under Test (UUT)
	uut: SlowClock PORT MAP(
--		rst => rst,
		clk => clk,
		en1M => en1M,
		SlowClockOut => SlowClockOut
	);
--pRST: process
--begin
--	rst <= '0';
--	wait for 100 ns;
--	rst <= '1';
--	wait;
--end process;

pCLK: process
begin
clk <= '0';
wait for 12 ns;
clk <= '1';
wait for 13 ns;
end process;

pCLK1M: process
begin
en1M <= '0';
wait for 975 ns;
en1M <= '1';
wait for 25 ns;
end process;



END;
