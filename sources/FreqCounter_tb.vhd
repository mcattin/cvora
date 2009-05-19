
--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   11:39:27 07/03/2006
-- Design Name:   FrequencyCounter
-- Module Name:   FreqCounter_tb.vhd
-- Project Name:  control
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: FrequencyCounter
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

ENTITY FreqCounter_tb_vhd IS
END FreqCounter_tb_vhd;

ARCHITECTURE behavior OF FreqCounter_tb_vhd IS 

	-- Component Declaration for the Unit Under Test (UUT)
	COMPONENT FrequencyCounter
	PORT(
		rst : IN std_logic;
		clk : IN std_logic;
		StartCounterPulse : IN std_logic;
		inclk : IN std_logic;          
		FreqCounterReady : out std_logic;
		FrequencyCounterReg : OUT std_logic_vector(31 downto 0)
		);
	END COMPONENT;

	--Inputs
	SIGNAL rst :  std_logic;
	SIGNAL clk :  std_logic;
	SIGNAL StartCounterPulse :  std_logic;
	SIGNAL inclk :  std_logic;
	signal FreqCounterReady : std_logic;

	--Outputs
	SIGNAL FrequencyCounter1 :  std_logic_vector(31 downto 0) := (others => '0');

BEGIN

	-- Instantiate the Unit Under Test (UUT)
	uut: FrequencyCounter PORT MAP(
		rst => rst,
		clk => clk,
		StartCounterPulse => StartCounterPulse,
		inclk => inclk,
		FreqCounterReady => FreqCounterReady,
		FrequencyCounterReg => FrequencyCounter1
	);

pCLK: process
begin
clk <= '0';
wait for 2 ns;
clk <= '1';
wait for 3 ns;
end process;

preset: process
begin
rst <= '0';
wait for 50 ns;
rst <= '1';
wait for 50 ns;
wait;
end process;

pen512: process
begin
StartCounterPulse <= '0';
wait for 100 ns;
StartCounterPulse <= '1';
wait for 5 ns;
StartCounterPulse <= '0';
wait for 4900 ns;
end process;

pinclk: process
begin
inclk <= '0';
wait for 10 ns;
inclk <= '1';
wait for 5 ns;
inclk <= '0';
wait for 10 ns;
end process;


END;
