
--------------------------------------------------------------------------------
-- Test bench for testing SimpleFrequency Meter component
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.all;
USE ieee.numeric_std.ALL;

ENTITY SimpleFrequencyMeter_tb_vhd IS
END SimpleFrequencyMeter_tb_vhd;

ARCHITECTURE behavior OF SimpleFrequencyMeter_tb_vhd IS 

	-- Component Declaration for the Unit Under Test (UUT)
	COMPONENT SimpleFrequencyMeter
	Generic ( QUARTZFREQ : integer := 40; --en Megahertz
				 ACQTIME : integer := 1000 --en millisecondesecondes
			);
	PORT(
		rst : IN std_logic;
		clk : IN std_logic;
		inclk : IN std_logic;          
		FreqCounterReady : out std_logic;
		FrequencyCounterReg : OUT std_logic_vector(31 downto 0)
		);
	END COMPONENT;

	--Inputs
	SIGNAL rst :  std_logic;
	SIGNAL clk :  std_logic;
	SIGNAL inclk :  std_logic;
	signal FreqCounterReady : std_logic;

	--Outputs
	SIGNAL FrequencyCounter1 :  std_logic_vector(31 downto 0) := (others => '0');

BEGIN

	-- Instantiate the Unit Under Test (UUT)
	uut: SimpleFrequencyMeter 
	generic map( QUARTZFREQ => 200,
	             ACQTIME => 1
	)
	PORT MAP(
		rst => rst,
		clk => clk,
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
