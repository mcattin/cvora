-- P.N. Version 1.1 le 08-05-2006 Add a signal which give a pulse when watchdog change (from version 3.7 of CIBM)
-- P.N. Version 1.2 le 19-04-2007 Version for the CVORA for slow frequency mesure

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use work.auxdef.all;

--  Uncomment the following lines to use the declarations that are
--  provided for instantiating Xilinx primitive components.
--library UNISIM;
--use UNISIM.VComponents.all;

entity FrequencyCounter is
    Port ( rst : in std_logic;
           clk : in std_logic;
			  StartCounterPulse : in std_logic;
           inclk : in std_logic;
			  FreqCounterReady : out std_logic;
			  FrequencyCounterReg : out std_logic_vector(31 downto 0));
end FrequencyCounter;

architecture Behavioral of FrequencyCounter is

signal iFrequencyCounter : std_logic_vector(31 downto 0);

begin
MESURE_PROC: process(rst, clk , StartCounterPulse, iFrequencyCounter, inclk)
begin
	if rst = RESET_ACTIVE then
		iFrequencyCounter <= (others => '0');
		FreqCounterReady <= '0';
	elsif rising_edge(clk) then
		if StartCounterPulse = '1' then
			FrequencyCounterReg <= iFrequencyCounter;
			iFrequencyCounter <= (others => '0');
			FreqCounterReady <= '1';
		elsif inclk = '1' then
			iFrequencyCounter <= iFrequencyCounter + x"0001";
		else
			FreqCounterReady <= '0';
		end if;
	end if;
end process;

end Behavioral;
