-- P.N. Version 1.1 le 08-05-2006 Add a signal which give a pulse when watchdog change (from version 3.7 of CIBM)
-- P.N. Version 1.2 le 19-04-2007 Version for the CVORA for slow frequency mesure
-- P.N. Version 1.3 le 26-04-2006 Simple Frequency Meter generic component with own acquisition time
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use work.auxdef.all;

--  Uncomment the following lines to use the declarations that are
--  provided for instantiating Xilinx primitive components.
--library UNISIM;
--use UNISIM.VComponents.all;

entity SimpleFrequencyMeter is
	Generic ( QUARTZFREQ : integer := 40; -- en Megahertz
				 ACQTIME : integer := 1000 --en millisecondesecondes
			);
    Port ( rst : in std_logic;
           clk : in std_logic;
           inclk : in std_logic;
			  FreqCounterReady : out std_logic;
			  FrequencyCounterReg : out std_logic_vector(31 downto 0));
end SimpleFrequencyMeter;

architecture Behavioral of SimpleFrequencyMeter is

constant ACQUISITION : integer := (QUARTZFREQ * ACQTIME * 1000) - 1;
signal iFrequencyCounter : std_logic_vector(31 downto 0);
signal StartCounterPulse : std_logic := '0';
signal acqCounter : integer range 0 to ACQUISITION := 0;

BEGIN
-- Construct free running acquisition  down Counter 
acq_Process: process(clk)
begin
	if rising_edge(clk) then
		if acqCounter = 0 then
			StartCounterPulse <= '1';
			acqCounter <= ACQUISITION;
		else
			StartCounterPulse <= '0';
			acqCounter <= acqCounter - 1;
		end if;
	end if;
end process;

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
