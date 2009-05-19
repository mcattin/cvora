----------------------------------------------------------------------------------
-- Company:  CERN
-- Engineer: Philippe Nouchi
-- 
-- Create Date:    10:03:57 10/17/2006 
-- Design Name: 
-- Module Name:    SlowClock - Behavioral 
-- Project Name:   CVORA (DPRam)
-- Target Devices: 
-- Tool versions:  
-- Description:    Just a diviser to output a slow clock (~1ms)
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
-- Version 1.0 PN le 17-10-2006 If en1M is 1Mhz SlowClock is about 1ms (1024uS)
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
---- Uncomment the following library declaration if instantiating
---- any Xilinx primitives in this code.
library UNISIM;
use UNISIM.VComponents.all;

entity SlowClock is
    Port ( --rst : in std_logic;
           clk : in  STD_LOGIC;
           en1M : in  STD_LOGIC;
           SlowClockOut : out  STD_LOGIC);
end SlowClock;

architecture Behavioral of SlowClock is
signal iSlowClock : std_logic;
signal counter : std_logic_vector(9 downto 0) := "0000000000";

begin
process( clk, en1M, counter)
begin
--	if rst = '0' then
--		counter := "0000000000";
--	els
	if rising_edge(clk) then
		if en1M = '1' then
			counter <= counter + "0000000001";
		end if;
	end if;
	iSlowClock <= counter(9);
end process;
	
GBUF_FOR_SLOWCLOCK: BUFG 
   port map (I => iSlowClock, 
             O => SlowClockOut);


end Behavioral;

