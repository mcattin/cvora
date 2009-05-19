----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    14:43:37 03/13/2007 
-- Design Name: 
-- Module Name:    LoadDacDelay - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 

-- the pulse to load the dac must be 45 ns minimum duration and start after 100ns
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use work.auxdef.all;

---- Uncomment the following library declaration if instantiating
---- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity LoadDacDelay is
    Port ( clk : in  STD_LOGIC;
           inputpulse : in  STD_LOGIC;
           output : out  STD_LOGIC);
end LoadDacDelay;

architecture Behavioral of LoadDacDelay is

signal ildac, ildacA, ldacLoad : std_logic := '0';

begin

MONOSTABLEldacStart: Monostable 
	Generic map( DURATION => 15,
					DEFAULT_PULSEPOLARITY => '1'
	)
	Port map( clk => clk,
				inputpulse => inputpulse, -- A clock wide pulse
				output => ildac
	);
process(clk, ildac)
begin
if rising_edge(clk) then
	ildacA <= ildac;
end if;
end process;
ldacLoad <= not ildac and  ildacA;

MONOSTABLEldac: Monostable
	Generic map( DURATION => 31,
					DEFAULT_PULSEPOLARITY => '1'
	)
	Port map( clk => clk,
				inputpulse => ldacLoad, -- A clock wide pulse
				output => output
	);


end Behavioral;

