----------------------------------------------------------------------------------
-- Company: 		CERN
-- Engineer: 		Philippe Nouchi
-- 
-- Create Date:    11:41:05 10/17/2006 
-- Design Name: 
-- Module Name:    Monostable - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 	A simple monostable
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
-- The monostable is retriggerable
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

---- Uncomment the following library declaration if instantiating
---- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity Monostable is
	Generic( DURATION : natural := 255;
				DEFAULT_PULSEPOLARITY : std_Logic := '1' -- Determine polarity of the inputs
				-- '0' is from TG8 (negative logic), '1' if inputs come from new timing modules
	); 		
	Port ( 	clk : in  STD_LOGIC;
				inputpulse : in  STD_LOGIC; -- A clock wide pulse
				output : out  STD_LOGIC);
end Monostable;

architecture Behavioral of Monostable is

signal iOutput : std_logic := '0';
signal uCounter : natural := 0;

begin

monostable_300mS: process( inputpulse, clk)
begin
if rising_edge(clk) then
	if inputpulse = '1' then  
		iOutput <= '1';
	elsif iOutput = '1' then
		if uCounter = DURATION then
			iOutput <= '0';
		   ucounter <= 0;
		else 
			ucounter <= ucounter + 1;
		end if;
	else
		iOutput <= '0';
	end if;
end if;
end process;
output <= iOutput;
end Behavioral;

