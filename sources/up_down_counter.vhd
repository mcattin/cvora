--------------------------------------------------------------------------------
-- Company: 		 CERN
-- Engineer:		 P. Nouchi
--
-- Create Date:    16:05:00 09/15/05
-- Design Name:    
-- Module Name:    up_down_counter - Behavioral
-- Project Name:   
-- Target Device:  
-- Tool versions:  
-- Description:
--
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--------------------------------------------------------------------------------
-- Version 1.0 P. Nouchi le 03-04-2007 Add CounterOverflow output
-- Version 1.1 P. Nouchi le 14-02-2008 Add a one clock wide delay to deliver the terminal count signal
--------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
--use work.auxdef.all;

---- Uncomment the following library declaration if instantiating
---- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity up_down_counter is
	Generic ( COUNTER_WIDTH : integer := 32);
    Port ( rst : in std_logic;
	 		  clk : in std_logic;
           clk1 : in std_logic;
           clk2 : in std_logic;
           CounterEnable : in std_logic;
			  loadValue : in std_logic_vector(COUNTER_WIDTH - 1 downto 0);
			  CounterOverflow : out std_logic;
           InstantValue : out std_logic_vector(COUNTER_WIDTH - 1 downto 0);
           TerminalCount : out std_logic);
end up_down_counter;

architecture Behavioral of up_down_counter is

constant EXTRESET_ACTIVE : std_logic := '0';
constant VALUEZERO : std_logic_vector(COUNTER_WIDTH - 1 downto 0) := (others => '0');
constant VALUEONE : std_logic_vector(COUNTER_WIDTH - 1 downto 0) := (0 => '1', others => '0');--x"000001";
--constant VALUETWO : std_logic_vector(COUNTER_WIDTH - 1 downto 0) := (1 => '1', others => '0');--x"000010";
constant UP : std_logic := '1'; 
constant DOWN : std_logic := '0';
constant OVERFLOW: std_logic_vector(COUNTER_WIDTH - 1 downto 0) := (others => '1'); 

signal tc : std_logic := '0';
signal iInstantValue : std_logic_vector(COUNTER_WIDTH - 1 downto 0) := (others => '0');
signal clk1A, clk2A, clk1B, clk2B, clk1RE, clk2RE : std_logic := '0';
signal UDCounterValid: std_logic := '0';

begin

UDCounterValid <= CounterEnable;
-- need a pulse at the rising_edge of ExtStart, clk1 and clk2
CLK1RE_PROC: process(rst, clk, clk1, clk2)
begin 
if rising_edge(clk) then
	clk1A <= clk1;
	clk2A <= clk2;
	clk1B <= clk1A;
	clk2B <= clk2A;
end if;
end process;
clk1RE <= CounterEnable and clk1A and not(clk1B);
clk2RE <= CounterEnable and clk2A and not(clk2B);

COUNT_PROC: process(rst, clk, clk1RE, clk2RE, LoadValue, UDCounterValid)
begin
if rising_edge(clk) then
	if rst = '1' then
		iInstantValue <= LoadValue;
	elsif UDCounterValid = '1' then
		if clk1RE = '1' and clk2RE = '0' then
			iInstantValue <= iInstantValue + VALUEONE;
		elsif clk2RE = '1' and clk1RE = '0' then
			iInstantValue <= iInstantValue - VALUEONE;
		end if;
	end if;
end if;
end process;

OVERFLOW_PROC: process(rst, clk, iInstantValue)
begin
	if rst = '1' then
		CounterOverflow <= '0';
	elsif rising_edge(clk) then
		if iInstantValue = OVERFLOW and CounterEnable = '1' then
			CounterOverflow <= '1';
		else 
			CounterOverflow <= '0';
		end if;
	end if;
end process;

-- just to have a signal clk wide to load the External DAC
tc <= clk1RE or clk2RE; --tcup when direction = UP else tcdown;
process(clk)
begin
if rising_edge(clk) then		
	TerminalCount <= tc; 
end if;
end process;
	
InstantValue <= iInstantValue;

end Behavioral;
