--==============================================================--
--Design Units : RS232_Tx declaration and architecture
--Size:		
--Speed:		 
--File Name: 	  RS232_Tx.vhd
--
--Purpose: this module takes in one byte, transmits it serially
--			  in the RS232 format 8N1
--			  eight data bits a start bit of '0'and a stop bit of '1'
--
--
--Note:	  This model was written and developed in ISE 4.1			  
--
--Limitations:	 
--
--Errors: 
--
--Libraries:		 	 
--
--Dependancies:
--
--Author: Benjamin Todd
--		    European Organisation for Nuclear Research
--		 	 SL SPS/LHC -- Control -- Timing Division
--		 	 CERN, Geneva, Switzerland,  CH-1211
--		 	 Building 864 Room 1 - A24
--
--Simulator:               ModelSim
--==============================================================--
--Revision List
--Version Author Date		 Changes
--
--1.0     BMT    25.01.2002 New Version
--2.0		 BMT	  04.02.2002 Baud rate analysis completed
--2.0		 BMT	  04.02.2002 Baud rate changed to 9600
--3.0		 PN	  04.02.2002 Reset Active High
--4.0		 BMT	  04.02.2002 Shift Register Optimised to 9 bits
--4.1		 BMT	  04.02.2002 Meta Stability issue removed by extending shift register to 10 bits
--5.0		 BMT	  05.02.2002 Baud rate optimised to 19200 BPS
--==============================================================--
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
--use IEEE.STD_LOGIC_UNSIGNED.ALL;
use work.auxdef.all;

entity RS232_Tx is
    Port ( Clock : in std_logic; -- 40 MHZ clock 
           Reset : in std_logic; -- active low reset
           Data_Ready : in std_logic; -- signal to indicate data ready on inputs
           Data : in std_logic_vector (7 downto 0); -- one byte of data
           RS232_Serial : out std_logic; -- serially generated code
			  Ready: out std_logic); -- signal to say a reload can take place when logic one
end RS232_Tx;

architecture Behavioral of RS232_Tx is

--==============================================================--
--constant RESET_ACTIVE : std_logic := '1';	in auxdef package
--Change the reset activity here
constant FULL_BIT_SAMPLE_TIME : natural := 2083; -- 2083 for 40 Mhz      --521 for 10 Mhz 
--HOW TO CALCULATE THE BAUD RATE CONSTANTS:
--1. Take the required baud rate in bps and double it. (i.e. 19200 doubles to 38400)
--2. Divide the clock frequency by this number (16384000/38400 = 427) rounded to nearest whole
		--This is the number of clock pulses in a half bit period
--3. Double this value, giving you the range of iCounter (854 in this case)
--==============================================================--

signal iData : std_logic_vector (9 downto 0); -- full data vector start and stop bits included
signal divided_clock : std_logic; -- divided clock is the baud rate
signal iReady : std_logic; -- this is the delayed version of Ready, assures that stop bit gets sent
signal vClockCounter : natural range 0 to FULL_BIT_SAMPLE_TIME;

begin
--==============================================================--
--small shift register
Transmit_Frame : process (clock, reset, Data_Ready, Data, divided_clock, iReady, iData)
begin
if Reset = RESET_ACTIVE then
iData <= (others => '1'); -- Reset the register to all ones
Ready <= '1'; -- set the unit to ready
elsif rising_edge (clock) then
	if Data_Ready = '1' then -- latch the data when its ready
	iData <= Data & '0' & '1'; -- add a start bit
	Ready <= '0';-- force ready to low, as the unit is not ready
	elsif Divided_Clock = '1' then
	Ready <= iReady; -- shift in the ready signal, i.e. so we make a stop bit by delaying
 	iData <= '1' & iData (9 downto 1); -- shift register
	end if;
end if;
end process Transmit_Frame;
RS232_Serial <= iData (0); -- take the RS232 as the last bit of the register
--==============================================================--
--Note: its actually less logic to transmit at a higher baud rate
--as counter can be shorter, changing the count to value here changes
--the baud rate.
divide_Clock: process (clock, reset, vClockCounter)
	begin
	if reset = RESET_ACTIVE then
	vClockCounter <= 0;
	Divided_Clock <= '0';
	elsif rising_edge (clock) then
		if vClockCounter = FULL_BIT_SAMPLE_TIME then -- <<CHANGE Baud Rate here
		Divided_Clock <= '1'; -- create a one shot every time the counter is reset
		vClockCounter <= 0; -- reset the counter
		else vClockCounter <= vClockCounter + 1; -- increment the counter
			  Divided_Clock <= '0'; -- clear the one shot
		end if;
	end if;
end process divide_clock;
--==============================================================--
Set_ready : process (clock, reset, iData)
begin
	if Reset = RESET_ACTIVE then
	iReady <= '1'; -- set i ready to one to enable the unit
	elsif rising_edge (clock) then
		if iData = "1111111111" then -- if the shift register has been cleared
		iReady <= '1'; -- then the unit can be reloaded
		else iReady <= '0'; -- else it cant be.
		end if;
	end if;
end process Set_ready;
--==============================================================--

end Behavioral;
