--==============================================================--
--Design Units : RS232_Rx declaration and architecture
--Size:		
--Speed:		 
--File Name: 	  RS232_Rx.vhd
--
--Purpose: RS232 Receiver to be used with the testbench
--			  for the transmitter
--
--Note:	  This model was written and developed in ISE 4.1			  
--
--Limitations:	 19200 BPS RS232 in this module, manybe some of the 
--				    signals can be made combinational instaed of rsgistered
--					 This could be an improvement at a later date.
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
--	Modified by P. Nouchi from version 3.1 (see under)
--
--Simulator:               ModelSim
--==============================================================--
--Revision List
--Version Author Date		 Changes
--
--1.0     BMT    05.02.2002 New Version
--2.0		 BMT	  15.02.2002 Instructions and constants added
--3.0		 BMT	  20.02.2002 Error Signal removed
--3.1		 P.N.	  20.06.2006 Adapted for the DPram card serial inputs from sampler-transceiver card
--                          128kBaud
--3.2		 P.N.	  31.10.2006 Generic for Number of bit, quartz frequency and baud rate
--==============================================================--
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use auxdef.RESET_ACTIVE;

entity RS232_Rx is
	 Generic ( 	NUMBER_OF_BIT : Integer := 9; -- without start and stop bits
					QUARTZFREQ : integer := 40;
					BAUD_RATE : integer := 128000
				);
    Port ( Clock : in std_logic;
           Reset : in std_logic;
           Serial : in std_logic;
           dOut : out std_logic_vector(NUMBER_OF_BIT - 1 downto 0);
           dReady : out std_logic);
end RS232_Rx;

architecture Behavioral of RS232_Rx is
signal iSerial, iSerial2 : std_logic; -- used to detect the edge of the serial code
signal iReady : std_logic;
signal Divided_Clock_Enable: std_logic; -- used to enable the sampler
signal number_of_Bits : natural range 0 to NUMBER_OF_BIT + 2; -- Add Start  and stop bits
signal iRegister: std_logic_vector (NUMBER_OF_BIT + 1 downto 0);
--==============================================================--
--constant RESET_ACTIVE : std_logic := '1'; --Change the reset to active high here
constant HALF_BIT_SAMPLE_TIME : natural := 156;--QUARTZFREQ * 500000 / BAUD_RATE;--156;
constant FULL_BIT_SAMPLE_TIME : natural := 312;-- 2 * HALF_BIT_SAMPLE_TIME;--312; -- 40000000/256000 * 2
--signal iCounter : natural range 0 to FULL_BIT_SAMPLE_TIME;
--HOW TO CALCULATE THE BAUD RATE CONSTANTS:
--1. Take the required baud rate in bps and double it. (i.e. 19200 doubles to 38400)
--2. Divide the clock frequency by this number (16384000/38400 = 427) rounded to nearest whole
		--This is the number of clock pulses in a half bit period
--3. Double this value, giving you the range of iCounter (854 in this case)
--==============================================================--
--Functionality
--as the data takes an 8N1 format, the unit waits for the serial
--data to have a falling edge, this ten triggers an internal counter
--that samples the information and shifts it into a register
--after the full number of samples has been made, the data ready signal
--is raised to show that the serial rs232 has been recieved correctly
begin
dOut <= iRegister (NUMBER_OF_BIT downto 1); -- input signal minus start and stop bits.
dReady <= iReady;
--First check for the falling edge of the serial code: two d-latches to provide 
--a synchronous one shot of the serial code
Detect_Edges: process (clock, reset)
begin
	if Reset = RESET_ACTIVE then
	iSerial <= '0';
	iSerial2 <= '0';
	elsif rising_edge (clock) then
	iSerial <= Serial;
	iSerial2 <= iSerial;
	end if;
end process Detect_Edges;
--Use the falling edge to enable the divided clock, and reset the divided clock enable
--signal when the number of bits received is ten.
Make_Sampler: process (clock, reset)
begin
	if Reset = RESET_ACTIVE then
	iReady <= '0';
	Divided_Clock_Enable <= '0';
	elsif rising_edge (clock) then
		if Number_of_Bits = NUMBER_OF_BIT + 2 then
			if iReady = '0' then -- and here we make the output a one-shot
				iReady <= '1'; -- enable the ready
			else iReady <= '0'; -- clear the ready if it has already been displayed
			end if;
			Divided_clock_Enable <= '0'; -- disable the divided clock, wait for another ready
		elsif iSerial = '0' and iSerial2 = '1' then -- i.e. if its the falling edge of the Serial
			iReady <= '0'; -- leave this here else you make an invisible latch
			Divided_Clock_Enable <= '1'; -- start the sampler, if we see a falling edge
		else iReady <= '0'; -- again control these signals in every
		end if;
	end if;
end process Make_Sampler;
-- Here generate the baud rate sampler, note that it resets to half of the baud rate 
--signal, this is so the sample occurs at exactly half way through the signal, this
--is now designed to a baud rate of 19200bps; from a 16384 Mhz clock.
Divided_Clock_Generation: process (clock, reset)
variable vCounter : natural range 0 to FULL_BIT_SAMPLE_TIME;
begin
	if Reset = RESET_ACTIVE then -- on an active low reset
	vCounter := HALF_BIT_SAMPLE_TIME; -- half of the 19200 max count value
--	iRegister <= (others => '0');
	Number_of_Bits <= 0;
	elsif rising_edge (clock) then
		if Divided_Clock_Enable = '1' then
			if vCounter = FULL_BIT_SAMPLE_TIME then -- 19200 max count value
				iRegister <= Serial & iRegister (NUMBER_OF_BIT + 1 downto 1);
				vCounter := 0;
				Number_of_Bits <= Number_of_Bits + 1;
			else  vCounter := vCounter + 1; -- increment counter
			end if;
	   else vCounter := HALF_BIT_SAMPLE_TIME;
			  Number_of_bits <= 0;
		end if;
	end if;
end process Divided_Clock_Generation;
end Behavioral;





