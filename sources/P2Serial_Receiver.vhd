----------------------------------------------------------------------------------
-- Company: 		CERN
-- Engineer: 		Philippe Nouchi
-- 
-- Create Date:    10:48:27 10/31/2006 
-- Design Name: 
-- Module Name:    P2Serial_Receiver - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: ise 9.1
-- Description:   The serial receiver of the CVORA card on P2 connector
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
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

entity P2Serial_Receiver is
    Port ( rst : in  STD_LOGIC;
           clk : in  STD_LOGIC;
			  Enable : in std_logic;
           SDataIn : in  STD_LOGIC;
           DataOut : out  STD_LOGIC_VECTOR (15 downto 0);
           DataReady : out  STD_LOGIC
	);
end P2Serial_Receiver;

architecture Behavioral of P2Serial_Receiver is

constant NUMBER_OF_BIT : integer := 9;

signal idOut: std_logic_vector( NUMBER_OF_BIT - 1 downto 0);
signal idReady: std_logic;
signal odready: std_logic := '0';
signal FirstByte, SecondByte: std_logic_vector(7 downto 0) := (others => '0');
signal iSDataIn: std_logic;

begin
iSDataIn <= SDataIn and Enable;
RS232_Rx: RS232_Rx
	Generic map ( 	NUMBER_OF_BIT => NUMBER_OF_BIT, -- without start and stop bits
						QUARTZFREQ => QUARTZFREQ,
						BAUD_RATE => 128000
				)
	Port map (	Clock => clk,
					Reset => rst,
					Serial => iSDataIn,
					dOut => idOut,
					dReady => idReady
				);
Store_Data_proc: process( clk, idOut, idReady)
begin
	if rising_edge(clk) then
		if idReady = '1' and Enable = '1' then 
			if idOut(8) = '0' then  -- if bit 8 = 1 means MSB else LSB
				FirstByte <= idOut(7 downto 0);
				odready <= '1';
			else 
				SecondByte <= idOut(7 downto 0);
			end if;
		else odready <= '0';
		end if;
	end if;
end process;

DataOut <= SecondByte & FirstByte;
DataReady <= odReady;
				
end Behavioral;

