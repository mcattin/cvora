----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    16:06:45 03/30/2007 
-- Design Name: 
-- Module Name:    RamManagerSTM - Behavioral 
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

entity RamManagerSTM is
    Port ( rst : in  STD_LOGIC;
           clk : in  STD_LOGIC;
           Mode : in  STD_LOGIC_VECTOR (2 downto 0);
           DataIn : in  P2Array;
           WriteRam : in  STD_LOGIC;
           WriteAdd : in  std_logic_vector(MEM_ADDRESS_LENGTH - 1 downto 0);
           ReadAdd : in  std_logic_vector(MEM_ADDRESS_LENGTH - 1 downto 0);
           readRam : in  STD_LOGIC;
			  ChannelReg :in std_logic_vector(31 downto 0);
			  DataWritten: in std_logic;
           MemPointer : out  STD_LOGIC_VECTOR (31 downto 0));
end RamManagerSTM;

architecture Behavioral of RamManagerSTM is

type BufferState_type is 
	(iddle, WriteChannel-1, WriteChannel-2, WriteChannel-3,WriteChannel-4, WriteChannel-5, WriteChannel-6,WriteChannel-7, WriteChannel-8, 
	WriteChannel-9,WriteChannel-10, WriteChannel-11, WriteChannel-12, WriteChannel-13, WriteChannel-14, WriteChannel-15,
	WriteChannel-16, WriteChannel-17, WriteChannel-18, WriteChannel-19, WriteChannel-20,WriteChannel-21, WriteChannel-22, 
	WriteChannel-23, WriteChannel-24, WriteChannel-25, WriteChannel-26, WriteChannel-27, WriteChannel-28,WriteChannel-29,
	WriteChannel-30, WriteChannel-31, WriteChannel-32); 

signal state, next_state : BufferState_type; 
signal DataReady : std_logic;
signal P2SdataIn: std_logic_vector(NUMBER_OF_CHANNEL - 1 downto 0);
signal P2SdataEnable: std_logic_vector(NUMBER_OF_CHANNEL - 1 downto 0);
signal P2DataOut : P2Array;
signal P2DataReady: P2DataReadyArray;

begin
-- ************************************
-- * Serial Data Inputs on P2 connector
-- ************************************
P2SdataEnable <= ChannelReg(NUMBER_OF_CHANNEL - 1 downto 0) when UsedMode = P2SERIALMODE else
						(others => '0');
P2SdataIn <= PDataIn( NUMBER_OF_CHANNEL - 1 downto 0);

P2SERIAL_LOOP: for I in 1 to NUMBER_OF_CHANNEL generate

P2Serial_Receiver:  P2Serial_Receiver
    Port map(	rst => rst,
					clk => clk,
					Enable => P2SdataEnable(I-1),
					SDataIn => P2SdataIn(I-1),
					DataOut => P2DataOut(I),
					DataReady => P2DataReady(I)
		);
end generate;


-- Synch the outputs				
SYNC_PROC: process (rst, clk)
begin
	if rst = RESET_ACTIVE then
		state <= iddle;
	elsif rising_edge(clk) then
		if Mode = P2SERIALMODE then
			state <= next_state;
		end if;
	end if;
end process;


NEXT_STATE_DECODE: process (state, dataWritten)
   begin
      next_state <= state;  --default is to stay in current state
      case (state) is
         when iddle =>
--            if StateEvent = '1' then
--               next_state <= Input;
--				elsif MaskEvent = '1' then
--					next_state <= Mask;
--				elsif SbfEvent = '1' then
--					next_state <= Sbf;
--				elsif TestEvent = '1' then
--					next_state <= Test;
--				elsif TimingEvent = '1' then
--					next_state <= Timing;
--				elsif LoopEvent = '1' then
--					next_state <= Loop10M;
--				elsif RearmEvent = '1' then
--					next_state <= Rearm;
--				else next_state <= iddle;
--            end if;
--			when Timing  =>
--            if DataWritten = '1' then
--					next_state <= Second;
--				end if;				
         when others => -- wait for the record is written (5 clock ticks)
            if DataWritten = '1' then
					next_state <= iddle;
				end if;
      end case;      
   end process;


end Behavioral;

