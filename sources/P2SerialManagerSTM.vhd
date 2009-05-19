----------------------------------------------------------------------------------
-- Company: 		CERN
-- Engineer: 		Philippe Nouchi
-- 
-- Create Date:    16:06:45 03/30/2007 
-- Design Name: 
-- Module Name:    P2SerialManagerSTM - Behavioral 
-- Project Name: 	 CVORA
-- Target Devices: 
-- Tool versions:  Ise9.1
-- Description:    State machine to serialize and write the input data from P2 connector to External RAM 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
-- Version 1.1 P.N. le 05-04-2007 Remove P2MemBusy input
-- Version 1.2 P.N. le 05-04-2007 Need a time out if one of the rear inputs are not cabled 
-- Version 1.3 P.N. le 10-04-2007 Need a buffer to store data when they arrive to avoid waiting for P2DataReady (No timeout needed now)
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use work.auxdef.all;
use work.IntContPackage.all;

---- Uncomment the following library declaration if instantiating
---- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity P2SerialManagerSTM is
    Port ( rst : in  STD_LOGIC;
           clk : in  STD_LOGIC;
           Mode : in  STD_LOGIC_VECTOR (2 downto 0);
           DataIn : in  std_logic_vector(31 downto 0);
			  DataInClk : std_logic;
			  ChannelReg :in std_logic_vector(31 downto 0);
			  DataWritten: in std_logic;   -- the Ram is ready to accept another write
			  ClearMem : in std_logic;
           WriteRam : out  STD_LOGIC;
           WriteAdd : out  std_logic_vector(MEM_ADDRESS_LENGTH - 1 downto 0);
			  P2RamAddOverflow: out std_logic;
           DatatoRAM : out  STD_LOGIC_VECTOR (31 downto 0));
end P2SerialManagerSTM;

architecture Behavioral of P2SerialManagerSTM is

type BufferState_type is 
	(iddle, WriteChannel, LoadLSB, LoadMSB, LoadMSBandLSB, WriteInRam, WaitRam, IncAdd, IncChannel, 
	AllChannelsProcessed, MemoryFull, EraseMemory);

signal state, next_state : BufferState_type; 
signal P2SdataIn: std_logic_vector(NUMBER_OF_CHANNEL - 1 downto 0);
signal P2SdataEnable: std_logic_vector(NUMBER_OF_CHANNEL - 1 downto 0);
signal P2DataOut : P2Array;
signal P2DataReady: P2DataReadyArray;
signal CurrentChannel: integer range 1 to NUMBER_OF_CHANNEL - 1 := 1; 
signal NextChannel: integer range 2 to NUMBER_OF_CHANNEL := 2; 
signal iWriteAdd : std_logic_vector(MEM_ADDRESS_LENGTH - 1 downto 0) := MEMEMPTY;
signal DataBuffer: P2Array;

begin
-- ************************************
-- * Serial Data Inputs on P2 connector
-- ************************************

process(ChannelReg, Mode)
begin 
	if Mode = P2SERIALMODE then
		P2SdataEnable <= ChannelReg(NUMBER_OF_CHANNEL - 1 downto 0);
	else
		P2SdataEnable <= (others => '0');
	end if;
end process;

P2SdataIn <= DataIn( NUMBER_OF_CHANNEL - 1 downto 0);

P2SERIAL_LOOP: for I in 1 to NUMBER_OF_CHANNEL generate

P2Serial_Receiver:  P2Serial_Receiver
    Port map(	rst => rst,
					clk => clk,
					Enable => P2SdataEnable(I-1),
					SDataIn => P2SdataIn(I-1),
					DataOut => P2DataOut(I),
					DataReady => P2DataReady(I)
		);
		
-- I need an intermediate buffer to store data when they are ready
process(rst, clk, P2DataReady(I), P2DataOut(I), P2SdataEnable(I-1))
begin
if rst = RESET_ACTIVE then
	DataBuffer(I) <= (others => '0');
elsif rising_edge(clk) then
	if P2DataReady(I) = '1' and P2SdataEnable(I-1) = '1' then
		DataBuffer(I) <= P2DataOut(I);
	elsif P2SdataEnable(I-1) = '0' then
		DataBuffer(I) <= (others => '0');
	end if;
end if;
end process;

end generate;


-- Synch the outputs	only in the P2SerialMODE			
SYNC_PROC: process (rst, clk, Mode)
begin
	if rst = RESET_ACTIVE then
		state <= iddle;
	elsif rising_edge(clk) then
		if Mode = P2SERIALMODE then
			state <= next_state;
		else state <= iddle;
		end if;
	end if;
end process;


NEXT_STATE_DECODE: process (state, dataWritten, DataInClk, iWriteAdd, CurrentChannel, NextChannel, ClearMem, ChannelReg)
   begin
      next_state <= state;  --default is to stay in current state
      case state is
         when iddle =>
				if ClearMem = '1' then
					next_state <= EraseMemory;
				elsif DataInClk = '1' then
					next_state <= WriteChannel;
				elsif iWriteAdd = EXTRAMFULL then
					next_state <= MemoryFull;
				end if;

			when WriteChannel  =>
				if ChannelReg(CurrentChannel downto (CurrentChannel - 1)) = "11" then
					next_state <= LoadMSBandLSB;
				elsif ChannelReg(CurrentChannel downto (CurrentChannel - 1)) = "10" then	
					next_state <= LoadMSB; 
				elsif ChannelReg(CurrentChannel downto (CurrentChannel - 1)) = "01" then
					next_state <= LoadLSB;
				else
					next_state <= IncChannel;
				end if;

			when LoadMSBandLSB => 
				next_state <= WriteInRam;
			
			when LoadLSB =>
				next_state <= WriteInRam;

			when LoadMSB =>
				next_state <= WriteInRam;

			when WriteInRam =>
				next_state <= WaitRam;

			when WaitRam => -- wait until the RAMManager has finished
				if ClearMem = '1' then
					next_state <= EraseMemory;
				elsif DataWritten = '1' then
					next_state <= IncAdd;
				end if;

			when IncAdd =>
				if iWriteAdd = EXTRAMFULL - 1 then
					next_state <= MemoryFull;
--				elsif CurrentChannel = 1 then
--					next_state <= iddle;
				else
					next_state <= IncChannel;
				end if;
			
			when IncChannel =>
				if CurrentChannel = NUMBER_OF_CHANNEL - 1 or NextChannel = NUMBER_OF_CHANNEL then
					next_state <= AllChannelsProcessed;
				else 
					next_state <= WriteChannel;
				end if;
				
			when MemoryFull =>  -- stay here until memory is cleared by external Reset
				if ClearMem = '1' then
					next_state <= EraseMemory;
				end if;

			when EraseMemory => -- the memory pointer is set back to the first address of the memory
				next_state <= iddle;

			when AllChannelsProcessed =>
				next_state <= iddle;
				
         when others => -- wait for the record is written (5 clock ticks)
				next_state <= iddle;
      end case;      
   end process;

Outputs_PROC: process(clk, state, P2DataReady, P2DataOut, NextChannel, CurrentChannel)
begin
	if rising_edge(clk) then
		case state is
			when iddle =>
				P2RamAddOverflow <= '0';
				WriteRam <= '0';
				CurrentChannel <= 1;
				NextChannel <= 2;

			when WriteChannel =>
				WriteRam <= '0';

			when LoadMSBandLSB =>
				if Nextchannel <= NUMBER_OF_CHANNEL and CurrentChannel <= NUMBER_OF_CHANNEL - 1 then
					DataToRam <= DataBuffer(NextChannel) & DataBuffer(CurrentChannel);
				end if;
				
			when LoadLSB =>
				if CurrentChannel <= NUMBER_OF_CHANNEL - 1 then
					DataToRam <= x"0000" & DataBuffer(CurrentChannel);
				end if;

			when LoadMSB =>
				if Nextchannel <= NUMBER_OF_CHANNEL then
					DataToRam <=  DataBuffer(NextChannel) & x"0000";
				end if;

			when WriteInRam =>
				WriteRam <= '1';				

			when WaitRam =>
				WriteRam <= '0';

			when IncAdd =>
				iWriteAdd <= iWriteAdd + EXTRAM_BUF_ONE;
				
			when IncChannel =>
				if CurrentChannel = NUMBER_OF_CHANNEL - 1 or NextChannel = NUMBER_OF_CHANNEL then
					CurrentChannel <= 1;
					NextChannel <= 2;
				else
					CurrentChannel <= CurrentChannel + 2;
					NextChannel <= NextChannel + 2;
				end if;
				
			when AllChannelsProcessed =>
				WriteRam <= '0';
				NextChannel <= 2;
				CurrentChannel <= 1;

			when MemoryFull =>  -- stay here until memory is cleared by external Reset
				WriteRam <= '0';
				P2RamAddOverflow <= '1';

			when EraseMemory => -- the memory pointer is set back to the first address of the memory
				iWriteAdd <= MEMEMPTY;
				P2RamAddOverflow <= '0';

			when others =>
				WriteRam <= '0';

		end case;
	end if;
end process;

WriteAdd <= iWriteAdd;
--P2MemBusy <= iP2MemBusy when Mode = P2SERIALMODE else '0';

end Behavioral;

