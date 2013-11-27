----------------------------------------------------------------------------------
-- Company:        CERN
-- Engineer:       Philippe Nouchi
--
-- Create Date:    10:48:27 10/31/2006
-- Design Name:
-- Module Name:    Serial_Receiver - Behavioral
-- Project Name:
-- Target Devices:
-- Tool versions:  ise 8.3
-- Description:    The serial receiver of the CVORA card
--
-- Dependencies:
--
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_ARITH.all;
use IEEE.STD_LOGIC_UNSIGNED.all;
use work.auxdef.all;


entity Serial_Receiver is
  port (
    rst           : in  std_logic;
    clk           : in  std_logic;
    mode          : in  std_logic;
    SDataIn1      : in  std_logic;
    SDataIn2      : in  std_logic;
    DataOut       : out std_logic_vector (31 downto 0);
    DataReady     : out std_logic;
    ChannelReady1 : out std_logic;
    ChannelReady2 : out std_logic
    );
end Serial_Receiver;


architecture Behavioral of Serial_Receiver is


  constant NUMBER_OF_BIT : integer := 9;

  signal idOut1, idOut2                               : std_logic_vector(NUMBER_OF_BIT - 1 downto 0);
  signal idReady1, idReady2                           : std_logic;
  signal odready1, odready2                           : std_logic                    := '0';
  signal FirstByte, SecondByte, ThirdByte, FourthByte : std_logic_vector(7 downto 0) := (others => '0');


begin


  RS232_Rx1 : RS232_Rx
    generic map (
      NUMBER_OF_BIT => NUMBER_OF_BIT,   -- without start and stop bits
      QUARTZFREQ    => QUARTZFREQ,
      BAUD_RATE     => 128000
      )
    port map (
      Clock  => clk,
      Reset  => rst,
      Serial => SDataIn1,
      dOut   => idOut1,
      dReady => idReady1
      );

  RS232_Rx2 : RS232_Rx
    generic map (
      NUMBER_OF_BIT => NUMBER_OF_BIT,   -- without start and stop bits
      QUARTZFREQ    => QUARTZFREQ,
      BAUD_RATE     => 128000
      )
    port map (
      Clock  => clk,
      Reset  => rst,
      Serial => SDataIn2,
      dOut   => idOut2,
      dReady => idReady2
      );

  Store_Data_proc1 : process(clk, idOut1, idReady1)
  begin
    if rising_edge(clk) then
      if idReady1 = '1' then
        if idOut1(8) = '0' then         -- if bit 8 = 1 means MSB else LSB
          FirstByte <= idOut1(7 downto 0);
          odready1  <= '1';
        else
          SecondByte <= idOut1(7 downto 0);
        end if;
      else odready1 <= '0';
      end if;
    end if;
  end process;

  Store_Data_proc2 : process(clk, idOut2, idReady2)
  begin
    if rising_edge(clk) then
      if idReady2 = '1' then
        if idOut2(8) = '0' then
          ThirdByte <= idOut2(7 downto 0);
          odready2  <= '1';
        else
          FourthByte <= idOut2(7 downto 0);
        end if;
      else odready2 <= '0';
      end if;
    end if;
  end process;

  DataOut       <= FourthByte & ThirdByte & SecondByte & FirstByte;
  DataReady     <= odReady1 when mode = '0' else odReady2;
  ChannelReady1 <= odReady1;
  ChannelReady2 <= odReady2;

end Behavioral;

