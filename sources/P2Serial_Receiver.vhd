----------------------------------------------------------------------------------
-- Company:        CERN
-- Engineer:       Philippe Nouchi
--
-- Create Date:    10:48:27 10/31/2006
-- Design Name:
-- Module Name:    P2Serial_Receiver - Behavioral
-- Project Name:
-- Target Devices:
-- Tool versions:  ise 9.1
-- Description:    The serial receiver of the CVORA card on P2 connector
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


entity P2Serial_Receiver is
  port (
    rst       : in  std_logic;
    clk       : in  std_logic;
    Enable    : in  std_logic;
    SDataIn   : in  std_logic;
    DataOut   : out std_logic_vector (15 downto 0);
    DataReady : out std_logic
    );
end P2Serial_Receiver;


architecture Behavioral of P2Serial_Receiver is


  constant NUMBER_OF_BIT : integer := 9;

  signal idOut      : std_logic_vector(NUMBER_OF_BIT - 1 downto 0);
  signal idReady    : std_logic;
  signal odready    : std_logic                    := '0';
  signal FirstByte  : std_logic_vector(7 downto 0) := (others => '0');
  signal SecondByte : std_logic_vector(7 downto 0) := (others => '0');
  signal iSDataIn   : std_logic;


begin


  iSDataIn <= SDataIn and Enable;
  RS232_Rx : RS232_Rx
    generic map (
      NUMBER_OF_BIT => NUMBER_OF_BIT,   -- without start and stop bits
      QUARTZFREQ    => QUARTZFREQ,
      BAUD_RATE     => 128000
      )
    port map (
      Clock  => clk,
      Reset  => rst,
      Serial => iSDataIn,
      dOut   => idOut,
      dReady => idReady
      );

  Store_Data_proc : process(clk, idOut, idReady)
  begin
    if rising_edge(clk) then
      if idReady = '1' and Enable = '1' then
        if idOut(8) = '0' then          -- if bit 8 = 1 means MSB else LSB
          FirstByte <= idOut(7 downto 0);
          odready   <= '1';
        else
          SecondByte <= idOut(7 downto 0);
        end if;
      else odready <= '0';
      end if;
    end if;
  end process;

  DataOut   <= SecondByte & FirstByte;
  DataReady <= odReady;

end Behavioral;

