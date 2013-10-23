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
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_ARITH.all;
use IEEE.STD_LOGIC_UNSIGNED.all;
use work.auxdef.all;


entity LoadDacDelay is
  port (
    clk        : in  std_logic;
    inputpulse : in  std_logic;
    output     : out std_logic);
end LoadDacDelay;


architecture Behavioral of LoadDacDelay is


  signal ildac, ildacA, ldacLoad : std_logic := '0';


begin

  MONOSTABLEldacStart : Monostable
    generic map(
      DURATION              => 15,
      DEFAULT_PULSEPOLARITY => '1'
      )
    port map(
      clk        => clk,
      inputpulse => inputpulse,         -- A clock wide pulse
      output     => ildac
      );

  process(clk, ildac)
  begin
    if rising_edge(clk) then
      ildacA <= ildac;
    end if;
  end process;
  ldacLoad <= not ildac and ildacA;

  MONOSTABLEldac : Monostable
    generic map(
      DURATION              => 31,
      DEFAULT_PULSEPOLARITY => '1'
      )
    port map(
      clk        => clk,
      inputpulse => ldacLoad,           -- A clock wide pulse
      output     => output
      );


end Behavioral;

