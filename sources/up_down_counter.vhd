--------------------------------------------------------------------------------
-- Company:       CERN
-- Engineer:      P. Nouchi
--
-- Create Date:   16:05:00 09/15/05
-- Design Name:
-- Module Name:   up_down_counter - Behavioral
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
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;


entity up_down_counter is
  generic (
    COUNTER_WIDTH : integer := 32);
  port (
    rst             : in  std_logic;
    clk             : in  std_logic;
    clk1            : in  std_logic;
    clk2            : in  std_logic;
    CounterEnable   : in  std_logic;
    loadValue       : in  std_logic_vector(COUNTER_WIDTH - 1 downto 0);
    CounterOverflow : out std_logic;
    InstantValue    : out std_logic_vector(COUNTER_WIDTH - 1 downto 0);
    TerminalCount   : out std_logic);
end up_down_counter;


architecture Behavioral of up_down_counter is


  constant OVERFLOW        : unsigned(COUNTER_WIDTH - 1 downto 0) := (others => '1');

  signal iInstantValue : unsigned(COUNTER_WIDTH - 1 downto 0) := (others => '0');

  signal tc             : std_logic := '0';
  signal clk1A          : std_logic;
  signal clk2A          : std_logic;
  signal clk1B          : std_logic;
  signal clk2B          : std_logic;
  signal clk1RE         : std_logic;
  signal clk2RE         : std_logic := '0';
  signal UDCounterValid : std_logic := '0';


begin

  UDCounterValid <= CounterEnable;
-- need a pulse at the rising_edge of ExtStart, clk1 and clk2
  CLK1RE_PROC : process(rst, clk, clk1, clk2)
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

  COUNT_PROC : process(rst, clk, clk1RE, clk2RE, LoadValue, UDCounterValid)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        iInstantValue <= unsigned(LoadValue);
      elsif UDCounterValid = '1' then
        if clk1RE = '1' and clk2RE = '0' then
          iInstantValue <= iInstantValue + 1;
        elsif clk2RE = '1' and clk1RE = '0' then
          iInstantValue <= iInstantValue - 1;
        end if;
      end if;
    end if;
  end process;

  OVERFLOW_PROC : process(rst, clk, iInstantValue)
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
  tc <= clk1RE or clk2RE;               --tcup when direction = UP else tcdown;
  process(clk)
  begin
    if rising_edge(clk) then
      TerminalCount <= tc;
    end if;
  end process;

  InstantValue <= std_logic_vector(iInstantValue);

end Behavioral;
