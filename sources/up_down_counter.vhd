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
--         2.0  M.Cattin  25.10.2013 make 
--------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;


entity up_down_counter is
  generic (
    g_width : integer := 32);
  port (
    rst_n_i    : in  std_logic;
    clk_i      : in  std_logic;
    clear_i    : in  std_logic;
    up_i       : in  std_logic;
    down_i     : in  std_logic;
    enable_i   : in  std_logic;
    overflow_o : out std_logic;
    value_o    : out std_logic_vector(g_width - 1 downto 0);
    valid_o    : out std_logic);
end up_down_counter;


architecture rtl of up_down_counter is


  signal cnt : unsigned(g_width - 1 downto 0);


begin


  p_cnt : process (clk_i, rst_n_i)
  begin
    if rst_n_i = '0' then
      cnt <= (others => '0');
    elsif rising_edge(clk_i) then
      if clear_i = '1' then
        cnt <= (others => '0');
      elsif enable_i = '1' then
        if up_i = '1' and down_i = '0' then
          cnt <= cnt + 1;
        elsif up_i = '0' and down_i = '1' then
          cnt <= cnt - 1;
        end if;
      end if;
    end if;
  end process p_cnt;

  value_o <= std_logic_vector(cnt);

  p_cnt_overflow : process (clk_i, rst_n_i)
  begin
    if rst_n_i = '0' then
      overflow_o <= '0';
    elsif rising_edge(clk_i) then
      if clear_i = '1' then
        overflow_o <= '0';
      elsif cnt = x"FFFFFFFF" and enable_i = '1' then
        overflow_o <= '1';
      else
        overflow_o <= '0';
      end if;
    end if;
  end process p_cnt_overflow;

  p_cnt_to_dac : process (clk_i, rst_n_i)
  begin
    if rst_n_i = '0' then
      valid_o <= '0';
    elsif rising_edge(clk_i) then
      if up_i = '1' or down_i = '1' then
        valid_o <= '1';
      else
        valid_o <= '0';
      end if;
    end if;
  end process p_cnt_to_dac;


end rtl;
