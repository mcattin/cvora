--------------------------------------------------------------------------------
-- CERN (BE-CO-HT)
-- Load signal delay for AD669 DAC
--------------------------------------------------------------------------------
--
-- unit name: dac_load_delay
--
-- author: Matthieu Cattin (matthieu.cattin@cern.ch)
--
-- date: 12-11-2013
--
-- description: Takes a 1 tick pulse as input, delays it for 125ns and extend it
--              to 50ns.
--              Requires a 40 MHz clock
--
-- dependencies: gc_extend_pulse (git://ohwr.org/hdl-core-lib/general-cores.git)
--
-- references: http://www.analog.com/static/imported-files/data_sheets/AD669.pdf
--
--------------------------------------------------------------------------------
-- GNU LESSER GENERAL PUBLIC LICENSE
--------------------------------------------------------------------------------
-- This source file is free software; you can redistribute it and/or modify it
-- under the terms of the GNU Lesser General Public License as published by the
-- Free Software Foundation; either version 2.1 of the License, or (at your
-- option) any later version. This source is distributed in the hope that it
-- will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
-- of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
-- See the GNU Lesser General Public License for more details. You should have
-- received a copy of the GNU Lesser General Public License along with this
-- source; if not, download it from http://www.gnu.org/licenses/lgpl-2.1.html
--------------------------------------------------------------------------------
-- last changes: see git log.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.gencores_pkg.all;


entity dac_load_delay is
  port (
    rst_n_i   : in  std_logic;
    clk_i     : in  std_logic;
    pulse_i   : in  std_logic;
    d_pulse_o : out std_logic);
end dac_load_delay;


architecture Behavioral of dac_load_delay is


  signal pulse_d : std_logic_vector(4 downto 0);


begin


  -- Delay by 125ns => 5*25ns
  p_pulse_delay: process (rst_n_i, clk_i)
  begin
    if rst_n_i = '0' then
      pulse_d <= (others => '0');
    elsif rising_edge(clk_i) then
      pulse_d <= pulse_d(3 downto 0) & pulse_i;
    end if;
  end process p_pulse_delay;

  -- Extend to 50ns => 2*25ns
  cmp_monostable_load : gc_extend_pulse
    generic map (
      g_width => 2)
    port map (
      clk_i      => clk_i,
      rst_n_i    => rst_n_i,
      pulse_i    => pulse_d(4),
      extended_o => d_pulse_o);


end Behavioral;

