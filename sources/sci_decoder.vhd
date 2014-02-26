--------------------------------------------------------------------------------
-- CERN (BE-CO-HT)
-- SCI protocol decoder
--------------------------------------------------------------------------------
--
-- unit name: sci_decoder
--
-- author: Matthieu Cattin (matthieu.cattin@cern.ch)
--
-- date: 25-10-2013
--
-- description: This module decodes the SCI (Serial Communication Interface)
--              protocol. It receives frames made of two bytes.
--              The frame format is the following:
--                - 128 kbaud
--                - start + byte 1 + '1' + stop + start + byte 2 + '0' + stop
--
-- dependencies:
--
-- references: [1] www.freescale.com/files/microcontrollers/doc/data_sheet/MC68HC05C8A.pdf
--
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

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;
use work.cvora_pkg.all;


entity sci_decoder is
  port (
    rst_n_i      : in  std_logic;
    clk_i        : in  std_logic;
    enable_i     : in  std_logic;
    data_i       : in  std_logic;
    data_o       : out std_logic_vector(15 downto 0);
    data_valid_o : out std_logic
    );
end sci_decoder;


architecture rtl of sci_decoder is


  constant NB_BITS : integer := 9;

  signal serial_data    : std_logic;
  signal tmp_data       : std_logic_vector(NB_BITS - 1 downto 0);
  signal tmp_data_valid : std_logic;

  signal idReady    : std_logic;
  signal odready    : std_logic                    := '0';
  signal FirstByte  : std_logic_vector(7 downto 0) := (others => '0');
  signal SecondByte : std_logic_vector(7 downto 0) := (others => '0');
  signal iSDataIn   : std_logic;


begin


  serial_data <= data_i and enable_i;

  cmp_sci_rx : RS232_Rx
    generic map (
      NUMBER_OF_BIT => NB_BITS,   -- without start and stop bits
      QUARTZFREQ    => QUARTZFREQ,
      BAUD_RATE     => 128000)
    port map (
      Clock  => clk_i,
      Reset  => rst_n_i,
      Serial => serial_data,
      dOut   => tmp_data,
      dReady => tmp_data_valid);

  p_frame : process(clk_i, rst_n_i)
  begin
    if rst_n_i = '0' then
      data_o <= (others => '0');
      data_valid_o <= '0';
    elsif rising_edge(clk_i) then
      if tmp_data_valid = '1' and enable_i = '1' then
        if tmp_data(8) = '0' then          -- if bit 8 = 1 means MSB else LSB
          data_o(7 downto 0) <= tmp_data(7 downto 0);
          data_valid_o   <= '1';
        else
          data_o(15 downto 8) <= tmp_data(7 downto 0);
        end if;
      else
        data_valid_o <= '0';
      end if;
    end if;
  end process p_frame;


end rtl;
