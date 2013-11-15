--------------------------------------------------------------------------------
-- CERN (BE-CO-HT)
-- RTM serial data manager
--------------------------------------------------------------------------------
--
-- unit name: rtm_serial_manager
--
-- author: Matthieu Cattin (matthieu.cattin@cern.ch)
--
-- date: 28-10-2013
--
-- description: Takes de-serialised data from rtm and writes them to RAM.
--
-- dependencies:
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

use work.cvora_pkg.all;


entity rtm_serial_manager is
  port (
    rst_n_i                   : in  std_logic;
    clk_i                     : in  std_logic;
    rtm_data_i                : in  std_logic_vector(31 downto 0);
    rtm_data_o                : out rtm_data_array_t;
    rtm_data_valid_o          : out std_logic_vector(31 downto 0);
    mode_i                    : in  std_logic_vector(3 downto 0);
    channel_en_i              : in  std_logic_vector(31 downto 0);
    data_clk_i                : in  std_logic;
    cvorb_pulse_width_thres_i : in  std_logic_vector(7 downto 0);
    cvorb_meas_pulse_width_o  : out cvorb_pulse_width_array_t;
    ram_data_o                : out std_logic_vector(31 downto 0);
    ram_data_valid_o          : out std_logic;
    ram_addr_o                : out std_logic_vector(RAM_ADDR_LENGTH-1 downto 0);
    ram_overflow_o            : out std_logic;
    ram_data_written_i        : in  std_logic;
    reset_ram_addr_i          : in  std_logic
    );
end rtm_serial_manager;


architecture rtl of rtm_serial_manager is


  signal rtm_sci_channel_en   : std_logic_vector(c_NB_RTM_CHAN-1 downto 0);
  signal rtm_data_sci         : rtm_data_array_t;
  signal rtm_data_sci_valid   : std_logic_vector(c_NB_RTM_CHAN-1 downto 0);
  signal rtm_cvorb_channel_en : std_logic_vector(c_NB_RTM_CHAN-1 downto 0);
  signal rtm_data_cvorb       : rtm_data_array_t;
  signal rtm_data_cvorb_valid : std_logic_vector(c_NB_RTM_CHAN-1 downto 0);
  signal rtm_data             : rtm_data_array_t;
  signal rtm_data_valid       : std_logic_vector(31 downto 0);
  signal rtm_channel_en       : std_logic_vector(31 downto 0);

  signal data_buffer : rtm_data_array_t;

  type fsm_state_t is (s_IDLE, s_WRITE_CHANNEL, s_LOAD_LSB, s_LOAD_MSB, s_LOAD_MSB_LSB,
                       s_WRITE_RAM, s_WAIT_RAM, s_INCR_ADDR, s_INCR_CHANNEL,
                       s_DONE, s_RAM_FULL, s_RESET_RAM_ADDR);
  signal fsm_state       : fsm_state_t;
  signal fsm_next_state  : fsm_state_t;
  signal current_channel : natural range 0 to c_NB_RTM_CHAN-1;
  signal ram_addr        : unsigned(RAM_ADDR_LENGTH-1 downto 0);


begin

  -- SCI protocol
  rtm_sci_channel_en <= channel_en_i when mode_i = c_RTM_SCI_M else (others => '0');

  l_rtm_sci_decoders : for I in 0 to c_NB_RTM_CHAN-1 generate
    cmp_rtm_sci_decoder : sci_decoder
      port map(
        rst_n_i      => rst_n_i,
        clk_i        => clk_i,
        enable_i     => rtm_sci_channel_en(I),
        data_i       => rtm_data_i(I),
        data_o       => rtm_data_sci(I),
        data_valid_o => rtm_data_sci_valid(I));
  end generate l_rtm_sci_decoders;

  -- CVORB protocol
  rtm_cvorb_channel_en <= channel_en_i when mode_i = c_RTM_CVORB_M else (others => '0');

  l_rtm_cvorb_decoders : for I in 0 to c_NB_RTM_CHAN-1 generate
    cmp_rtm_cvorb_decoder : cvorb_decoder
      port map(
        rst_n_i             => rst_n_i,
        clk_i               => clk_i,
        enable_i            => rtm_cvorb_channel_en(I),
        data_i              => rtm_data_i(I),
        zero_test_o         => open,
        one_test_o          => open,
        strobe_test_o       => open,
        pulse_width_thres_i => cvorb_pulse_width_thres_i,
        pulse_width_o       => cvorb_meas_pulse_width_o(I),
        data_o              => rtm_data_cvorb(I),
        data_valid_o        => rtm_data_cvorb_valid(I));
  end generate l_rtm_cvorb_decoders;

  -- Select protocol (sci or cvorb)
  rtm_data <= rtm_data_sci when mode_i = c_RTM_SCI_M else
              rtm_data_cvorb when mode_i = c_RTM_CVORB_M else
              (others => (others => '0'));

  rtm_data_valid <= rtm_data_sci_valid when mode_i = c_RTM_SCI_M else
                    rtm_data_cvorb_valid when mode_i = c_RTM_CVORB_M else
                    (others => '0');

  rtm_channel_en <= rtm_sci_channel_en when mode_i = c_RTM_SCI_M else
                    rtm_cvorb_channel_en when mode_i = c_RTM_CVORB_M else
                    (others => '0');

  rtm_data_o       <= rtm_data;
  rtm_data_valid_o <= rtm_data_valid;

  -- Data buffering
  l_data_buffer : for I in 0 to c_NB_RTM_CHAN-1 generate
    p_data_buffer : process (rst_n_i, clk_i)
    begin
      if rst_n_i = '0' then
        data_buffer(I) <= (others => '0');
      elsif rising_edge(clk_i) then
        if rtm_data_valid(I) = '1' and rtm_channel_en(I) = '1' then
          data_buffer(I) <= rtm_data(I);
        elsif rtm_channel_en(I) = '0' then
          data_buffer(I) <= (others => '0');
        end if;
      end if;
    end process p_data_buffer;
  end generate l_data_buffer;

  -- RAM managment FSM
  p_fsm_clk : process (rst_n_i, clk_i)
  begin
    if rst_n_i = '0' then
      fsm_state <= s_IDLE;
    elsif rising_edge(clk_i) then
      if (mode_i = c_RTM_SCI_M) or (mode_i = c_RTM_CVORB_M) then
        fsm_state <= fsm_next_state;
      else
        fsm_state <= s_IDLE;
      end if;
    end if;
  end process;


  p_fsm_next_state : process (fsm_state, reset_ram_addr_i, data_clk_i, ram_addr,
                              rtm_channel_en, current_channel, ram_data_written_i)
  begin
    fsm_next_state <= fsm_state;        --default is to stay in current state
    case fsm_state is
      when s_IDLE =>
        if reset_ram_addr_i = '1' then
          fsm_next_state <= s_RESET_RAM_ADDR;
        elsif data_clk_i = '1' then
          fsm_next_state <= s_WRITE_CHANNEL;
        elsif ram_addr = EXTRAMFULL then
          fsm_next_state <= s_RAM_FULL;
        end if;

      when s_WRITE_CHANNEL =>
        if rtm_channel_en(current_channel+1 downto current_channel) = "11" then
          fsm_next_state <= s_LOAD_MSB_LSB;
        elsif rtm_channel_en(current_channel+1 downto current_channel) = "10" then
          fsm_next_state <= s_LOAD_MSB;
        elsif rtm_channel_en(current_channel+1 downto current_channel) = "01" then
          fsm_next_state <= s_LOAD_LSB;
        else
          fsm_next_state <= s_INCR_CHANNEL;
        end if;

      when s_LOAD_MSB_LSB =>
        fsm_next_state <= s_WRITE_RAM;

      when s_LOAD_LSB =>
        fsm_next_state <= s_WRITE_RAM;

      when s_LOAD_MSB =>
        fsm_next_state <= s_WRITE_RAM;

      when s_WRITE_RAM =>
        fsm_next_state <= s_WAIT_RAM;

      when s_WAIT_RAM =>                -- wait until the RAMManager has finished
        if reset_ram_addr_i = '1' then
          fsm_next_state <= s_RESET_RAM_ADDR;
        elsif ram_data_written_i = '1' then
          fsm_next_state <= s_INCR_ADDR;
        end if;

      when s_INCR_ADDR =>
        if ram_addr = EXTRAMFULL - 1 then
          fsm_next_state <= s_RAM_FULL;
        else
          fsm_next_state <= s_INCR_CHANNEL;
        end if;

      when s_INCR_CHANNEL =>
        if current_channel = c_NB_RTM_CHAN-2 then
          fsm_next_state <= s_DONE;
        else
          fsm_next_state <= s_WRITE_CHANNEL;
        end if;

      when s_RAM_FULL =>                -- stay here until memory is cleared by external Reset
        if reset_ram_addr_i = '1' then
          fsm_next_state <= s_RESET_RAM_ADDR;
        end if;

      when s_RESET_RAM_ADDR =>          -- the memory pointer is set back to the first address of the memory
        fsm_next_state <= s_IDLE;

      when s_DONE =>
        fsm_next_state <= s_IDLE;

      when others =>                    -- wait for the record is written (5 clock ticks)
        fsm_next_state <= s_IDLE;
    end case;
  end process;

  p_fsm_outputs : process(clk_i)
  begin
    if rising_edge(clk_i) then
      case fsm_state is
        when s_IDLE =>
          ram_overflow_o   <= '0';
          ram_data_valid_o <= '0';
          current_channel  <= 0;

        when s_WRITE_CHANNEL =>
          ram_data_valid_o <= '0';

        when s_LOAD_MSB_LSB =>
          ram_data_o <= data_buffer(current_channel+1) & data_buffer(current_channel);

        when s_LOAD_LSB =>
          ram_data_o <= x"0000" & data_buffer(current_channel);

        when s_LOAD_MSB =>
          ram_data_o <= data_buffer(current_channel+1) & x"0000";

        when s_WRITE_RAM =>
          ram_data_valid_o <= '1';

        when s_WAIT_RAM =>
          ram_data_valid_o <= '0';

        when s_INCR_ADDR =>
          ram_addr <= ram_addr + EXTRAM_BUF_ONE;

        when s_INCR_CHANNEL =>
          current_channel <= current_channel + 2;

        when s_DONE =>
          ram_data_valid_o <= '0';
          current_channel  <= 0;

        when s_RAM_FULL =>              -- stay here until memory is cleared by external Reset
          ram_data_valid_o <= '0';
          ram_overflow_o   <= '1';

        when s_RESET_RAM_ADDR =>        -- the memory pointer is set back to the first address of the memory
          ram_addr       <= MEMEMPTY;
          ram_overflow_o <= '0';

        when others =>
          ram_data_valid_o <= '0';

      end case;
    end if;
  end process;

  ram_addr_o <= std_logic_vector(ram_addr);


end rtl;
