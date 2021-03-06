------------------------------------------
------------------------------------------
-- Date        : Fri Nov 04 11:41:34 2011
--
-- Author      : Damien Perrelet
--               Heiko Damerau
--
-- Company     : CERN BE/RF/FB
--
-- Description : CVORB receiver
--
-- "pulse_width_i" is the measured
-- pulses length of zeroes and ones of the
-- stream.
-- The average between these two values
-- has to be set to "pulse_width_thres_i" input.
------------------------------------------
------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity cvorb_decoder is
  port (
    rst_n_i             : in  std_logic;
    clk_i               : in  std_logic;
    enable_i            : in  std_logic;
    data_i              : in  std_logic;
    zero_test_o         : out std_logic;
    one_test_o          : out std_logic;
    strobe_test_o       : out std_logic;
    pulse_width_thres_i : in  std_logic_vector(7 downto 0);
    pulse_width_o       : out std_logic_vector(7 downto 0);
    data_o              : out std_logic_vector(15 downto 0);
    data_valid_o        : out std_logic);
end cvorb_decoder;


architecture rtl of cvorb_decoder is


  signal serial_data : std_logic_vector(2 downto 0);

-- DOUBLE FLIP FLOP
  signal data_ff_pos_one : std_logic;
  signal data_ff_neg_one : std_logic;
  signal data_pos        : std_logic;
  signal data_neg        : std_logic;
-- PULSE LENGTH SAMPLE
  signal pulse_length    : std_logic_vector(7 downto 0);
  signal strobe          : std_logic;

  signal pulse_length_counter : unsigned(7 downto 0);

  type   states1 is (idle, acquire);
  signal state1               : states1;

-- CVORB BIT DECODER

  --signal short_pulse_limit : std_logic_vector(3 downto 0);
  signal data_out_cvorb       : std_logic_vector (15 downto 0);
  --signal zero_test_o : std_logic;
  --signal one_test_o : std_logic;
  signal shift_register       : std_logic_vector(15 downto 0);
  signal current_bit_position : unsigned(4 downto 0);
  signal timeout_delay        : unsigned(7 downto 0);  -- Timeout: 400 ns
  signal bit_updated          : std_logic;

  type   states2 is (idle, wait_bit, output);
  signal state2 : states2;


begin

  -- Synchronise input data to clock domain
  p_serial_data_sync: process (clk_i, rst_n_i)
  begin
    if rst_n_i = '0' then
      serial_data <= (others => '0');
    elsif rising_edge(clk_i) then
      serial_data <= serial_data(serial_data'left-1 downto 0) & (data_i and enable_i);
    end if;
  end process p_serial_data_sync;

-- DOUBLE FLIP FLOP
-- IN: data_i, clk_i, rst_n_i
-- OUT: data_pos_out,data_neg_out

  process(clk_i, rst_n_i)
  begin
    if rst_n_i = '0' then
      data_ff_pos_one <= '0';
    elsif (rising_edge(clk_i)) then
      data_ff_pos_one <= serial_data(serial_data'left);
    end if;
  end process;
  process(clk_i, rst_n_i)
  begin
    if rst_n_i = '0' then
      data_ff_neg_one <= '0';
    elsif (falling_edge(clk_i)) then
      data_ff_neg_one <= serial_data(serial_data'left);
    end if;
  end process;

  process(clk_i, rst_n_i)
  begin
    if rst_n_i = '0' then
      data_pos <= '0';
      data_neg <= '0';
    elsif (rising_edge(clk_i)) then
      data_pos <= data_ff_pos_one;
      data_neg <= data_ff_neg_one;
    end if;
  end process;





-- PULSE LENGTH SAMPLE
-- IN: data_pos_out => data_pos_in, data_neg_out => data_neg_in
-- OUT: pulse_length_out, strobe out

  process(clk_i, rst_n_i)
  begin
    if rst_n_i = '0' then
      pulse_length_counter <= X"00";
      pulse_length         <= X"00";
      strobe               <= '0';
      state1               <= idle;
    elsif (rising_edge(clk_i)) then
      if (state1 = idle and data_pos = '0' and data_neg = '0') then
        pulse_length_counter <= X"00";
        strobe               <= '0';
        state1               <= idle;
      elsif (data_pos xor data_neg) = '1' then
        pulse_length_counter <= pulse_length_counter + 1;
        state1               <= acquire;
      elsif (data_pos = '1' and data_neg = '1') then
        pulse_length_counter <= pulse_length_counter + 2;
        state1               <= acquire;
      elsif (state1 = acquire and data_pos = '0' and data_neg = '0') then
        pulse_length <= std_logic_vector(pulse_length_counter);
        strobe       <= '1';
        state1       <= idle;
      else
        state1 <= idle;
        strobe <= '0';
      end if;
    end if;
  end process;

  strobe_test_o <= strobe;



-- CVORB BIT DECODER
-- IN: Pulse_Length_In, Strobe_In, Short_pulse_width_thres_i, clk_i, rst_n_i
-- OUT: Data_Out_CVORB, data_valid_o, Zero_Test_O, One_Test_O
  process(clk_i, rst_n_i)
  begin
    if rst_n_i = '0' then
      shift_register       <= X"0000";
      current_bit_position <= "00000";
      timeout_delay        <= X"00";
      bit_updated          <= '0';
      data_out_cvorb       <= X"0000";
      data_valid_o         <= '0';
      data_o               <= X"0000";
--      zero_test_o          <= '0';
--      one_test_o           <= '0';
      state2               <= idle;
    elsif (rising_edge(clk_i)) then
      if (state2 = idle and strobe = '0') then
        shift_register       <= X"0000";
        current_bit_position <= "00000";
        timeout_delay        <= X"00";
        bit_updated          <= '0';
        data_out_cvorb       <= X"0000";
        data_valid_o         <= '0';
        zero_test_o          <= '0';
        one_test_o           <= '0';
        state2               <= idle;
      elsif (strobe = '1' and bit_updated = '0') then

        if (pulse_length > pulse_width_thres_i) then
          shift_register(15 downto 0) <= shift_register(14 downto 0) & '1';
          one_test_o                  <= '1';
        else
          shift_register(15 downto 0) <= shift_register(14 downto 0) & '0';
          zero_test_o                 <= '1';
        end if;
        timeout_delay        <= X"00";
        bit_updated          <= '1';
        current_bit_position <= current_bit_position + 1;
        state2               <= wait_bit;
      elsif state2 = wait_bit then
        zero_test_o <= '0';
        one_test_o  <= '0';
        if strobe = '0' then
          bit_updated <= '0';
        end if;
        if current_bit_position = "10000" then
          data_out_cvorb <= shift_register;
          state2         <= output;
        elsif timeout_delay = X"20" then
          state2 <= idle;
        else
          timeout_delay <= timeout_delay + 1;
          state2        <= wait_bit;
        end if;
      elsif state2 = output then
        data_valid_o <= '1';
        data_o       <= data_out_cvorb;
        state2       <= idle;
      else
        state2 <= idle;
      end if;
    end if;
  end process;

  pulse_width_o <= pulse_length;



end rtl;
