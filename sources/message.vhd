--
-- Component to send a message and a obtain the rs232 stream.
--
-- Philippe Nouchi le 06 Mars 2002
--
-- Version 1.0
-- Version 1.1 PN 12/03/2004 Could be used with any clock (set it in RS232_Tx)
--
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

use work.cvora_pkg.all;
use work.message_pkg.all;


entity message is
  port (
    rst         : in  std_logic;
    clk         : in  std_logic;
    rs232_start : in  std_logic;        -- signal to start the message (no effect when message_stop = '1')
    message     : in  message_array;    -- An array of ASCII character (std_logic_vector(7 downto 0)) define row and column in auxdef
    message_env : out std_logic;        -- '1' when the message when is being sent on rs232out
    rs232out    : out std_logic         -- the rs232 output
    );
end message;


architecture Behavioral of message is


  component RS232_Tx port (
    Clock        : in  std_logic;                      -- 16.384 MHZ clock
    Reset        : in  std_logic;                      -- active low reset
    Data_Ready   : in  std_logic;                      -- signal to indicate data ready on inputs
    Data         : in  std_logic_vector (7 downto 0);  -- one byte of data
    RS232_Serial : out std_logic;                      -- serially generated code
    Ready        : out std_logic);                     -- signal to say a reload can take place
  end component;

  signal rs232_Data_Ready          : std_logic;                           -- put at 1 to start serial data sending
  signal rs232_Data                : std_logic_vector (7 downto 0);       -- ASCII character to send
  signal rs232_Ready, rs232_ReadyB : std_logic;                           -- signal to say a reload can take place (if = '1')
  signal rs232_stop                : std_logic;
  signal iindex                    : natural range 1 to LINE_NUMBER + 1;  -- index of the line number
  signal jindex                    : natural range 0 to LINE_LENGTH;  -- - 1;                  -- index of the character in the line
  signal message_stop              : std_logic;
  signal imess                     : message_array;


begin


  RS232_TX1 : RS232_Tx
    port map (
      Clock        => clk,
      Reset        => rst,
      Data_Ready   => rs232_Data_Ready,
      Data         => rs232_Data,
      RS232_Serial => rs232out,
      Ready        => rs232_Ready
      );

-- D-latch for the message
  process(clk, rs232_start, message)
  begin
    if falling_edge(clk) then
      if rs232_start = '1' then
        imess <= message;
      end if;
    end if;
  end process;

  -- when an ascii character is finishing
  process(rst, clk, rs232_Ready)
  begin
    if rst = '0' then
      rs232_ReadyB <= '0';
    elsif rising_edge(clk) then
      rs232_ReadyB <= rs232_Ready;
    end if;
  end process;
  rs232_stop <= rs232_Ready and not(rs232_ReadyB);  -- Rising_edge of rs232_Ready

  -- send message
  MESS_SEND : process(rst, clk, rs232_start, rs232_stop, imess)
  begin
    if rst = '0' then
      iindex           <= 1;
      jindex           <= 0;              -- character in the line
      message_stop     <= '0';
      rs232_Data_Ready <= '0';
      rs232_Data       <= (others => '0');
    elsif rising_edge(clk) then
      if rs232_start = '1' and message_stop = '0' then
        iindex           <= 1;
        jindex           <= 1;
        message_stop     <= '1';
        rs232_Data       <= imess(1)(0);
        rs232_Data_Ready <= '1';
      elsif rs232_stop = '1' and message_stop = '1' then
        rs232_Data <= imess(iindex)(jindex);
        jindex     <= jindex + 1;
        if jindex = LINE_LENGTH - 1 then  -- end of line
          jindex <= 0;
          iindex <= iindex + 1;
          if iindex = LINE_NUMBER then    -- end of message
            message_stop <= '0';
          end if;
        end if;
        rs232_Data_Ready <= '1';
      else rs232_Data_Ready <= '0';
      end if;
    end if;
  end process;

  message_env <= message_stop;

end Behavioral;
