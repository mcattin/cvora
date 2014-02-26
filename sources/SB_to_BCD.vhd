--==============================================================--
--Design Units : SB to BCD component declaration and architecture
--
--File Name:     SB_to_BCD.vhd
--
--Purpose: This design takes in a N-bit SB number and converts it to BCD
--
--Note:   This model was written and developed in ISE 4.1
--
--Limitations:  generics are used for the port map
--
--Errors:
--
--Libraries:
--
--Dependancies:
--
--Author: Benjamin Todd
--        European Organisation for Nuclear Research
--        SL SPS/LHC -- Control -- Timing Division
--        CERN, Geneva, Switzerland,  CH-1211
--        Building 864 Room 1 - A24
--
--Simulator:               Microsim
--==============================================================--
--Revision List
--Version Author Date            Changes
--
--1.0            BMT      16.04.2002 New Version
--==============================================================--

library IEEE;                           -- include standard libraries
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_ARITH.all;
use IEEE.STD_LOGIC_UNSIGNED.all;

use work.cvora_pkg.all;
use work.message_package.all;           -- declares unconstrained BCD_Vector_Type


entity SB_to_BCD is

  generic(
    SB_Bits     : natural := 9;   -- these are the default values assumed if not given
    BCD_Numbers : natural := 3
    );
  port (
    Clock      : in  std_logic;                                     -- external clock, not speed dependant
    Reset      : in  std_logic;                                     -- external reset, activity defined in package
    SB_Number  : in  std_logic_vector ((SB_Bits-1) downto 0);       -- straight binary number
    SB_Ready   : in  std_logic;                                     -- signal to indicate that the conversion should start
    BCD_Vector : out BCD_vector_TYPE ((BCD_Numbers - 1) downto 0);  -- resulting BCD
    BCD_Ready  : out std_logic);                                    -- signal to indicate that the converison is done

end SB_to_BCD;


--NOTES
--The BCD Vector contains the resulting BCD numbers in an ARRAY, where
--BCD_Vector(N) represents 10^N

architecture Dynamic of SB_to_BCD is


  signal iSB_Ready, iSB_Ready2, Ready_Re : std_logic;                                      -- rising_edge detection of the input signal
  signal INIT, Hold                      : std_logic;                                      -- control bits
  signal ShiftRegister                   : std_logic_vector ((SB_Bits-1) downto 0);        -- shift register
  signal iBCD                            : BCD_Vector_Type ((BCD_Numbers - 1) downto 0);   -- internal BCD vectors
  signal iModOut                         : std_logic_vector ((BCD_Numbers - 1) downto 0);  -- internal "carry" logic
  signal BitCounter                      : natural range 0 to (SB_Bits);                   -- bit counter, range augmented for simulation
  signal Q0, Q1, Q2, Q3                  : std_logic_vector ((BCD_Numbers -1) downto 0);   -- d-latches
  signal MUXQ1, MUXQ2, MUXQ3             : std_logic_vector ((BCD_Numbers - 1) downto 0);  -- 2 to 1 muxes
  signal GreaterThanFive                 : std_logic_vector ((BCD_Numbers - 2) downto 0);  -- 4 bit comparators


begin  -- architecture of dynamic sb to bcd converter


  BCD_Vector <= iBCD;                   -- map the internal signals to the external port

--Firstly the edge of the ready signal is detected, by sampling
  Edge_Detect : process (clock, reset)
  begin
    if Reset = '0' then
      iSB_Ready  <= '0';
      iSB_Ready2 <= '0';
    elsif rising_edge (clock) then
      iSB_Ready  <= SB_Ready;
      iSB_Ready2 <= iSB_ready;
    end if;
  end process Edge_Detect;
  Ready_Re <= iSB_Ready and (not iSb_ready2);

--once the edge is detected then the SHIFT register is loaded with the BCD data
--and the conversion process begins, the shift register shift is also controlled
--here
  Latch_Data : process (clock, reset)
  begin
    if Reset = '0' then
      ShiftRegister <= (others => '0');
      Init          <= '0';
    elsif rising_edge (clock) then
      if Ready_Re = '1' then
        Init          <= '0';
        ShiftRegister <= SB_Number;
      else
        ShiftRegister ((SB_Bits -1) downto 1) <= ShiftRegister ((SB_Bits-2) downto 0);
        Init                                  <= '1';
      end if;
    end if;
  end process Latch_Data;
  iModOut(0) <= ShiftRegister (SB_Bits - 1);

--the two control bits are defined
--once all the bits have been moved from the register, then the HOLD is activated
  generate_Ready_and_Hold : process (clock, reset)
  begin
    if reset = '0' then
      Hold       <= '1';
      BCD_Ready  <= '0';
      BitCounter <= 0;
    elsif rising_edge (clock) then
      if Hold = '1' then
        if Ready_Re = '1' then
          BitCounter <= 0;
          Hold       <= '0';
        end if;
      else BitCounter <= BitCounter + 1;
      end if;
      if BitCounter = (SB_Bits -1) then
        Hold      <= '1';
        BCD_Ready <= '1';
      else BCD_Ready <= '0';
      end if;
    end if;
  end process generate_Ready_and_Hold;

--the last converion block is defined explicitly, as it has no modout
  Convert_last_Block : process (clock, reset, iBCD, INIT, Q0, Q1, Q2, Q3)
  begin
    if Reset = '0' then
      Q0(BCD_Numbers - 1) <= '0';
      Q1(BCD_Numbers - 1) <= '0';
      Q2(BCD_Numbers - 1) <= '0';
      Q3(BCD_Numbers - 1) <= '0';
    elsif rising_edge (clock) then
      if Hold = '0' then
        Q0(BCD_Numbers - 1) <= iModOut((BCD_Numbers - 1));
        Q1(BCD_Numbers - 1) <= MUXQ1(BCD_Numbers - 1) and INIT;
        Q2(BCD_Numbers - 1) <= MUXQ2(BCD_Numbers - 1) and INIT;
        Q3(BCD_Numbers - 1) <= MUXQ3(BCD_Numbers - 1) and INIT;
      end if;
    end if;
    if iBCD((BCD_Numbers - 1)) >= "0101" then
      MUXQ1(BCD_Numbers - 1) <= not Q0(BCD_Numbers - 1);
      MUXQ2(BCD_Numbers - 1) <= Q0(BCD_Numbers - 1) xnor Q1(BCD_Numbers - 1);
      MUXQ3(BCD_Numbers - 1) <= Q0(BCD_Numbers - 1) and Q3(BCD_Numbers - 1);
    else
      MUXQ1(BCD_Numbers - 1) <= Q0(BCD_Numbers - 1);
      MUXQ2(BCD_Numbers - 1) <= Q1(BCD_Numbers - 1);
      MUXQ3(BCD_Numbers - 1) <= Q2(BCD_Numbers - 1);
    end if;
    iBCD((BCD_Numbers - 1)) <= Q3(BCD_Numbers - 1) & Q2(BCD_Numbers - 1) & Q1(BCD_Numbers - 1) & Q0(BCD_Numbers - 1);
  end process Convert_last_Block;

--all the other blocks are generated here
  BCD_Conversion_Blocks : for NUMBER in 0 to (BCD_Numbers - 2) generate
    process (clock, reset, iBCD, INIT, Q0, Q1, Q2, Q3)
    begin
      if Reset = '0' then
        Q0(NUMBER) <= '0';
        Q1(NUMBER) <= '0';
        Q2(NUMBER) <= '0';
        Q3(NUMBER) <= '0';
      elsif rising_edge (clock) then
        if Hold = '0' then
          Q0(NUMBER) <= iModOut(NUMBER);
          Q1(NUMBER) <= MUXQ1(NUMBER) and INIT;
          Q2(NUMBER) <= MUXQ2(NUMBER) and INIT;
          Q3(NUMBER) <= MUXQ3(NUMBER) and INIT;
        end if;
      end if;
      if iBCD(NUMBER) >= "0101" then
        MUXQ1(NUMBER)           <= not Q0(NUMBER);
        MUXQ2(NUMBER)           <= Q0(NUMBER) xnor Q1(NUMBER);
        MUXQ3(NUMBER)           <= Q0(NUMBER) and Q3(NUMBER);
        GreaterThanFive(NUMBER) <= '1';
      else
        MUXQ1(NUMBER)           <= Q0(NUMBER);
        MUXQ2(NUMBER)           <= Q1(NUMBER);
        MUXQ3(NUMBER)           <= Q2(NUMBER);
        GreaterThanFive(NUMBER) <= '0';
      end if;
      iModOut(NUMBER + 1) <= GreaterThanFive(NUMBER) and INIT;
      iBCD(NUMBER)        <= Q3(NUMBER) & Q2(NUMBER) & Q1(NUMBER) & Q0(NUMBER);
    end process;
  end generate BCD_Conversion_Blocks;

--the blocks are all generated dynamically, hence changing the size of the generic will
--change the resulting component
end Dynamic;
