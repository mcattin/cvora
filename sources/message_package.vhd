
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

package message_package is
  constant LINE_NUMBER : natural := 5;
  constant LINE_LENGTH : natural := 18;

  type    line is array(0 to LINE_LENGTH - 1) of std_logic_vector(7 downto 0);
  type    message_array is array(1 to LINE_NUMBER) of line;
  type    calendar_type is array(5 downto 0) of std_logic_vector(7 downto 0);
  type    threechar is array(0 to 2) of std_logic_vector(7 downto 0);
  subtype Ram_Address is std_logic_vector(9 downto 0);
  subtype BCD_TYPE is std_logic_vector (3 downto 0);
  type    BCD_Vector_TYPE is array (natural range <>) of BCD_TYPE;

  constant Carriage_Return : std_logic_vector (7 downto 0) := X"0D";
  constant Line_Feed       : std_logic_vector (7 downto 0) := X"0A";
  constant semicolon       : std_logic_vector (7 downto 0) := X"3A";
  constant slash           : std_logic_vector (7 downto 0) := X"2F";
  constant space           : std_logic_vector (7 downto 0) := X"20";
  constant dot             : std_logic_vector (7 downto 0) := X"2e";
  constant Letter_at       : std_logic_vector (7 downto 0) := X"40";
--   constant Dollar_Symbol: std_logic_vector (7 downto 0) := X"24";
  constant LETTER_A        : std_logic_vector(7 downto 0)  := X"41";
  constant LETTER_B        : std_logic_vector(7 downto 0)  := X"42";
  constant Letter_C        : std_logic_vector (7 downto 0) := X"43";
  constant Letter_D        : std_logic_vector (7 downto 0) := X"44";
  constant LETTER_E        : std_logic_vector(7 downto 0)  := X"45";
  constant LETTER_F        : std_logic_vector(7 downto 0)  := X"46";
  constant LETTER_I        : std_logic_vector(7 downto 0)  := X"49";
  constant Letter_K        : std_logic_vector (7 downto 0) := X"4b";
  constant Letter_L        : std_logic_vector (7 downto 0) := X"4c";
  constant LETTER_M        : std_logic_vector(7 downto 0)  := X"4d";
  constant LETTER_N        : std_logic_vector(7 downto 0)  := X"4e";
  constant Letter_O        : std_logic_vector (7 downto 0) := X"4f";
  constant Letter_P        : std_logic_vector (7 downto 0) := X"50";
  constant Letter_Q        : std_logic_vector (7 downto 0) := X"51";
  constant LETTER_T        : std_logic_vector(7 downto 0)  := X"54";
  constant Letter_U        : std_logic_vector (7 downto 0) := X"55";
  constant Letter_R        : std_logic_vector (7 downto 0) := X"52";
  constant Letter_S        : std_logic_vector (7 downto 0) := X"53";
  constant Letter_V        : std_logic_vector (7 downto 0) := X"56";
  --constant Letter_W : std_logic_vector (7 downto 0) := X"57";
  constant LETTER_0        : std_logic_vector (7 downto 0) := X"30";
  constant LETTER_1        : std_logic_vector (7 downto 0) := X"31";
  constant LETTER_2        : std_logic_vector (7 downto 0) := X"32";
  constant LETTER_3        : std_logic_vector (7 downto 0) := X"33";
  constant LETTER_6        : std_logic_vector (7 downto 0) := X"36";
  constant Letter_amin     : std_logic_vector (7 downto 0) := X"61";
  constant Letter_emin     : std_logic_vector (7 downto 0) := X"65";
  constant Letter_hmin     : std_logic_vector (7 downto 0) := X"68";
  constant Letter_imin     : std_logic_vector (7 downto 0) := X"69";
  constant Letter_jmin     : std_logic_vector (7 downto 0) := X"6A";
  constant Letter_kmin     : std_logic_vector (7 downto 0) := X"6B";
  constant Letter_lmin     : std_logic_vector (7 downto 0) := X"6C";
  constant Letter_mmin     : std_logic_vector (7 downto 0) := X"6D";
  constant Letter_nmin     : std_logic_vector (7 downto 0) := X"6E";
  constant Letter_omin     : std_logic_vector (7 downto 0) := X"6F";
  constant Letter_rmin     : std_logic_vector (7 downto 0) := X"72";
  constant Letter_smin     : std_logic_vector (7 downto 0) := X"73";
  constant Letter_tmin     : std_logic_vector (7 downto 0) := X"74";
  --constant Letter_vmin : std_logic_vector (7 downto 0) := X"76";
  constant Letter_xmin     : std_logic_vector (7 downto 0) := X"78";
  constant Letter_zmin     : std_logic_vector (7 downto 0) := X"7A";
  constant Letter_plus     : std_logic_vector (7 downto 0) := X"2B";
  constant Letter_moins    : std_logic_vector (7 downto 0) := X"2D";

  constant NMAXMON : integer                            := 9;
  constant IONEMON : std_logic_vector(NMAXMON downto 0) := (0      => '1', others => '0');
  constant TOPCMON : std_logic_vector(NMAXMON downto 0) := (others => '1');
  constant INJEC   : std_logic_vector(7 downto 0)       := X"A0";
  constant EXTRAC  : std_logic_vector(7 downto 0)       := X"A1";
  -- Header constant
  constant HMS     : std_logic_vector(7 downto 0)       := X"01";
  constant HSEC    : std_logic_vector(7 downto 0)       := X"02";
  constant HMIN    : std_logic_vector(7 downto 0)       := X"03";
  constant HHOUR   : std_logic_vector(7 downto 0)       := X"04";
  constant HDAY    : std_logic_vector(7 downto 0)       := X"05";
  constant HMONTH  : std_logic_vector(7 downto 0)       := X"06";
  constant HYEAR   : std_logic_vector(7 downto 0)       := X"07";
  constant HSSC    : std_logic_vector(7 downto 0)       := X"20";
  constant HEVSPS  : std_logic_vector(7 downto 0)       := X"21";
  -- constant for pointer incrment
  constant TROIS   : Ram_Address                        := "0000000011";
  constant QUATRE  : Ram_Address                        := "0000000100";
  constant DEUX    : Ram_Address                        := "0000000010";
  constant UN      : Ram_Address                        := "0000000001";
  constant ZERO    : Ram_Address                        := "0000000000";

  -- "FP OP16 SCI     "
  constant c_FP_OP16_SCI_LINE : line := (LETTER_F, LETTER_P, space, LETTER_O, LETTER_P,
                                         LETTER_1, LETTER_6, space, LETTER_S, LETTER_C,
                                         LETTER_I, space, space, space, space, space,
                                         Carriage_Return, Line_Feed);
  -- "FP CU16 SCI     "
  constant c_FP_CU16_SCI_LINE : line := (LETTER_F, LETTER_P, space, LETTER_C, LETTER_U,
                                         LETTER_1, LETTER_6, space, LETTER_S, LETTER_C,
                                         LETTER_I, space, space, space, space, space,
                                         Carriage_Return, Line_Feed);
  -- "CNT 32 Bits     "
  constant c_CNT32_LINE : line := (LETTER_C, LETTER_N, LETTER_T, space, LETTER_3,
                                   LETTER_2, space, LETTER_B, LETTER_imin, LETTER_tmin,
                                   LETTER_smin, space, space, space, space, space,
                                   Carriage_Return, Line_Feed);
  -- "RTM PARALLEL    "
  constant c_RTM_PARALLEL_LINE : line := (LETTER_R, LETTER_T, LETTER_M, space, LETTER_P,
                                          LETTER_A, LETTER_R, LETTER_A, LETTER_L, LETTER_L,
                                          LETTER_E, LETTER_L, space, space, space, space,
                                          Carriage_Return, Line_Feed);
  -- "FP OP32 SCI     "
  constant c_FP_OP32_SCI_LINE : line := (LETTER_F, LETTER_P, space, LETTER_O, LETTER_P,
                                         LETTER_3, LETTER_2, space, LETTER_S, LETTER_C,
                                         LETTER_I, space, space, space, space, space,
                                         Carriage_Return, Line_Feed);
  -- "FP CU32 SCI     "
  constant c_FP_CU32_SCI_LINE : line := (LETTER_F, LETTER_P, space, LETTER_C, LETTER_U,
                                         LETTER_3, LETTER_2, space, LETTER_S, LETTER_C,
                                         LETTER_I, space, space, space, space, space,
                                         Carriage_Return, Line_Feed);
  -- "RTM SCI         "
  constant c_RTM_SCI_LINE : line := (LETTER_R, LETTER_T, LETTER_M, space, LETTER_S,
                                     LETTER_C, LETTER_I, space, space, space, space, space,
                                     space, space, space, space,
                                     Carriage_Return, Line_Feed);
  -- "FP OP16 CVORB   "
  constant c_FP_OP16_CVORB_LINE : line := (LETTER_F, LETTER_P, space, LETTER_O, LETTER_P,
                                           LETTER_1, LETTER_6, space, LETTER_C, LETTER_V,
                                           LETTER_O, LETTER_R, LETTER_B, space, space, space,
                                           Carriage_Return, Line_Feed);
  -- "FP CU16 CVORB   "
  constant c_FP_CU16_CVORB_LINE : line := (LETTER_F, LETTER_P, space, LETTER_C, LETTER_U,
                                           LETTER_1, LETTER_6, space, LETTER_C, LETTER_V,
                                           LETTER_O, LETTER_R, LETTER_B, space, space, space,
                                           Carriage_Return, Line_Feed);
  -- "CNT 2x16 Bits   "
  constant c_CNT2X16_LINE : line := (LETTER_C, LETTER_N, LETTER_T, space, LETTER_2,
                                     LETTER_xmin, LETTER_1, LETTER_6, space, LETTER_B,
                                     LETTER_imin, LETTER_tmin, LETTER_smin, space, space, space,
                                     Carriage_Return, Line_Feed);
  -- "FP OP32 CVORB   "
  constant c_FP_OP32_CVORB_LINE : line := (LETTER_F, LETTER_P, space, LETTER_O, LETTER_P,
                                           LETTER_3, LETTER_2, space, LETTER_C, LETTER_V,
                                           LETTER_O, LETTER_R, LETTER_B, space, space, space,
                                           Carriage_Return, Line_Feed);
  -- "FP CU32 CVORB   "
  constant c_FP_CU32_CVORB_LINE : line := (LETTER_F, LETTER_P, space, LETTER_C, LETTER_U,
                                           LETTER_3, LETTER_2, space, LETTER_C, LETTER_V,
                                           LETTER_O, LETTER_R, LETTER_B, space, space, space,
                                           Carriage_Return, Line_Feed);
  -- "RTM CVORB       "
  constant c_RTM_CVORB_LINE : line := (LETTER_R, LETTER_T, LETTER_M, space, LETTER_C,
                                       LETTER_V, LETTER_O, LETTER_R, LETTER_B, space,
                                       space, space, space, space, space, space,
                                       Carriage_Return, Line_Feed);
  -- "RESERVED MODE   "
  constant c_RESERVED_LINE : line := (LETTER_R, LETTER_E, LETTER_S, LETTER_E, LETTER_R,
                                      LETTER_V, LETTER_E, LETTER_D, space, LETTER_M,
                                      LETTER_O, LETTER_D, LETTER_E, space, space, space,
                                      Carriage_Return, Line_Feed);

end message_package;


package body message_package is

end message_package;
