--==============================================================
--Design Units : BIC Control and Statistics
--
--Purpose: This entity generates an addres maping between the
--         bus and the registers and memory.
--         It also multiplexes the registers to the bus interface.

--
--Note:   This model was written and developed in ISE 4.2.
--        The core of the case when has been generated using
--        the tcl script creaArrayConst2.do
--
--Limitations:
--
--Errors:
--
--Libraries:
--
--Dependancies:
--
--Author: Pablo Antonio Alvarez Sanchez
--        European Organisation for Nuclear Research
--        SL SPS/LHC -- Control -- Timing Division
--        CERN, Geneva, Switzerland,  CH-1211
--        Building 864 Room 1 - A24
--
-- Mod by P. Nouchi to adapt this module to the specific BIC-core module mapping addresses
--
--Simulator:               ModelSim XE 5.5e_p1
--==============================================================
--Revision List
--Version        Author Date        Changes
--1.0            PAAS   30.09.2002  Added comments, tested with the rest of the design
--2.0            PN     13.02.2003  For BIC-core module
--2.1            PN     13.02.2003  For CVORA module
--==============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.std_logic_unsigned.Conv_Integer;
use work.IntContPackage.all;
use work.vme_pkg.all;


entity ROMMUX is
  port (
    IntAdd    : in  IntAddrOutType;
    DoingOpC  : in  std_logic;
    DoneRam   : in  SelectedRamPosType;
    DataArray : in  MuxDataArrType;
    Sel       : out SelectedPosType;
    Done      : out std_logic;
    DataOut   : out IntDataType);
end ROMMUX;


architecture CaseWhen of ROMMUX is


begin


  process(IntAdd, DoingOpC, DataArray, DoneRam)
    variable vAdd : integer;            -- range 0 to
  begin
    vAdd := Conv_Integer(IntAdd);
    case vAdd is
      -- Registers
      when CSR_REG_ADDR =>
        Sel     <= (CSR_REG_P + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
        DataOut <= DataArray(CSR_REG_P);
        Done    <= DoingOpC;

      when MEM_PTR_REG_ADDR =>
        Sel     <= (MEM_PTR_REG_P + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
        DataOut <= DataArray(MEM_PTR_REG_P);
        Done    <= DoingOpC;

      when MODE_REG_ADDR =>
        Sel     <= (MODE_REG_P + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
        DataOut <= DataArray(MODE_REG_P);
        Done    <= DoingOpC;

      when CHAN_EN_REG_ADDR =>
        Sel     <= (CHAN_EN_REG_P + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
        DataOut <= DataArray(CHAN_EN_REG_P);
        Done    <= DoingOpC;

      when CLK_FREQ_REG_ADDR =>
        Sel     <= (CLK_FREQ_REG_P + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
        DataOut <= DataArray(CLK_FREQ_REG_P);
        Done    <= DoingOpC;

      when CHAN_SEL_REG_ADDR =>
        Sel     <= (CHAN_SEL_REG_P + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
        DataOut <= DataArray(CHAN_SEL_REG_P);
        Done    <= DoingOpC;

      when CVORB_REG_ADDR =>
        Sel     <= (CVORB_REG_P + FIRSTREGPOS => '1', REGOPPOS => '1', others => '0');
        DataOut <= DataArray(CVORB_REG_P);
        Done    <= DoingOpC;
      -- External RAM
      when EXT_RAM_ADDRL to EXT_RAM_ADDRH =>
        Sel     <= (EXT_RAM_P + FIRSTRAMPOS => '1', RAMOPPOS => '1', others => '0');
        DataOut <= DataArray(EXT_RAM_P + FIRSTRAMPOS - FIRSTREGPOS);
        Done    <= DoneRam(EXT_RAM_P);

      when others =>
        Sel     <= (RAMOPPOS => '0', REGOPPOS => '0', others => '0');
        DataOut <= (others   => '-');
        Done    <= '0';

    end case;
  end process;


end CaseWhen;

