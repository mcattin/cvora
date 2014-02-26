--==============================================================--
--Design Units : CTX1 Control and Statistics
--
--File Name:     IntControlRegRdComb.vhd
--
--Purpose: The purpose of this entity is to make a link between
--         the VME bus and the registers and ram of the CTX1.
--
--         Basically it consists of an address decoder and
--         a data multiplexor implemented in a case when structure.
--
--Note:   This model was written and developed in ISE 4.2
--
--Limitations:  Records are used for the port map
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
--Simulator:               ModelSim XE 5.5e_p1
--==============================================================--
--Revision List
--Version Author Date           Changes
--
--1.0     PAAS   30.09.2002     Added comments, tested with the rest of the design
--1.1     PN     17-03-2003     Modification for the BIC-core module
--==============================================================--

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.std_logic_unsigned.all;

use work.bus_int_pkg.all;
use work.vme_pkg.all;
use work.cvora_pkg.all;


entity BusIntControl is
  port (
    Clk   : in std_logic;
    RstNA : in std_logic;

    -- Interface
    IntRead     : in std_logic;         -- Interface Read Signal
    IntWrite    : in std_logic;         -- Interface Write Signal
    DataFromInt : in IntDataType;       -- Data From interface
    IntAdd      : in IntAddrOutType;    -- Address From interface
    OpDone    : out std_logic;          -- Operation Done, Read or Write Finished
    DataToInt : out IntDataType;        -- Data going from Control to the Interface

    -- Registers
    ContToRegs : out ContToRegsType;    -- Data going from Control to the Registers
                                        -- This consists of Data + Write Enable Siganal
    RegsToCont : in  RegsToContType;    -- Data Array From the Registers to the Control

    -- Memory
    ContToMem : out ContToMemType;      -- Data going from Control to the Registers
                                        -- This consists of Data + Enable + Read + Write

    MemToCont : in MemToContType        -- Data Array  From the Registers to the Control
                                        -- Data + Done
    );
end BusIntControl;


architecture Comb of BusIntControl is


  component ROMMUX
    port (
      IntAdd    : in  IntAddrOutType;
      DoingOpC  : in  std_logic;
      DoneRam   : in  SelectedRamPosType;
      DataArray : in  MuxDataArrType;
      Sel       : out SelectedPosType;
      Done      : out std_logic;
      DataOut   : out IntDataType
      );
  end component ROMMUX;

  signal nxSelectedPos                   : SelectedPosType;
  signal dataFromIntDelayed              : IntDataType;
  signal intAddDelayed                   : IntAddrOutType;
  signal intReadDelayed, intWriteDelayed : std_logic;
  signal iOpDone                         : std_logic;
  signal doingOpC, doingOpR              : std_logic;
  signal iDataToInt                      : IntDataType;
  signal eRegSelectedPos                 : std_logic_vector(NUMREGS - 1 downto 0);
  signal eRamSelectedPos                 : std_logic_vector(NUMRAM - 1 downto 0);
  signal muxSelPos                       : MuxSelType;
  signal dataMuxArrayIn                  : MuxDataArrType;
  signal doneRam                         : SelectedRamPosType;
  signal RdMemFlag                       : std_logic;

begin


-- Address Decoder + Data Multiplexor
  UROMMUX : ROMMUX
    port map(
      IntAdd    => IntAdd,
      DoingOpC  => doingOpC,
      DoneRam   => doneRam,
      DataArray => dataMuxArrayIn,
      Sel       => nxSelectedPos,
      Done      => iOpDone,
      DataOut   => iDataToInt
      );


  process(RstNA, Clk)
  begin
    if RstNA = '0' then

      ContToRegs.Sel <= (others => '0');

      ContToMem.SelectedPos <= (others => '0');
      dataFromIntDelayed    <= (others => '0');
      intAddDelayed         <= (others => '0');
      intReadDelayed        <= '0';
      intWriteDelayed       <= '0';
      OpDone                <= '0';

      DataToInt <= (others => '0');
      OpDone    <= '0';
      RdMemFlag <= '0';
    elsif rising_edge(Clk) then


      for I in FIRSTREGPOS to FIRSTRAMPOS - 1 loop
        ContToRegs.Sel(I - FIRSTREGPOS) <= nxSelectedPos(I);  -- and IntWrite;
      end loop;
      ContToMem.SelectedPos <= nxSelectedPos(nxSelectedPos'left downto FIRSTRAMPOS);
      dataFromIntDelayed    <= DataFromInt;
      intAddDelayed         <= IntAdd;
      intReadDelayed        <= IntRead;
      intWriteDelayed       <= IntWrite;
      if IntRead = '1' then
        RdMemFlag <= '1';
      elsif iOpDone = '1' then
        RdMemFlag <= '0';
      end if;

      DataToInt <= iDataToInt;
      OpDone    <= iOpDone;
    end if;
  end process;

  doingOpC <= IntRead or IntWrite;
  G0 : for I in 0 to NUMRAM - 1 generate
    doneRam(I) <= MemToCont(I).RdDone;
  end generate;


  G1 : for I in 0 to NUMREGS - 1 generate
    dataMuxArrayIn(I) <= RegsToCont(I);
  end generate;

  G2 : for I in NUMREGS to NUMREGS + NUMRAM - 1 generate
    dataMuxArrayIn(I) <= MemToCont(I - NUMREGS).Data;
  end generate;

  ContToRegs.Data <= dataFromIntDelayed;
  ContToMem.Data  <= dataFromIntDelayed;
  ContToMem.Add   <= intAddDelayed;
  ContToMem.Rd    <= RdMemFlag and not(iOpDone);
  ContToMem.Wr    <= intWriteDelayed;
  ContToRegs.Rd   <= intReadDelayed;
  ContToRegs.Wr   <= intWriteDelayed;


end Comb;
