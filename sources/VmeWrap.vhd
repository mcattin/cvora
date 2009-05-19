----------------------------------------------------
--  
--  Unit    Name :  VmeWrap
--  Unit    Type :  Text Unit
--  
-- Unit Description:
--
-- This unit trnasforms some signals from VmeIntfce.vhd (VmeDtackN and VmeDir)into tristate signals
-- to allow tying several Vme interfaces together and therefore sharing these pins.
-- If several Vme interfaces must be tied together, this entity must be placed between the VME bus
-- and the different VME interfaces.
--
--  Supports: 
--
--     *Different Direction polarity for VME data bus (dir='0' means read from the VME slave or 
--   dir='0' means write into the VME slave).
--     *As many Vme interfaces as needed may be tied together since the outputs are tri-state. 
--
--  Interface details:
--
--     *VME side       : bi-directional buffers are infered inside this entity. Note that either
--        external (in the board) or internal (in the UCF constraints file) pull-ups/pull-downs must be
--        put in order to have a '1' or a '0' during steady state/idle cycles. The pull-up or pull-down
--        will depend on the needed polarity for the Direction pin.
--
--     *Intfce side    : VmeDirFloat indicates that the VmeDirTri signal must go into 'Z'. Note
--                       that when it goes from '0' into 'Z', the pin is forced to '1' during 1 clk cycle
--                       So: '0'->'1'->'Z'. This is to have a fast transition from '0' to '1', instead of
--                       waiting for the pull-ups to do the job. The opposite thing happens if pull-downs
--                       are needed instead of pull-ups ( see DirSamePolarity generic)
--
--  Generics:
--     *DirSamePolarity: Bidirectional buffers in the board (like ABT16245)need a direction pin and
--          bidirectional data pins.The direction pin may have the same polarity as the bidirectional 
--          buffers in Xilinx Spartan or not.
--          if direction='1' means the VME bus IS WRITING  INTO the SLAVE then SET DirSamePolarity='0'.
--             (then external or internal PULL-UPs are needed in VmeDirTri and VmeDtackN pins).
--
--          If direction='1' means the VME bus IS READING  FROM the SLAVE then SET DirSamePolarity='1'.
--             (Then external or internal PULL-DOWNs are needed in VmeDirTri and VmeDtackN pins).
--
--
--  Notation : see Notation.vhd
--
--
--
--
--  Author  :     David Dominguez Montejos
--  Division:     AB/CO 
--
--  Revisions:    1.0
--
--
--  For any bug or comment, please send an e-mail to David.Dominguez@cern.ch	
------------------------------------------------------



library ieee;
use ieee.STD_LOGIC_1164.all;
use ieee.STD_LOGIC_ARITH.all;
use ieee.STD_LOGIC_UNSIGNED.all;



entity VmeWrap is


 generic (
     DirSamePolarity: std_logic :='0');

  port (
     ResetNA       : in std_logic;
     Clk           : in std_logic;
     VmeDirTri     : out std_logic;
     VmeDtackN     : out std_logic;
     IntfceDir     : in std_logic;
     VmeDirFloat   : in std_logic;
     IntfceDtackN  : in std_logic
        );
end VmeWrap;

architecture simple of VmeWrap is

signal intfceDirDly   : std_logic;
signal intfceDtackDlyN: std_logic;
signal dirPolarity    :std_logic;

begin

gen1: if DirSamePolarity='0' generate
         dirPolarity <= '1';
end generate gen1;

gen2: if DirSamePolarity='1' generate
         dirPolarity <= '0';
end generate gen2;


process (ResetNA, Clk,intfceDtackN, intfceDir,dirpolarity)
begin
 if ResetNA ='0' then 
   intfceDirDly <=dirpolarity;
   intfceDtackDlyN <='1';
 elsif rising_edge (clk) then
   intfceDtackDlyN <= intfceDtackN;
   intfceDirDly <= intfceDir;
 end if;
end process;

process (ResetNA, IntfceDir,IntfceDtackN, intfceDirDly, intfceDtackdlyN,VmeDirFloat, dirPolarity)

begin
  if ResetNA ='0' then
   VmeDirTri <= 'Z';
  elsif (intfceDir = not(dirPolarity)  and VmeDirFloat ='0') then 
   VmeDirTri <= not(dirPolarity);
  elsif (intfceDir = dirPolarity and VmeDirFloat ='1' and intfceDirDly =not(dirPolarity)) then
     VmeDirTri <= dirPolarity;
  elsif (intfceDir = dirPolarity and VmeDirFloat='0') then
     VmeDirTri <= dirPolarity; 
  else
     VmeDirTri <= 'Z';
  end if;
  
  if ResetNA ='0' then
   VmeDtackN <='Z';
  elsif intfceDtackN ='0' then
   VmeDtackN <= '0';
  elsif intfceDtackN='1' and intfceDtackdlyN ='0' then
   VmeDtackN <= '1';
  else
   VmeDtackN <='Z';
  end if; 
end process;

end simple;
   

