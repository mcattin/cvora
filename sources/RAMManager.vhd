------------------------------------------------------------------
-- This block arbitrates access to the IDT 71V35761 RAM chip.
-- The block can receive a write access from the History block,
-- a read access from the Trigger block or R/W acces from the 
-- bus control block. For read operations, this block presents the
-- data and a dataValid line that validates the data. This line is 
-- active for one tick only, and data are only guaranteed to be
-- correct during this tick. For write operations there is also a 
-- handshake (dataWritten goes high when the write is done). 
-- Priority is first given to access from the trigger block, then 
-- to the Bus Control and finally to the History block.
------------------------------------------------------------------
-- P. Nouchi le 12-03-2007 Modification to adapt the CTRV RamManager to the CVORA
-- Version 1.0 le 12-03-2007 Remove Interface from Trigger
-- Version 1.1 le 05-04-2007 Remove Signal P2MemBusy

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity RAMManager is
    Port ( RstN : in std_logic;
           Clk : in std_logic;

           -- Interface with History block
           DataFromHistory : in std_logic_vector(31 downto 0);
           AddrFromHistory : in std_logic_vector(16 downto 0);
           WriteFromHistory : in std_logic;
           DataFromHistoryWritten: out std_logic;

           -- Interface with Bus Control block
           AddrFromCont : in std_logic_vector(16 downto 0);
           ReadFromCont : in std_logic;
-- Not used in CVORA Memory is read only from the VME bus
--           WriteFromCont : in std_logic;  
--           DataFromCont : in std_logic_vector(31 downto 0);
--           DataFromContWritten: out std_logic;
           DataToCont : out std_logic_vector(31 downto 0);
           DataToContValid : out std_logic;
			
           -- Interface with IDT 71V35761 RAM chip
           RAMAddr : out std_logic_vector(16 downto 0);
           RAMData : inout std_logic_vector(31 downto 0);
           RAMOEN : out std_logic;
           RAMGWN : out std_logic;
           RAMADSCN : out std_logic;

           -- Other IDT 71V35761 that we don't really use 
           RAMCEN : out std_logic;
           RAMCS0 : out std_logic;
           RAMCS1N : out std_logic;
           RAMBWEN : out std_logic;
           RAMBWN : out std_logic_vector(4 downto 1);
           RAMADVN : out std_logic;
--           RAMPar : in std_logic_vector(4 downto 1);   -- to be left floating
           RAMLBON : out std_logic;
           RAMZZ : out std_logic;
           RAMADSPN : out std_logic);




end RAMManager;

architecture RTL of RAMManager is

type StateType is (Idle, contRead, contWait, contDataValid, --contWrite1, contWrite2, contWrite3,
                   historyWrite1, historyWrite2, historyWrite3);

signal currentState, nextState: StateType;

signal contPendingAddr: std_logic_vector(16 downto 0);
--signal contPendingData: std_logic_vector(31 downto 0);
--signal contPendingWrite, 
signal contPendingRead: std_logic;

signal historyPendingAddr: std_logic_vector(16 downto 0);
signal historyPendingData: std_logic_vector(31 downto 0);
signal historyPendingWrite: std_logic;

signal writing: std_logic;
signal ramDataIn, ramDataOut: std_logic_vector(31 downto 0);

signal dataToFPGA : std_logic_vector(31 downto 0);

--attribute syn_useioff : boolean;
--attribute syn_useioff of ramDataIn : signal is true;
--attribute syn_useioff of ramDataOut : signal is true;

begin

RAMADSPN <= '1';
RAMADVN <= '1';
RAMBWN <= "1111";
RAMLBON <= '1';
RAMBWEN <= '1';  
RAMZZ <= '0';
RAMCS0 <= '1';
RAMCS1N <= '0';
RAMCEN <= '0';
RAMADSCN <= '0';

RAMData <= ramDataOut when writing='1' else (others=>'Z');
ramDataIn <= RAMData;


PendingCont: process(Clk)
begin
if Clk'event and Clk='1' then
 if RstN='0' then
--   contPendingAddr <= (others=>'0');
--   contPendingData <= (others=>'0');
--   contPendingWrite <= '0';
   contPendingRead <= '0';
 else
  if ReadFromCont='1' then
   contPendingAddr <= AddrFromCont;
   contPendingRead <= '1';
--  elsif WriteFromCont='1' then
--   contPendingAddr <= AddrFromCont;
--   contPendingData <= DataFromCont;
--   contPendingWrite <= '1';
  elsif currentState=contDataValid then-- or currentState=contWrite3 then
--   contPendingWrite <= '0';
   contPendingRead <= '0';
  end if;  
 end if;
 end if;
end process PendingCont;

--PendingTrigger: process(Clk)
--begin
--if Clk'event and Clk='1' then
-- if RstN='0' then
----   triggerPendingAddr <= (others=>'0');
--   triggerPendingRead <= '0';
-- else
--  if ReadFromTrigger='1' then
--   triggerPendingRead <= '1';
--  elsif currentState=triggerDataValid then
--   triggerPendingRead <= '0';
--  end if;  
--	if ReadFromTrigger='1' then
--	   triggerPendingAddr <= AddrFromTrigger;
--	end if;
-- end if;
-- end if;
--end process PendingTrigger;

PendingHistory: process(RstN, Clk)
begin
if Clk'event and Clk='1' then
 if RstN='0' then
--   historyPendingAddr <= (others=>'0');
--   historyPendingData <= (others=>'0');
   historyPendingWrite <= '0';
 else
  if WriteFromHistory='1' then
   historyPendingAddr <= AddrFromHistory;
   historyPendingData <= DataFromHistory;
   historyPendingWrite <= '1';
  elsif currentState=historyWrite3 then
   historyPendingWrite <= '0';
  end if;  
 end if;
 end if;
end process PendingHistory;

States: process(RstN, Clk)
begin
if Clk'event and Clk='1' then
 if RstN='0' then
  currentState <= Idle;
 else
  currentState <= nextState;
 end if; 
 end if; 
end process States;

Transitions: process(currentState, contPendingRead,-- contPendingWrite,
                     historyPendingWrite)
begin
 case currentState is 
  when Idle =>
--   if contPendingWrite='1' then
--    nextState <= contWrite1;
--   els
	if contPendingRead = '1' then
    nextState <= contRead;
   elsif historyPendingWrite='1' then
    nextState <= historyWrite1;
   else 
    nextState <= Idle;
   end if;
--  when contWrite1 =>
--   nextState <= contWrite2; 
--  when contWrite2 =>
--   nextState <= contWrite3;
--  when contWrite3 =>
--   nextState <= Idle; 
  when contRead =>
   nextState <= contWait;
  when contWait =>
   nextState <= contDataValid;
  when contDataValid =>
   nextState <= Idle;
  when historyWrite1 =>
   nextState <= historyWrite2;
  when historyWrite2 =>
   nextState <= historyWrite3;
  when historyWrite3 =>
   nextState <= Idle;

  when others =>
   nextState <= Idle;
 end case;
end process Transitions;

Outputs: process(Clk)
begin
 if Clk'event and Clk='1' then
  case currentState is
   when Idle =>
    writing <= '0';
    RAMOEN <= '1';
    RAMGWN <= '1';
    DataFromHistoryWritten <= '0';
    DataToContValid <= '0';
--    DataFromContWritten <= '0';
   when historyWrite1 =>
    RAMAddr <= historyPendingAddr;
    ramDataOut <= historyPendingData;
    writing <= '1';
    RAMGWN <= '1';
   when historyWrite2 =>
    RAMGWN <= '0';
   when historyWrite3 =>
    RAMGWN <= '0';
    DataFromHistoryWritten <= '1';
   when contRead =>
    RAMAddr <= contPendingAddr;
    RAMOEN <= '0'; 
   when contWait =>
   when contDataValid =>
    DataToFPGA <= ramDataIn;
    DataToContValid <= '1';
--   when contWrite1 =>
--    RAMAddr <= contPendingAddr;
--    ramDataOut <= contPendingData;
--    writing <= '1';
--    RAMGWN <= '1';
--   when contWrite2 =>
--    RAMGWN <= '0';
--    DataFromContWritten <= '0';		
--   when contWrite3 =>
--    RAMGWN <= '0';
--    DataFromContWritten <= '1';			   
   when others =>
    writing <= '0';
    RAMOEN <= '1';
    RAMGWN <= '1';
    DataFromHistoryWritten <= '0';
    DataToContValid <= '0';
--    DataFromContWritten <= '0';
  end case;
 end if;
end process Outputs;

DataToCont <= DataToFPGA;

end RTL;
