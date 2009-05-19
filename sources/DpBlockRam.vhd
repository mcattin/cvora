--==============================================================--
--Design Units : CTX1 Control and Statistics
--Size:		
--Speed:	 	 
--File Name: 	 MebRam.vhd
--
--Purpose:	The dpblockram implements a synthetisable model of a 
--				dual port RAM.
--				There are an input data and addr ports to allow the
--				writing at the reception of a GMT frame.
--				The output data and addr ports allow a simultanous
--				reading of the circular buffer by the user, at the
--				same time than it is beeing written.
--				
--				The frame and the millisecond stamp are stored in the
--				same ram word. It is the task of the MEB block to 
--				separate the frame data from the millisecond stamp data. 
--
--Limitations:	
--
--Errors:
--
--Libraries: 
--
--Dependancies:  It instantiates a synthetisable model of a DPRAM
--					  See MebRam.vhd
--
--Author: Pablo Antonio Alvarez Sanchez
--		  	 European Organisation for Nuclear Research
--		 	 SL SPS/LHC -- Control -- Timing Division
--		 	 CERN, Geneva, Switzerland,  CH-1211
--		 	 Building 864 Room 1 - A24
--
--Simulator:               ModelSim XE 5.5e_p1
--==============================================================--
--Revision List
--Version Author Date		 Changes
--
--1.0		 PAAS	  30.09.2002 Added comments, tested with the
--										rest of the design
--==============================================================--

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
 
 entity dpblockram is
 generic (dl : integer := 32; 		-- Length of the data word 
 			 al : integer := 10;			-- Size of the addr map (10 = 1024 words)
			 nw : integer := 1024);    -- Number of words
			 									-- 'nw' has to be coherent with 'al'

 port (clk  : in std_logic; 			-- Global Clock
 	we   : in std_logic; 				-- Write Enable
 	aw    : in std_logic_vector(al - 1 downto 0); -- Write Address 
 	ar : in std_logic_vector(al - 1 downto 0); 	 -- Read Address
 	di   : in std_logic_vector(dl - 1 downto 0);  -- Data input
 	dw  : out std_logic_vector(dl - 1 downto 0);  -- Data write, normaly open
 	do  : out std_logic_vector(dl - 1 downto 0)); 	 -- Data output
 end dpblockram; 												 -- DATA OUTPUT NOT REGISTERED!
 
 architecture syn of dpblockram is 
 
 type ram_type is array (nw - 1 downto 0) of std_logic_vector (dl - 1 downto 0); 
 signal RAM : ram_type; 
 signal read_a : std_logic_vector(al - 1 downto 0); 
 signal read_ar : std_logic_vector(al - 1 downto 0); 
 
 begin 
 process (clk) 
 begin 
 	if falling_edge(clk) then  
 		if (we = '1') then 
 			RAM(conv_integer(aw)) <= di; 
 		end if; 
 		read_a <= aw; 
 		read_ar <= ar; 
 	end if; 
 end process; 
 
 dw <= RAM(conv_integer(read_a)); 
 do <= RAM(conv_integer(read_ar)); -- Notice that the Data Output is not registered
 
 end syn;

 
