----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    23:34:41 12/06/2017 
-- Design Name: 
-- Module Name:    sim_iobuf - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
-- IO Buf for Simulation uses. Defined locally to allow easy simulation with ghdl
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;




entity sim_iobuf is
    Port ( I : in  STD_LOGIC;
           O : out  STD_LOGIC;
           T : in  STD_LOGIC;
           IO : inout  STD_LOGIC);
end sim_iobuf;

architecture Behavioral of sim_iobuf is

begin

   O <= IO;

   process(I,T) begin
     if T='1' then
       IO <= 'Z';
     else
       IO <= I;
     end if;  
     
   end process;


end Behavioral;

