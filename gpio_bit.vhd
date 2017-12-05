----------------------------------------------------------------------------------
-- Company:
-- Engineer:
--
-- Create Date:    20:02:29 12/05/2017
-- Design Name:
-- Module Name:    gpio_bit - Behavioral
-- Project Name:
-- Target Devices:
-- Tool versions:
-- Description:
-- Models a single bit of the GPIO Port. See chapter 17. of the SiFive FE310-G000 Manual
-- Dependencies:
--
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity gpio_bit is
    Port ( rst_i: in STD_LOGIC;
           clk_i : in STD_LOGIC;
           pin_value_o : out  STD_LOGIC;
           input_en_i : in  STD_LOGIC;
           output_en_i : in  STD_LOGIC;
           port_value_i : in  STD_LOGIC;
           out_xor_i : in  STD_LOGIC;

           rising_o : out STD_LOGIC;
           falling_o : out STD_LOGIC;

           -- IO Block control - connect to a IO Block
           iob_o : out  STD_LOGIC; -- Output
           iob_i : in STD_LOGIC;  -- Input
           iob_t : out STD_LOGIC   -- Tri-State out

           );
end gpio_bit;

architecture Behavioral of gpio_bit is

signal pv1, pv2, pin_value : std_logic; -- Input synchronizers

signal edge : std_logic; --  for Edge detector
signal rising, falling : std_logic;

begin

-- Input
 process(clk_i) begin
    if rising_edge(clk_i) then
      pv1 <= iob_i and input_en_i;
      pv2 <= pv1;
      pin_value <= pv2;

      -- Edge detector
      rising <= '0';
      falling <= '0';
      if edge='0' and pin_value='1' then
        rising <= '1';
      elsif edge='1' and pin_value='0' then
        falling <= '1';
      end if;

      edge <= pin_value;
    end if;

 end process;

 pin_value_o <= pin_value;

 -- Output

 iob_o <=  port_value_i xor out_xor_i;
 iob_t <=  output_en_i;


  rising_o <= rising;
  falling_o <= falling;









end Behavioral;

