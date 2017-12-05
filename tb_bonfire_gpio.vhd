--------------------------------------------------------------------------------
-- Company:
-- Engineer:
--
-- Create Date:   23:25:02 12/05/2017
-- Design Name:
-- Module Name:   /home/thomas/fusesoc_projects/bonfire/bonfire-gpio/tb_bonfire_gpio.vhd
-- Project Name:  bonfire-soc_0
-- Target Device:
-- Tool versions:
-- Description:
--
-- VHDL Test Bench Created by ISE for module: bonfire_gpio
--
-- Dependencies:
--
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes:
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;

ENTITY tb_bonfire_gpio IS
END tb_bonfire_gpio;

ARCHITECTURE behavior OF tb_bonfire_gpio IS

    -- Component Declaration for the Unit Under Test (UUT)

    COMPONENT bonfire_gpio
    PORT(
         wb_clk_i : IN  std_logic;
         wb_rst_i : IN  std_logic;
         wb_dat_o : OUT  std_logic_vector(31 downto 0);
         wb_dat_i : IN  std_logic_vector(31 downto 0);
         wb_adr_i : IN  std_logic_vector(7 downto 2);
         wb_we_i : IN  std_logic;
         wb_cyc_i : IN  std_logic;
         wb_stb_i : IN  std_logic;
         wb_ack_o : OUT  std_logic;
         wb_inta_o : OUT  std_logic;
         gpio_o : OUT  std_logic_vector(31 downto 0);
         gpio_i : IN  std_logic_vector(31 downto 0);
         gpio_t : OUT  std_logic_vector(31 downto 0)
        );
    END COMPONENT;


   --Inputs
   signal wb_clk_i : std_logic := '0';
   signal wb_rst_i : std_logic := '0';
   signal wb_dat_i : std_logic_vector(31 downto 0) := (others => '0');
   signal wb_adr_i : std_logic_vector(7 downto 2) := (others => '0');
   signal wb_we_i : std_logic := '0';
   signal wb_cyc_i : std_logic := '0';
   signal wb_stb_i : std_logic := '0';
   signal gpio_i : std_logic_vector(31 downto 0) := (others => '0');

    --Outputs
   signal wb_dat_o : std_logic_vector(31 downto 0);
   signal wb_ack_o : std_logic;
   signal wb_inta_o : std_logic;
   signal gpio_o : std_logic_vector(31 downto 0);
   signal gpio_t : std_logic_vector(31 downto 0);


   subtype t_adr_s is std_logic_vector(7 downto 0);

   constant clk_period : time := 10 ns;

BEGIN

    -- Instantiate the Unit Under Test (UUT)
   uut: bonfire_gpio PORT MAP (
          wb_clk_i => wb_clk_i,
          wb_rst_i => wb_rst_i,
          wb_dat_o => wb_dat_o,
          wb_dat_i => wb_dat_i,
          wb_adr_i => wb_adr_i,
          wb_we_i => wb_we_i,
          wb_cyc_i => wb_cyc_i,
          wb_stb_i => wb_stb_i,
          wb_ack_o => wb_ack_o,
          wb_inta_o => wb_inta_o,
          gpio_o => gpio_o,
          gpio_i => gpio_i,
          gpio_t => gpio_t
        );

   -- Clock process definitions
   clk_process :process
   begin
        wb_clk_i <= '0';
        wait for clk_period/2;
        wb_clk_i <= '1';
        wait for clk_period/2;
   end process;


 -- Input
    input_proc: process
    begin

    wait for clk_period*3;

    -- Simulate input pattern
    for i in  gpio_i'range loop
       gpio_i(i) <= '1';
       wait for clk_period*8.33;
       gpio_i(i) <= '0';
    end loop;

    wait;

    end process;



   -- Stimulus process
   stim_proc: process

   procedure wb_write(address : in t_adr_s; data : in std_logic_vector(wb_dat_i'range)) is
         begin
            wait until rising_edge(wb_clk_i);
            wb_adr_i <= std_logic_vector(address(wb_adr_i'range));
            wb_dat_i <= data;
            wb_we_i <= '1';
            wb_cyc_i <= '1';
            wb_stb_i <= '1';
          

            wait  until rising_edge(wb_clk_i) and wb_ack_o = '1' ;
            wb_stb_i <= '0';
            wb_cyc_i <= '0';

        end procedure;

       procedure wb_read(address : in t_adr_s;
                          data: out std_logic_vector(wb_dat_o'range) )  is
         begin
            wait until rising_edge(wb_clk_i);
            wb_adr_i <= std_logic_vector(address(wb_adr_i'range));
            wb_we_i <= '1';
            wb_cyc_i <= '1';
            wb_stb_i <= '1';
            wb_we_i <= '0';
          
            wait until rising_edge(wb_clk_i) and wb_ack_o = '1';
            data:= wb_dat_o;
            wb_stb_i <= '0';
            wb_cyc_i <= '0';

        end procedure;



   begin
      -- hold reset state for 100 ns.
      wb_write(X"04",X"FFFFFFFF"); -- Input Enable
      wb_write(X"18",X"40000000"); -- rise_ie on bit 30






      wait;
   end process;

END;
