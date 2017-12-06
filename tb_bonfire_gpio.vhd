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
USE ieee.numeric_std.ALL;

use work.txt_util.all;

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

         rise_irq_o : out std_logic;
         fall_irq_o : out std_logic;
         high_irq_o : out std_logic;
         low_irq_o : out std_logic;

         gpio_o : OUT  std_logic_vector(31 downto 0);
         gpio_i : IN  std_logic_vector(31 downto 0);
         gpio_t : OUT  std_logic_vector(31 downto 0)
        );
    END COMPONENT;

   COMPONENT sim_iobuf
    PORT(
        I : IN std_logic;
        T : IN std_logic;
        IO : INOUT std_logic;
        O : OUT std_logic
        );
    END COMPONENT;


   --Inputs
   signal wb_clk_i : std_logic := '0';
   signal wb_rst_i : std_logic := '0';
   signal wb_dat_i : std_logic_vector(31 downto 0) := (others => 'X');
   signal wb_adr_i : std_logic_vector(7 downto 2) := (others => 'X');
   signal wb_we_i : std_logic := '0';
   signal wb_cyc_i : std_logic := '0';
   signal wb_stb_i : std_logic := '0';
   signal gpio_i : std_logic_vector(31 downto 0);

    --Outputs
   signal wb_dat_o : std_logic_vector(31 downto 0);
   signal wb_ack_o : std_logic;
   signal wb_inta_o : std_logic;
   signal gpio_o : std_logic_vector(31 downto 0);
   signal gpio_t : std_logic_vector(31 downto 0);


   -- Simulated IO

   signal pads : std_logic_vector(31 downto 0);

   -- interrupt signals
   signal rise_irq, high_irq, fall_irq,low_irq : std_logic;


   subtype t_adr_s is std_logic_vector(7 downto 0);

   constant clk_period : time := 10 ns;

BEGIN

   -- Wire pads

   padgen: for i in pads'range generate

   pad_iobuf: sim_iobuf PORT MAP(
        I => gpio_o(i),
        O => gpio_i(i),
        T => gpio_t(i),
        IO => pads(i)
    );


   end generate;


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

          rise_irq_o => rise_irq,
          fall_irq_o => fall_irq,
          high_irq_o => high_irq,
          low_irq_o => low_irq,

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
    pads <= (others=>'0');

    -- Simulate input pattern
    for i in  pads'range loop
       pads(i) <= '1';
       wait for clk_period*8.33;
       pads(i) <= '0';
    end loop;
    wait for clk_period*8.33;
    pads <= (others => 'Z');

    wait;

    end process;



   -- Stimulus process
   stim_proc: process
   variable dat : std_logic_vector(wb_dat_o'range);
   variable chk : boolean;

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

        procedure check_register(adr: in t_adr_s;
                                v: in std_logic_vector(wb_dat_o'range);
                                 r: out boolean)  is

        variable d : std_logic_vector(wb_dat_o'range);
        begin
          wb_read(adr,d);
          r:= d=v;
        end;



        procedure clear_ip_reg(adr : in t_adr_s;
                               r: out boolean )  is
        variable d : std_logic_vector(wb_dat_o'range);
        begin
          wb_write(adr,X"FFFFFFFF"); -- clear all pending flags
          check_register(adr,X"00000000",r); -- Check if all cleared
        end;



   begin
      -- hold reset state for 100 ns.
      wb_write(X"04",X"FFFFFFFF"); -- Input Enable
      wb_write(X"18",X"40000000"); -- rise_ie on bit 30

      wait until rising_edge(wb_clk_i) and rise_irq='1';  -- Wait until Interrupt has occured
      print("RISE IRQ on bit 30");
      wb_write(X"1C",X"C0000000"); -- Clear pending interrupts
      wb_read(X"1C",dat); -- Check Pending flag
      assert dat(30)='0'
      report "Interupt pending Flag clear failed"
      severity failure;
      wb_write(X"18",X"00000000");


      wb_write(X"20",X"00000001"); -- Enable Fall Interrupt on port 0
      wait until rising_edge(wb_clk_i) and fall_irq='1';  -- Wait until Interrupt has occured

      wb_read(X"24",dat); -- Read fall_ip
      check_register(X"24",X"FFFFFFFF",chk);
      assert chk report "Error on Fall_ip register "
        severity failure;
      check_register(X"2C",X"FFFFFFFF",chk);
      assert chk  report "Error on high_ip register "
        severity failure;

      clear_ip_reg(X"1C",chk); assert chk report "Failure on clear rise_ip"
                                 severity failure;

      clear_ip_reg(X"24",chk); assert chk report "Failure on clear fall_ip"
                                 severity failure;

      clear_ip_reg(X"2C",chk); assert chk report "Failure on clear high_ip"
                                 severity failure;
      -- the result of the following statement can only be seen
      -- in the timeing diagram for 1 clock, because the low_ip bits
      -- will be triggered again immediatly from the inputs
      clear_ip_reg(X"34",chk);

      -- Output Test
      wb_write(X"0C",X"55AA55BB"); -- port register - output
      wb_write(X"28",X"FFFFFFFF"); -- high_ie enable
      wb_write(X"08",X"FFFFFFFF"); -- output_en register
      wait until rising_edge(wb_clk_i) and high_irq='1';
      assert pads=X"55AA55BB" report "Failure on Output Test"
        severity failure;

      check_register(X"00",X"55AA55BB",chk); -- value register
      assert chk report "Readback of output value on input failed" severity failure;
      wb_write(X"04",X"00000000"); -- input_en register (disable all inputs)
      wait for clk_period*5; -- let input synchronizers settle
      check_register(X"00",X"00000000",chk);
      assert chk report "Disabling of inputs failed" severity failure;
      print("Simulation sucessfully finished");
      wait;
   end process;

END;
