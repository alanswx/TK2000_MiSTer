--
-- Multicore 2 / Multicore 2+
--
-- Copyright (c) 2017-2020 - Victor Trucco
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- Redistributions of source code must retain the above copyright notice,
-- this list of conditions and the following disclaimer.
--
-- Redistributions in synthesized form must reproduce the above copyright
-- notice, this list of conditions and the following disclaimer in the
-- documentation and/or other materials provided with the distribution.
--
-- Neither the name of the author nor the names of other contributors may
-- be used to endorse or promote products derived from this software without
-- specific prior written permission.
--
-- THIS CODE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- You are responsible for any legal issues arising from your use of this code.
--
		
--
-- Terasic DE1 top-level
--

-- altera message_off 10540 10541

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Generic top-level 	 for Altera DE1 board
	entity tk2000_de1 is
	port (
		-- Clocks
		CLOCK_24       : in    std_logic_vector(1 downto 0);
		CLOCK_27       : in    std_logic_vector(1 downto 0);
		CLOCK_50       : in    std_logic;
		EXT_CLOCK      : in    std_logic;

		-- Switches
		SW             : in    std_logic_vector(9 downto 0);
		-- Buttons
		KEY            : in    std_logic_vector(3 downto 0);

		-- 7 segment displays
		HEX0           : out   std_logic_vector(6 downto 0)		:= (others => '1');
		HEX1           : out   std_logic_vector(6 downto 0)		:= (others => '1');
		HEX2           : out   std_logic_vector(6 downto 0)		:= (others => '1');
		HEX3           : out   std_logic_vector(6 downto 0)		:= (others => '1');
		-- Red LEDs
		LEDR           : out   std_logic_vector(9 downto 0)		:= (others => '0');
		-- Green LEDs
		LEDG           : out   std_logic_vector(7 downto 0)		:= (others => '0');

		-- VGA
		VGA_R          : out   std_logic_vector(3 downto 0)		:= (others => '0');
		VGA_G          : out   std_logic_vector(3 downto 0)		:= (others => '0');
		VGA_B          : out   std_logic_vector(3 downto 0)		:= (others => '0');
		VGA_HS         : out   std_logic									:= '1';
		VGA_VS         : out   std_logic									:= '1';

		-- Serial
		UART_RXD       : in    std_logic;
		UART_TXD       : out   std_logic									:= '1';

		-- PS/2 Keyboard
		PS2_CLK        : inout std_logic									:= 'Z';
		PS2_DAT        : inout std_logic									:= 'Z';

		-- I2C
		I2C_SCLK       : inout std_logic									:= 'Z';
		I2C_SDAT       : inout std_logic									:= 'Z';

		-- Audio
		AUD_XCK        : out   std_logic									:= '0';
		AUD_BCLK       : out   std_logic									:= '0';
		AUD_ADCLRCK    : out   std_logic									:= '0';
		AUD_ADCDAT     : in    std_logic;
		AUD_DACLRCK    : out   std_logic									:= '0';
		AUD_DACDAT     : out   std_logic									:= '0';

		-- SRAM
		SRAM_ADDR      : out   std_logic_vector(17 downto 0)		:= (others => '0');
		SRAM_DQ        : inout std_logic_vector(15 downto 0)		:= (others => 'Z');
		SRAM_CE_N      : out   std_logic									:= '1';
		SRAM_OE_N      : out   std_logic									:= '1';
		SRAM_WE_N      : out   std_logic									:= '1';
		SRAM_UB_N      : out   std_logic									:= '1';
		SRAM_LB_N      : out   std_logic									:= '1';

		-- SDRAM
		DRAM_ADDR      : out   std_logic_vector(11 downto 0)		:= (others => '0');
		DRAM_DQ        : inout std_logic_vector(15 downto 0)		:= (others => 'Z');
		DRAM_BA_0      : out   std_logic									:= '1';
		DRAM_BA_1      : out   std_logic									:= '1';
		DRAM_CAS_N     : out   std_logic									:= '1';
		DRAM_CKE       : out   std_logic									:= '1';
		DRAM_CLK       : out   std_logic									:= '1';
		DRAM_CS_N      : out   std_logic									:= '1';
		DRAM_LDQM      : out   std_logic									:= '1';
		DRAM_RAS_N     : out   std_logic									:= '1';
		DRAM_UDQM      : out   std_logic									:= '1';
		DRAM_WE_N      : out   std_logic									:= '1';

		-- Flash
		FL_ADDR        : out   std_logic_vector(21 downto 0)		:= (others => '0');
		FL_DQ          : inout std_logic_vector(7 downto 0)		:= (others => 'Z');
		FL_RST_N       : out   std_logic									:= '1';
		FL_OE_N        : out   std_logic									:= '1';
		FL_WE_N        : out   std_logic									:= '1';
		FL_CE_N        : out   std_logic									:= '1';

		-- SD card (SPI mode)
		SD_nCS         : out   std_logic									:= '1';
		SD_MOSI        : out   std_logic									:= '1';
		SD_SCLK        : out   std_logic									:= '1';
		SD_MISO        : in    std_logic;

		-- GPIO
		GPIO_0         : inout std_logic_vector(35 downto 0)		:= (others => 'Z');
		GPIO_1         : inout std_logic_vector(35 downto 0)		:= (others => 'Z')
	);
end 	;

architecture behavior of tk2000_de1 is

	-- Clocks
	signal clock_28_s			: std_logic;		-- Dobro para scandoubler
	signal clock_14_s			: std_logic;		-- Master
	signal phi0_s				: std_logic;		-- phase 0
	signal phi1_s				: std_logic;		-- phase 1
	signal phi2_s				: std_logic;		-- phase 2
	signal clock_2M_s			: std_logic;		-- Clock Q3

	-- Resets
	signal pll_locked_s		: std_logic;
	signal por_reset_s		: std_logic;
	signal softreset_s		: std_logic;
	signal reset_s				: std_logic;

	-- ROM
	signal rom_addr_s				: std_logic_vector(13 downto 0);
	signal rom_data_from_s		: std_logic_vector(7 downto 0);
	signal rom_oe_s				: std_logic;
	signal rom_we_s				: std_logic;
	signal ipl_s					: std_logic;
	signal iplrom_data_from_s	: std_logic_vector(7 downto 0);
	signal rrom_data_from_s		: std_logic_vector(7 downto 0);

	-- RAM
	signal ram_addr_s			: std_logic_vector(15 downto 0);
	signal ram_data_to_s		: std_logic_vector(7 downto 0);
	signal ram_oe_s			: std_logic;
	signal ram_we_s			: std_logic;

	-- SRAM
	signal sram_addr_s		: std_logic_vector(17 downto 0);
	signal sram_data_from_s	: std_logic_vector(7 downto 0);
--	signal sram_data_to_s	: std_logic_vector(7 downto 0);
	signal sram_ce_s			: std_logic;
	signal sram_oe_s			: std_logic;
	signal sram_we_s			: std_logic;

	-- Keyboard
	signal kbd_ctrl_s			: std_logic;
	signal kbd_rows_s			: std_logic_vector(7 downto 0);
	signal kbd_cols_s			: std_logic_vector(5 downto 0);
	signal FKeys_s				: std_logic_vector(12 downto 1);

	-- Audio
	signal spk_s				: std_logic;

	-- K7
	signal cas_i_s				: std_logic;
	signal cas_o_s				: std_logic;
	signal cas_motor_s		: std_logic_vector(1 downto 0);

	-- Video
	signal video_r_s			: std_logic_vector(7 downto 0);
	signal video_g_s			: std_logic_vector(7 downto 0);
	signal video_b_s			: std_logic_vector(7 downto 0);
	signal video_ro_s			: std_logic_vector(7 downto 0);
	signal video_go_s			: std_logic_vector(7 downto 0);
	signal video_bo_s			: std_logic_vector(7 downto 0);
	signal video_color_s		: std_logic;
	signal video_bit_s		: std_logic;
	signal video_hsync_n_s	: std_logic;
	signal video_vsync_n_s	: std_logic;
	signal video_hbl_s		: std_logic;
	signal video_vbl_s		: std_logic;
	signal video_ld194_s		: std_logic;

	-- LPT
	signal lpt_stb_s			: std_logic;
	signal lpt_busy_s			: std_logic;

	-- Periferico
	signal per_disrom_s		: std_logic;
	signal per_iosel_n_s		: std_logic;
	signal per_devsel_n_s	: std_logic;
	signal per_we_s			: std_logic;
	signal per_addr_s			: std_logic_vector(7 downto 0);
	signal per_data_from_s	: std_logic_vector(7 downto 0);
	signal per_data_to_s		: std_logic_vector(7 downto 0);

	-- Debug
	signal D_cpu_pc_s			: std_logic_vector(15 downto 0);
--	signal D_track_s			: std_logic_vector(5 downto 0);
	signal D_display_s		: std_logic_vector(15 downto 0);
	signal D_cpu_addr_s		: std_logic_vector(15 downto 0);

begin

	-- PLL
	pll: work.pll1
	port map (
		inclk0		=> CLOCK_50,
		c0				=> clock_28_s,
		c1				=> clock_14_s,
		locked		=> pll_locked_s
	);

	-- TK2000
	tk2000_inst: work.tk2000
	port map (
		clock_14_i			=> clock_14_s,
		reset_i				=> reset_s,
		-- RAM
		ram_addr_o			=> ram_addr_s,
		ram_data_to_o		=> ram_data_to_s,
		ram_data_from_i	=> sram_data_from_s,
		ram_oe_o				=> ram_oe_s,
		ram_we_o				=> ram_we_s,
		-- ROM
		rom_addr_o			=> rom_addr_s,
		rom_data_from_i	=> rom_data_from_s,
		rom_oe_o				=> rom_oe_s,
		rom_we_o				=> rom_we_s,
		-- Keyboard
		kbd_rows_o			=> kbd_rows_s,
		kbd_cols_i			=> kbd_cols_s,
		kbd_ctrl_o			=> kbd_ctrl_s,
		-- Audio
		spk_o					=> spk_s,
		-- Video
		video_color_o		=> video_color_s,
		video_bit_o			=> video_bit_s,
		video_hsync_n_o	=> open,
		video_vsync_n_o	=> open,
		video_hbl_o			=> video_hbl_s,
		video_vbl_o			=> video_vbl_s,
		video_ld194_o		=> video_ld194_s,
		-- Cassete
		cas_i					=> cas_i_s,
		cas_o					=> cas_o_s,
		cas_motor_o			=> cas_motor_s,
		-- LPT
		lpt_stb_o			=> lpt_stb_s,
		lpt_busy_i			=> lpt_busy_s,
		-- Periferico
		phi0_o				=> phi0_s,					-- fase 0 __|---|___|---
		phi1_o				=> phi1_s,					-- fase 1 ---|___|---|___
		phi2_o				=> phi2_s,					-- fase 2 ___|---|___|---
		clock_2m_o			=> clock_2M_s,
		read_write_o		=> per_we_s,
		irq_n_i				=> '1',
		nmi_n_i				=> '1',
		dis_rom_i			=> per_disrom_s,
		io_select_n_o		=> per_iosel_n_s,
		dev_select_n_o		=> per_devsel_n_s,
		per_addr_o			=> per_addr_s,
		per_data_from_i	=> per_data_from_s,
		per_data_to_o		=> per_data_to_s,
		-- Debug
		D_cpu_pc_o			=> D_cpu_pc_s
	);

	-- Keyboard
	kb: work.keyboard
	generic map (
		clkfreq_g			=> 28000
	)
	port map (
		clock_i				=> clock_28_s,
		reset_i				=> por_reset_s,
		ps2_clk_io			=> PS2_CLK,						-- Externo
		ps2_data_io			=> PS2_DAT,						-- Externo
		rows_i				=> kbd_rows_s,
		row_ctrl_i			=> kbd_ctrl_s,
		cols_o				=> kbd_cols_s,
		FKeys_o				=> FKeys_s
	);

	----------------
	-- Gerenciador de audio com CODEC WM8731
	----------------
	sound: work.Audio_WM8731
	port map (
		reset_i			=> reset_s,							-- Reset geral
		clock_i			=> CLOCK_24(0),					-- Clock 24 MHz
		mic_i				=> cas_o_s,							-- Entrada 1 bit MIC
		spk_i				=> spk_s,							-- Entrada 1 bit Speaker
		ear_o				=> cas_i_s,							-- Saida 1 bit EAR

		i2s_xck_o		=>	AUD_XCK,							-- Ligar nos pinos do TOP
		i2s_bclk_o		=> AUD_BCLK,
		i2s_adclrck_o	=> AUD_ADCLRCK,
		i2s_adcdat_i	=> AUD_ADCDAT,
		i2s_daclrck_o	=> AUD_DACLRCK,
		i2s_dacdat_o	=> AUD_DACDAT,
		
		i2c_sda_io		=> I2C_SDAT,							-- Ligar no pino I2C SDA
		i2c_scl_io		=> I2C_SCLK								-- Ligar no pino I2C SCL
	);

	-- ROM
	rrom: entity work.tk2000_rom
	port map (
		clock		=> clock_28_s,
		address	=> rom_addr_s,
		q			=> rrom_data_from_s
	);

	rom: entity work.ipl_rom
	port map (
		clock		=> clock_28_s,
		address	=> rom_addr_s(12 downto 0),
		q			=> iplrom_data_from_s
	);

	-- VGA
	vga : work.vga_controller
	port map (
		clock_28_i		=> clock_28_s,
		video_i			=> video_bit_s,
		color_i			=> video_color_s,
		hbl_i				=> video_hbl_s,
		vbl_i				=> video_vbl_s,
		ld194_i			=> video_ld194_s,
		color_type_i	=> SW(8),
		vga_hs_n_o		=> video_hsync_n_s,
		vga_vs_n_o		=> video_vsync_n_s,
		vga_blank_n_o	=> open,
		vga_r_o			=> video_r_s,
		vga_g_o			=> video_g_s,
		vga_b_o			=> video_b_s
	);
	
	VGA_HS	<= video_hsync_n_s;
	VGA_VS	<= video_vsync_n_s;
	VGA_R		<= video_r_s(7 downto 4);
	VGA_G		<= video_g_s(7 downto 4);
	VGA_B		<= video_b_s(7 downto 4);

	-- SPI
	spi: entity work.per_spi
	port map (
		por_i					=> por_reset_s,
		reset_i				=> reset_s,
		clock_i				=> clock_14_s,
		clock_q3_i			=> clock_2m_s,
		phi0_i				=> phi0_s,
		addr_i				=> per_addr_s,
		data_i				=> per_data_to_s,
		data_o				=> per_data_from_s,
		read_write_i		=> per_we_s,
		dev_sel_n_i			=> per_devsel_n_s,
		-- IPL
		key_opt_i			=> SW(0),
		ipl_o					=> ipl_s,
		reset_o				=> softreset_s,
		-- SD card interface
		spi_cs_n_o			=> SD_nCS,
		spi_sclk_o			=> SD_SCLK,
		spi_mosi_o			=> SD_MOSI,
		spi_miso_i			=> SD_MISO,
		-- Debug
		debug_o				=> LEDG
	);


	-- Glue Logic

	por_reset_s	<= '1' when pll_locked_s = '0' or KEY(3) = '0'															else '0';
	reset_s		<= '1' when por_reset_s = '1'  or KEY(0) = '0' or FKeys_s(4) = '1' or softreset_s = '1'	else '0';

	-- RAM e ROM
	sram_addr_s	<= "1000" & rom_addr_s		when rom_we_s = '1' and ipl_s = '1' 	else
						"1000" & rom_addr_s		when rom_oe_s = '1' 							else
						"00"   & ram_addr_s;

	rom_data_from_s	<= iplrom_data_from_s	when ipl_s = '1' else
--								rrom_data_from_s;
								sram_data_from_s;

	sram_ce_s	<= '1';
	sram_we_s	<= ram_we_s or (rom_we_s and ipl_s);
	sram_oe_s	<= ram_oe_s or (rom_oe_s and not ipl_s);

	SRAM_DQ		<= "00000000" & ram_data_to_s when sram_we_s = '1' else
						(others => 'Z');

	SRAM_ADDR	<= sram_addr_s;
	sram_data_from_s <= SRAM_DQ(7 downto 0);
	SRAM_UB_N 	<= '1';
	SRAM_LB_N 	<= '0';
	SRAM_CE_N 	<= not sram_ce_s;
	SRAM_WE_N 	<= not sram_we_s;
	SRAM_OE_N 	<= not sram_oe_s;

	-- Perifericos
	lpt_busy_s		<= '0';
	per_disrom_s	<= SW(9);

	-- Debug
	D_display_s	<= D_cpu_pc_s;
--	D_display_s	<= D_Cpu_addr_s;
--	D_display_s	<= "0000000000" & disk2_track_s;

	-- Display de leds: Mapeamos o barramento de endereço para debug
	ld3: 	 work.seg7 port map(D_display_s(15 downto 12), HEX3);
	ld2: 	 work.seg7 port map(D_display_s(11 downto  8), HEX2);
	ld1: 	 work.seg7 port map(D_display_s( 7 downto  4), HEX1);
	ld0: 	 work.seg7 port map(D_display_s( 3 downto  0), HEX0);

--	LEDR(9)	<= not per_iosel_n_s;
--	LEDR(8)	<= not per_devsel_n_s;
	
--	LEDR(0) <= ipl_s;
--	LEDR(1) <= ram_oe_s;
--	LEDR(2) <= ram_we_s;
--	LEDR(3) <= rom_oe_s;
--	LEDR(4) <= rom_we_s;
--	LEDR(5) <= por_reset_s;
--	LEDR(6) <= reset_s;

end architecture;