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
--
--

-- altera message_off 10540 10541

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;

entity tk2000_de2 is
	port (
		-- Clocks
		CLOCK_27       : in    std_logic;
		CLOCK_50       : in    std_logic;
		EXT_CLOCK      : in    std_logic;
		-- Switches
		SW             : in    std_logic_vector(17 downto 0);
		-- Buttons
		KEY            : in    std_logic_vector(3 downto 0);
		-- 7 segment displays
		HEX0           : out   std_logic_vector(6 downto 0)		:= (others => '1');
		HEX1           : out   std_logic_vector(6 downto 0)		:= (others => '1');
		HEX2           : out   std_logic_vector(6 downto 0)		:= (others => '1');
		HEX3           : out   std_logic_vector(6 downto 0)		:= (others => '1');
		HEX4           : out   std_logic_vector(6 downto 0)		:= (others => '1');
		HEX5           : out   std_logic_vector(6 downto 0)		:= (others => '1');
		HEX6           : out   std_logic_vector(6 downto 0)		:= (others => '1');
		HEX7           : out   std_logic_vector(6 downto 0)		:= (others => '1');
		-- Red LEDs
		LEDR           : out   std_logic_vector(17 downto 0)		:= (others => '0');
		-- Green LEDs
		LEDG           : out   std_logic_vector(8 downto 0)		:= (others => '0');
		-- Serial
		UART_RXD       : in    std_logic;
		UART_TXD       : out   std_logic									:= '1';
		IRDA_RXD       : in    std_logic;
		IRDA_TXD       : out   std_logic									:= '0';
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
		-- SRAM
		SRAM_ADDR      : out   std_logic_vector(17 downto 0)		:= (others => '0');
		SRAM_DQ        : inout std_logic_vector(15 downto 0)		:= (others => 'Z');
		SRAM_CE_N      : out   std_logic									:= '1';
		SRAM_OE_N      : out   std_logic									:= '1';
		SRAM_WE_N      : out   std_logic									:= '1';
		SRAM_UB_N      : out   std_logic									:= '1';
		SRAM_LB_N      : out   std_logic									:= '1';
		--	ISP1362 Interface
		OTG_ADDR       : out   std_logic_vector(1 downto 0)		:= (others => '0');	--	ISP1362 Address 2 Bits
		OTG_DATA       : inout std_logic_vector(15 downto 0)		:= (others => 'Z');	--	ISP1362 Data bus 16 Bits
		OTG_CS_N       : out   std_logic									:= '1';					--	ISP1362 Chip Select
		OTG_RD_N       : out   std_logic									:= '1';					--	ISP1362 Write
		OTG_WR_N       : out   std_logic									:= '1';					--	ISP1362 Read
		OTG_RST_N      : out   std_logic									:= '1';					--	ISP1362 Reset
		OTG_FSPEED     : out   std_logic									:= 'Z';					--	USB Full Speed,	0 = Enable, Z = Disable
		OTG_LSPEED     : out   std_logic									:= 'Z';					--	USB Low Speed, 	0 = Enable, Z = Disable
		OTG_INT0       : in    std_logic;															--	ISP1362 Interrupt 0
		OTG_INT1       : in    std_logic;															--	ISP1362 Interrupt 1
		OTG_DREQ0      : in    std_logic;															--	ISP1362 DMA Request 0
		OTG_DREQ1      : in    std_logic;															--	ISP1362 DMA Request 1
		OTG_DACK0_N    : out   std_logic									:= '1';					--	ISP1362 DMA Acknowledge 0
		OTG_DACK1_N    : out   std_logic									:= '1';					--	ISP1362 DMA Acknowledge 1
		--	LCD Module 16X2
		LCD_ON         : out   std_logic									:= '0';					--	LCD Power ON/OFF, 0 = Off, 1 = On
		LCD_BLON       : out   std_logic									:= '0';					--	LCD Back Light ON/OFF, 0 = Off, 1 = On
		LCD_DATA       : inout std_logic_vector(7 downto 0)		:= (others => '0');	--	LCD Data bus 8 bits
		LCD_RW         : out   std_logic									:= '1';					--	LCD Read/Write Select, 0 = Write, 1 = Read
		LCD_EN         : out   std_logic									:= '1';					--	LCD Enable
		LCD_RS         : out   std_logic									:= '1';					--	LCD Command/Data Select, 0 = Command, 1 = Data
		--	SD_Card Interface
		SD_DAT         : inout std_logic									:= 'Z';					--	SD Card Data (SPI MISO)
		SD_DAT3        : inout std_logic									:= 'Z';					--	SD Card Data 3 (SPI /CS)
		SD_CMD         : inout std_logic									:= 'Z';					--	SD Card Command Signal (SPI MOSI)
		SD_CLK         : out   std_logic									:= '1';					--	SD Card Clock (SPI SCLK)
		-- I2C
		I2C_SCLK       : inout std_logic									:= 'Z';
		I2C_SDAT       : inout std_logic									:= 'Z';
		-- PS/2 Keyboard
		PS2_CLK        : inout std_logic									:= 'Z';
		PS2_DAT        : inout std_logic									:= 'Z';
		-- VGA
		VGA_R          : out   std_logic_vector(9 downto 0)		:= (others => '0');
		VGA_G          : out   std_logic_vector(9 downto 0)		:= (others => '0');
		VGA_B          : out   std_logic_vector(9 downto 0)		:= (others => '0');
		VGA_HS         : out   std_logic									:= '1';
		VGA_VS         : out   std_logic									:= '1';
		VGA_BLANK		: out   std_logic									:= '1';
		VGA_SYNC			: out   std_logic									:= '0';
		VGA_CLK		   : out   std_logic									:= '0';
		-- Ethernet Interface
		ENET_CLK       : out   std_logic									:= '0';					--	DM9000A Clock 25 MHz
		ENET_DATA      : inout std_logic_vector(15 downto 0)		:= (others => 'Z');	--	DM9000A DATA bus 16Bits
		ENET_CMD       : out   std_logic									:= '0';					--	DM9000A Command/Data Select, 0 = Command, 1 = Data
		ENET_CS_N      : out   std_logic									:= '1';					--	DM9000A Chip Select
		ENET_WR_N      : out   std_logic									:= '1';					--	DM9000A Write
		ENET_RD_N      : out   std_logic									:= '1';					--	DM9000A Read
		ENET_RST_N     : out   std_logic									:= '1';					--	DM9000A Reset
		ENET_INT       : in    std_logic;															--	DM9000A Interrupt
		-- Audio
		AUD_XCK        : out   std_logic									:= '0';
		AUD_BCLK       : out   std_logic									:= '0';
		AUD_ADCLRCK    : out   std_logic									:= '0';
		AUD_ADCDAT     : in    std_logic;
		AUD_DACLRCK    : out   std_logic									:= '0';
		AUD_DACDAT     : out   std_logic									:= '0';
		-- TV Decoder
		TD_DATA        : in    std_logic_vector(7 downto 0);									--	TV Decoder Data bus 8 bits
		TD_HS          : in    std_logic;															--	TV Decoder H_SYNC
		TD_VS          : in    std_logic;															--	TV Decoder V_SYNC
		TD_RESET       : out   std_logic									:= '1';					--	TV Decoder Reset
		-- GPIO
		GPIO_0         : inout std_logic_vector(35 downto 0)		:= (others => 'Z');
		GPIO_1         : inout std_logic_vector(35 downto 0)		:= (others => 'Z')
	);
end entity;

architecture behavior of tk2000_de2 is

	-- Alias
	alias SD_nCS  is SD_DAT3;
	alias SD_MISO is SD_DAT;
	alias SD_MOSI is SD_CMD;
	alias SD_SCLK is SD_CLK;

	-- Clocks
	signal clock_28_s			: std_logic;		-- Dobro para scandoubler
	signal clock_14_s			: std_logic;		-- Master
	signal clock_audio_s		: std_logic;		-- 24MHz para codec
	signal phi0_s				: std_logic;		-- phase 0
	signal phi1_s				: std_logic;		-- phase 1
	signal phi2_s				: std_logic;		-- phase 2
	signal clock_2M_s			: std_logic;		-- Clock Q3

	-- Reset geral
	signal pll_locked_s		: std_logic;
	signal por_reset_s		: std_logic;
	signal reset_s				: std_logic;

	-- ROM
	signal rom_addr_s			: std_logic_vector(13 downto 0);
	signal rom_data_from_s	: std_logic_vector(7 downto 0);

	-- RAM
	signal ram_addr_s			: std_logic_vector(15 downto 0);
	signal ram_data_to_s		: std_logic_vector(7 downto 0);
	signal ram_data_from_s	: std_logic_vector(7 downto 0);
	signal ram_oe_s			: std_logic;
	signal ram_we_s			: std_logic;

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
	signal video_blank_s		: std_logic;

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

	-- Disk II
	signal image_num_s		: unsigned( 9 downto 0)				:= (others => '0');
	signal track_num_s		: unsigned( 5 downto 0);
	signal track_addr_s		: unsigned(13 downto 0);
	signal disk1_en_s			: std_logic;
	signal disk2_en_s			: std_logic;
	signal track_ram_addr_s	: unsigned(13 downto 0);
	signal track_ram_data_s	: unsigned( 7 downto 0);
	signal track_ram_we_s	: std_logic;

	-- OSD
	signal osd_visible_s		: std_logic								:= '1';
	signal osd_pixel_s		: std_logic;
	signal osd_green_s		: std_logic_vector(7 downto 0);								-- OSD byte signal
	signal btn_up_s			: std_logic := '1'; 
	signal btn_down_s			: std_logic := '1'; 
	signal timer_osd_s		: unsigned(21 downto 0)				:= (others => '1');

	-- Debug
	signal D_cpu_pc_s			: std_logic_vector(15 downto 0);
	signal D_display_s		: std_logic_vector(15 downto 0);

begin

	-- PLL
	pll: entity work.pll1
	port map (
		inclk0		=> CLOCK_50,
		c0				=> clock_28_s,
		c1				=> clock_14_s,
		locked		=> pll_locked_s
	);

	pllaudio: entity work.pll2
	port map (
		inclk0		=> CLOCK_27,
		c0				=> clock_audio_s
	);

	-- TK2000
	tk2000_inst: work.tk2000
	port map (
		clock_14_i			=> clock_14_s,
		reset_i				=> reset_s,
		-- RAM
		ram_addr_o			=> ram_addr_s,
		ram_data_to_o		=> ram_data_to_s,
		ram_data_from_i	=> ram_data_from_s,
		ram_oe_o				=> ram_oe_s,
		ram_we_o				=> ram_we_s,
		-- ROM
		rom_addr_o			=> rom_addr_s,
		rom_data_from_i	=> rom_data_from_s,
		rom_oe_o				=> open,
		rom_we_o				=> open,
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
	sound: entity work.Audio_WM8731
	port map (
		reset_i			=> reset_s,
		clock_i			=> clock_audio_s,
		mic_i				=> cas_o_s,
		spk_i				=> spk_s,
		ear_o				=> cas_i_s,

		i2s_xck_o		=>	AUD_XCK,
		i2s_bclk_o		=> AUD_BCLK,
		i2s_adclrck_o	=> AUD_ADCLRCK,
		i2s_adcdat_i	=> AUD_ADCDAT,
		i2s_daclrck_o	=> AUD_DACLRCK,
		i2s_dacdat_o	=> AUD_DACDAT,

		i2c_sda_io		=> I2C_SDAT,
		i2c_scl_io		=> I2C_SCLK
	);

	-- ROM
	rom: entity work.tk2000_rom
	port map (
		clock		=> clock_28_s,
		address	=> rom_addr_s,
		q			=> rom_data_from_s
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
		color_type_i	=> SW(1),
		vga_hs_n_o		=> video_hsync_n_s,
		vga_vs_n_o		=> video_vsync_n_s,
		vga_blank_n_o	=> video_blank_s,
		vga_r_o			=> video_r_s,
		vga_g_o			=> video_g_s,
		vga_b_o			=> video_b_s
	);

	-- OSD overlay for the green channel
	osd_inst: entity work.osd
	generic map (									-- workaround for wrong video size
		C_digits			=> 3,						-- number of hex digits to show
		C_resolution_x	=> 565
	)
	port map (
		clk_pixel		=> clock_28_s,
		vsync				=> not video_vsync_n_s,	-- positive sync
		fetch_next		=> video_blank_s,			-- '1' when video_active
		probe_in			=> "00" & std_logic_vector(image_num_s),
		osd_out			=> osd_pixel_s
	);

	osd_green_s		<= (others => (osd_pixel_s and osd_visible_s));

	VGA_HS		<= video_hsync_n_s;
	VGA_VS		<= video_vsync_n_s;
	VGA_R			<= video_r_s & "00";
	VGA_G			<= (video_g_s or osd_green_s) & "00";
	VGA_B			<= video_b_s & "00";
	VGA_SYNC		<= '0';
	VGA_CLK		<= clock_28_s;

	-- Button Down
	btndw: entity work.debounce
	generic map (
		counter_size_g	=> 16
	)
	port map (
		clk_i				=> clock_14_s,
		button_i			=> KEY(2),
		result_o			=> btn_down_s
	);

	-- Button Up
 	btnup: entity work.debounce
	generic map (
		counter_size_g	=> 16
	)
	port map (
		clk_i				=> clock_14_s,
		button_i			=> KEY(1),
		result_o			=> btn_up_s
	);

	disk : entity work.disk_ii
	port map (
		CLK_14M			=> clock_14_s,
		CLK_2M			=> clock_2M_s,
		PRE_PHASE_ZERO	=> phi0_s,
		IO_SELECT		=> not per_iosel_n_s,
		DEVICE_SELECT	=> not per_devsel_n_s,
		RESET				=> reset_s,
		A					=> unsigned(per_addr_s),
		D_IN				=> unsigned(per_data_to_s),
		std_logic_vector(D_OUT)	=> per_data_from_s,
		TRACK				=> track_num_s,
		TRACK_ADDR		=> track_addr_s,
		D1_ACTIVE		=> disk1_en_s,
		D2_ACTIVE		=> disk2_en_s,
		ram_write_addr	=> track_ram_addr_s,
		ram_di			=> track_ram_data_s,
		ram_we			=> track_ram_we_s
	);

	sdcard_interface : entity work.spi_controller
	port map (
		CLK_14M        => clock_14_s,
		RESET          => reset_s,
		CS_N           => SD_nCS,
		MOSI           => SD_MOSI,
		MISO           => SD_MISO,
		SCLK           => SD_SCLK,
		track          => track_num_s,
		image          => image_num_s,
		ram_write_addr => track_ram_addr_s,
		ram_di         => track_ram_data_s,
		ram_we         => track_ram_we_s
	);

	-- Glue Logic
	por_reset_s	<= '1' when pll_locked_s = '0' or KEY(3) = '0'								else '0';
	reset_s		<= '1' when por_reset_s = '1'  or KEY(0) = '0' or FKeys_s(4) = '1'  	else '0';

	-- RAM
	ram_data_from_s	<= SRAM_DQ(7 downto 0);
	SRAM_UB_N 			<= '1';
	SRAM_LB_N 			<= '0';

	process (por_reset_s, ram_addr_s, ram_we_s, ram_oe_s, ram_data_to_s)
	begin
		if por_reset_s = '1' then
			SRAM_ADDR	<= "00" & X"03F4";
			SRAM_CE_N	<= '0';
			SRAM_WE_N	<= '0';
			SRAM_OE_N	<= '0';
			SRAM_DQ		<= "ZZZZZZZZ" & X"00";
		else
			SRAM_ADDR	<= "00" & ram_addr_s;
			SRAM_CE_N	<= '0';
			SRAM_WE_N	<= not ram_we_s;
			SRAM_OE_N	<= not ram_oe_s;
			if ram_we_s = '1' then
				SRAM_DQ	<= "ZZZZZZZZ" & ram_data_to_s;
			else
				SRAM_DQ	<= (others => 'Z');
			end if;
		end if;
	end process;

	-- Perifericos
	lpt_busy_s		<= '0';
	per_disrom_s	<= SW(9);

	-- Image and OSD
	-- dectect falling edge of the buttons
	process (por_reset_s, clock_14_s)
		variable btn_up_de_v		: std_logic_vector(1 downto 0) := "11";
		variable btn_down_de_v	: std_logic_vector(1 downto 0) := "11";
	begin
		if por_reset_s = '1' then
			image_num_s	<= (others => '0');
		elsif rising_edge(clock_14_s) then  
			if    btn_up_de_v = "10" and btn_down_s = '1' then
				image_num_s <= image_num_s + 1;
			elsif btn_down_de_v = "10" and btn_up_s = '1' then  
				image_num_s <= image_num_s - 1;
			end if;
			btn_up_de_v 	:= btn_up_de_v(0)   & btn_up_s;
			btn_down_de_v	:= btn_down_de_v(0) & btn_down_s;
		end if;
	end process;

	-- OSD timer
	process (clock_2M_s, btn_up_s, btn_down_s)
	begin
		if rising_edge(clock_2M_s) then
			if btn_up_s = '0' or btn_down_s = '0' then
				timer_osd_s		<= (others => '1');
				osd_visible_s	<= '1';
			elsif timer_osd_s > 0 then
				timer_osd_s		<= timer_osd_s - 1;
				osd_visible_s	<= '1';
			else
				osd_visible_s	<= '0';
			end if;
		end if;
	end process;

	-- Debug
	D_display_s	<= D_cpu_pc_s;

	-- Display de leds
	ld7: entity work.seg7 port map(X"0", HEX7);
	ld6: entity work.seg7 port map(X"0", HEX6);
	ld5: entity work.seg7 port map(X"0", HEX5);
	ld4: entity work.seg7 port map(X"0", HEX4);
	ld3: entity work.seg7 port map(D_display_s(15 downto 12), HEX3);
	ld2: entity work.seg7 port map(D_display_s(11 downto  8), HEX2);
	ld1: entity work.seg7 port map(D_display_s( 7 downto  4), HEX1);
	ld0: entity work.seg7 port map(D_display_s( 3 downto  0), HEX0);

end architecture;
