//============================================================================
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [45:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM (USE_FB=1 in qsf)
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = '0;  

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_SCALER = 0;
assign HDMI_FREEZE = 0;

assign AUDIO_S = 0;
assign AUDIO_L = { 2'b0,spk_s,spk_s,12'b0};
assign AUDIO_R = AUDIO_L;
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[9:8];

assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;

`include "build_id.v" 
localparam CONF_STR = {
	"TK2000;;",
	"-;",
	"O89,Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O2,TV Mode,NTSC,PAL;",
	"O34,Noise,White,Red,Green,Blue;",
	"-;",
	"S0,NIBDSKDO PO ;",
	"H0F2,NIB,IOCTL NIB;",
	"OA,Dis Rom,On,Off;",
	"OB,Color Mode,On,Off;",
	"-;",
	"R0,Reset;",
	"J,Fire 1,Fire 2;",
	"V,v",`BUILD_DATE 
};

wire forced_scandoubler;
wire  [1:0] buttons;
wire [31:0] status;
wire [10:0] ps2_key;
wire [31:0] joy1, joy2;


wire [31:0] sd_lba[2];
reg   [1:0] sd_rd;
reg   [1:0] sd_wr;
wire  [1:0] sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din[2];
wire        sd_buff_wr;
wire  [1:0] img_mounted;
wire        img_readonly;
wire [63:0] img_size;


wire        ioctl_download;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_data;
wire  [7:0] ioctl_index;


hps_io #(.CONF_STR(CONF_STR),.PS2DIV(1000),.VDNUM(2)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask({status[5]}),
	.joystick_0(joy1),
	.joystick_1(joy2),

	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),
	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),

	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_data),
	.ioctl_index(ioctl_index),
	
	
	.ps2_key(ps2_key),
   .ps2_kbd_clk_out    ( ps2_kbd_clk    ),
   .ps2_kbd_data_out   ( ps2_kbd_data   )
);
wire ps2_kbd_clk;
wire ps2_kbd_data;

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys = clock_28_s;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clock_28_s),
	.outclk_1(clock_14_s),
	.locked(pll_locked_s)
);

    
wire reset = RESET | status[0] | buttons[1];

//////////////////////////////////////////////////////////////////


wire HBlank;
wire HSync;
wire VBlank;
wire VSync;
wire [7:0] video;


assign CLK_VIDEO = clk_sys;

/*
reg ce_pix;
always @(posedge clk_sys) begin
        reg [1:0] div;

        div <= div + 1'd1;
	ce_pix <= div == 0;
end
*/
reg ce_pix;
always @(posedge clk_sys) begin
        reg div;

        div <= ~div ;
	ce_pix <= div == 0;
end
assign CE_PIXEL = ce_pix;
/*
assign VGA_DE = ~(HBlank | VBlank);
assign VGA_HS = HSync;
assign VGA_VS = VSync;
assign VGA_G  = (!col || col == 2) ? video : 8'd0;
assign VGA_R  = (!col || col == 1) ? video : 8'd0;
assign VGA_B  = (!col || col == 3) ? video : 8'd0;
*/
assign LED_USER    = 1'b0;


 
    //-- Clocks
    wire clock_28_s;      // Dobro para scandoubler
    wire clock_14_s;      // Master
    wire phi0_s;        // phase 0
    wire phi1_s;        // phase 1
    wire phi2_s;        // phase 2
    wire clock_2M_s;       // Clock Q3
    wire clock_dvi_s;

    wire clk_kbd_s;
    wire [2:0] div_s;


    // Resets
    wire pll_locked_s;
    reg por_reset_s =1'b1 ;  //   : std_logic := '1';
    wire reset_s;

    // ROM
    wire [13:0] rom_addr_s;
    wire [7:0] rom_data_from_s;
//--  signal rom_oe_s         : std_logic;
//--  signal rom_we_s         : std_logic;

    // RAM
    wire [15:0] ram_addr_s;
    wire [7:0] ram_data_to_s;
    wire [7:0] ram_data_from_s;
    wire ram_oe_s;
    wire ram_we_s;
    wire [15:0] ram_addr;
    wire [7:0] ram_data;
    wire ram_we;
        
    // Keyboard
    wire kbd_ctrl_s;
    wire [7:0] kbd_rows_s;
    wire [5:0] kbd_cols_s;
    wire [12:1] FKeys_s;
    wire [7:0] osd_s;

    // Audio
    wire spk_s;

    // K7
    wire cas_o_s;
//--  signal cas_motor_s      : std_logic_vector(1 downto 0);

   // -- Video
    wire [7:0] video_r_s;
    wire [7:0] video_g_s ;
    wire [7:0] video_b_s;
    wire [7:0] video_ro_s;
    wire [7:0] video_go_s;
    wire [7:0] video_bo_s;
    wire video_color_s ;
    wire video_bit_s;
    wire video_hsync_n_s ;
    wire video_vsync_n_s;
    wire video_blank_s  ;
    wire video_hbl_s  ;
    wire video_vbl_s   ;
    wire video_ld194_s   ;
wire per_iosel_n_s;
wire per_devsel_n_s;
wire per_we_s;
wire [7:0] per_addr_s;
wire [7:0] per_data_from_s;
wire [7:0] per_data_to_s;  // Disk II
wire [9:0] image_num_s ;
wire [5:0] track_num_s;
wire [13:0] track_addr_s;
wire disk1_en_s;
wire disk2_en_s;
wire [13:0] track_ram_addr_s;
wire [7:0] track_ram_data_s;
wire track_ram_we_s;  // OSD
reg osd_visible_s = 1'b1;
wire osd_pixel_s;
wire [4:0] osd_green_s;  // OSD byte signal
wire btn_up_s ;
wire btn_down_s  ;
reg [21:0] timer_osd_s = 1'b1;  // Debug
wire [15:0] D_cpu_pc_s;  //
wire [3:0] color_index;
wire [1:0] scanlines_en_s;
wire [4:0] vga_r_s;
wire [4:0] vga_g_s;
wire [4:0] vga_b_s;
wire [4:0] osd_r_s;
wire [4:0] osd_g_s;
wire [4:0] osd_b_s;
wire [9:0] vga_x_s;
wire [9:0] vga_y_s;
reg [22:0] flash_clk = 1'b0;
wire [31:0] menu_status;
wire mc_ack = 1'b0;
wire odd_line_s;
wire step_sound_s;
reg [5:0] kbd_joy_s;  // Data pump
reg pump_active_s = 1'b0  ;
wire sram_we_s ;

wire [18:0] sram_addr_s ;
wire [7:0] sram_data_s;
wire [18:0] disk_addr_s;
wire [7:0] disk_data_s ;
wire [7:0] pcm_out_s;  // HDMI
wire [9:0] tdms_r_s;
wire [9:0] tdms_g_s;
wire [9:0] tdms_b_s;
wire [3:0] tdms_p_s;
wire [3:0] tdms_n_s;  
// SDISKII
wire [3:0] motor_phase_s ;
wire drive_en_s;
wire rd_pulse_s;





    tk2000 tk2000 (
    .clock_14_i(clock_14_s),
    .reset_i(reset_s),
    .CPU_WAIT(cpu_wait_fdd),
    // RAM
    .ram_addr_o(ram_addr_s),
    .ram_data_to_o(ram_data_to_s),
    .ram_data_from_i(ram_data_from_s),
    .ram_oe_o(ram_oe_s),
    .ram_we_o(ram_we_s),
    // ROM
    .rom_addr_o(rom_addr_s),
    .rom_data_from_i(rom_data_from_s),
    .rom_oe_o(/* open */),
    //rom_oe_s,
    .rom_we_o(/* open */),
    //rom_we_s,
    // Keyboard
    .kbd_rows_o(kbd_rows_s),
    .kbd_cols_i(kbd_joy_s),
    .kbd_ctrl_o(kbd_ctrl_s),
    // Audio
    .spk_o(spk_s),
    // Video
    .video_color_o(video_color_s),
    .video_bit_o(video_bit_s),
    .video_hsync_n_o(/* open */),
    .video_vsync_n_o(/* open */),
    .video_hbl_o(video_hbl_s),
    .video_vbl_o(video_vbl_s),
    .video_ld194_o(video_ld194_s),
    // Cassete
    .cas_i(ear_i),
    .cas_o(cas_o_s),
    .cas_motor_o(/* open */),
    //cas_motor_s,
    // LPT
    .lpt_stb_o(/* open */),
    .lpt_busy_i(1'b0),
    // Periferico
    .phi0_o(phi0_s),
    // fase 0 __|---|___|---
    .phi1_o(phi1_s),
    // fase 1 ---|___|---|___
    .phi2_o(phi2_s),
    // fase 2 ___|---|___|---
    .clock_2m_o(clock_2M_s),
    .read_write_o(per_we_s),
    .irq_n_i(1'b1),
    .nmi_n_i(1'b1),
    .dis_rom_i(~status[10]),
    // 1 enable peripheral
    .io_select_n_o(per_iosel_n_s),
    .dev_select_n_o(per_devsel_n_s),
    .per_addr_o(per_addr_s),
    .per_data_from_i(per_data_from_s),
    .per_data_to_o(per_data_to_s),
    // Debug
    .D_cpu_pc_o(D_cpu_pc_s));
   
 // Keyboard
  keyboard #(
      .clkfreq_g(28000)) 
  kb(
      .clock_i(clock_28_s), //  --clock_28_s,
    .reset_i(por_reset_s),
    .ps2_clk_io(ps2_kbd_clk),
    .ps2_data_io(ps2_kbd_data),
    .rows_i(kbd_rows_s),
    .row_ctrl_i(kbd_ctrl_s),
    .cols_o(kbd_cols_s),
    .FKeys_o(FKeys_s),
    .osd_o(osd_s));

	 
	 

	 
  always @(posedge clock_28_s) begin
    kbd_joy_s <= kbd_cols_s;
/*    if((kbd_rows_s[6] == 1'b1 && joy1_up_i == 1'b0) || (kbd_rows_s[5] == 1'b1 && joy1_down_i == 1'b0) || (kbd_rows_s[4] == 1'b1 && joy1_right_i == 1'b0) || (kbd_rows_s[3] == 1'b1 && joy1_left_i == 1'b0)) begin
      kbd_joy_s <= kbd_joy_s | 6'b000001;
    end
    if((kbd_rows_s[7] == 1'b1 && joy1_p6_i == 1'b0) || (kbd_rows_s[7] == 1'b1 && joy1_p9_i == 1'b0)) begin
      kbd_joy_s <= kbd_joy_s | 6'b010000;
    end
 */
 //generate a slower clock for the keyboard
    div_s <= div_s + 1;
    clk_kbd_s <= div_s[1];
    // 7 mhz
  end
    


  // ROM
  tk2000_rom rom(
      .clock(clock_28_s),
    .address(rom_addr_s),
    .q(rom_data_from_s));



  // VGA
  vga_controller vga(
    .clock_28_i(clock_28_s),
    .video_i(video_bit_s),
    .color_i(video_color_s),
    .hbl_i(video_hbl_s),
    .vbl_i(video_vbl_s),
    .ld194_i(video_ld194_s),
    .color_type_i(~status[11]),
    .vga_hs_n_o(video_hsync_n_s),
    .vga_vs_n_o(video_vsync_n_s),
    .vga_blank_n_o(video_blank_s),
	 .VGA_HBL(HBlank),
	 .VGA_VBL(VBlank),
	 
    .vga_r_o(video_r_s),
    .vga_g_o(video_g_s),
    .vga_b_o(video_b_s),
    .vga_odd_line_o(odd_line_s),
    .color_index(color_index));
	
	assign VGA_HS	= video_hsync_n_s;
	assign VGA_VS	= video_vsync_n_s;
	assign VGA_R	= video_r_s;
	assign VGA_G	= video_g_s;
	assign VGA_B	= video_b_s;	 
  assign VGA_DE = (video_blank_s);

`define diskinram
  
`ifdef diskinram

 disk_ii disk(
    .CLK_14M(clock_14_s),
    .CLK_2M(clock_2M_s),
    .PRE_PHASE_ZERO(phi0_s),
	 
    .IO_SELECT(~per_iosel_n_s),
    .DEVICE_SELECT(~per_devsel_n_s),
	 
    .RESET(reset_s),
    .A(per_addr_s),
    .D_IN(per_data_to_s),
    .D_OUT(per_data_from_s),
	 
	 
    .TRACK(track_num_s),
    .TRACK_ADDR(track_addr_s),
	 
    .D1_ACTIVE(disk1_en_s),
    .D2_ACTIVE(disk2_en_s),
    .ram_write_addr(track_ram_addr_s),
    .ram_di(track_ram_data_s),
    .ram_we(track_ram_we_s),
	 
    .step_sound_o(step_sound_s),
    //------------------------------------------------------------------------------
    .motor_phase_o(motor_phase_s),
    .drive_en_o(drive_en_s),
    .rd_pulse_o(rd_pulse_s));

	 
	 

  image_controller image_ctrl(
      // System Interface -------------------------------------------------------
    .CLK_14M(clock_14_s),
    .reset(reset_s),
    // SRAM Interface ---------------------------------------------------------
    .buffer_addr_i(disk_addr_s),
    .buffer_data_i(disk_data_s),
    // Track buffer Interface -------------------------------------------------
    .ram_write_addr(track_ram_addr_s[12:0]),
    // out
    .ram_di(track_ram_data_s),
    // out
    .ram_we(track_ram_we_s),
    // out
    .track(track_num_s),
    .image(1'b0));


//
//Disk Buffer
//
//Hold entire disk image here - from ioctl
//
//
    
dpram2 #(.addr_width_g(18),.data_width_g(8))
address_table(
	.clock_a(clk_sys),
	.address_a(ioctl_addr[17:0]),
	.data_a(ioctl_data), 
	.wren_a(ioctl_wr ),
	
	.clock_b(clock_14_s),
	.address_b(disk_addr_s[17:0]),
	.q_b(disk_data_s)
);

    
`else



 disk_ii disk(
    .CLK_14M(clock_14_s),
    .CLK_2M(clock_2M_s),
    .PRE_PHASE_ZERO(phi0_s),
	 
    .IO_SELECT(~per_iosel_n_s),
    .DEVICE_SELECT(~per_devsel_n_s),
	 
    .RESET(reset_s),
    .A(per_addr_s),
    .D_IN(per_data_to_s),
    .D_OUT(per_data_from_s),
	 
	 
    .TRACK(track),
    .TRACK_ADDR(track_addr_s),
	 
    .D1_ACTIVE(disk1_en_s),
    .D2_ACTIVE(disk2_en_s),
    .ram_write_addr({track_sec, sd_buff_addr}),
    .ram_di(sd_buff_dout),
    .ram_we(sd_buff_wr & sd_ack[0]),
	 
    .step_sound_o(step_sound_s),
    //------------------------------------------------------------------------------
    .motor_phase_o(motor_phase_s),
    .drive_en_o(drive_en_s),
    .rd_pulse_o(rd_pulse_s));
	 
assign sd_buff_din[1] = 0;
assign      sd_lba[1] = 0;
assign      sd_rd[1] = 0;
assign      sd_wr[1] = 0;
	 
assign sd_buff_din[0] = 0;
assign      sd_lba[0] = lba_fdd;
wire  [5:0] track;
reg   [3:0] track_sec;
reg         cpu_wait_fdd = 0;
reg  [31:0] lba_fdd;

always @(posedge clk_sys) begin
	reg       state = 0;
	reg [5:0] cur_track;
	reg       fdd_mounted = 0;
	reg       old_ack = 0;
	
	old_ack <= sd_ack[0];
	fdd_mounted <= fdd_mounted | img_mounted[0];
	sd_wr[0] <= 0;

	if(reset_s) begin
		state <= 0;
		cpu_wait_fdd <= 0;
		sd_rd[0] <= 0;
	end
	else if(!state) begin
		if((cur_track != track) || (fdd_mounted && ~img_mounted[0])) begin
			cur_track <= track;
			fdd_mounted <= 0;
			if(img_size) begin
				track_sec <= 0;
				lba_fdd <= 13 * track;
				state <= 1;
				sd_rd[0] <= 1;
				cpu_wait_fdd <= 1;
			end
		end
	end
	else begin
		if(~old_ack & sd_ack[0]) begin
			if(track_sec >= 12) sd_rd[0] <= 0;
			lba_fdd <= lba_fdd + 1'd1;
		end else if(old_ack & ~sd_ack[0]) begin
			track_sec <= track_sec + 1'd1;
			if(~sd_rd[0]) state <= 0;
			cpu_wait_fdd <= 0;
		end
	end
end

`endif
 	 
assign joy1_right_i  = joy1[0];
assign joy1_left_i   = joy1[1];
assign joy1_down_i   = joy1[2];
assign joy1_up_i     = joy1[3];
assign joy1_p6_i= joy1[4];
assign joy1_p9_i= joy1[5];


  assign joy2_right_i = motor_phase_s[3];
  assign joy2_left_i = motor_phase_s[2];
  assign joy2_down_i = motor_phase_s[1];
  assign joy2_up_i = motor_phase_s[0];
  assign joy2_p6_i = drive_en_s;

 //--    joy2_p9_i    <= rd_pulse_s;

      
  assign scanlines_en_s = 2'b00;
    


 // Glue Logic
  // In the Apple ][, this was a 555 timer
  always @(posedge clock_14_s) begin
    reset_s <= por_reset_s |RESET |  buttons[1] | status[0];
	 /*
    if((btn_n_i[4] == 1'b0 && btn_n_i[3] == 1'b0) || menu_status[0] == 1'b1 || pump_active_s == 1'b1) begin
      por_reset_s <= 1'b1;
      flash_clk <= {23{1'b0}};
    end
    else*/ 
	 
	 if(buttons[1] || status[0] || RESET || pump_active_s == 1'b1) begin
      por_reset_s <= 1'b1;
      flash_clk <= {23{1'b0}};
    end
    else
	 begin
      if(flash_clk[22] == 1'b1) begin
        por_reset_s <= 1'b0;
      end
      flash_clk <= flash_clk + 1;
    end
  end



  dpram2  #( .addr_width_g(16),.data_width_g(8))
  ram (
	.address_a(ram_addr),
	.clock_a(clock_28_s),
	.clock_b(~clock_2M_s),
	.data_a(ram_data),
	.q_a(ram_data_from_s),
	.wren_a(ram_we)
  );
  


  assign ram_we = por_reset_s == 1'b0 ? ram_we_s : 1'b1;
  assign ram_data = por_reset_s == 1'b0 ? ram_data_to_s : 8'b00000000;
  assign ram_addr = por_reset_s == 1'b0 ? ram_addr_s: 16'h3F4;//std_logic_vector(to_unsigned(1012,ram_addr'length)); -- $3F4


    
  assign mic_o = cas_o_s;

  

endmodule
