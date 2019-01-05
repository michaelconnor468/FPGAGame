// --------------------------------------------------------------------
// Copyright (c) 2005 by Terasic Technologies Inc. 
// --------------------------------------------------------------------
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development 
//   Kits made by Terasic.  Other use of this code, including the selling 
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use 
//   or functionality of this code.
//
// --------------------------------------------------------------------
//           
//                     Terasic Technologies Inc
//                     356 Fu-Shin E. Rd Sec. 1. JhuBei City,
//                     HsinChu County, Taiwan
//                     302
//
//                     web: http://www.terasic.com/
//                     email: support@terasic.com
//
// --------------------------------------------------------------------
//
// Major Functions:	DE2 TOP LEVEL
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date :| Changes Made:
//   V1.0 :| Johnny Chen       :| 05/08/19  :|      Initial Revision
// --------------------------------------------------------------------

module DE2_TOP
	(
		////////////////////	Clock Input	 	////////////////////	 
		CLOCK_27,						//	On Board 27 MHz
		CLOCK_50,						//	On Board 50 MHz
		EXT_CLOCK,						//	External Clock
		////////////////////	Push Button		////////////////////
		KEY,							//	Push Button[3:0]
		////////////////////	DPDT Switch		////////////////////
		SW,								//	DPDT Switch[17:0]
		////////////////////	7-SEG Dispaly	////////////////////
		HEX0,							//	Seven Segment Digital 0
		HEX1,							//	Seven Segment Digital 1
		HEX2,							//	Seven Segment Digital 2
		HEX3,							//	Seven Segment Digital 3
		HEX4,							//	Seven Segment Digital 4
		HEX5,							//	Seven Segment Digital 5
		HEX6,							//	Seven Segment Digital 6
		HEX7,							//	Seven Segment Digital 7
		////////////////////////	LED		////////////////////////
		LEDG,							//	LED Green[8:0]
		LEDR,							//	LED Red[17:0]
		////////////////////////	UART	////////////////////////
		UART_TXD,						//	UART Transmitter
		UART_RXD,						//	UART Receiver
		////////////////////////	IRDA	////////////////////////
		IRDA_TXD,						//	IRDA Transmitter
		IRDA_RXD,						//	IRDA Receiver
		/////////////////////	SDRAM Interface		////////////////
		DRAM_DQ,						//	SDRAM Data bus 16 Bits
		DRAM_ADDR,						//	SDRAM Address bus 12 Bits
		DRAM_LDQM,						//	SDRAM Low-byte Data Mask 
		DRAM_UDQM,						//	SDRAM High-byte Data Mask
		DRAM_WE_N,						//	SDRAM Write Enable
		DRAM_CAS_N,						//	SDRAM Column Address Strobe
		DRAM_RAS_N,						//	SDRAM Row Address Strobe
		DRAM_CS_N,						//	SDRAM Chip Select
		DRAM_BA_0,						//	SDRAM Bank Address 0
		DRAM_BA_1,						//	SDRAM Bank Address 1
		DRAM_CLK,						//	SDRAM Clock
		DRAM_CKE,						//	SDRAM Clock Enable
		////////////////////	Flash Interface		////////////////
		FL_DQ,							//	FLASH Data bus 8 Bits
		FL_ADDR,						//	FLASH Address bus 20 Bits
		FL_WE_N,						//	FLASH Write Enable
		FL_RST_N,						//	FLASH Reset
		FL_OE_N,						//	FLASH Output Enable
		FL_CE_N,						//	FLASH Chip Enable
		////////////////////	SRAM Interface		////////////////
		SRAM_DQ,						//	SRAM Data bus 16 Bits
		SRAM_ADDR,						//	SRAM Address bus 18 Bits
		SRAM_UB_N,						//	SRAM High-byte Data Mask
		SRAM_LB_N,						//	SRAM Low-byte Data Mask  
		SRAM_WE_N,						//	SRAM Write Enable
		SRAM_CE_N,						//	SRAM Chip Enable
		SRAM_OE_N,						//	SRAM Output Enable
		////////////////////	ISP1362 Interface	////////////////
		OTG_DATA,						//	ISP1362 Data bus 16 Bits
		OTG_ADDR,						//	ISP1362 Address 2 Bits
		OTG_CS_N,						//	ISP1362 Chip Select
		OTG_RD_N,						//	ISP1362 Write
		OTG_WR_N,						//	ISP1362 Read
		OTG_RST_N,						//	ISP1362 Reset
		OTG_FSPEED,						//	USB Full Speed,	0 = Enable, Z = Disable
		OTG_LSPEED,						//	USB Low Speed, 	0 = Enable, Z = Disable
		OTG_INT0,						//	ISP1362 Interrupt 0
		OTG_INT1,						//	ISP1362 Interrupt 1
		OTG_DREQ0,						//	ISP1362 DMA Request 0
		OTG_DREQ1,						//	ISP1362 DMA Request 1
		OTG_DACK0_N,					//	ISP1362 DMA Acknowledge 0
		OTG_DACK1_N,					//	ISP1362 DMA Acknowledge 1
		////////////////////	LCD Module 16X2		////////////////
		LCD_ON,							//	LCD Power ON/OFF
		LCD_BLON,						//	LCD Back Light ON/OFF
		LCD_RW,							//	LCD Read/Write Select, 0 = Write, 1 = Read
		LCD_EN,							//	LCD Enable
		LCD_RS,							//	LCD Command/Data Select, 0 = Command, 1 = Data
		LCD_DATA,						//	LCD Data bus 8 bits
		////////////////////	SD_Card Interface	////////////////
		SD_DAT,							//	SD Card Data
		SD_DAT3,						//	SD Card Data 3
		SD_CMD,							//	SD Card Command Signal
		SD_CLK,							//	SD Card Clock
		////////////////////	USB JTAG link	////////////////////
		TDI,  							//	CPLD -> FPGA (Data in)
		TCK,  							//	CPLD -> FPGA (Clock)
		TCS,  							//	CPLD -> FPGA (CS)
	    TDO,  							//	FPGA -> CPLD (Data out)
		////////////////////	I2C		////////////////////////////
		I2C_SDAT,						//	I2C Data
		I2C_SCLK,						//	I2C Clock
		////////////////////	PS2		////////////////////////////
		PS2_DAT,						//	PS2 Data
		PS2_CLK,						//	PS2 Clock
		////////////////////	VGA		////////////////////////////
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK,						//	VGA BLANK
		VGA_SYNC,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,  						//	VGA Blue[9:0]
		////////////	Ethernet Interface	////////////////////////
		ENET_DATA,						//	DM9000A DATA bus 16Bits
		ENET_CMD,						//	DM9000A Command/Data Select, 0 = Command, 1 = Data
		ENET_CS_N,						//	DM9000A Chip Select
		ENET_WR_N,						//	DM9000A Write
		ENET_RD_N,						//	DM9000A Read
		ENET_RST_N,						//	DM9000A Reset
		ENET_INT,						//	DM9000A Interrupt
		ENET_CLK,						//	DM9000A Clock 25 MHz
		////////////////	Audio CODEC		////////////////////////
		AUD_ADCLRCK,					//	Audio CODEC ADC LR Clock
		AUD_ADCDAT,						//	Audio CODEC ADC Data
		AUD_DACLRCK,					//	Audio CODEC DAC LR Clock
		AUD_DACDAT,						//	Audio CODEC DAC Data
		AUD_BCLK,						//	Audio CODEC Bit-Stream Clock
		AUD_XCK,						//	Audio CODEC Chip Clock
		////////////////	TV Decoder		////////////////////////
		TD_DATA,    					//	TV Decoder Data bus 8 bits
		TD_HS,							//	TV Decoder H_SYNC
		TD_VS,							//	TV Decoder V_SYNC
		TD_RESET,						//	TV Decoder Reset
		////////////////////	GPIO	////////////////////////////
		GPIO_0,							//	GPIO Connection 0
		GPIO_1							//	GPIO Connection 1
	);

////////////////////////	Clock Input	 	////////////////////////
input			CLOCK_27;					//	27 MHz
input			CLOCK_50;					//	50 MHz
input			EXT_CLOCK;				//	External Clock
////////////////////////	Push Button		////////////////////////
input	[3:0]	KEY;					//	Button[3:0]
////////////////////////	DPDT Switch		////////////////////////
input	[17:0]	SW;				//	DPDT Switch[17:0]
////////////////////////	7-SEG Display	////////////////////////
output	[6:0]	HEX0;					//	Seven Segment Digital 0
output	[6:0]	HEX1;					//	Seven Segment Digital 1
output	[6:0]	HEX2;					//	Seven Segment Digital 2
output	[6:0]	HEX3;					//	Seven Segment Digital 3
output	[6:0]	HEX4;					//	Seven Segment Digital 4
output	[6:0]	HEX5;					//	Seven Segment Digital 5
output	[6:0]	HEX6;					//	Seven Segment Digital 6
output	[6:0]	HEX7;					//	Seven Segment Digital 7
////////////////////////////	LED		////////////////////////////
output	[8:0]	LEDG;				//	LED Green[8:0]
output	[17:0]	LEDR;				//	LED Red[17:0]
////////////////////////////	UART	////////////////////////////
output			UART_TXD;				//	UART Transmitter
input			UART_RXD;				//	UART Rceiver
////////////////////////////	IRDA	////////////////////////////
output			IRDA_TXD;				//	IRDA Transmitter
input			IRDA_RXD;				//	IRDA Rceiver
///////////////////////		SDRAM Interface	////////////////////////
inout	[15:0]	DRAM_DQ;				//	SDRAM Data bus 16 Bits
output	[11:0]	DRAM_ADDR;				//	SDRAM Address bus 12 Bits
output			DRAM_LDQM;				//	SDRAM Low-byte Data Mask 
output			DRAM_UDQM;				//	SDRAM High-byte Data Mask
output			DRAM_WE_N;				//	SDRAM Write Enable
output			DRAM_CAS_N;				//	SDRAM Column Address Strobe
output			DRAM_RAS_N;				//	SDRAM Row Address Strobe
output			DRAM_CS_N;				//	SDRAM Chip Select
output			DRAM_BA_0;				//	SDRAM Bank Address 0
output			DRAM_BA_1;				//	SDRAM Bank Address 0
output			DRAM_CLK;				//	SDRAM Clock
output			DRAM_CKE;				//	SDRAM Clock Enable
////////////////////////	Flash Interface	////////////////////////
inout	[7:0]	FL_DQ;					//	FLASH Data bus 8 Bits
output	[19:0]	FL_ADDR;				//	FLASH Address bus 20 Bits
output			FL_WE_N;				//	FLASH Write Enable
output			FL_RST_N;				//	FLASH Reset
output			FL_OE_N;				//	FLASH Output Enable
output			FL_CE_N;				//	FLASH Chip Enable
////////////////////////	SRAM Interface	////////////////////////
inout	[15:0]	SRAM_DQ;				//	SRAM Data bus 16 Bits
output	[17:0]	SRAM_ADDR;				//	SRAM Adress bus 18 Bits
output			SRAM_UB_N;				//	SRAM Low-byte Data Mask 
output			SRAM_LB_N;				//	SRAM High-byte Data Mask 
output			SRAM_WE_N;				//	SRAM Write Enable
output			SRAM_CE_N;				//	SRAM Chip Enable
output			SRAM_OE_N;				//	SRAM Output Enable
////////////////////	ISP1362 Interface	////////////////////////
inout	[15:0]	OTG_DATA;				//	ISP1362 Data bus 16 Bits
output	[1:0]	OTG_ADDR;				//	ISP1362 Address 2 Bits
output			OTG_CS_N;				//	ISP1362 Chip Select
output			OTG_RD_N;				//	ISP1362 Write
output			OTG_WR_N;				//	ISP1362 Read
output			OTG_RST_N;				//	ISP1362 Reset
output			OTG_FSPEED;				//	USB Full Speed,	0 = Enable, Z = Disable
output			OTG_LSPEED;				//	USB Low Speed, 	0 = Enable, Z = Disable
output			OTG_INT0;				//	ISP1362 Interrupt 0
output			OTG_INT1;				//	ISP1362 Interrupt 1
output			OTG_DREQ0;				//	ISP1362 DMA Request 0
output			OTG_DREQ1;				//	ISP1362 DMA Request 1
output			OTG_DACK0_N;			//	ISP1362 DMA Acknowledge 0
output			OTG_DACK1_N;			//	ISP1362 DMA Acknowledge 1
////////////////////	LCD Module 16X2	////////////////////////////
inout	[7:0]	LCD_DATA;				//	LCD Data bus 8 bits
output			LCD_ON;					//	LCD Power ON/OFF
output			LCD_BLON;				//	LCD Back Light ON/OFF
output			LCD_RW;					//	LCD Read/Write Select, 0 = Write, 1 = Read
output			LCD_EN;					//	LCD Enable
output			LCD_RS;					//	LCD Command/Data Select, 0 = Command, 1 = Data
////////////////////	SD Card Interface	////////////////////////
inout			SD_DAT;					//	SD Card Data
inout			SD_DAT3;				//	SD Card Data 3
inout			SD_CMD;					//	SD Card Command Signal
output			SD_CLK;					//	SD Card Clock
////////////////////////	I2C		////////////////////////////////
inout			I2C_SDAT;				//	I2C Data
output			I2C_SCLK;				//	I2C Clock
////////////////////////	PS2		////////////////////////////////
input		 	PS2_DAT;				//	PS2 Data
input			PS2_CLK;				//	PS2 Clock
////////////////////	USB JTAG link	////////////////////////////
input  			TDI;					// CPLD -> FPGA (data in)
input  			TCK;					// CPLD -> FPGA (clk)
input  			TCS;					// CPLD -> FPGA (CS)
output 			TDO;					// FPGA -> CPLD (data out)
////////////////////////	VGA			////////////////////////////
output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK;				//	VGA BLANK
output			VGA_SYNC;				//	VGA SYNC
output	[9:0]	VGA_R;   				//	VGA Red[9:0]
output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
////////////////	Ethernet Interface	////////////////////////////
inout	[15:0]	ENET_DATA;				//	DM9000A DATA bus 16Bits
output			ENET_CMD;				//	DM9000A Command/Data Select, 0 = Command, 1 = Data
output			ENET_CS_N;				//	DM9000A Chip Select
output			ENET_WR_N;				//	DM9000A Write
output			ENET_RD_N;				//	DM9000A Read
output			ENET_RST_N;				//	DM9000A Reset
input			ENET_INT;				//	DM9000A Interrupt
output			ENET_CLK;				//	DM9000A Clock 25 MHz
////////////////////	Audio CODEC		////////////////////////////
inout			AUD_ADCLRCK;			//	Audio CODEC ADC LR Clock
input			AUD_ADCDAT;				//	Audio CODEC ADC Data
inout			AUD_DACLRCK;			//	Audio CODEC DAC LR Clock
output			AUD_DACDAT;				//	Audio CODEC DAC Data
inout			AUD_BCLK;				//	Audio CODEC Bit-Stream Clock
output			AUD_XCK;				//	Audio CODEC Chip Clock
////////////////////	TV Devoder		////////////////////////////
input	[7:0]	TD_DATA;    			//	TV Decoder Data bus 8 bits
input			TD_HS;					//	TV Decoder H_SYNC
input			TD_VS;					//	TV Decoder V_SYNC
output			TD_RESET;				//	TV Decoder Reset
////////////////////////	GPIO	////////////////////////////////
inout	[35:0]	GPIO_0;					//	GPIO Connection 0
inout	[35:0]	GPIO_1;					//	GPIO Connection 1

//	Turn on all display
assign	HEX5		=	7'h00;
assign	HEX6		=	7'h00;
assign	HEX7		=	7'h00;
assign	LEDG	=	9'h1FF;
assign	LCD_ON		=	1'b1;
assign	LCD_BLON	=	1'b1;

//	All inout port turn to tri-state
assign	DRAM_DQ		=	16'hzzzz;
assign	FL_DQ		=	8'hzz;
assign	SRAM_DQ		=	16'hzzzz;
assign	OTG_DATA	=	16'hzzzz;
assign	LCD_DATA	=	8'hzz;
assign	SD_DAT		=	1'bz;
assign	I2C_SDAT	=	1'bz;
assign	ENET_DATA	=	16'hzzzz;
assign	AUD_ADCLRCK	=	1'bz;
assign	AUD_DACLRCK	=	1'bz;
assign	AUD_BCLK	=	1'bz;
assign	GPIO_0		=	36'hzzzzzzzzz;
assign	GPIO_1		=	36'hzzzzzzzzz;
	
	wire resetn;
	assign resetn = KEY[0];
	
	// Create the color, x, y and writeEn wires that are inputs to the controller.

	wire [2:0] color;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;
	wire[2:0] func;
	wire select_color;
	wire enable;
	wire update;
	wire[7:0] x_in;
	wire[6:0] y_in;

	// Temporarily assign inputs to ground. Remove these lines later.

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	VgaAdapter vga_controller(resetn, CLOCK_50, color, x, y, writeEn,
			VGA_R, VGA_G, VGA_B, VGA_HS, VGA_VS, VGA_BLANK, VGA_SYNC, VGA_CLK);
	defparam vga_controller.COLOR_CHANNEL_DEPTH = 1;
	defparam vga_controller.BACKGROUND_IMAGE = "black.mif";
		
	// Put your code here. Your code should produce signals x,y,color and writeEn
	// for the VGA controller, in addition to any other functionality your design may require.
	
	// An example of such code is:
 	// DisplayExample2(resetn, CLOCK_50, color, x, y, writeEn, KEY[0]);
	wire update_1;
	wire update_2;
	wire update_3;
	wire update_4;
	wire change;
	wire reset2;
	wire select_output;
	wire change2;
	wire[2:0] func2;
	wire[7:0] x1, x2;
	wire[6:0] y1, y2;
	wire[2:0] color1, color2;
	
	counter_1 c130(
		.enable(enable),
		.reset_n(resetn),
		.update(update_1)
	);
	
	counter_2 c110(
		.enable(enable),
		.reset_n(resetn),
		.update(update_2)
	);

	counter_3 c121(
		.enable(enable),
		.reset_n(resetn),
		.update(update_3)
	);

	counter_4 c85(
		.enable(enable),
		.reset_n(resetn),
		.update(update_4)
	);
	 control c0(
	 .clock(CLOCK_50),
	 .enable(enable),
	 .plot_out(writeEn),
	 .func(func),
	 .reset_n(resetn),
	 .x(x),
	 .y(y),
	 .func2(func2),
	 .select_output(select_output),
	 .gameover(gameover),
	 .LEDR(LEDR)
	 );
	 
	frame_counter f1(
		.clock(CLOCK_50),
		.enable(enable),
		.reset_n(resetn)
		);
	delay_counter d2(
		.enable(enable),
		.reset_n(resetn),
		.update(update)
	);
	datapath d1(
	.clock(CLOCK_50),
	.func(func),
	.color_in(bufferarrayvalue),
	.color_out(color1),
	.x(x1),
	.y(y1),
	.reset_n(resetn),
	.change(change),
	.color_in2(saout),
	.change2(change2),
	.color_in3(1'b1)
	);
	
	datapath2 d22(
	.clock(CLOCK_50),
	.func(func2),
	.x_in(topleftx),
	.y_in(toplefty),
	.color_out(color2),
	.x(x2),
	.y(y2),
	.reset_n(1'b1),
	.change(change)
	);

	mux m(
		.x1(x1),
		.y1(y1),
		.x2(x2),
		.y2(y2),
		.color1(color1),
		.color2(color2),
		.x(x),
		.y(y),
		.color(color),
		.select_output(select_output)
	);
	
	control2 c222(
		.clock(enable),
		.jump(~KEY[1]),
		.direction(direction),
		.move_clock(moveclock),
		.reset_n(reset2),
		.collide(iscollide),
		.initialize(initialize)
	);
	
	hex_decoder h11(
		.hex_digit(digit0[3:0]),
		.segments(HEX0)
	);
	hex_decoder h12(
		.hex_digit(digit1[3:0]),
		.segments(HEX1)
	);
	
	control3 c33(
		.clock(CLOCK_50),
		.gameover(gameover),
		.button(~KEY[2]),
		.change(change),
		.change2(change2),
		.reset(reset2)
	);
	
	assign LEDR[4] = gameover;
	assign LEDR[5] = change2;
	wire[159:0] p1out;
	wire[159:0] p2out;
	wire[159:0] p3out;
	wire[159:0] p4out;
	wire[159:0] p5out;
	wire[6:0] score1wir;
	wire[6:0] score2wir;
	wire[6:0] score3wir;
	wire bufferarrayvalue;
	//Five Platforms
	platformregister p1(.drawplatform(1'b1), .clock1(update_1), .regvalue(p1out[159:0]));
	platformregister p2(.drawplatform(1'b1), .clock1(update_2), .regvalue(p2out[159:0]));
	platformregister p3(.drawplatform(1'b1), .clock1(update_3), .regvalue(p3out[159:0]));
	platformregister p4(.drawplatform(1'b1), .clock1(update_1), .regvalue(p4out[159:0]));
	platformregister p5(.drawplatform(1'b1), .clock1(update_4), .regvalue(p5out[159:0]));
	//Buffer Array
	hex_decoder hp0(.hex_digit(digit0[3:0]), .segments(score1wir));
	hex_decoder hp1(.hex_digit(digit1[3:0]), .segments(score2wir));
	hex_decoder hp2(.hex_digit(digit2[3:0]), .segments(score3wir));
	platformarray ar0(.score1(score1wir), .score2(score2wir), .score3(score3wir), .register1(p1out[159:0]), .register2(p2out[159:0]), .register3(p3out[159:0]), .register4(p4out[159:0]), .register5(p5out[159:0]), .clock(CLOCK_50), .xposition(x), .yposition(y), .arrayvalue(bufferarrayvalue));

	
	wire initialize;
	wire iscollide;
	wire moveclock;
	wire direction;
	wire[6:0] verticallocation;
	wire gameover;
	wire[7:0] topleftx;
	wire[6:0] toplefty;
	person pers(.topleftx(topleftx[7:0]), .iscollide(iscollide), .toplefty(toplefty[6:0]), .initialize(initialize), .reg1(p1out[159:0]), .reg2(p2out[159:0]), .reg3(p3out[159:0]), .reg4(p4out[159:0]), .reg5(p5out[159:0]), .moveclock(moveclock), .direction(direction), .verticallocation(verticallocation[6:0]), .gameover(gameover));

	wire scoreinitialize;
	wire secondclock;
	wire[3:0] digit0;
	wire[3:0] digit1;
	wire[3:0] digit2;
	secondtimer st(.enable(CLOCK_50), .reset_n(resetn), .update(secondclock));
	scorecounter(.initialize(initialize), .secondclock(secondclock & ~gameover), .digit0(digit0[3:0]), .digit1(digit1[3:0]), .digit2(digit2[3:0]));
	
	wire[7:0] saxpos;
	wire[6:0] saypos;
	assign saxpos = x1;
	assign saypos = y1;
	wire saout;
	startingarray sa00(.score1(score1wir), .score2(score2wir), .score3(score3wir), .clock(CLOCK_50), .xposition(saxpos), .yposition(saypos), .toggle(change2), .arrayvalue(saout));
	
	wire[7:0] rx_data;
	wire garbagestuff;
	ps2_rx ps2rxx(.clk(CLOCK_50), .reset(rclock), .ps2d(PS2_DAT), .ps2c(PS2_CLK), .rx_en(1'b1), .rx_done_tick(garbagestuff), .rx_data(rx_data[7:0]));
	
	wire enterkey;
	wire wkey;
	keydecoder kdcdr(.data(rx_data[7:0]), .clock(CLOCK_50), .enter(enterkey), .up(wkey));

	wire rclock;
	resetClock(
		.clock(CLOCK_50),
		.rclock(rclock)
	);
	
	wire endarrayval;
	endarray endarr(.score1(score1wir), .score2(score2wir), .score3(score3wir), .xposition(x1), .yposition(y1), .arrayvalue(endarrayval));
endmodule

module control3(clock, gameover, button, change, reset, change, change2);
	input clock, gameover, button;
	output change, reset, change2;
	reg[5:0] current_state;
	reg[5:0] next_state;
	reg change, reset, change2;
	localparam s1 = 6'd0, s2=6'd1, s3=6'd2, s4=6'd3, s5=6'd4, s6=6'd5, s7=6'd6, s8=6'd7, s9=6'd8, s10=6'd9, s11=6'd10, s12=6'd11,
	s13=6'd12, s14=6'd13, s15=6'd14, s16=6'd15, s17=6'd16, s18=6'd17, s19=6'd18, s20=6'd19, s21=6'd20, s22=6'd21, s23=6'd22, s24=6'd23,
	s25=6'd24, s26=6'd25, s27=6'd26, s28=6'd27, s29=6'd28;
	always @(posedge clock)
	begin
			current_state <= next_state;
	end
	always @(*)
	begin
		case(current_state)
			s1: next_state = button ? s2 : s1;
			s2: next_state = s3;
			s3: next_state = s4;
			s4: next_state = gameover ? s5 : s2;
			s5: next_state = button ? s2 : s5;
		endcase
	end
	
	always @(*)
	begin
	change = 0;
	reset = 0;
	change2 = 0;
	case(current_state)
		s1:begin
			change = 1;
			reset = 1;
		end
		s5: begin
			change2 = 1;
			change = 1;
			reset = 1;
		end
	endcase
	end
endmodule
	

module control2(clock, jump, direction, move_clock, reset_n, initialize, collide);
	input clock;
	input jump;
	input collide;
	input reset_n;
	output direction;
	output move_clock;
	output initialize;
	reg[5:0] current_state;
	reg[5:0] next_state;
	reg direction;
	reg move_clock;
	reg[4:0] counter;
	reg initialize;
	localparam s1 = 6'd0, s2=6'd1, s3=6'd2, s4=6'd3, s5=6'd4, s6=6'd5, s7=6'd6, s8=6'd7, s9=6'd8, s10=6'd9, s11=6'd10, s12=6'd11,
	s13=6'd12, s14=6'd13, s15=6'd14, s16=6'd15, s17=6'd16, s18=6'd17, s19=6'd18, s20=6'd19, s21=6'd20, s22=6'd21, s23=6'd22, s24=6'd23,
	s25=6'd24, s26=6'd25, s27=6'd26, s28=6'd27, s29=6'd28;
	always @(*)
	begin
		case(current_state)
			s1: next_state = s2;
			s2: next_state = jump ? s3 : s2;
			s3: next_state = (jump & collide) ? s13 : s4;
			s4: next_state = (jump & collide) ? s13 : s5;
			s5: next_state = (jump & collide) ? s13 : s6;
			s6: next_state = (jump & collide) ? s13 : s7;
			s7: next_state = (jump & collide) ? s13 : s3;
			s13: next_state = s8;
			s8: next_state = s9;
			s9: next_state = s10;
			s10: next_state = s11;
			s11: next_state = (counter > 4) ? s8 : s3;
		endcase
	end
	always @(posedge clock)
	begin
		if(reset_n == 1)
			current_state <= s1;
		else
			current_state <= next_state;
		if(next_state == s13)
			counter <= 5'd32;
		else if (next_state == s10)
			counter <= counter - 1;
	end
	always @(*)
	begin
	direction = 0;
	move_clock = 0;
	initialize = 0;
	case(current_state)
		s1:begin
			initialize = 1;
			move_clock = 0;
		end
		s2: begin
			initialize = 1;
			move_clock = 1;
		end
		s6: begin
			move_clock = 1;
		end
		s8:begin
			direction = 1;
			move_clock = 0;
		end
		s9: begin
			direction = 1;
			move_clock = 1;
		end
	endcase
	end
endmodule
module control(clock, reset_n, plot_out, func, enable, x, y, func2, select_output, gameover, LEDR);
	input clock, enable, reset_n;
	output[2:0] LEDR;
	reg[2:0] LEDR;
	input[7:0] x;
	input[6:0] y;
	input gameover;
	output plot_out;
	output select_output;
	reg select_output;
	output[2:0] func;
	output[2:0] func2;
	reg[2:0] func2;
	reg plot_out;
	reg[2:0] func;
	reg[5:0] current_state, next_state;
	localparam s1 = 6'd0, s2=6'd1, s3=6'd2, s4=6'd3, s5=6'd4, s6=6'd5, s7=6'd6, s8=6'd7, s9=6'd8, s10=6'd9, s11=6'd10, s12=6'd11,
	s13=6'd12, s14=6'd13, s15=6'd14, s16=6'd15, s17=6'd16, s18=6'd17, s19=6'd18, s20=6'd19, s21=6'd20, s22=6'd21, s23=6'd22;
	always @(*)
	begin
		case(current_state)
			s1: next_state = enable ? s2 : s1;
			s2: next_state = x >= 159 ? s3 : s2;
			s3: next_state = s4;
			s4: next_state = y >= 119 ? s5 : s2;
			s5: next_state = s6;
			s6: next_state = gameover ? s1 : s7;
			s7: next_state = s8;
			s8: next_state = s9;
			s9: next_state = s10;
			s10: next_state = s11;
			s11: next_state = s12;
			s12: next_state = s13;
			s13: next_state = s14;
			s14: next_state = s15;
			s15: next_state = s16;
			s16: next_state = s17;
			s17: next_state = s18;
			s18: next_state = s19;
			s19: next_state = s20;
			s20: next_state = s21;
			s21: next_state = s22;
			s22: next_state = s23;
			s23: next_state = s1;
		endcase
	end
	
	always @(posedge clock)
	begin
		if(reset_n == 0)
			current_state <= s1;
		else
			current_state <= next_state;
	end
	
	always @(*)
	begin
	plot_out = 1;
	func = 0;
	func2 = 0;
	select_output = 0;
		case(current_state)
			s1: begin 
				plot_out = 0;
			end
			s2: begin
				func = 1;
			end
			s3:begin
			end
			s4: begin
				func = 2;
			end
			s6: begin
				func = 5;
			end
			s7: begin
				plot_out = 0;
				select_output = 1;
				func2 = 5;
			end
			s8: begin
				select_output = 1;
				func2 = 1;
			end
			s9: begin
				select_output = 1;
				func2 = 1;
			end
			s10: begin
				select_output = 1;
				func2 = 1;
			end
			s11: begin
				select_output = 1;
				func2 = 2;
			end
			s12: begin
				select_output = 1;
				func2 = 3;
			end
			s13: begin
				select_output = 1;
				func2 = 3;
			end
			s14: begin
				select_output = 1;
				func2 = 3;
			end
			s15: begin
				select_output = 1;
				func2 = 2;
			end
			s16: begin
				select_output = 1;
				func2 = 1;
				plot_out = 0;
			end
			s17: begin
				select_output = 1;
				func2 = 1;
			end		
			s18: begin
				select_output = 1;
				func2 = 1;
			end
			s19: begin
				select_output = 1;
				func2 = 2;
				plot_out = 0;
			end
			s20: begin
				select_output = 1;
				func2 = 3;
			end
			s21: begin
				select_output = 1;
				func2 = 3;
				plot_out = 0;
			end
			s22: begin
				select_output = 1;
				func2 = 3;
				plot_out = 0;
			end	
			s23: begin
				select_output = 1;
				func2 = 3;
			end
		endcase
	end
endmodule 

module datapath(clock, reset_n, func, color_in, color_out, x, y, change, change2, color_in2, color_in3);
	input clock, reset_n;
	input[2:0] func;
	input color_in, color_in2, color_in3;
	input change, change2;
	output[2:0] color_out;
	reg[2:0] color_out;
	output[7:0] x;
	output[6:0] y;
	reg[7:0] x;
	reg[6:0] y;
	
	always @(posedge clock, negedge reset_n)
	begin
		if(reset_n == 0)
		begin
			x <= 0;
			y <= 0;
			color_out = 3'b000;
		end
		else
		begin
			case(func)
				0: begin
				end
				1: begin
					x <= x + 1;
				end
				2: begin
					x <= 0;
					y <= y + 1;
				end
				3: begin
					x <= x - 1; 
				end
				4: begin
					y <= y - 1;
				end
				5: begin
					x <= 0;
					y <= 0;
				end
			endcase
			if(change == 0)
			begin
				if(color_in == 0)
					color_out <= 3'b000;
				else
					color_out <= 3'b111;
			end
			if(change == 1)
			begin
				if(color_in2 == 0)
					color_out <= 3'b000;
				else
					color_out <= 3'b111;		
			end
		end
	end
endmodule 

module counter_1(enable, reset_n, update);
	input enable;
	input reset_n;
	output update;
	reg[8:0] num;
	always @(posedge enable, negedge reset_n)
	begin
		if(reset_n == 0)
			num <= 9'd24;
		else if(num == 0)
			num <= 9'd24;
		else
			num <= num - 1;
	end 
	assign update = ~(|num);
endmodule 

module counter_2(enable, reset_n, update);
	input enable;
	input reset_n;
	output update;
	reg[8:0] num;
	always @(posedge enable, negedge reset_n)
	begin
		if(reset_n == 0)
			num <= 9'd29;
		else if(num == 0)
			num <= 9'd29;
		else
			num <= num - 1;
	end 
	assign update = ~(|num);
endmodule 

module counter_3(enable, reset_n, update);
	input enable;
	input reset_n;
	output update;
	reg[8:0] num;
	always @(posedge enable, negedge reset_n)
	begin
		if(reset_n == 0)
			num <= 9'd19;
		else if(num == 0)
			num <= 9'd19;
		else
			num <= num - 1;
	end 
	assign update = ~(|num);
endmodule 


module delay_counter(enable, reset_n, update);
	input enable;
	input reset_n;
	output update;
	reg[4:0] num;
	always @(posedge enable, negedge reset_n)
	begin
		if(reset_n == 0)
			num <= 5'd14;
		else if(num == 0)
			num <= 5'd14;
		else
			num <= num - 1;
	end 
	assign update = ~(|num);
endmodule 


module counter_4(enable, reset_n, update);
	input enable;
	input reset_n;
	output update;
	reg[8:0] num;
	always @(posedge enable, negedge reset_n)
	begin
		if(reset_n == 0)
			num <= 9'd26;
		else if(num == 0)
			num <= 9'd26;
		else
			num <= num - 1;
	end 
	assign update = ~(|num);
endmodule 

module frame_counter(clock, enable, reset_n);
	input clock;
	input reset_n;
	output enable;
	reg[20:0] num;
	always @(posedge clock, negedge reset_n)
	begin
		if(reset_n == 0)
			num <= 833333;
		else if(num == 0)
			num <= 833333;
		else
		begin
			num <= num - 1;
		end
	end
	assign enable = ~(|num);
endmodule 
	
module platformarray(input[6:0] score1, input[6:0] score2, input[6:0] score3, input[159:0] register1, input[159:0] register2, input[159:0] register3, input[159:0] register4, input[159:0] register5, input clock, input[159:0] xposition, input[119:0] yposition, output reg arrayvalue);
	reg [119:0] array [159:0];
	reg [15:0] i;
	reg [15:0] j;
	
	always @(posedge clock)
	begin
		for (i = 0; i < 160; i = i + 1)
		begin
			for (j = 0; j < 120; j = j + 1)
			begin
				array[i][j] <= 0;
			end
		end
		
		for (j = 15; j < 18; j = j + 1)
		begin
			for (i = 0; i < 160; i = i + 1)
			begin
				array[i][j] <= register1[i];
			end
		end
		
		for (j = 35; j < 38; j = j + 1)
		begin
			for (i = 0; i < 160; i = i + 1)
			begin
				array[i][j] <= register2[i];
			end
		end
		
		for (j = 55; j < 58; j = j + 1)
		begin
			for (i = 0; i < 160; i = i + 1)
			begin
				array[i][j] <= register3[i];
			end
		end
		
		for (j = 75; j < 78; j = j + 1)
		begin
			for (i = 0; i < 160; i = i + 1)
			begin
				array[i][j] <= register4[i];
			end
		end
		
		for (j = 95; j < 98; j = j + 1)
		begin
			for (i = 0; i < 160; i = i + 1)
			begin
				array[i][j] <= register5[i];
			end
		end
		
		if(score1[0] == 0)
		begin
			array[155][1] <= 1;
			array[154][1] <= 1;
			array[153][1] <= 1;
		end
		if(score1[1] == 0)
		begin
			array[155][1] <= 1;
			array[155][2] <= 1;
			array[155][3] <= 1;
		end
		if(score1[2] == 0)
		begin
			array[155][3] <= 1;
			array[155][4] <= 1;
			array[155][5] <= 1;
		end
		if(score1[3] == 0)
		begin
			array[155][5] <= 1;
			array[154][5] <= 1;
			array[153][5] <= 1;
		end
		if(score1[4] == 0)
		begin
			array[153][3] <= 1;
			array[153][4] <= 1;
			array[153][5] <= 1;
		end
		if(score1[5] == 0)
		begin
			array[153][1] <= 1;
			array[153][2] <= 1;
			array[153][3] <= 1;
		end
		if(score1[6] == 0)
		begin
			array[155][3] <= 1;
			array[154][3] <= 1;
			array[153][3] <= 1;
		end
		
		
		
		if(score2[0] == 0)
		begin
			array[151][1] <= 1;
			array[150][1] <= 1;
			array[149][1] <= 1;
		end
		if(score2[1] == 0)
		begin
			array[151][1] <= 1;
			array[151][2] <= 1;
			array[151][3] <= 1;
		end
		if(score2[2] == 0)
		begin
			array[151][3] <= 1;
			array[151][4] <= 1;
			array[151][5] <= 1;
		end
		if(score2[3] == 0)
		begin
			array[151][5] <= 1;
			array[150][5] <= 1;
			array[149][5] <= 1;
		end
		if(score2[4] == 0)
		begin
			array[149][3] <= 1;
			array[149][4] <= 1;
			array[149][5] <= 1;
		end
		if(score2[5] == 0)
		begin
			array[149][1] <= 1;
			array[149][2] <= 1;
			array[149][3] <= 1;
		end
		if(score2[6] == 0)
		begin
			array[151][3] <= 1;
			array[150][3] <= 1;
			array[149][3] <= 1;
		end
		
		
		
		if(score3[0] == 0)
		begin
			array[147][1] <= 1;
			array[146][1] <= 1;
			array[145][1] <= 1;
		end
		if(score3[1] == 0)
		begin
			array[147][1] <= 1;
			array[147][2] <= 1;
			array[147][3] <= 1;
		end
		if(score3[2] == 0)
		begin
			array[147][3] <= 1;
			array[147][4] <= 1;
			array[147][5] <= 1;
		end
		if(score3[3] == 0)
		begin
			array[147][5] <= 1;
			array[146][5] <= 1;
			array[145][5] <= 1;
		end
		if(score3[4] == 0)
		begin
			array[145][3] <= 1;
			array[145][4] <= 1;
			array[145][5] <= 1;
		end
		if(score3[5] == 0)
		begin
			array[145][1] <= 1;
			array[145][2] <= 1;
			array[145][3] <= 1;
		end
		if(score3[6] == 0)
		begin
			array[147][3] <= 1;
			array[146][3] <= 1;
			array[145][3] <= 1;
		end
		arrayvalue <= array[xposition][yposition];
	end
endmodule 

module platformregister(input drawplatform, input clock1, output reg[159:0] regvalue);
	reg[164:0] innerreg;
	reg shift;
	reg[4:0] draw;
	always @(posedge clock1)
	begin
		if (drawplatform == 1 && draw[4] == 1 && draw[3] == 1 && shift == 0)
			begin
			innerreg[164:160] <= 15'b111111111111111;
			shift <= 1;
			end 
		else
		begin
			innerreg <= innerreg >> 1'b1;
			shift <= 0;
			draw <= draw + 1;
		end
		regvalue[159:0] <= innerreg[159:0];
	end
endmodule 

module datapath2(clock, reset_n, func, x_in, y_in, color_out, x, y, change);
	input clock, reset_n;
	input change;
	input[2:0] func;
	input[7:0] x_in;
	input[6:0] y_in;
	output[2:0] color_out;
	reg[2:0] color_out;
	output[7:0] x;
	output[6:0] y;
	reg[7:0] x;
	reg[6:0] y;
	
	always @(posedge clock, negedge reset_n)
	begin
		if(reset_n == 0)
		begin
			x <= 10;
			y <= 60;
			color_out = 3'b000;
		end
		else
		begin
			case(func)
				0: begin
				end
				1: begin
					x <= x + 1;
				end
				2: begin
					y <= y + 1;
				end
				3: begin
					x <= x - 1; 
				end
				4: begin
					y <= y - 1;
				end
				5: begin
					x <= x_in;
					y <= y_in;
				end
			endcase
			if(change == 0)
				color_out <= 3'b111;
			else
				color_out <= 3'b000;
		end
	end
endmodule 

module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule


module person(input initialize, input[159:0] reg1, input[159:0] reg2, input[159:0] reg3, input[159:0] reg4, input[159:0] reg5, input moveclock, input direction, output reg iscollide, output reg[6:0] toplefty, output reg[6:0] verticallocation, output reg[7:0] topleftx , output reg gameover);
	// Note that the person will be 3 pixels wide and 5 pixels tall
	// input initialize to set the person to his initial position
	// input moveclock timed however fast you would like to move the person
	// input direction 1 to move up, 0 to move down
	// input reg the output of the 5 shift registers goes here
	// verticallocation will denote the position of the bottom pixels on the person
	// gameover will be 1 if the game is to be ended
	
	// player will start at vertical location 8 and move about in column horizontal location 15 - 17
	
	always @(posedge moveclock)
	begin
		if(initialize == 1)
		begin
			verticallocation <= 7'b0001000;
			gameover <= 0;
			iscollide <= 0;
		end
		else
		begin
			if((direction == 1) & (gameover == 0))
			begin
				iscollide <= 0;
				if(verticallocation > 4)
				begin
					verticallocation <= verticallocation - 1;
				end
			end
			else if((direction == 0) & (gameover == 0))
			begin
				if(verticallocation == 15)
				begin
					if((reg1[15] == 1) | (reg1[18] == 1))
					begin
						verticallocation <= verticallocation;
						iscollide <= 1;
					end
					else
					begin
						verticallocation <= verticallocation + 1;
					end
				end
				else if(verticallocation == 35)
				begin
					if((reg2[15] == 1) | (reg2[18] == 1))
					begin
						verticallocation <= verticallocation;
						iscollide <= 1;
					end
					else
					begin
						verticallocation <= verticallocation + 1;
					end
				end
				else if(verticallocation == 55)
				begin
					if((reg3[15] == 1) | (reg3[18] == 1))
					begin
						verticallocation <= verticallocation;
						iscollide <= 1;
					end
					else
					begin
						verticallocation <= verticallocation + 1;
					end
				end
				else if(verticallocation == 75)
				begin
					if((reg4[15] == 1) | (reg4[18] == 1))
					begin
						verticallocation <= verticallocation;
						iscollide <= 1;
					end
					else
					begin
						verticallocation <= verticallocation + 1;
					end
				end
				else if(verticallocation == 95)
				begin
					if((reg5[15] == 1) | (reg5[18] == 1))
					begin
						verticallocation <= verticallocation;
						iscollide <= 1;
					end
					else
					begin
						verticallocation <= verticallocation + 1;
					end
				end
				else
				begin
					verticallocation <= verticallocation + 1;
				end
			end
			toplefty <= verticallocation - 3;
			topleftx <= 15;
			if(verticallocation > 110)
			begin
				gameover <= 1;
			end
		end
	end
endmodule

module secondtimer(enable, reset_n, update);
	input enable;
	input reset_n;
	output update;
	reg[26:0] num;
	always @(posedge enable, negedge reset_n)
	begin
		if(reset_n == 0)
			num <= 26'd50000000;
		else if(num == 0)
			num <= 26'd50000000;
		else
			num <= num - 1;
	end 
	assign update = ~(|num);
endmodule 

module scorecounter(input initialize, input secondclock, output reg[3:0] digit0, output reg[3:0] digit1, output reg[3:0] digit2);
	// score will have 3 digits that increment each second in the form [digit2][digit1][digit0]
	// input initialize starts the score counter_1
	// output digit - digit of score
	always @(posedge secondclock)
	begin
		if(initialize == 1)
		begin
			digit0 <= 0;
			digit1 <= 0;
			digit2 <= 0;
		end
		else if(digit0 < 9)
		begin
			digit0 <= digit0 + 1;
		end
		else if(digit1 < 9)
		begin
			digit1 <= digit1 + 1;
			digit0 <= 0;
		end
		else if(digit2 < 9)
		begin
			digit2 <= digit2 + 1;
			digit1 <= 0;
			digit0 <= 0;
		end
		else
		begin
			digit0 <= 0;
			digit1 <= 0;
			digit2 <= 0;
		end
	end
endmodule

module mux(select_output, x1, y1, x2, y2, color1, color2, x, y, color);
	input[7:0] x1, x2;
	input[6:0] y1, y2;
	input select_output;
	input[2:0] color1, color2;
	output[7:0] x;
	output[6:0] y;
	output[2:0] color;
	reg[7:0] x;
	reg[6:0] y;
	reg[2:0] color;
	
	always @(*)
	begin
		if(select_output == 0)
		begin
			x = x1;
			y = y1;
			color = color1;
		end
		else
		begin
			x = x2;
			y = y2;
			color = color2;
		end
	end
endmodule 

module startingarray(input[6:0] score1, input[6:0] score2, input[6:0] score3, input clock, input[7:0] xposition, input[6:0] yposition, input toggle, output reg arrayvalue);
	reg [119:0] array [159:0];
	reg [15:0] i;
	reg [15:0] j;
	
	always @(posedge clock)
	begin
		for (i = 0; i < 160; i = i + 1)
		begin
			for (j = 0; j < 120; j = j + 1)
			begin
				array[i][j] <= 0;
			end
		end
		if(toggle == 0)
		begin
		array[66][55] <= 1;
		array[66][56] <= 1;
		array[66][57] <= 1;
		array[66][58] <= 0;
		array[66][59] <= 1;
		
		array[67][55] <= 1;
		array[67][56] <= 0;
		array[67][57] <= 1;
		array[67][58] <= 0;
		array[67][59] <= 1;
		
		array[68][55] <= 1;
		array[68][56] <= 0;
		array[68][57] <= 1;
		array[68][58] <= 1;
		array[68][59] <= 1;
		
		array[71][55] <= 1;
		array[71][56] <= 0;
		array[71][57] <= 0;
		array[71][58] <= 0;
		array[71][59] <= 0;
		
		array[72][55] <= 1;
		array[72][56] <= 1;
		array[72][57] <= 1;
		array[72][58] <= 1;
		array[72][59] <= 1;
		
		array[73][55] <= 1;
		array[73][56] <= 0;
		array[73][57] <= 0;
		array[73][58] <= 0;
		array[73][59] <= 0;
		
		array[74][55] <= 1;
		array[74][56] <= 0;
		array[74][57] <= 0;
		array[74][58] <= 0;
		array[74][59] <= 0;
		
		array[77][55] <= 1;
		array[77][56] <= 1;
		array[77][57] <= 1;
		array[77][58] <= 1;
		array[77][59] <= 1;
		
		array[78][55] <= 1;
		array[78][56] <= 0;
		array[78][57] <= 1;
		array[78][58] <= 0;
		array[78][59] <= 0;
		
		array[79][55] <= 1;
		array[79][56] <= 1;
		array[79][57] <= 1;
		array[79][58] <= 1;
		array[79][59] <= 1;
		
		array[82][55] <= 1;
		array[82][56] <= 1;
		array[82][57] <= 1;
		array[82][58] <= 1;
		array[82][59] <= 1;
		
		array[83][55] <= 1;
		array[83][56] <= 0;
		array[83][57] <= 1;
		array[83][58] <= 1;
		array[83][59] <= 0;
		
		array[84][55] <= 1;
		array[84][56] <= 1;
		array[84][57] <= 1;
		array[84][58] <= 0;
		array[84][59] <= 1;
		
		array[87][55] <= 1;
		array[87][56] <= 0;
		array[87][57] <= 0;
		array[87][58] <= 0;
		array[87][59] <= 0;
		
		array[88][55] <= 1;
		array[88][56] <= 1;
		array[88][57] <= 1;
		array[88][58] <= 1;
		array[88][59] <= 1;
		
		array[89][55] <= 1;
		array[89][56] <= 0;
		array[89][57] <= 0;
		array[89][58] <= 0;
		array[89][59] <= 0;
		
		array[92][55] <= 1;
		array[92][56] <= 1;
		array[92][57] <= 1;
		array[92][58] <= 0;
		array[92][59] <= 0;
		
		array[93][55] <= 0;
		array[93][56] <= 0;
		array[93][57] <= 1;
		array[93][58] <= 0;
		array[93][59] <= 0;
		
		array[94][55] <= 0;
		array[94][56] <= 0;
		array[94][57] <= 1;
		array[94][58] <= 0;
		array[94][59] <= 0;
		
		array[95][55] <= 0;
		array[95][56] <= 0;
		array[95][57] <= 1;
		array[95][58] <= 0;
		array[95][59] <= 0;
		
		array[96][55] <= 0;
		array[96][56] <= 0;
		array[96][57] <= 1;
		array[96][58] <= 0;
		array[96][59] <= 0;
		
		array[97][55] <= 0;
		array[97][56] <= 0;
		array[97][57] <= 1;
		array[97][58] <= 0;
		array[97][59] <= 0;
		
		array[98][55] <= 1;
		array[98][56] <= 1;
		array[98][57] <= 1;
		array[98][58] <= 1;
		array[98][59] <= 1;
		
		array[99][55] <= 0;
		array[99][56] <= 1;
		array[99][57] <= 1;
		array[99][58] <= 1;
		array[99][59] <= 0;
		
		array[100][55] <= 0;
		array[100][56] <= 0;
		array[100][57] <= 1;
		array[100][58] <= 0;
		array[100][59] <= 0;
		end
		else
		begin
		if(score1[0] == 0)
		begin
			array[155][1] <= 1;
			array[154][1] <= 1;
			array[153][1] <= 1;
		end
		if(score1[1] == 0)
		begin
			array[155][1] <= 1;
			array[155][2] <= 1;
			array[155][3] <= 1;
		end
		if(score1[2] == 0)
		begin
			array[155][3] <= 1;
			array[155][4] <= 1;
			array[155][5] <= 1;
		end
		if(score1[3] == 0)
		begin
			array[155][5] <= 1;
			array[154][5] <= 1;
			array[153][5] <= 1;
		end
		if(score1[4] == 0)
		begin
			array[153][3] <= 1;
			array[153][4] <= 1;
			array[153][5] <= 1;
		end
		if(score1[5] == 0)
		begin
			array[153][1] <= 1;
			array[153][2] <= 1;
			array[153][3] <= 1;
		end
		if(score1[6] == 0)
		begin
			array[155][3] <= 1;
			array[154][3] <= 1;
			array[153][3] <= 1;
		end
		
		
		
		if(score2[0] == 0)
		begin
			array[151][1] <= 1;
			array[150][1] <= 1;
			array[149][1] <= 1;
		end
		if(score2[1] == 0)
		begin
			array[151][1] <= 1;
			array[151][2] <= 1;
			array[151][3] <= 1;
		end
		if(score2[2] == 0)
		begin
			array[151][3] <= 1;
			array[151][4] <= 1;
			array[151][5] <= 1;
		end
		if(score2[3] == 0)
		begin
			array[151][5] <= 1;
			array[150][5] <= 1;
			array[149][5] <= 1;
		end
		if(score2[4] == 0)
		begin
			array[149][3] <= 1;
			array[149][4] <= 1;
			array[149][5] <= 1;
		end
		if(score2[5] == 0)
		begin
			array[149][1] <= 1;
			array[149][2] <= 1;
			array[149][3] <= 1;
		end
		if(score2[6] == 0)
		begin
			array[151][3] <= 1;
			array[150][3] <= 1;
			array[149][3] <= 1;
		end
		
		
		
		if(score3[0] == 0)
		begin
			array[147][1] <= 1;
			array[146][1] <= 1;
			array[145][1] <= 1;
		end
		if(score3[1] == 0)
		begin
			array[147][1] <= 1;
			array[147][2] <= 1;
			array[147][3] <= 1;
		end
		if(score3[2] == 0)
		begin
			array[147][3] <= 1;
			array[147][4] <= 1;
			array[147][5] <= 1;
		end
		if(score3[3] == 0)
		begin
			array[147][5] <= 1;
			array[146][5] <= 1;
			array[145][5] <= 1;
		end
		if(score3[4] == 0)
		begin
			array[145][3] <= 1;
			array[145][4] <= 1;
			array[145][5] <= 1;
		end
		if(score3[5] == 0)
		begin
			array[145][1] <= 1;
			array[145][2] <= 1;
			array[145][3] <= 1;
		end
		if(score3[6] == 0)
		begin
			array[147][3] <= 1;
			array[146][3] <= 1;
			array[145][3] <= 1;
		end
		
		array[60][40] <= 1;
		array[60][41] <= 1;
		array[60][42] <= 1;
		array[60][43] <= 1;
		array[60][44] <= 1;
		
		array[61][40] <= 1;
		array[61][41] <= 0;
		array[61][42] <= 1;
		array[61][43] <= 0;
		array[61][44] <= 1;
		
		array[62][40] <= 1;
		array[62][41] <= 0;
		array[62][42] <= 1;
		array[62][43] <= 0;
		array[62][44] <= 1;
		
		array[63][40] <= 1;
		array[63][41] <= 0;
		array[63][42] <= 1;
		array[63][43] <= 0;
		array[63][44] <= 1;
		
		array[65][40] <= 1;
		array[65][41] <= 1;
		array[65][42] <= 1;
		array[65][43] <= 1;
		array[65][44] <= 1;
		
		array[66][40] <= 0;
		array[66][41] <= 1;
		array[66][42] <= 0;
		array[66][43] <= 0;
		array[66][44] <= 0;
		
		array[67][40] <= 0;
		array[67][41] <= 0;
		array[67][42] <= 1;
		array[67][43] <= 1;
		array[67][44] <= 0;
		
		array[68][40] <= 1;
		array[68][41] <= 1;
		array[68][42] <= 1;
		array[68][43] <= 1;
		array[68][44] <= 1;
		
		array[70][40] <= 1;
		array[70][41] <= 1;
		array[70][42] <= 1;
		array[70][43] <= 1;
		array[70][44] <= 1;
		
		array[71][40] <= 1;
		array[71][41] <= 0;
		array[71][42] <= 0;
		array[71][43] <= 0;
		array[71][44] <= 1;
		
		array[72][40] <= 1;
		array[72][41] <= 0;
		array[72][42] <= 0;
		array[72][43] <= 0;
		array[72][44] <= 1;
		
		array[73][40] <= 0;
		array[73][41] <= 1;
		array[73][42] <= 1;
		array[73][43] <= 1;
		array[73][44] <= 0;
		end
		arrayvalue <= array[xposition][yposition];
	end
endmodule

module ps2_rx
	(
		input wire clk, reset, 
		input wire ps2d, ps2c, rx_en,    // ps2 data and clock inputs, receive enable input
		output reg rx_done_tick,         // ps2 receive done tick
		output wire [7:0] rx_data        // data received 
	);
	
	// FSMD state declaration
	localparam 
		idle = 1'b0,
		rx   = 1'b1;
		
	// internal signal declaration
	reg state_reg, state_next;          // FSMD state register
	reg [7:0] filter_reg;               // shift register filter for ps2c
	wire [7:0] filter_next;             // next state value of ps2c filter register
	reg f_val_reg;                      // reg for ps2c filter value, either 1 or 0
	wire f_val_next;                    // next state for ps2c filter value
	reg [3:0] n_reg, n_next;            // register to keep track of bit number 
	reg [10:0] d_reg, d_next;           // register to shift in rx data
	wire neg_edge;                      // negative edge of ps2c clock filter value
	
	// register for ps2c filter register and filter value
	always @(posedge clk, posedge reset)
		if (reset)
			begin
			filter_reg <= 0;
			f_val_reg  <= 0;
			end
		else
			begin
			filter_reg <= filter_next;
			f_val_reg  <= f_val_next;
			end

	// next state value of ps2c filter: right shift in current ps2c value to register
	assign filter_next = {ps2c, filter_reg[7:1]};
	
	// filter value next state, 1 if all bits are 1, 0 if all bits are 0, else no change
	assign f_val_next = (filter_reg == 8'b11111111) ? 1'b1 :
			    (filter_reg == 8'b00000000) ? 1'b0 :
			    f_val_reg;
	
	// negative edge of filter value: if current value is 1, and next state value is 0
	assign neg_edge = f_val_reg & ~f_val_next;
	
	// FSMD state, bit number, and data registers
	always @(posedge clk, posedge reset)
		if (reset)
			begin
			state_reg <= idle;
			n_reg <= 0;
			d_reg <= 0;
			end
		else
			begin
			state_reg <= state_next;
			n_reg <= n_next;
			d_reg <= d_next;
			end
	
	// FSMD next state logic
	always @*
		begin
		
		// defaults
		state_next = state_reg;
		rx_done_tick = 1'b0;
		n_next = n_reg;
		d_next = d_reg;
		
		case (state_reg)
			
			idle:
				if (neg_edge & rx_en)                 // start bit received
					begin
					n_next = 4'b1010;             // set bit count down to 10
					state_next = rx;              // go to rx state
					end
				
			rx:                                           // shift in 8 data, 1 parity, and 1 stop bit
				begin
				if (neg_edge)                         // if ps2c negative edge...
					begin
					d_next = {ps2d, d_reg[10:1]}; // sample ps2d, right shift into data register
					n_next = n_reg - 1;           // decrement bit count
					end
			
				if (n_reg==0)                         // after 10 bits shifted in, go to done state
                                        begin
					 rx_done_tick = 1'b1;         // assert dat received done tick
					 state_next = idle;           // go back to idle
					 end
				end
		endcase
		end
		
	assign rx_data = d_reg[8:1]; // output data bits 
endmodule

module keydecoder(input[7:0] data, input clock, output reg enter, output reg up);
	always @(*)
	begin
		if(data == 8'b01011010)
		begin
			enter <= 1;
			up <= 0;
		end
		else if(data == 8'b00011101)
		begin
			enter <= 0;
			up <= 1;
		end
		else
		begin
			enter <= 0;
			up <= 0;
		end
	end
endmodule


module resetClock(clock, rclock);
	input clock;
	output rclock;
	reg[20:0] count;
	always @(posedge clock)
	begin
		count <= count + 1;
	end
	assign rclock = (count == 0);
endmodule 

module endarray(input[6:0] score1, input[6:0] score2, input[6:0] score3, input clock, input[7:0] xposition, input[6:0] yposition, output reg arrayvalue);
	reg [119:0] array [159:0];
	reg [15:0] i;
	reg [15:0] j;
	
	always @(posedge clock)
	begin
		for (i = 0; i < 160; i = i + 1)
		begin
			for (j = 0; j < 120; j = j + 1)
			begin
				array[i][j] <= 0;
			end
		end
		
		if(score1[0] == 0)
		begin
			array[155][1] <= 1;
			array[154][1] <= 1;
			array[153][1] <= 1;
		end
		if(score1[1] == 0)
		begin
			array[155][1] <= 1;
			array[155][2] <= 1;
			array[155][3] <= 1;
		end
		if(score1[2] == 0)
		begin
			array[155][3] <= 1;
			array[155][4] <= 1;
			array[155][5] <= 1;
		end
		if(score1[3] == 0)
		begin
			array[155][5] <= 1;
			array[154][5] <= 1;
			array[153][5] <= 1;
		end
		if(score1[4] == 0)
		begin
			array[153][3] <= 1;
			array[153][4] <= 1;
			array[153][5] <= 1;
		end
		if(score1[5] == 0)
		begin
			array[153][1] <= 1;
			array[153][2] <= 1;
			array[153][3] <= 1;
		end
		if(score1[6] == 0)
		begin
			array[155][3] <= 1;
			array[154][3] <= 1;
			array[153][3] <= 1;
		end
		
		
		
		if(score2[0] == 0)
		begin
			array[151][1] <= 1;
			array[150][1] <= 1;
			array[149][1] <= 1;
		end
		if(score2[1] == 0)
		begin
			array[151][1] <= 1;
			array[151][2] <= 1;
			array[151][3] <= 1;
		end
		if(score2[2] == 0)
		begin
			array[151][3] <= 1;
			array[151][4] <= 1;
			array[151][5] <= 1;
		end
		if(score2[3] == 0)
		begin
			array[151][5] <= 1;
			array[150][5] <= 1;
			array[149][5] <= 1;
		end
		if(score2[4] == 0)
		begin
			array[149][3] <= 1;
			array[149][4] <= 1;
			array[149][5] <= 1;
		end
		if(score2[5] == 0)
		begin
			array[149][1] <= 1;
			array[149][2] <= 1;
			array[149][3] <= 1;
		end
		if(score2[6] == 0)
		begin
			array[151][3] <= 1;
			array[150][3] <= 1;
			array[149][3] <= 1;
		end
		
		
		
		if(score3[0] == 0)
		begin
			array[147][1] <= 1;
			array[146][1] <= 1;
			array[145][1] <= 1;
		end
		if(score3[1] == 0)
		begin
			array[147][1] <= 1;
			array[147][2] <= 1;
			array[147][3] <= 1;
		end
		if(score3[2] == 0)
		begin
			array[147][3] <= 1;
			array[147][4] <= 1;
			array[147][5] <= 1;
		end
		if(score3[3] == 0)
		begin
			array[147][5] <= 1;
			array[146][5] <= 1;
			array[145][5] <= 1;
		end
		if(score3[4] == 0)
		begin
			array[145][3] <= 1;
			array[145][4] <= 1;
			array[145][5] <= 1;
		end
		if(score3[5] == 0)
		begin
			array[145][1] <= 1;
			array[145][2] <= 1;
			array[145][3] <= 1;
		end
		if(score3[6] == 0)
		begin
			array[147][3] <= 1;
			array[146][3] <= 1;
			array[145][3] <= 1;
		end
		
		array[60][40] <= 1;
		array[60][41] <= 1;
		array[60][42] <= 1;
		array[60][43] <= 1;
		array[60][44] <= 1;
		
		array[61][40] <= 1;
		array[61][41] <= 0;
		array[61][42] <= 1;
		array[61][43] <= 0;
		array[61][44] <= 1;
		
		array[62][40] <= 1;
		array[62][41] <= 0;
		array[62][42] <= 1;
		array[62][43] <= 0;
		array[62][44] <= 1;
		
		array[63][40] <= 1;
		array[63][41] <= 0;
		array[63][42] <= 1;
		array[63][43] <= 0;
		array[63][44] <= 1;
		
		array[65][40] <= 1;
		array[65][41] <= 1;
		array[65][42] <= 1;
		array[65][43] <= 1;
		array[65][44] <= 1;
		
		array[66][40] <= 0;
		array[66][41] <= 1;
		array[66][42] <= 0;
		array[66][43] <= 0;
		array[66][44] <= 0;
		
		array[67][40] <= 0;
		array[67][41] <= 0;
		array[67][42] <= 1;
		array[67][43] <= 1;
		array[67][44] <= 0;
		
		array[68][40] <= 1;
		array[68][41] <= 1;
		array[68][42] <= 1;
		array[68][43] <= 1;
		array[68][44] <= 1;
		
		array[70][40] <= 1;
		array[70][41] <= 1;
		array[70][42] <= 1;
		array[70][43] <= 1;
		array[70][44] <= 1;
		
		array[71][40] <= 1;
		array[71][41] <= 0;
		array[71][42] <= 0;
		array[71][43] <= 0;
		array[71][44] <= 1;
		
		array[72][40] <= 1;
		array[72][41] <= 0;
		array[72][42] <= 0;
		array[72][43] <= 0;
		array[72][44] <= 1;
		
		array[73][40] <= 0;
		array[73][41] <= 1;
		array[73][42] <= 1;
		array[73][43] <= 1;
		array[73][44] <= 0;
		arrayvalue <= array[xposition][yposition];
	end
endmodule 
