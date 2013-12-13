module audio3(
  // Clock Input (50 MHz)
  input CLOCK_50, // 50 MHz
  input CLOCK_27, // 27 MHz
  //  Push Buttons
  input  [3:0]  KEY,
  //  DPDT Switches 
  input  [17:0]  SW,
  //  7-SEG Displays
  output  [6:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7,
  //  LEDs
  output  [8:0]  LEDG,  //  LED Green[8:0]
  output  [17:0]  LEDR, //  LED Red[17:0]
  // TV Decoder
  output TD_RESET, // TV Decoder Reset
  // I2C
  inout  I2C_SDAT, // I2C Data
  output I2C_SCLK, // I2C Clock
  // Audio CODEC
  output/*inout*/ AUD_ADCLRCK, // Audio CODEC ADC LR Clock
  input	 AUD_ADCDAT,  // Audio CODEC ADC Data
  output /*inout*/  AUD_DACLRCK, // Audio CODEC DAC LR Clock
  output AUD_DACDAT,  // Audio CODEC DAC Data
  inout	 AUD_BCLK,    // Audio CODEC Bit-Stream Clock
  output AUD_XCK     // Audio CODEC Chip Clock
);


wire enable;

wire [6:0] myclock;
wire RST;
assign RST = KEY[0];

// reset delay gives some time for peripherals to initialize
wire DLY_RST;
Reset_Delay r0(	.iCLK(CLOCK_50),.oRESET(DLY_RST) );

assign	TD_RESET = 1'b1;  // Enable 27 MHz

VGA_Audio_PLL 	p1 (	
	.areset(~DLY_RST),
	.inclk0(CLOCK_27),
	.c0(VGA_CTRL_CLK),
	.c1(AUD_CTRL_CLK),
	.c2(VGA_CLK)
);

I2C_AV_Config u3(	
//	Host Side
  .iCLK(CLOCK_50),
  .iRST_N(KEY[0]),
//	I2C Side
  .I2C_SCLK(I2C_SCLK),
  .I2C_SDAT(I2C_SDAT)	
);

assign	AUD_ADCLRCK	=	AUD_DACLRCK;
assign	AUD_XCK		=	AUD_CTRL_CLK;

audio_clock u4(	
//	Audio Side
   .oAUD_BCK(AUD_BCLK),
   .oAUD_LRCK(AUD_DACLRCK),
//	Control Signals
  .iCLK_18_4(AUD_CTRL_CLK),
   .iRST_N(DLY_RST)	
);

audio_converter u5(
	// Audio side
	.AUD_BCK(AUD_BCLK),       // Audio bit clock
	.AUD_LRCK(AUD_DACLRCK), // left-right clock
	.AUD_ADCDAT(AUD_ADCDAT),
	.AUD_DATA(AUD_DACDAT),
	// Controller side
	.iRST_N(DLY_RST),  // reset
	.AUD_outL(audio_outL),
	.AUD_outR(audio_outR),
	.AUD_inL(audio_inL),
	.AUD_inR(audio_inR)
);

wire [15:0] audio_inL, audio_inR;
wire [15:0] audio_outL, audio_outR;
wire [15:0] signal;

//set up DDS frequency
//Use switches to set freq
wire [31:0] dds_incrR;
wire [31:0] dds_incrL;
//wire [31:0] freq = SW[3:0]+10*SW[7:4]+100*SW[11:8]+1000*SW[15:12]+10000*SW[17:16];

wire [31:0] freqR = 32'd1000;  //Right Sound Frequency in Hz
wire [31:0] freqL = 32'd2500;	//Left Sound Frequency in Hz

assign dds_incrR = freqR * 91626 ; //91626 = 2^32/46875 so SW is in Hz
assign dds_incrL = freqL * 91626 ; //91626 = 2^32/46875 so SW is in Hz

reg [31:0] dds_phaseR;
reg [31:0] dds_phaseL;

always @(negedge AUD_DACLRCK or negedge DLY_RST)
	if (!DLY_RST) dds_phaseR <= 0;
	else dds_phaseR <= dds_phaseR + dds_incrR;

always @(negedge AUD_DACLRCK or negedge DLY_RST)
	if (!DLY_RST) dds_phaseL <= 0;
	else dds_phaseL <= dds_phaseL + dds_incrL;	
	
	
wire [7:0] indexR = dds_phaseR[31:24];
wire [7:0] indexL = dds_phaseL[31:24];


wire [15:0] ONaudio_outR;
wire [15:0] ONaudio_outL;
 
sine_table sigR(
	.index(indexR),
	.signal(ONaudio_outR)
);

sine_table sigL(
	.index(indexL),
	.signal(ONaudio_outL)
);


	//audio_outR <= audio_inR;

//always @(posedge AUD_DACLRCK)
//assign audio_outL = 15'h0000;

assign audio_outR = onR ? ONaudio_outR : 15'h0000;
assign audio_outL = onL ? ONaudio_outL : 15'h0000;

assign LEDG[8] = enable;

reg [25:0] count;
reg onR, onL;

always @ (posedge CLOCK_50 or negedge KEY[0]) begin
	if(KEY[0] == 1'b0) begin
	end
	else if (CLOCK_50 == 1'b1) begin
		if(enable == 1'b1)begin
			count <= count + 1'b1;
			if(count == 26'd50000000) begin
				count <= 26'b00000000000000000000000000;
			end
			else if(count < 26'd25000000) begin
				onR <= 1'b1;
				onL <= 1'b0;
			end
			else if(count >= 26'd25000000) begin
				onR <= 1'b0;
				onL <= 1'b1;
			end
		end
		else begin
			onR <= 1'b0;
			onL <= 1'b0;			
		end
	end

end

clock(CLOCK_50, KEY[0], HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7, KEY[3], KEY[2], SW[0], KEY[1], enable, LEDR);

endmodule

module clock (clk, rst,
HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7,
addHR, addMIN, alrmSET, alrmRST, alarm,
lights
);


input clk;
input rst;

output reg[17:0] lights;

input alrmSET;

input addHR, addMIN; // KEY3, KEY2
reg addHR_EN, addMIN_EN;

output reg [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7;

output reg alarm;
input alrmRST; // KEY1

reg [25:0] count;
reg [25:0] toSec;
reg [3:0]  sO, sT, mO, mT, hO, hT;
reg [3:0]  AmO, AmT, AhO, AhT;

reg am_pmSW;
reg Aam_pmSW;
reg am_pm; // 0: AM 1: PM
reg Aam_pm;

wire [6:0] am_pmALPHA;
wire [6:0] Aam_pmALPHA;


assign am_pmALPHA = am_pm ? (7'b0001100) : (7'b0001000);
assign Aam_pmALPHA = Aam_pm ? (7'b0001100) : (7'b0001000);

assign zero = alrmSET ? Aam_pmALPHA : am_pmALPHA;

wire [6:0] Tzero, Tone, Ttwo, Tthree, Tfour, Tfive, Tsix, Tseven;
wire [6:0] Azero, Aone, Atwo, Athree, Afour, Afive, Asix, Aseven;
wire [6:0] zero, one, two, three, four, five, six, seven;


SevenSegmentDisp hrT(hT, Tseven);
SevenSegmentDisp hrO(hO, Tsix);
SevenSegmentDisp miT(mT, Tfive);
SevenSegmentDisp miO(mO, Tfour);
SevenSegmentDisp scT(sT, Tthree);
SevenSegmentDisp scO(sO, Ttwo);

SevenSegmentDisp AhrT(AhT, Aseven);
SevenSegmentDisp AhrO(AhO, Asix);
SevenSegmentDisp AmiT(AmT, Afive);
SevenSegmentDisp AmiO(AmO, Afour);

assign seven = alrmSET ? Aseven  : Tseven;
assign six = alrmSET   ? Asix    : Tsix;
assign five = alrmSET  ? Afive   : Tfive;
assign four = alrmSET  ? Afour   : Tfour;
assign three = alrmSET ? (7'hFF) : Tthree;
assign two = alrmSET   ? (7'hFF) : Ttwo;
assign one = alrmSET   ? (7'hFF) : (7'hEF);
//assign zero = alrmSET  ? ()




always @(posedge clk, negedge rst)
begin
	if(rst == 1'b0) begin
		hT <= 4'd1;
		hO <= 4'd2;
		mT <= 4'd0;
		mO <= 4'd0;
		sT <= 4'd0;
		sO <= 4'd0;
		
		AhT <= 4'd0;
		AhO <= 4'd0;
		AmT <= 4'd0;
		AmO <= 4'd0;
		
		alarm <= 1'b0;
		
		am_pm <= 1'b0;
		Aam_pm <= 1'b0;
	end
	else begin
		if(clk == 1'b1) begin
			
			//Alarm goes off
			if(hT == AhT && hO == AhO && mT == AmT && mO == AmO && sT == 4'd0 && sO == 4'd0 && am_pm == Aam_pm) begin
				alarm <= 1'b1;
			end
			else if(alrmRST == 1'b0) begin
				alarm <= 1'b0;
			end
			
			/*
			//dims lights on
			if(alarm <= 1'b1) begin

					
					//always @ (posedge clock) begin
						counter = counter + 1;
						if (counter <= steps) lights <= 18'b111111111111111111;
						else lights <= 0;
						if (counter >= 50000) counter <= 0;
					//end
			end
			*/
			
			
			//adds time per key press (one unit per press)
			//Hour
			if(addHR == 1'b0 && addHR_EN == 1'b1) begin
				if(alrmSET == 1'b0)
					hO <= hO + 1'b1;
				else if(alrmSET == 1'b1)
					AhO <= AhO + 1'b1;
				addHR_EN <= 1'b0;
			end
			else if(addHR == 1'b1)
				addHR_EN <= 1'b1;
				
			//Minute
			if(addMIN == 1'b0 && addMIN_EN == 1'b1) begin
				if(alrmSET == 1'b0)
					mO <= mO + 1'b1;
				else if(alrmSET == 1'b1)
					AmO <= AmO + 1'b1;
				addMIN_EN <= 1'b0;
			end
			else if(addMIN == 1'b1)
				addMIN_EN <= 1'b1;
				
			//Uses the clock to count seconds
			count <= count + 1'b1;
			if(count == 26'd50000000) begin
				count <= 26'b00000000000000000000000000;
				sO <= sO + 1'b1;
			end
			
			//Converts numbers into correct time values
			if(hT == 1 && hO == 3) begin
				hO <= 4'd1;
				hT <= 4'd0;
				mT <= 4'd0;
				mO <= 4'd0;
				sT <= 4'd0;
				sO <= 4'd0;
			end
			else if(hO == 10) begin
				hO <= 4'd0;
				hT <= hT + 1'b1;
			end
			else if(mT == 6) begin
				mT <= 4'd0;
				hO <= hO + 1'b1;
			end
			else if(mO == 10) begin
				mO <= 4'd0;
				mT <= mT + 1'b1;
			end
			else if(sT == 6) begin
				sT <= 4'd0;
				mO <= mO + 1'b1;
			end
			else if(sO == 10) begin
				sO <= 4'd0;
				sT <= sT + 1'b1;
			end
						
			//Coverts the alarm numbers to time values
			if(AhT == 1 && AhO == 3) begin
				AhO <= 4'd1;
				AhT <= 4'd0;
			end
			else if(AhO == 10) begin
				AhO <= 4'd0;
				AhT <= AhT + 1'b1;
			end
			else if(AmT == 6) begin
				AmT <= 4'd0;
				AhO <= AhO + 1'b1;
			end
			else if(AmO == 10) begin
				AmO <= 4'd0;
				AmT <= AmT + 1'b1;
			end
			
			//Switches between AM and PM
			
			if(hT == 0 && hO == 1)
				am_pmSW <= 1'b1;
			else if(hT == 1 && hO == 2 && am_pmSW == 1'b1) begin
				am_pm <= am_pm + 1'b1;
				am_pmSW <= 1'b0;
			end
				
			if(AhT == 0 && AhO == 1)
				Aam_pmSW <= 1'b1;
			else if(AhT == 1 && AhO == 2 && Aam_pmSW == 1'b1) begin
				Aam_pm <= Aam_pm + 1'b1;
				Aam_pmSW <= 1'b0;
			end			
			
			
			//The hex display
			HEX0 <= zero;
			HEX1 <= 7'b1111111;
			HEX2 <= two;
			HEX3 <= three;
			HEX4 <= four;
			HEX5 <= five;
			HEX6 <= six;
			HEX7 <= seven;
				
		end
	end
end

endmodule
