module regfile (
    clock,
    ctrl_writeEnable,
    ctrl_reset, ctrl_writeReg,
    ctrl_readRegA, ctrl_readRegB, data_writeReg,
    data_readRegA, data_readRegB
);

   input clock, ctrl_writeEnable, ctrl_reset;
   input [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
   input [31:0] data_writeReg;

   output [31:0] data_readRegA, data_readRegB;
	
	wire [31:0] regWrite, regEnable;
	wire [31:0] bus0,  bus1,  bus2,  bus3,  bus4,  bus5,  bus6,  bus7,
					bus8,  bus9,  bus10, bus11, bus12, bus13, bus14, bus15,
					bus16, bus17, bus18, bus19, bus20, bus21, bus22, bus23,
					bus24, bus25, bus26, bus27, bus28, bus29, bus30, bus31;
		
	decoder writeDecode(regWrite, ctrl_writeReg); 
		
	and and0 (regEnable[0],  1'b0, ctrl_writeEnable);	// Cannot write to reg 0
	and and1 (regEnable[1],  regWrite[1],  ctrl_writeEnable);
	and and2 (regEnable[2],  regWrite[2],  ctrl_writeEnable);
	and and3 (regEnable[3],  regWrite[3],  ctrl_writeEnable);
	and and4 (regEnable[4],  regWrite[4],  ctrl_writeEnable);
	and and5 (regEnable[5],  regWrite[5],  ctrl_writeEnable);
	and and6 (regEnable[6],  regWrite[6],  ctrl_writeEnable);
	and and7 (regEnable[7],  regWrite[7],  ctrl_writeEnable);
	and and8 (regEnable[8],  regWrite[8],  ctrl_writeEnable);
	and and9 (regEnable[9],  regWrite[9],  ctrl_writeEnable);
	and and10(regEnable[10], regWrite[10], ctrl_writeEnable);
	and and11(regEnable[11], regWrite[11], ctrl_writeEnable);
	and and12(regEnable[12], regWrite[12], ctrl_writeEnable);
	and and13(regEnable[13], regWrite[13], ctrl_writeEnable);
	and and14(regEnable[14], regWrite[14], ctrl_writeEnable);
	and and15(regEnable[15], regWrite[15], ctrl_writeEnable);
	and and16(regEnable[16], regWrite[16], ctrl_writeEnable);
	and and17(regEnable[17], regWrite[17], ctrl_writeEnable);
	and and18(regEnable[18], regWrite[18], ctrl_writeEnable);
	and and19(regEnable[19], regWrite[19], ctrl_writeEnable);
	and and20(regEnable[20], regWrite[20], ctrl_writeEnable);
	and and21(regEnable[21], regWrite[21], ctrl_writeEnable);
	and and22(regEnable[22], regWrite[22], ctrl_writeEnable);
	and and23(regEnable[23], regWrite[23], ctrl_writeEnable);
	and and24(regEnable[24], regWrite[24], ctrl_writeEnable);
	and and25(regEnable[25], regWrite[25], ctrl_writeEnable);
	and and26(regEnable[26], regWrite[26], ctrl_writeEnable);
	and and27(regEnable[27], regWrite[27], ctrl_writeEnable);
	and and28(regEnable[28], regWrite[28], ctrl_writeEnable);
	and and29(regEnable[29], regWrite[29], ctrl_writeEnable);
	and and30(regEnable[30], regWrite[30], ctrl_writeEnable);
	and and31(regEnable[31], regWrite[31], ctrl_writeEnable);
	
	registerUnit reg0 (clock, regEnable[0],  ctrl_reset, data_writeReg, bus0);
	registerUnit reg1 (clock, regEnable[1],  ctrl_reset, data_writeReg, bus1);
	registerUnit reg2 (clock, regEnable[2],  ctrl_reset, data_writeReg, bus2);
	registerUnit reg3 (clock, regEnable[3],  ctrl_reset, data_writeReg, bus3);
	registerUnit reg4 (clock, regEnable[4],  ctrl_reset, data_writeReg, bus4);
	registerUnit reg5 (clock, regEnable[5],  ctrl_reset, data_writeReg, bus5);
	registerUnit reg6 (clock, regEnable[6],  ctrl_reset, data_writeReg, bus6);
	registerUnit reg7 (clock, regEnable[7],  ctrl_reset, data_writeReg, bus7);
	registerUnit reg8 (clock, regEnable[8],  ctrl_reset, data_writeReg, bus8);
	registerUnit reg9 (clock, regEnable[9],  ctrl_reset, data_writeReg, bus9);
	registerUnit reg10(clock, regEnable[10], ctrl_reset, data_writeReg, bus10);
	registerUnit reg11(clock, regEnable[11], ctrl_reset, data_writeReg, bus11);
	registerUnit reg12(clock, regEnable[12], ctrl_reset, data_writeReg, bus12);
	registerUnit reg13(clock, regEnable[13], ctrl_reset, data_writeReg, bus13);
	registerUnit reg14(clock, regEnable[14], ctrl_reset, data_writeReg, bus14);
	registerUnit reg15(clock, regEnable[15], ctrl_reset, data_writeReg, bus15);
	registerUnit reg16(clock, regEnable[16], ctrl_reset, data_writeReg, bus16);
	registerUnit reg17(clock, regEnable[17], ctrl_reset, data_writeReg, bus17);
	registerUnit reg18(clock, regEnable[18], ctrl_reset, data_writeReg, bus18);
	registerUnit reg19(clock, regEnable[19], ctrl_reset, data_writeReg, bus19);
	registerUnit reg20(clock, regEnable[20], ctrl_reset, data_writeReg, bus20);
	registerUnit reg21(clock, regEnable[21], ctrl_reset, data_writeReg, bus21);
	registerUnit reg22(clock, regEnable[22], ctrl_reset, data_writeReg, bus22);
	registerUnit reg23(clock, regEnable[23], ctrl_reset, data_writeReg, bus23);
	registerUnit reg24(clock, regEnable[24], ctrl_reset, data_writeReg, bus24);
	registerUnit reg25(clock, regEnable[25], ctrl_reset, data_writeReg, bus25);
	registerUnit reg26(clock, regEnable[26], ctrl_reset, data_writeReg, bus26);
	registerUnit reg27(clock, regEnable[27], ctrl_reset, data_writeReg, bus27);
	registerUnit reg28(clock, regEnable[28], ctrl_reset, data_writeReg, bus28);
	registerUnit reg29(clock, regEnable[29], ctrl_reset, data_writeReg, bus29);
	registerUnit reg30(clock, regEnable[30], ctrl_reset, data_writeReg, bus30);
	registerUnit reg31(clock, regEnable[31], ctrl_reset, data_writeReg, bus31);
	
	mux muxA(data_readRegA, ctrl_readRegA, 
					bus0,  bus1,  bus2,  bus3,  bus4,  bus5,  bus6,  bus7,
					bus8,  bus9,  bus10, bus11, bus12, bus13, bus14, bus15,
					bus16, bus17, bus18, bus19, bus20, bus21, bus22, bus23,
					bus24, bus25, bus26, bus27, bus28, bus29, bus30, bus31);
					
	mux muxB(data_readRegB, ctrl_readRegB, 
					bus0,  bus1,  bus2,  bus3,  bus4,  bus5,  bus6,  bus7,
					bus8,  bus9,  bus10, bus11, bus12, bus13, bus14, bus15,
					bus16, bus17, bus18, bus19, bus20, bus21, bus22, bus23,
					bus24, bus25, bus26, bus27, bus28, bus29, bus30, bus31);
				
endmodule


module registerUnit (clk, en, rst, writeData, readData);

	input clk, en, rst;
	input [31:0] writeData;
	
	output [31:0] readData;

	dflipflopReg flops [31:0] (readData, writeData, clk, en, rst);
	
endmodule

module mux16input (out, s, in0,  in1,  in2,  in3,  in4,  in5,  in6,  in7,
									  in8,  in9,  in10, in11, in12, in13, in14, in15);
	
	input [31:0] in0,  in1,  in2,  in3,  in4,  in5,  in6,  in7,
					 in8,  in9,  in10, in11, in12, in13, in14, in15;
	input [3:0] s;
	output [31:0] out;
	
	wire [31:0] mux0_7out, mux8_15out;
	
	mux8input mux0_7 (mux0_7out,  s[2:0], in0,  in1,  in2,  in3,  in4,  in5,  in6,  in7);
	mux8input mux8_15(mux8_15out, s[2:0], in8,  in9,  in10, in11, in12, in13, in14, in15);
	mux2input muxout(out, s[3], mux0_7out, mux8_15out);
	
endmodule


module mux8input (out, s, in0, in1, in2, in3, in4, in5, in6, in7);
	
	input [31:0] in0, in1, in2, in3, in4, in5, in6, in7;
	input [2:0] s;
	output [31:0] out;
	
	wire [31:0] mux0_3out, mux4_7out;
	
	mux4input mux0_3(mux0_3out, s[1:0], in0, in1, in2, in3);
	mux4input mux4_7(mux4_7out, s[1:0], in4, in5, in6, in7);
	mux2input muxout(out, s[2], mux0_3out, mux4_7out);
	
endmodule


module mux4input (out, s, in0, in1, in2, in3);
	
	input [31:0] in0, in1, in2, in3;
	input [1:0] s;
	output [31:0] out;
	
	wire [31:0] mux01out, mux23out;
	
	mux2input mux01(mux01out, s[0], in0, in1);
	mux2input mux23(mux23out, s[0], in2, in3);
	mux2input mux0123(out, s[1], mux01out, mux23out);
	
endmodule


module mux2input(out, s, ina, inb);

	input [31:0] ina, inb;
	input s;
	output [31:0] out;
	
	wire [31:0] andA, andB;
	wire notS;
	
	not inverter(notS, s);
	
	and andAs [31:0] (andA, ina, notS);
	
	and andBs [31:0] (andB, inb, s);
	
	or ors [31:0] (out, andA, andB);
	
endmodule


module mux(out, s, in0,  in1,  in2,  in3,  in4,  in5,  in6,  in7,
							in8,  in9,  in10, in11, in12, in13, in14, in15,
							in16, in17, in18, in19, in20, in21, in22, in23,
							in24, in25, in26, in27, in28, in29, in30, in31);

	input [31:0] in0,  in1,  in2,  in3,  in4,  in5,  in6,  in7,
					 in8,  in9,  in10, in11, in12, in13, in14, in15,
					 in16, in17, in18, in19, in20, in21, in22, in23,
					 in24, in25, in26, in27, in28, in29, in30, in31;
	input [4:0] s;
	
	output [31:0] out;
	
	wire [31:0] mux0_15out, mux16_31out;
			
	mux16input mux0_15  (mux0_15out,   s[3:0], in0,  in1,  in2,  in3,  in4,  in5,  in6,  in7, in8,  in9,  in10, in11, in12, in13, in14, in15);
	mux16input mux16_31 (mux16_31out, s[3:0], in16, in17, in18, in19, in20, in21, in22, in23, in24, in25, in26, in27, in28, in29, in30, in31);
	
	mux2input muxout (out, s[4], mux0_15out, mux16_31out);

					
endmodule


module dflipflopReg (q, d, clock, en, reset);

	input d, clock, en, reset;
	output q;

	reg q;
	
	always @(posedge clock or posedge reset) begin
     if (reset)
        q <= 1'b0;
	  else
			if(en)
				q <= d;
	end
endmodule


module decoder(out, in);
	input [4:0] in;
	output [31:0] out;

	wire [4:0] notIn;
	
	not inverters [4:0] (notIn, in);
	
	and a0 (out[0],  notIn[4], notIn[3], notIn[2], notIn[1], notIn[0]);	//00000
	and a1 (out[1],  notIn[4], notIn[3], notIn[2], notIn[1],    in[0]);	//00001
	and a2 (out[2],  notIn[4], notIn[3], notIn[2],    in[1], notIn[0]);	//00010
	and a3 (out[3],  notIn[4], notIn[3], notIn[2],    in[1],    in[0]);	//00011
	and a4 (out[4],  notIn[4], notIn[3],    in[2], notIn[1], notIn[0]);	//00100
	and a5 (out[5],  notIn[4], notIn[3],    in[2], notIn[1],    in[0]);	//00101
	and a6 (out[6],  notIn[4], notIn[3],    in[2],    in[1], notIn[0]);	//00110
	and a7 (out[7],  notIn[4], notIn[3],    in[2],    in[1],    in[0]);	//00111
	and a8 (out[8],  notIn[4],    in[3], notIn[2], notIn[1], notIn[0]);	//01000
	and a9 (out[9],  notIn[4],    in[3], notIn[2], notIn[1],    in[0]);	//01001
	and a10(out[10], notIn[4],    in[3], notIn[2],    in[1], notIn[0]);	//01010
	and a11(out[11], notIn[4],    in[3], notIn[2],    in[1],    in[0]);	//01011
	and a12(out[12], notIn[4],    in[3],    in[2], notIn[1], notIn[0]);	//01100
	and a13(out[13], notIn[4],    in[3],    in[2], notIn[1],    in[0]);	//01101
	and a14(out[14], notIn[4],    in[3],    in[2],    in[1], notIn[0]);	//01110
	and a15(out[15], notIn[4],    in[3],    in[2],    in[1],    in[0]);	//01111
	and a16(out[16],    in[4], notIn[3], notIn[2], notIn[1], notIn[0]);	//10000
	and a17(out[17],    in[4], notIn[3], notIn[2], notIn[1],    in[0]);	//10001
	and a18(out[18],    in[4], notIn[3], notIn[2],    in[1], notIn[0]);	//10010
	and a19(out[19],    in[4], notIn[3], notIn[2],    in[1],    in[0]);	//10011
	and a20(out[20],    in[4], notIn[3],    in[2], notIn[1], notIn[0]);	//10100
	and a21(out[21],    in[4], notIn[3],    in[2], notIn[1],    in[0]);	//10101
	and a22(out[22],    in[4], notIn[3],    in[2],    in[1], notIn[0]);	//10110
	and a23(out[23],    in[4], notIn[3],    in[2],    in[1],    in[0]);	//10111
	and a24(out[24],    in[4],    in[3], notIn[2], notIn[1], notIn[0]);	//11000
	and a25(out[25],    in[4],    in[3], notIn[2], notIn[1],    in[0]);	//11001
	and a26(out[26],    in[4],    in[3], notIn[2],    in[1], notIn[0]);	//11010
	and a27(out[27],    in[4],    in[3], notIn[2],    in[1],    in[0]);	//11011
	and a28(out[28],    in[4],    in[3],    in[2], notIn[1], notIn[0]);	//11100
	and a29(out[29],    in[4],    in[3],    in[2], notIn[1],    in[0]);	//11101
	and a30(out[30],    in[4],    in[3],    in[2],    in[1], notIn[0]);	//11110
	and a31(out[31],    in[4],    in[3],    in[2],    in[1],    in[0]);	//11111
		
endmodule
