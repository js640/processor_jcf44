module alu(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan, overflow);

   input [31:0] data_operandA, data_operandB;
   input [4:0] ctrl_ALUopcode, ctrl_shiftamt;

   output [31:0] data_result;
   output isNotEqual, isLessThan, overflow;
	wire lessWire;
	wire [31:0] adderOut, andOut, orOut, leftOut, rightOut;

   full32AdderALU adderBlock(adderOut, overflow, data_operandA, data_operandB, ctrl_ALUopcode[0]);	
	
	and ander [31:0] (andOut, data_operandA, data_operandB);
	
	or orer   [31:0] (orOut,  data_operandA, data_operandB);
	
	barrelShifterLeft leftshifter(leftOut, data_operandA, ctrl_shiftamt);
	
	barrelShifterRight rightshifter(rightOut, data_operandA, ctrl_shiftamt);
	
	mux8inputALU mux(data_result, ctrl_ALUopcode[2:0], adderOut, adderOut, andOut, orOut, leftOut, rightOut, adderOut, adderOut);
	
	or notEqual(isNotEqual, adderOut);
	
	and lessCheck(lessWire, ctrl_ALUopcode[0], overflow);
	or lessThan(isLessThan, lessWire, adderOut[31]);

endmodule


module shiftRight16 (out, in, en);
	input [31:0] in;
	input en;
	
	output [31:0] out;
	
	mux2inputALU mux0 (out[0], en, in[0], in[16]);
	mux2inputALU mux1 (out[1], en, in[1], in[17]);
	mux2inputALU mux2 (out[2], en, in[2], in[18]);
	mux2inputALU mux3 (out[3], en, in[3], in[19]);
	mux2inputALU mux4 (out[4], en, in[4], in[20]);
	mux2inputALU mux5 (out[5], en, in[5], in[21]);
	mux2inputALU mux6 (out[6], en, in[6], in[22]);
	mux2inputALU mux7 (out[7], en, in[7], in[23]);
	mux2inputALU mux8 (out[8], en, in[8], in[24]);
	mux2inputALU mux9 (out[9], en, in[9], in[25]);
	mux2inputALU mux10(out[10], en, in[10], in[26]);
	mux2inputALU mux11(out[11], en, in[11], in[27]);
	mux2inputALU mux12(out[12], en, in[12], in[28]);
	mux2inputALU mux13(out[13], en, in[13], in[29]);
	mux2inputALU mux14(out[14], en, in[14], in[30]);
	mux2inputALU mux15(out[15], en, in[15], in[31]);
	mux2inputALU mux16(out[16], en, in[16], in[31]);
	mux2inputALU mux17(out[17], en, in[17], in[31]);
	mux2inputALU mux18(out[18], en, in[18], in[31]);
	mux2inputALU mux19(out[19], en, in[19], in[31]);
	mux2inputALU mux20(out[20], en, in[20], in[31]);
	mux2inputALU mux21(out[21], en, in[21], in[31]);
	mux2inputALU mux22(out[22], en, in[22], in[31]);
	mux2inputALU mux23(out[23], en, in[23], in[31]);
	mux2inputALU mux24(out[24], en, in[24], in[31]);
	mux2inputALU mux25(out[25], en, in[25], in[31]);
	mux2inputALU mux26(out[26], en, in[26], in[31]);
	mux2inputALU mux27(out[27], en, in[27], in[31]);
	mux2inputALU mux28(out[28], en, in[28], in[31]);
	mux2inputALU mux29(out[29], en, in[29], in[31]);
	mux2inputALU mux30(out[30], en, in[30], in[31]);
	mux2inputALU mux31(out[31], en, in[31], in[31]);

endmodule


module shiftRight8 (out, in, en);
	input [31:0] in;
	input en;
	
	output [31:0] out;
	
	mux2inputALU mux0 (out[0], en, in[0], in[8]);
	mux2inputALU mux1 (out[1], en, in[1], in[9]);
	mux2inputALU mux2 (out[2], en, in[2], in[10]);
	mux2inputALU mux3 (out[3], en, in[3], in[11]);
	mux2inputALU mux4 (out[4], en, in[4], in[12]);
	mux2inputALU mux5 (out[5], en, in[5], in[13]);
	mux2inputALU mux6 (out[6], en, in[6], in[14]);
	mux2inputALU mux7 (out[7], en, in[7], in[15]);
	mux2inputALU mux8 (out[8], en, in[8], in[16]);
	mux2inputALU mux9 (out[9], en, in[9], in[17]);
	mux2inputALU mux10(out[10], en, in[10], in[18]);
	mux2inputALU mux11(out[11], en, in[11], in[19]);
	mux2inputALU mux12(out[12], en, in[12], in[20]);
	mux2inputALU mux13(out[13], en, in[13], in[21]);
	mux2inputALU mux14(out[14], en, in[14], in[22]);
	mux2inputALU mux15(out[15], en, in[15], in[23]);
	mux2inputALU mux16(out[16], en, in[16], in[24]);
	mux2inputALU mux17(out[17], en, in[17], in[25]);
	mux2inputALU mux18(out[18], en, in[18], in[26]);
	mux2inputALU mux19(out[19], en, in[19], in[27]);
	mux2inputALU mux20(out[20], en, in[20], in[28]);
	mux2inputALU mux21(out[21], en, in[21], in[29]);
	mux2inputALU mux22(out[22], en, in[22], in[30]);
	mux2inputALU mux23(out[23], en, in[23], in[31]);
	mux2inputALU mux24(out[24], en, in[24], in[31]);
	mux2inputALU mux25(out[25], en, in[25], in[31]);
	mux2inputALU mux26(out[26], en, in[26], in[31]);
	mux2inputALU mux27(out[27], en, in[27], in[31]);
	mux2inputALU mux28(out[28], en, in[28], in[31]);
	mux2inputALU mux29(out[29], en, in[29], in[31]);
	mux2inputALU mux30(out[30], en, in[30], in[31]);
	mux2inputALU mux31(out[31], en, in[31], in[31]);

endmodule


module shiftRight4 (out, in, en);
	input [31:0] in;
	input en;
	
	output [31:0] out;
	
	mux2inputALU mux0 (out[0], en, in[0], in[4]);
	mux2inputALU mux1 (out[1], en, in[1], in[5]);
	mux2inputALU mux2 (out[2], en, in[2], in[6]);
	mux2inputALU mux3 (out[3], en, in[3], in[7]);
	mux2inputALU mux4 (out[4], en, in[4], in[8]);
	mux2inputALU mux5 (out[5], en, in[5], in[9]);
	mux2inputALU mux6 (out[6], en, in[6], in[10]);
	mux2inputALU mux7 (out[7], en, in[7], in[11]);
	mux2inputALU mux8 (out[8], en, in[8], in[12]);
	mux2inputALU mux9 (out[9], en, in[9], in[13]);
	mux2inputALU mux10(out[10], en, in[10], in[14]);
	mux2inputALU mux11(out[11], en, in[11], in[15]);
	mux2inputALU mux12(out[12], en, in[12], in[16]);
	mux2inputALU mux13(out[13], en, in[13], in[17]);
	mux2inputALU mux14(out[14], en, in[14], in[18]);
	mux2inputALU mux15(out[15], en, in[15], in[19]);
	mux2inputALU mux16(out[16], en, in[16], in[20]);
	mux2inputALU mux17(out[17], en, in[17], in[21]);
	mux2inputALU mux18(out[18], en, in[18], in[22]);
	mux2inputALU mux19(out[19], en, in[19], in[23]);
	mux2inputALU mux20(out[20], en, in[20], in[24]);
	mux2inputALU mux21(out[21], en, in[21], in[25]);
	mux2inputALU mux22(out[22], en, in[22], in[26]);
	mux2inputALU mux23(out[23], en, in[23], in[27]);
	mux2inputALU mux24(out[24], en, in[24], in[28]);
	mux2inputALU mux25(out[25], en, in[25], in[29]);
	mux2inputALU mux26(out[26], en, in[26], in[30]);
	mux2inputALU mux27(out[27], en, in[27], in[31]);
	mux2inputALU mux28(out[28], en, in[28], in[31]);
	mux2inputALU mux29(out[29], en, in[29], in[31]);
	mux2inputALU mux30(out[30], en, in[30], in[31]);
	mux2inputALU mux31(out[31], en, in[31], in[31]);

endmodule


module shiftRight2 (out, in, en);
	input [31:0] in;
	input en;
	
	output [31:0] out;
	
	mux2inputALU mux0 (out[0], en, in[0], in[2]);
	mux2inputALU mux1 (out[1], en, in[1], in[3]);
	mux2inputALU mux2 (out[2], en, in[2], in[4]);
	mux2inputALU mux3 (out[3], en, in[3], in[5]);
	mux2inputALU mux4 (out[4], en, in[4], in[6]);
	mux2inputALU mux5 (out[5], en, in[5], in[7]);
	mux2inputALU mux6 (out[6], en, in[6], in[8]);
	mux2inputALU mux7 (out[7], en, in[7], in[9]);
	mux2inputALU mux8 (out[8], en, in[8], in[10]);
	mux2inputALU mux9 (out[9], en, in[9], in[11]);
	mux2inputALU mux10(out[10], en, in[10], in[12]);
	mux2inputALU mux11(out[11], en, in[11], in[13]);
	mux2inputALU mux12(out[12], en, in[12], in[14]);
	mux2inputALU mux13(out[13], en, in[13], in[15]);
	mux2inputALU mux14(out[14], en, in[14], in[16]);
	mux2inputALU mux15(out[15], en, in[15], in[17]);
	mux2inputALU mux16(out[16], en, in[16], in[18]);
	mux2inputALU mux17(out[17], en, in[17], in[19]);
	mux2inputALU mux18(out[18], en, in[18], in[20]);
	mux2inputALU mux19(out[19], en, in[19], in[21]);
	mux2inputALU mux20(out[20], en, in[20], in[22]);
	mux2inputALU mux21(out[21], en, in[21], in[23]);
	mux2inputALU mux22(out[22], en, in[22], in[24]);
	mux2inputALU mux23(out[23], en, in[23], in[25]);
	mux2inputALU mux24(out[24], en, in[24], in[26]);
	mux2inputALU mux25(out[25], en, in[25], in[27]);
	mux2inputALU mux26(out[26], en, in[26], in[28]);
	mux2inputALU mux27(out[27], en, in[27], in[29]);
	mux2inputALU mux28(out[28], en, in[28], in[30]);
	mux2inputALU mux29(out[29], en, in[29], in[31]);
	mux2inputALU mux30(out[30], en, in[30], in[31]);
	mux2inputALU mux31(out[31], en, in[31], in[31]);

endmodule


module shiftRight1 (out, in, en);
	input [31:0] in;
	input en;
	
	output [31:0] out;
	
	mux2inputALU mux0 (out[0], en, in[0], in[1]);
	mux2inputALU mux1 (out[1], en, in[1], in[2]);
	mux2inputALU mux2 (out[2], en, in[2], in[3]);
	mux2inputALU mux3 (out[3], en, in[3], in[4]);
	mux2inputALU mux4 (out[4], en, in[4], in[5]);
	mux2inputALU mux5 (out[5], en, in[5], in[6]);
	mux2inputALU mux6 (out[6], en, in[6], in[7]);
	mux2inputALU mux7 (out[7], en, in[7], in[8]);
	mux2inputALU mux8 (out[8], en, in[8], in[9]);
	mux2inputALU mux9 (out[9], en, in[9], in[10]);
	mux2inputALU mux10(out[10], en, in[10], in[11]);
	mux2inputALU mux11(out[11], en, in[11], in[12]);
	mux2inputALU mux12(out[12], en, in[12], in[13]);
	mux2inputALU mux13(out[13], en, in[13], in[14]);
	mux2inputALU mux14(out[14], en, in[14], in[15]);
	mux2inputALU mux15(out[15], en, in[15], in[16]);
	mux2inputALU mux16(out[16], en, in[16], in[17]);
	mux2inputALU mux17(out[17], en, in[17], in[18]);
	mux2inputALU mux18(out[18], en, in[18], in[19]);
	mux2inputALU mux19(out[19], en, in[19], in[20]);
	mux2inputALU mux20(out[20], en, in[20], in[21]);
	mux2inputALU mux21(out[21], en, in[21], in[22]);
	mux2inputALU mux22(out[22], en, in[22], in[23]);
	mux2inputALU mux23(out[23], en, in[23], in[24]);
	mux2inputALU mux24(out[24], en, in[24], in[25]);
	mux2inputALU mux25(out[25], en, in[25], in[26]);
	mux2inputALU mux26(out[26], en, in[26], in[27]);
	mux2inputALU mux27(out[27], en, in[27], in[28]);
	mux2inputALU mux28(out[28], en, in[28], in[29]);
	mux2inputALU mux29(out[29], en, in[29], in[30]);
	mux2inputALU mux30(out[30], en, in[30], in[31]);
	mux2inputALU mux31(out[31], en, in[31], in[31]);

endmodule


module shiftLeft16 (out, in, en);
	input [31:0] in;
	input en;
	
	output [31:0] out;
	
	mux2inputALU mux0 (out[0], en, in[0], 1'b0);
	mux2inputALU mux1 (out[1], en, in[1], 1'b0);
	mux2inputALU mux2 (out[2], en, in[2], 1'b0);
	mux2inputALU mux3 (out[3], en, in[3], 1'b0);
	mux2inputALU mux4 (out[4], en, in[4], 1'b0);
	mux2inputALU mux5 (out[5], en, in[5], 1'b0);
	mux2inputALU mux6 (out[6], en, in[6], 1'b0);
	mux2inputALU mux7 (out[7], en, in[7], 1'b0);
	mux2inputALU mux8 (out[8], en, in[8], 1'b0);
	mux2inputALU mux9 (out[9], en, in[9], 1'b0);
	mux2inputALU mux10(out[10], en, in[10], 1'b0);
	mux2inputALU mux11(out[11], en, in[11], 1'b0);
	mux2inputALU mux12(out[12], en, in[12], 1'b0);
	mux2inputALU mux13(out[13], en, in[13], 1'b0);
	mux2inputALU mux14(out[14], en, in[14], 1'b0);
	mux2inputALU mux15(out[15], en, in[15], 1'b0);
	mux2inputALU mux16(out[16], en, in[16], in[0]);
	mux2inputALU mux17(out[17], en, in[17], in[1]);
	mux2inputALU mux18(out[18], en, in[18], in[2]);
	mux2inputALU mux19(out[19], en, in[19], in[3]);
	mux2inputALU mux20(out[20], en, in[20], in[4]);
	mux2inputALU mux21(out[21], en, in[21], in[5]);
	mux2inputALU mux22(out[22], en, in[22], in[6]);
	mux2inputALU mux23(out[23], en, in[23], in[7]);
	mux2inputALU mux24(out[24], en, in[24], in[8]);
	mux2inputALU mux25(out[25], en, in[25], in[9]);
	mux2inputALU mux26(out[26], en, in[26], in[10]);
	mux2inputALU mux27(out[27], en, in[27], in[11]);
	mux2inputALU mux28(out[28], en, in[28], in[12]);
	mux2inputALU mux29(out[29], en, in[29], in[13]);
	mux2inputALU mux30(out[30], en, in[30], in[14]);
	mux2inputALU mux31(out[31], en, in[31], in[15]);

endmodule



module shiftLeft8 (out, in, en);
	input [31:0] in;
	input en;
	
	output [31:0] out;
	
	mux2inputALU mux0 (out[0], en, in[0], 1'b0);
	mux2inputALU mux1 (out[1], en, in[1], 1'b0);
	mux2inputALU mux2 (out[2], en, in[2], 1'b0);
	mux2inputALU mux3 (out[3], en, in[3], 1'b0);
	mux2inputALU mux4 (out[4], en, in[4], 1'b0);
	mux2inputALU mux5 (out[5], en, in[5], 1'b0);
	mux2inputALU mux6 (out[6], en, in[6], 1'b0);
	mux2inputALU mux7 (out[7], en, in[7], 1'b0);
	mux2inputALU mux8 (out[8], en, in[8], in[0]);
	mux2inputALU mux9 (out[9], en, in[9], in[1]);
	mux2inputALU mux10(out[10], en, in[10], in[2]);
	mux2inputALU mux11(out[11], en, in[11], in[3]);
	mux2inputALU mux12(out[12], en, in[12], in[4]);
	mux2inputALU mux13(out[13], en, in[13], in[5]);
	mux2inputALU mux14(out[14], en, in[14], in[6]);
	mux2inputALU mux15(out[15], en, in[15], in[7]);
	mux2inputALU mux16(out[16], en, in[16], in[8]);
	mux2inputALU mux17(out[17], en, in[17], in[9]);
	mux2inputALU mux18(out[18], en, in[18], in[10]);
	mux2inputALU mux19(out[19], en, in[19], in[11]);
	mux2inputALU mux20(out[20], en, in[20], in[12]);
	mux2inputALU mux21(out[21], en, in[21], in[13]);
	mux2inputALU mux22(out[22], en, in[22], in[14]);
	mux2inputALU mux23(out[23], en, in[23], in[15]);
	mux2inputALU mux24(out[24], en, in[24], in[16]);
	mux2inputALU mux25(out[25], en, in[25], in[17]);
	mux2inputALU mux26(out[26], en, in[26], in[18]);
	mux2inputALU mux27(out[27], en, in[27], in[19]);
	mux2inputALU mux28(out[28], en, in[28], in[20]);
	mux2inputALU mux29(out[29], en, in[29], in[21]);
	mux2inputALU mux30(out[30], en, in[30], in[22]);
	mux2inputALU mux31(out[31], en, in[31], in[23]);

endmodule


module shiftLeft4 (out, in, en);
	input [31:0] in;
	input en;
	
	output [31:0] out;
	
	mux2inputALU mux0 (out[0], en, in[0], 1'b0);
	mux2inputALU mux1 (out[1], en, in[1], 1'b0);
	mux2inputALU mux2 (out[2], en, in[2], 1'b0);
	mux2inputALU mux3 (out[3], en, in[3], 1'b0);
	mux2inputALU mux4 (out[4], en, in[4], in[0]);
	mux2inputALU mux5 (out[5], en, in[5], in[1]);
	mux2inputALU mux6 (out[6], en, in[6], in[2]);
	mux2inputALU mux7 (out[7], en, in[7], in[3]);
	mux2inputALU mux8 (out[8], en, in[8], in[4]);
	mux2inputALU mux9 (out[9], en, in[9], in[5]);
	mux2inputALU mux10(out[10], en, in[10], in[6]);
	mux2inputALU mux11(out[11], en, in[11], in[7]);
	mux2inputALU mux12(out[12], en, in[12], in[8]);
	mux2inputALU mux13(out[13], en, in[13], in[9]);
	mux2inputALU mux14(out[14], en, in[14], in[10]);
	mux2inputALU mux15(out[15], en, in[15], in[11]);
	mux2inputALU mux16(out[16], en, in[16], in[12]);
	mux2inputALU mux17(out[17], en, in[17], in[13]);
	mux2inputALU mux18(out[18], en, in[18], in[14]);
	mux2inputALU mux19(out[19], en, in[19], in[15]);
	mux2inputALU mux20(out[20], en, in[20], in[16]);
	mux2inputALU mux21(out[21], en, in[21], in[17]);
	mux2inputALU mux22(out[22], en, in[22], in[18]);
	mux2inputALU mux23(out[23], en, in[23], in[19]);
	mux2inputALU mux24(out[24], en, in[24], in[20]);
	mux2inputALU mux25(out[25], en, in[25], in[21]);
	mux2inputALU mux26(out[26], en, in[26], in[22]);
	mux2inputALU mux27(out[27], en, in[27], in[23]);
	mux2inputALU mux28(out[28], en, in[28], in[24]);
	mux2inputALU mux29(out[29], en, in[29], in[25]);
	mux2inputALU mux30(out[30], en, in[30], in[26]);
	mux2inputALU mux31(out[31], en, in[31], in[27]);

endmodule


module shiftLeft2 (out, in, en);
	input [31:0] in;
	input en;
	
	output [31:0] out;
	
	mux2inputALU mux0 (out[0], en, in[0], 1'b0);
	mux2inputALU mux1 (out[1], en, in[1], 1'b0);
	mux2inputALU mux2 (out[2], en, in[2], in[0]);
	mux2inputALU mux3 (out[3], en, in[3], in[1]);
	mux2inputALU mux4 (out[4], en, in[4], in[2]);
	mux2inputALU mux5 (out[5], en, in[5], in[3]);
	mux2inputALU mux6 (out[6], en, in[6], in[4]);
	mux2inputALU mux7 (out[7], en, in[7], in[5]);
	mux2inputALU mux8 (out[8], en, in[8], in[6]);
	mux2inputALU mux9 (out[9], en, in[9], in[7]);
	mux2inputALU mux10(out[10], en, in[10], in[8]);
	mux2inputALU mux11(out[11], en, in[11], in[9]);
	mux2inputALU mux12(out[12], en, in[12], in[10]);
	mux2inputALU mux13(out[13], en, in[13], in[11]);
	mux2inputALU mux14(out[14], en, in[14], in[12]);
	mux2inputALU mux15(out[15], en, in[15], in[13]);
	mux2inputALU mux16(out[16], en, in[16], in[14]);
	mux2inputALU mux17(out[17], en, in[17], in[15]);
	mux2inputALU mux18(out[18], en, in[18], in[16]);
	mux2inputALU mux19(out[19], en, in[19], in[17]);
	mux2inputALU mux20(out[20], en, in[20], in[18]);
	mux2inputALU mux21(out[21], en, in[21], in[19]);
	mux2inputALU mux22(out[22], en, in[22], in[20]);
	mux2inputALU mux23(out[23], en, in[23], in[21]);
	mux2inputALU mux24(out[24], en, in[24], in[22]);
	mux2inputALU mux25(out[25], en, in[25], in[23]);
	mux2inputALU mux26(out[26], en, in[26], in[24]);
	mux2inputALU mux27(out[27], en, in[27], in[25]);
	mux2inputALU mux28(out[28], en, in[28], in[26]);
	mux2inputALU mux29(out[29], en, in[29], in[27]);
	mux2inputALU mux30(out[30], en, in[30], in[28]);
	mux2inputALU mux31(out[31], en, in[31], in[29]);

endmodule


module shiftLeft1 (out, in, en);
	input [31:0] in;
	input en;
	
	output [31:0] out;
	
	mux2inputALU mux0 (out[0], en, in[0], 1'b0);
	mux2inputALU mux1 (out[1], en, in[1], in[0]);
	mux2inputALU mux2 (out[2], en, in[2], in[1]);
	mux2inputALU mux3 (out[3], en, in[3], in[2]);
	mux2inputALU mux4 (out[4], en, in[4], in[3]);
	mux2inputALU mux5 (out[5], en, in[5], in[4]);
	mux2inputALU mux6 (out[6], en, in[6], in[5]);
	mux2inputALU mux7 (out[7], en, in[7], in[6]);
	mux2inputALU mux8 (out[8], en, in[8], in[7]);
	mux2inputALU mux9 (out[9], en, in[9], in[8]);
	mux2inputALU mux10(out[10], en, in[10], in[9]);
	mux2inputALU mux11(out[11], en, in[11], in[10]);
	mux2inputALU mux12(out[12], en, in[12], in[11]);
	mux2inputALU mux13(out[13], en, in[13], in[12]);
	mux2inputALU mux14(out[14], en, in[14], in[13]);
	mux2inputALU mux15(out[15], en, in[15], in[14]);
	mux2inputALU mux16(out[16], en, in[16], in[15]);
	mux2inputALU mux17(out[17], en, in[17], in[16]);
	mux2inputALU mux18(out[18], en, in[18], in[17]);
	mux2inputALU mux19(out[19], en, in[19], in[18]);
	mux2inputALU mux20(out[20], en, in[20], in[19]);
	mux2inputALU mux21(out[21], en, in[21], in[20]);
	mux2inputALU mux22(out[22], en, in[22], in[21]);
	mux2inputALU mux23(out[23], en, in[23], in[22]);
	mux2inputALU mux24(out[24], en, in[24], in[23]);
	mux2inputALU mux25(out[25], en, in[25], in[24]);
	mux2inputALU mux26(out[26], en, in[26], in[25]);
	mux2inputALU mux27(out[27], en, in[27], in[26]);
	mux2inputALU mux28(out[28], en, in[28], in[27]);
	mux2inputALU mux29(out[29], en, in[29], in[28]);
	mux2inputALU mux30(out[30], en, in[30], in[29]);
	mux2inputALU mux31(out[31], en, in[31], in[30]);

endmodule


module mux8inputALU (out, s, in0, in1, in2, in3, in4, in5, in6, in7);
	
	input [31:0] in0, in1, in2, in3, in4, in5, in6, in7;
	input [2:0] s;
	output [31:0] out;
	
	wire [31:0] mux0_3out, mux4_7out;
	
	mux4inputALU mux0_3(mux0_3out, s[1:0], in0, in1, in2, in3);
	mux4inputALU mux4_7(mux4_7out, s[1:0], in4, in5, in6, in7);
	mux2inputBusALU muxout(out, s[2], mux0_3out, mux4_7out);
	
endmodule


module mux4inputALU (out, s, in0, in1, in2, in3);
	
	input [31:0] in0, in1, in2, in3;
	input [1:0] s;
	output [31:0] out;
	
	wire [31:0] mux01out, mux23out;
	
	mux2inputBusALU mux01(mux01out, s[0], in0, in1);
	mux2inputBusALU mux23(mux23out, s[0], in2, in3);
	mux2inputBusALU mux0123(out, s[1], mux01out, mux23out);
	
endmodule



module mux2inputBusALU(out, s, ina, inb);

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


module mux2inputALU(out, s, ina, inb);

	input ina, inb;
	input s;
	output out;
	
	wire andA, andB;
	wire notS;
	
	not inverter(notS, s);
	
	and andAGate(andA, ina, notS);
	
	and andBGate(andB, inb, s);
	
	or orAB(out, andA, andB);
	
endmodule


module full32AdderALU(out, c, a, b, sub);
	input [31:0] a, b;
	input sub;
	output [31:0] out;
	output c;

	wire [3:0] G, P;
	wire [31:0] bin;
	wire c8, c16, c24, c32, P0c, P1G0, P1P0c, P2G1, P2P1G0, P2P1P0c, P3G2, P3P2G1, P3P2P1G0, P3P2P1P0c, notA, notB, notC;
	wire over1, over2, over3, notSub;
	
	xor xorB [31:0] (bin, b, sub);
	
	cla8Block cla8Block0(out[7:0], G[0], P[0], a[7:0], bin[7:0], sub);
	
	and and0(P0c, P[0], sub);
	or or0(c8, G[0], P0c);
	
	cla8Block cla8Block1(out[15:8], G[1], P[1], a[15:8], bin[15:8], c8);
	
	and and1(P1P0c, P[1], P0c);
	and and2(P1G0, P[1], G[0]);
	or or1(c16, G[1], P1G0, P1P0c);
	
	cla8Block cla8Block2(out[23:16], G[2], P[2], a[23:16], bin[23:16], c16);
	
	and and3(P2P1P0c, P[2], P1P0c);
	and and4(P2P1G0, P[2], P1G0);
	and and5(P2G1, P[2], G[1]);
	or or2(c24, G[2], P2G1, P2P1G0, P2P1P0c);
	
	cla8Block cla8Block3(out[31:24], G[3], P[3], a[31:24], bin[31:24], c24);
	
	and and6(P3P2P1P0c, P[3], P2P1P0c);
	and and7(P3P2P1G0, P[3], P2P1G0);
	and and8(P3P2G1, P[3], P2G1);
	and and9(P3G2, P[3], G[2]);
	or or3(c32, G[3], P3G2, P3P2G1, P3P2P1G0, P3P2P1P0c);

	not anot(notA, a[31]);
	not bnot(notB, b[31]);
	not cnot(notC, out[31]);
	not nnot(notSub, sub);
	
	and and10(over1, notA, notB, out[31], notSub);
	and and11(over2, a[31], b[31], notC, notSub);
	and and12(over3, a[31], notB, notC);
	or or4(c, over1, over2, over3);
	
endmodule


module cla8Block(s, bigG, bigP, a, b, c);
	input [7:0] a, b;
	input c;
	output [7:0] s;
	output bigG, bigP;
	wire p0c, c1, c2, c3, c4, c5, c6, c7, p1p0c, p1g0, p2p1p0c, p2p1g0, p2g1, p3p2p1p0c, p3p2p1g0, p3p2g1, p3g2;
	wire p4p3p2p1p0c, p4p3p2p1g0, p4p3p2g1, p4p3g2, p4g3, p5p4p3p2p1p0c, p5p4p3p2p1g0, p5p4p3p2g1, p5p4p3g2, p5p4g3, p5g4;
	wire p6p5p4p3p2p1p0c, p6p5p4p3p2p1g0, p6p5p4p3p2g1, p6p5p4p3g2, p6p5p4g3, p6p5g4, p6g5, p7g6;
	wire p7p6p5p4p3p2p1g0, p7p6p5p4p3p2g1, p7p6p5p4p3g2, p7p6p5p4g3, p7p6p5g4, p7p6g5;
	wire [7:0] g, p;
	
	adderALU adder0(s[0], p[0], g[0], a[0], b[0], c);
	
	and and0(p0c, p[0], c);
	or   or0(c1, p0c, g[0]);
	
	adderALU adder1(s[1], p[1], g[1], a[1], b[1], c1);
	
	and and1(p1p0c, p0c, p[1]);
	and and2(p1g0, p[1], g[0]);
	or or1(c2, g[1], p1g0, p1p0c);
	
	adderALU adder2(s[2], p[2], g[2], a[2], b[2], c2);
	
	and and3(p2p1p0c, p[2], p1p0c);
	and and4(p2p1g0, p[2], p1g0);
	and and5(p2g1, p[2], g[1]);
	or or2(c3, g[2], p2g1, p2p1g0, p2p1p0c);
	
	adderALU adder3(s[3], p[3], g[3], a[3], b[3], c3);
	
	and and6(p3p2p1p0c, p[3], p2p1p0c);
	and and7(p3p2p1g0, p[3], p2p1g0);
	and and8(p3p2g1, p[3], p2g1);
	and and9(p3g2, p[3], g[2]);
	or or3(c4, p3p2p1p0c, p3p2p1g0, p3p2g1, p3g2, g[3]);
	
	adderALU adder4(s[4], p[4], g[4], a[4], b[4], c4);
	
	and and10(p4p3p2p1p0c, p[4], p3p2p1p0c);
	and and11(p4p3p2p1g0, p[4], p3p2p1g0);
	and and12(p4p3p2g1, p[4], p3p2g1);
	and and13(p4p3g2, p[4], p3g2);
	and and14(p4g3, p[4], g[3]);
	or or4(c5, p4p3p2p1p0c, p4p3p2p1g0, p4p3p2g1, p4p3g2, p4g3, g[4]);
	
	adderALU adder5(s[5], p[5], g[5], a[5], b[5], c5);
	
	and and15(p5p4p3p2p1p0c, p[5], p4p3p2p1p0c);
	and and16(p5p4p3p2p1g0, p[5], p4p3p2p1g0);
	and and17(p5p4p3p2g1, p[5], p4p3p2g1);
	and and18(p5p4p3g2, p[5], p4p3g2);
	and and19(p5p4g3, p[5], p4g3);
	and and20(p5g4, p[5], g[4]);
	or or5(c6, p5p4p3p2p1p0c, p5p4p3p2p1g0, p5p4p3p2g1, p5p4p3g2, p5p4g3, p5g4, g[5]);
	
	adderALU adder6(s[6], p[6], g[6], a[6], b[6], c6);
	
	and and21(p6p5p4p3p2p1p0c, p[6], p5p4p3p2p1p0c);
	and and22(p6p5p4p3p2p1g0, p[6], p5p4p3p2p1g0);
	and and23(p6p5p4p3p2g1, p[6], p5p4p3p2g1);
	and and24(p6p5p4p3g2, p[6], p5p4p3g2);
	and and25(p6p5p4g3, p[6], p5p4g3);
	and and26(p6p5g4, p[6], p5g4);
	and and27(p6g5, p[6], g[5]);
	or or6(c7, p6p5p4p3p2p1p0c, p6p5p4p3p2p1g0, p6p5p4p3p2g1, p6p5p4p3g2, p6p5p4g3, p6p5g4, p6g5, g[6]);
	
	adderALU adder7(s[7], p[7], g[7], a[7], b[7], c7);
	
	and and28(bigP, p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0]);
	
	and and29(p7p6p5p4p3p2p1g0, p[7], p6p5p4p3p2p1g0);
	and and30(p7p6p5p4p3p2g1, p[7], p6p5p4p3p2g1);
	and and31(p7p6p5p4p3g2, p[7], p6p5p4p3g2);
	and and32(p7p6p5p4g3, p[7], p6p5p4g3);
	and and33(p7p6p5g4, p[7], p6p5g4);
	and and34(p7p6g5, p[7], p6g5);
	and and35(p7g6, p[7], g[6]);
	or or7(bigG, g[7], p7g6, p7p6g5, p7p6p5g4, p7p6p5p4g3, p7p6p5p4p3g2, p7p6p5p4p3p2g1, p7p6p5p4p3p2p1g0);
	
endmodule


module barrelShifterRight(out, in, shift_amt);

	input [31:0] in;
	input [4:0] shift_amt;
	output [31:0] out;
	
	wire [31:0] out1, out2, out4, out8;

	shiftRight1 shifter1(out1, in, shift_amt[0]);
	shiftRight2 shifter2(out2, out1, shift_amt[1]);
	shiftRight4 shifter4(out4, out2, shift_amt[2]);
	shiftRight8 shifter8(out8, out4, shift_amt[3]);
	shiftRight16 shifter16(out, out8, shift_amt[4]);

endmodule


module barrelShifterLeft(out, in, shift_amt);

	input [31:0] in;
	input [4:0] shift_amt;
	output [31:0] out;
	
	wire [31:0] out1, out2, out4, out8;

	shiftLeft1 shifter1(out1, in, shift_amt[0]);
	shiftLeft2 shifter2(out2, out1, shift_amt[1]);
	shiftLeft4 shifter4(out4, out2, shift_amt[2]);
	shiftLeft8 shifter8(out8, out4, shift_amt[3]);
	shiftLeft16 shifter16(out, out8, shift_amt[4]);

endmodule


module adderALU(out, p, g, a, b, cin);
	input a, b, cin;
	output out, p, g;
	
	xor outxor(out, a, b, cin);
	and andg(g, a, b);
	or orp(p, a, b);
	
endmodule