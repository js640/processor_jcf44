module multdiv(data_operandA, data_operandB, ctrl_MULT, ctrl_DIV, clock, data_result, data_exception, data_resultRDY);
   input [31:0] data_operandA, data_operandB;
   input ctrl_MULT, ctrl_DIV, clock;

   output [31:0] data_result;
   output data_exception, data_resultRDY;	
	
	wire mult_adder_en, mult_adder_sub, buffer_bit, product_enable, count_enable, exception, sign, en_int_1, en_int_2,
			isZeroA, isZeroB, noZero, notStateEn, n_enable, isMaxA, isMaxB, maxOverflow, exception1, state_enable, state,
			notState, rqb_enable, adderSub, gt_or_et, div_exception;
	wire [63:0] product_in, product_out, product_unshifted, product_shifted, rqb_unshifted, rqb_shifted;
	wire [31:0] adderOut, shifted_product_anded, shifted_product_ored, adderIn, rqb_in, p_in, rqbr, inverted_a, inverted_b, 
					in_a, in_b, uninverted_div_out, div_out;
	wire [5:0] count, countIncremented, notCount;
	
	
	// Make inputs positive for div, invert final product if necessary
	inverter32Mult inva(inverted_a, data_operandA, data_operandA[31]);
	inverter32Mult invb(inverted_b, data_operandB, data_operandB[31]);
	inverter32Mult invp(uninverted_div_out, rqb_shifted[31:0], sign);
				
	// Chooses inputs based on mult or div
	muxBusMult mina(in_a, inverted_a, data_operandA, state);
	muxBusMult minb(in_b, inverted_b, data_operandB, state);
				
	// Sets output to 0 if divisor is 0
	muxBusMult divo(div_out, 32'h0, uninverted_div_out, isZeroB);
												
	// Stores state as either multiplying (0) or dividing (1)
	or stateEn(state_enable, ctrl_MULT, ctrl_DIV);
	not enInv(n_enable, state_enable);
	dflipflopMult storeState(ctrl_DIV, clock, 1'b1, 1'b1, state_enable, state);
	not nState(notState, state);
		
	// Exception checking
	xor excxor (sign, data_operandA[31], data_operandB[31]); 	// Sign is 1 if negative, 0 if positive
	xor excand (exception, sign, data_result[31]);
	checkIfZeroMult checkA(isZeroA, data_operandA);
	checkIfZeroMult checkB(isZeroB, data_operandB);
	isMaxMult checkMaxA(isMaxA, data_operandA);
	isMaxMult checkMaxB(isMaxB, data_operandB);
	and checkMaxOver(maxOverflow, isMaxA, isMaxB, notState);
	nor checkNoZero(noZero, isZeroA, isZeroB);
	and excnot (exception1, exception, data_resultRDY, noZero, notState);	
	and divideByZero(div_exception, isZeroB, state);
	or exor(data_exception, exception1, maxOverflow, div_exception);	
	
	
	// Counts to 31, sets result ready
	not notCo[5:0](notCount, count);
	not outRDY(data_resultRDY, count_enable);
	nand cEnable(count_enable, notCount[0], notCount[1], notCount[2], notCount[3], notCount[4], count[5]);
	increment6bitMult incrementer(countIncremented, count);
	register6Mult counter(count, countIncremented, state_enable, clock, count_enable);	// Counts every step, resets at new signal
	
	// Allows product register to change
	or pEnable(product_enable, count_enable, state_enable);

	// For multiplication: shifting the combined adder output/product
	assign product_unshifted[63:32] = adderOut;
	assign product_unshifted[31:0] = product_out[31:0];
	shifterMult shift (product_shifted, product_unshifted);
	
	// For multiplication: initializing product if ctrl_MULT is asserted, or updating if multiplying
	and andDiv [31:0] (rqb_in, n_enable, rqb_shifted[63:32]);
	and ands1[31:0] (p_in, n_enable, product_shifted[63:32]);
	muxBusMult orgsg(product_in[63:32], rqb_in, p_in, state);
	and ands3 [31:0] (shifted_product_anded, n_enable, product_shifted[31:0], notState);
	and ands2[31:0](shifted_product_ored, state_enable, in_a);
	and anfaf[31:0](rqbr, rqb_shifted[31:0], state, n_enable);
	or ors1 [31:0] (product_in[31:0], shifted_product_ored, shifted_product_anded, rqbr);
	
	// Wires correct result out
	muxBusMult resultOut(data_result, div_out, product_out[31:0], state);
	
	// Sets sub and enable values for multiplication adder
	xor xorE(mult_adder_en, buffer_bit, product_out[0]);
	not notS(mult_adder_sub, buffer_bit);
	or addSub(adderSub, mult_adder_sub, state);
	
	and cond1(en_int_1, mult_adder_en, n_enable, notState);
	or cond2(en_int_2, en_int_1, state);
	
	// Stores booth buffer bit in register
	dflipflopMult bb(product_out[0], clock, n_enable, 1'b1, 1'b1, buffer_bit);
	
	// Product/RQB register and adder/comparator
	register64Mult productReg(product_out, product_in, 1'b0, clock, product_enable);
	full32AdderMult adder(adderOut, product_out[63:32], in_b, adderSub, en_int_2);
		 
	// Logic for greater than or equal to output of comparator
	assign gt_or_et = (~adderOut[31]) & ((product_out[63] & in_b[31]) + ((~product_out[63]) & (~in_b[31]))) +
							((~product_out[63]) & in_b[31]);
							
	muxBusMult ifgtoret(rqb_unshifted[63:32], adderOut, product_out[63:32], gt_or_et);
	assign rqb_unshifted[31:0] = product_out[31:0];
	
	leftShift1Mult leftshifter(rqb_shifted, rqb_unshifted, gt_or_et);
							
		 
endmodule



module shifterMult(out, in);
	input [63:0] in;
	output [63:0] out;
	
	wire [63:0] intermediate;
	
	assign intermediate = in >>> 1;
	
	assign out[63] = intermediate[62];
	assign out[62:0] = intermediate[62:0];
	
endmodule




module register64Mult(data_out, data_in, rst, clk, en);
	input clk, en, rst;
	input [63:0] data_in;
	
	output [63:0] data_out;
	
	wire reset;
	
	not notrst(reset, rst);
	
	dflipflopMult flops [63:0] (data_in, clk, reset, 1'b1, en, data_out);
	
endmodule



module register6Mult(data_out, data_in, rst, clk, en);
	input clk, en, rst;
	input [5:0] data_in;
	
	output [5:0] data_out;
	
	wire reset;
	
	not notrst(reset, rst);
	
	dflipflopMult flops [5:0] (data_in, clk, reset, 1'b1, en, data_out);
	
endmodule



module muxBusMult(out, ina, inb, s);
	input [31:0] ina, inb;
	input s;
	output [31:0] out;
	
	assign out = (s) ? ina : inb;
	
endmodule



module leftShift1Mult(out, in, lastVal);
	input [63:0] in;
	input lastVal;
	output [63:0] out;
	
	assign out[63:1] = in[62:0];
	assign out[0] = lastVal;
	
endmodule


module isMaxMult(out, in);
	input [31:0] in;
	output out;
	
	assign out = ~in[31] & in[30] & in[29] & in[28] & in[27] & in[26] & in[25] & in[24] &
					  in[23] & in[22] & in[21] & in[20] & in[19] & in[18] & in[17] & in[16] &
					  in[15] & in[14] & in[13] & in[12] & in[11] & in[10] & in[9] & in[8] &
					  in[7] & in[6] & in[5] & in[4] & in[3] & in[2] & in[1] & in[0];

endmodule


module inverter32Mult(out, in, en);
	input [31:0] in;
	input en;
	output [31:0] out;
		
	full32AdderMult add(out, 32'b0, in, en, 1'b1);
	
endmodule



module increment6bitMult(s, a);
	input [5:0] a;
	wire [5:0] b;
	wire c;
	
	
	output [5:0] s;
	
	assign b = 5'b000001;
	assign c = 1'b0;
	
	wire [5:0] p,g;
	
	wire c1, c2, c3, c4, c5;
	wire p0c, p1p0c, p1g0, p2p1p0c, p2p1g0, p2g1, p3p2p1p0c, p3p2p1g0, p3p2g1, p3g2, p4p3p2p1p0c, p4p3p2p1g0, p4p3p2g1, p4p3g2, p4g3;
	
	adderMult adder0(s[0], p[0], g[0], a[0], b[0], c);
	
	and and0(p0c, p[0], c);
	or   or0(c1, p0c, g[0]);
	
	adderMult adder1(s[1], p[1], g[1], a[1], b[1], c1);
	
	and and1(p1p0c, p0c, p[1]);
	and and2(p1g0, p[1], g[0]);
	or or1(c2, g[1], p1g0, p1p0c);
	
	adderMult adder2(s[2], p[2], g[2], a[2], b[2], c2);
	
	and and3(p2p1p0c, p[2], p1p0c);
	and and4(p2p1g0, p[2], p1g0);
	and and5(p2g1, p[2], g[1]);
	or or2(c3, g[2], p2g1, p2p1g0, p2p1p0c);
	
	adderMult adder3(s[3], p[3], g[3], a[3], b[3], c3);
	
	and and6(p3p2p1p0c, p[3], p2p1p0c);
	and and7(p3p2p1g0, p[3], p2p1g0);
	and and8(p3p2g1, p[3], p2g1);
	and and9(p3g2, p[3], g[2]);
	or or3(c4, p3p2p1p0c, p3p2p1g0, p3p2g1, p3g2, g[3]);
	
	adderMult adder4(s[4], p[4], g[4], a[4], b[4], c4);
	

	and and10(p4p3p2p1p0c, p[4], p3p2p1p0c);
	and and11(p4p3p2p1g0, p[4], p3p2p1g0);
	and and12(p4p3p2g1, p[4], p3p2g1);
	and and13(p4p3g2, p[4], p3g2);
	and and14(p4g3, p[4], g[3]);
	or or4(c5, p4p3p2p1p0c, p4p3p2p1g0, p4p3p2g1, p4p3g2, p4g3, g[4]);
	
	adderMult adder5(s[5], p[5], g[5], a[5], b[5], c5);
	
endmodule


module full32AdderMult(out, a, b, sub, en);
	
	input [31:0] a, b;
	input sub, en;
	output [31:0] out;

	wire [3:0] G, P;
	wire [31:0] bin, bin1;
	wire c8, c16, c24, c32, P0c, P1G0, P1P0c, P2G1, P2P1G0, P2P1P0c, P3G2, P3P2G1, P3P2P1G0, P3P2P1P0c, notA, notB, notC;
	wire over1, over2, over3, notSub, subin;
	
	and(subin, sub, en);
	
	xor xorB [31:0] (bin1, b, subin);
	
	and andB [31:0] (bin, bin1, en);
	
	cla8BlockMult cla8Block0(out[7:0], G[0], P[0], a[7:0], bin[7:0], subin);
	
	and and0(P0c, P[0], subin);
	or or0(c8, G[0], P0c);
	
	cla8BlockMult cla8Block1(out[15:8], G[1], P[1], a[15:8], bin[15:8], c8);
	
	and and1(P1P0c, P[1], P0c);
	and and2(P1G0, P[1], G[0]);
	or or1(c16, G[1], P1G0, P1P0c);
	
	cla8BlockMult cla8Block2(out[23:16], G[2], P[2], a[23:16], bin[23:16], c16);
	
	and and3(P2P1P0c, P[2], P1P0c);
	and and4(P2P1G0, P[2], P1G0);
	and and5(P2G1, P[2], G[1]);
	or or2(c24, G[2], P2G1, P2P1G0, P2P1P0c);
	
	cla8BlockMult cla8Block3(out[31:24], G[3], P[3], a[31:24], bin[31:24], c24);
	
	
endmodule


module dflipflopMult(d, clk, clrn, prn, ena, q);
    input d, clk, ena, clrn, prn;
    wire clr;
    wire pr;

    output q;
    reg q;

    assign clr = ~clrn;
    assign pr = ~prn;

    initial
    begin
        q = 1'b0;
    end

    always @(posedge clk) begin
        if (q == 1'bx) begin
            q <= 1'b0;
        end else if (clr) begin
            q <= 1'b0;
        end else if (ena) begin
            q <= d;
        end
    end
endmodule



module cla8BlockMult(s, bigG, bigP, a, b, c);
	input [7:0] a, b;
	input c;
	output [7:0] s;
	output bigG, bigP;
	wire p0c, c1, c2, c3, c4, c5, c6, c7, p1p0c, p1g0, p2p1p0c, p2p1g0, p2g1, p3p2p1p0c, p3p2p1g0, p3p2g1, p3g2;
	wire p4p3p2p1p0c, p4p3p2p1g0, p4p3p2g1, p4p3g2, p4g3, p5p4p3p2p1p0c, p5p4p3p2p1g0, p5p4p3p2g1, p5p4p3g2, p5p4g3, p5g4;
	wire p6p5p4p3p2p1p0c, p6p5p4p3p2p1g0, p6p5p4p3p2g1, p6p5p4p3g2, p6p5p4g3, p6p5g4, p6g5, p7g6;
	wire p7p6p5p4p3p2p1g0, p7p6p5p4p3p2g1, p7p6p5p4p3g2, p7p6p5p4g3, p7p6p5g4, p7p6g5;
	wire [7:0] g, p;
	
	adderMult adder0(s[0], p[0], g[0], a[0], b[0], c);
	
	and and0(p0c, p[0], c);
	or   or0(c1, p0c, g[0]);
	
	adderMult adder1(s[1], p[1], g[1], a[1], b[1], c1);
	
	and and1(p1p0c, p0c, p[1]);
	and and2(p1g0, p[1], g[0]);
	or or1(c2, g[1], p1g0, p1p0c);
	
	adderMult adder2(s[2], p[2], g[2], a[2], b[2], c2);
	
	and and3(p2p1p0c, p[2], p1p0c);
	and and4(p2p1g0, p[2], p1g0);
	and and5(p2g1, p[2], g[1]);
	or or2(c3, g[2], p2g1, p2p1g0, p2p1p0c);
	
	adderMult adder3(s[3], p[3], g[3], a[3], b[3], c3);
	
	and and6(p3p2p1p0c, p[3], p2p1p0c);
	and and7(p3p2p1g0, p[3], p2p1g0);
	and and8(p3p2g1, p[3], p2g1);
	and and9(p3g2, p[3], g[2]);
	or or3(c4, p3p2p1p0c, p3p2p1g0, p3p2g1, p3g2, g[3]);
	
	adderMult adder4(s[4], p[4], g[4], a[4], b[4], c4);
	
	and and10(p4p3p2p1p0c, p[4], p3p2p1p0c);
	and and11(p4p3p2p1g0, p[4], p3p2p1g0);
	and and12(p4p3p2g1, p[4], p3p2g1);
	and and13(p4p3g2, p[4], p3g2);
	and and14(p4g3, p[4], g[3]);
	or or4(c5, p4p3p2p1p0c, p4p3p2p1g0, p4p3p2g1, p4p3g2, p4g3, g[4]);
	
	adderMult adder5(s[5], p[5], g[5], a[5], b[5], c5);
	
	and and15(p5p4p3p2p1p0c, p[5], p4p3p2p1p0c);
	and and16(p5p4p3p2p1g0, p[5], p4p3p2p1g0);
	and and17(p5p4p3p2g1, p[5], p4p3p2g1);
	and and18(p5p4p3g2, p[5], p4p3g2);
	and and19(p5p4g3, p[5], p4g3);
	and and20(p5g4, p[5], g[4]);
	or or5(c6, p5p4p3p2p1p0c, p5p4p3p2p1g0, p5p4p3p2g1, p5p4p3g2, p5p4g3, p5g4, g[5]);
	
	adderMult adder6(s[6], p[6], g[6], a[6], b[6], c6);
	
	and and21(p6p5p4p3p2p1p0c, p[6], p5p4p3p2p1p0c);
	and and22(p6p5p4p3p2p1g0, p[6], p5p4p3p2p1g0);
	and and23(p6p5p4p3p2g1, p[6], p5p4p3p2g1);
	and and24(p6p5p4p3g2, p[6], p5p4p3g2);
	and and25(p6p5p4g3, p[6], p5p4g3);
	and and26(p6p5g4, p[6], p5g4);
	and and27(p6g5, p[6], g[5]);
	or or6(c7, p6p5p4p3p2p1p0c, p6p5p4p3p2p1g0, p6p5p4p3p2g1, p6p5p4p3g2, p6p5p4g3, p6p5g4, p6g5, g[6]);
	
	adderMult adder7(s[7], p[7], g[7], a[7], b[7], c7);
	
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



module checkIfZeroMult(out, in);
	input [31:0] in;
	output out;
	
	nor check(out, in[0], in[1], in[2], in[3], in[4], in[5], in[6], in[7], 
						in[8], in[9], in[10],in[11],in[12],in[13],in[14],in[15],
						in[16],in[17],in[18],in[19],in[20],in[21],in[22],in[23],
						in[24],in[25],in[26],in[27],in[28],in[29],in[30],in[31]);
	
endmodule



module adderMult(out, p, g, a, b, cin);
	input a, b, cin;
	output out, p, g;
	
	xor outxor(out, a, b, cin);
	and andg(g, a, b);
	or orp(p, a, b);
	
endmodule