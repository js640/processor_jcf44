/**
 * READ THIS DESCRIPTION!
 *
 * The processor takes in several inputs from a skeleton file.
 *
 * Inputs
 * clock: this is the clock for your processor at 50 MHz
 * reset: we should be able to assert a reset to start your pc from 0 (sync or
 * async is fine)
 *
 * Imem: input data from imem
 * Dmem: input data from dmem
 * Regfile: input data from regfile
 *
 * Outputs
 * Imem: output control signals to interface with imem
 * Dmem: output control signals and data to interface with dmem
 * Regfile: output control signals and data to interface with regfile
 *
 * Notes
 *
 * Ultimately, your processor will be tested by subsituting a master skeleton, imem, dmem, so the
 * testbench can see which controls signal you active when. Therefore, there needs to be a way to
 * "inject" imem, dmem, and regfile interfaces from some external controller module. The skeleton
 * file acts as a small wrapper around your processor for this purpose.
 *
 * You will need to figure out how to instantiate two memory elements, called
 * "syncram," in Quartus: one for imem and one for dmem. Each should take in a
 * 12-bit address and allow for storing a 32-bit value at each address. Each
 * should have a single clock.
 *
 * Each memory element should have a corresponding .mif file that initializes
 * the memory element to certain value on start up. These should be named
 * imem.mif and dmem.mif respectively.
 *
 * Importantly, these .mif files should be placed at the top level, i.e. there
 * should be an imem.mif and a dmem.mif at the same level as process.v. You
 * should figure out how to point your generated imem.v and dmem.v files at
 * these MIF files.
 *
 * imem
 * Inputs:  12-bit address, 1-bit clock enable, and a clock
 * Outputs: 32-bit instruction
 *
 * dmem
 * Inputs:  12-bit address, 1-bit clock, 32-bit data, 1-bit write enable
 * Outputs: 32-bit data at the given address
 *
 */
module processor(
    // Control signals
    clock,                          // I: The master clock
    reset,                          // I: A reset signal

    // Imem
    address_imem,                   // O: The address of the data to get from imem
    q_imem,                         // I: The data from imem

    // Dmem
    address_dmem,                   // O: The address of the data to get or put from/to dmem
    data,                           // O: The data to write to dmem
    wren,                           // O: Write enable for dmem
    q_dmem,                         // I: The data from dmem

    // Regfile
    ctrl_writeEnable,               // O: Write enable for regfile
    ctrl_writeReg,                  // O: Register to write to in regfile
    ctrl_readRegA,                  // O: Register to read from port A of regfile
    ctrl_readRegB,                  // O: Register to read from port B of regfile
    data_writeReg,                  // O: Data to write to for regfile
    data_readRegA,                  // I: Data from port A of regfile
    data_readRegB                   // I: Data from port B of regfile
	 
	 
	 
	 , probe, probe1, probe2
);
    
	 // Test signal
	 output [31:0] probe, probe1, probe2;
	 
	 // Control signals
    input clock, reset;

    // Imem
    output [11:0] address_imem;
    input [31:0] q_imem;

    // Dmem
    output [11:0] address_dmem;
    output [31:0] data;
    output wren;
    input [31:0] q_dmem;

    // Regfile
    output ctrl_writeEnable;
    output [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
    output [31:0] data_writeReg;
    input [31:0] data_readRegA, data_readRegB;

	 
	 // Test Values ----------------------
	 
	 assign probe = ctrl_writeEnable;
	 assign probe1 = ctrl_writeReg;
	 assign probe2 = data_writeReg;
	 
	 // -----------------------------------
	 
	 
// WIRES
	 
	 // Hazard wires
	 wire stall, flush;
	 wire d_DXRARD, d_DXRBRD, 
			x_dataHazard, x_dataHazard_stall, x_XMRSRD, x_MWRSRD, x_XMRTRD, x_MWRTRD, x_XMRDRD, x_MWRDRD,
			m_dataHazard, m_exception,
			w_dataHazard, w_exception;
	 
	 // Instruction type wires
	 wire d_btype, d_store, d_jr, d_bex, d_load, 
			x_rtype, x_load, x_addi, x_jal, x_setx, x_itype, x_store, x_ttype, x_jr, x_bex, x_btype,
			m_store, m_load,
			w_load, w_rtype, w_addi, w_jal, w_setx;	
			
	 // Instruction wires
	 wire [31:0] f_insn, f_insnFlushed, d_insn, d_insnFlushed, x_insn, x_insnStall, m_insn, w_insn;
	 
	 // PC wires
	 wire [31:0] pc_extend;
	 wire [11:0] pc_in, pc_increment, branchedPC, f_pc, d_pc, x_pc, m_pc, w_pc, target, bPC;
	 wire branch, m_branch, bex_trigger;
	 
	 // Output wires
	 wire [31:0] d_aFlushed, d_bFlushed, x_a, x_b, x_o, m_o, m_b, w_o, w_read;
	 
	 // Alu/MultDiv wires
	 wire [31:0] alu_inA, alu_inB, immediate, alu_out, mult_out, alu_out_stall, status, d_immediate, d_immediateFlushed;
	 wire [4:0] alu_op, alu_shiftAmt;
	 wire alu_ine, alu_ilt, alu_ovf, mult_exc, mult_rdy, x_mult, x_div, mult_stall, multing;
	 
	 // Bypass wires
	 wire [31:0] mw_aBypass, xm_aBypass, mw_bBypass, xm_bBypass, xm_bBypass_stall;
	 
	 // Temporary wires
	 wire [31:0] alu_inBTemp, x_bTemp, data_writeRegTemp1, data_writeRegTemp2;
	 wire [4:0] read_bTemp, ctrl_readRegATemp, ctrl_readRegBTemp, alu_opTemp, w_regTemp;
	 wire  x_multTemp, x_divTemp;
	 
	 
	 
// Stall / flush logic
	 
	 assign flush = branch | m_branch; 	// ASSUME NOT TAKEN
	 assign stall = (x_load & (d_DXRARD | (d_DXRBRD & ~d_store))) | (x_store & d_load) | (m_store & d_load);

	 
	 
// FETCH
	 
	 assign pc_in = (branch) ? branchedPC : pc_increment;
	 
	 register12bit PC (f_pc, pc_in, clock, ~stall & ~mult_stall, reset);
	 
	 assign address_imem = f_pc;
	 assign f_insn = q_imem;
	 incrementBy1 incrementer(pc_increment, f_pc, 1'b1);
	 
	 
	 
// F/D
	 	 
	 assign f_insnFlushed = (flush) ? 32'h00000000 : f_insn;
	 
	 registerUnit FDInsn (clock, ~stall & ~mult_stall, reset, f_insnFlushed, d_insn);
	 register12bit PCFD (d_pc, f_pc, clock, ~stall & ~mult_stall, reset);		// was pc_in
	 
	 
	 
// DECODE
	 
	  
	 assign ctrl_readRegATemp = (d_btype | d_jr) ? d_insn[26:22] : d_insn[21:17];
	 assign ctrl_readRegA = (d_bex) ? 5'b11110 : ctrl_readRegATemp;
	 assign ctrl_readRegBTemp = (d_store) ? d_insn[26:22] : d_insn[16:12];
	 assign ctrl_readRegB = (d_btype) ? d_insn[21:17] : ctrl_readRegBTemp;

	 addrCompare compA (d_DXRARD, ctrl_readRegA, x_insn[26:22]);
	 addrCompare compB (d_DXRBRD, ctrl_readRegB, x_insn[26:22]);
	 
	 assign d_immediate [31:17] = (d_insn[16]) ? 15'h7FFF : 15'h0;
	 assign d_immediate [16:0] = d_insn[16:0];
	 
	 
	 
// D/X
	 	 
	 assign d_insnFlushed = (stall | flush) ? 32'h00000000 : d_insn;
	 assign d_aFlushed = (stall | flush) ? 32'h00000000 : data_readRegA;
	 assign d_bFlushed = (stall | flush) ? 32'h00000000 : data_readRegB;
	 assign d_immediateFlushed = (stall | flush) ? 32'h00000000 : d_immediate;
	 
	 registerUnit DXInsn (clock, ~mult_stall, reset, d_insnFlushed, x_insn);
	 register12bit PCDX (x_pc, d_pc, clock, ~mult_stall, reset);			// Don't need to nop
	 registerUnit DXA (clock, ~mult_stall, reset, d_aFlushed, x_a);
	 registerUnit DXB (clock, ~mult_stall, reset, d_bFlushed, x_b);
	 registerUnit DXdwq (clock, ~mult_stall, reset, d_immediateFlushed, immediate);
	 
	 
	 
// X-ECUTE	 	 
	 
	 // Hazard catching
	 assign x_dataHazard = x_rtype | x_load | x_addi | x_jal | x_setx;
	 
	 addrCompare xmsd(x_XMRSRD, x_insn[21:17], m_insn[26:22]);
	 addrCompare mwsd(x_MWRSRD, x_insn[21:17], w_insn[26:22]);
	 addrCompare xmtd(x_XMRTRD, x_insn[16:12], m_insn[26:22]);
	 addrCompare mwtd(x_MWRTRD, x_insn[16:12], w_insn[26:22]);
	 addrCompare xmdd(x_XMRDRD, x_insn[26:22], m_insn[26:22]);
	 addrCompare mwdd(x_MWRDRD, x_insn[26:22], w_insn[26:22]);
	 
	
	 
	 assign x_bTemp = (x_load | x_addi | x_store) ? immediate : x_b;

	 
	 assign alu_shiftAmt = (x_rtype) ? x_insn[11:7] : 5'b0;
	 
	 assign alu_opTemp = (x_btype) ? 5'b00001 : 5'b0;	// Sub if bne or blt
	 assign alu_op = (x_rtype) ? x_insn[6:2] : alu_opTemp;
    
		
		
	 // Bypassing
	 assign mw_aBypass = (w_dataHazard & (((x_rtype | x_itype) & x_MWRSRD) | (x_btype & x_MWRDRD))) ? data_writeReg : x_a;
	 assign xm_aBypass = (m_dataHazard & (((x_rtype | x_itype) & x_XMRSRD) | (x_btype & x_XMRDRD))) ? m_o : mw_aBypass;
	 	 
	 assign mw_bBypass = (w_dataHazard & ((x_rtype & x_MWRTRD) | (x_btype & x_MWRSRD))) ? data_writeReg : x_bTemp;
	 assign xm_bBypass = (m_dataHazard & ((x_rtype & x_XMRTRD) | (x_btype & x_XMRSRD))) ? m_o : mw_bBypass;
	 
	 
	 alu aluModule(xm_aBypass, 	// I		Includes mw bypass, but prefers xm hazards 
					   xm_bBypass,		// I
						alu_op,	 		// I
						alu_shiftAmt, 	// I
						alu_out, 		// O
						alu_ine, 		// O
						alu_ilt, 		// O
						alu_ovf);		// O
						
						
	 
	 assign x_multTemp = x_rtype & (~x_insn[6] & ~x_insn[5] & x_insn[4] & x_insn[3] & ~x_insn[2]);
	 assign x_divTemp = x_rtype & (~x_insn[6] & ~x_insn[5] & x_insn[4] & x_insn[3] & x_insn[2]);
	 
	 assign x_mult = x_multTemp & ~multing;
	 assign x_div = x_divTemp & ~multing;
						
	 
						
	 multdiv mdModule(xm_aBypass, 			// I
							xm_bBypass, 			// I
							x_mult, 		// I
							x_div, 		// I
							clock, 		// I
							mult_out, 	// O
							mult_exc,	// O
							mult_rdy);	// O
	 
	 
	 
	 assign mult_stall = (multing | x_multTemp | x_divTemp) & ~mult_rdy;
	 
	 
	 dflipflopReg ismulting(multing, (x_multTemp | x_divTemp) & ~mult_rdy, clock, x_multTemp | x_divTemp, reset);
	 
	 checkIfZeroMult check(bex_trigger, xm_aBypass);
	 
	 assign branch = (x_btype & ~x_insn[29] & alu_ine) | (x_btype & x_insn[29] & alu_ilt) | x_ttype | x_jr | (~bex_trigger & x_bex);
	 
	 assign target = (x_jr) ? xm_aBypass[11:0] : x_insn[11:0];
	 
	 branchAdder bAdd (bPC, x_pc, x_insn[16:0]);
	 
	 assign branchedPC = (x_ttype | x_jr | (~bex_trigger & x_bex)) ? target : bPC;
	 
	 	 
	 assign x_insnStall = (mult_stall) ? 32'h0 : x_insn;
	 
	 assign x_o = (multing) ? mult_out : alu_out;
	 
	 assign alu_out_stall = (mult_stall) ? 32'h0 : x_o;
	 assign xm_bBypass_stall = (mult_stall) ? 32'h0 : xm_bBypassTemp2;
	 assign x_dataHazard_stall = (mult_stall) ? 1'b0 : x_dataHazard;
	

	 wire [31:0] xm_bBypassTemp1, xm_bBypassTemp2;
	 
	 assign xm_bBypassTemp1 = (x_MWRDRD & x_store) ? data_writeReg : x_b;
	 assign xm_bBypassTemp2 = (x_XMRDRD & x_store) ? m_o : xm_bBypassTemp1;
	
	
// X/M
	 	 
	 registerUnit XMInsn (clock, 1'b1, reset, x_insnStall, m_insn);
	 registerUnit XMOut (clock, 1'b1, reset, alu_out_stall, m_o);
	 registerUnit XMB   (clock, 1'b1, reset, xm_bBypass_stall, m_b);
	 dflipflopReg nam1(m_dataHazard, x_dataHazard_stall, clock, 1'b1, reset);
	 register12bit PCDadX (m_pc, x_pc, clock, 1'b1, reset);
	 dflipflopReg Except (m_exception, (mult_exc & mult_rdy) | (alu_ovf & (x_rtype | x_addi)), clock, 1'b1, reset);
	 dflipflopReg Excepadt (m_branch, branch, clock, 1'b1, reset);
	 
	 
	 
// MEMORY	 
	 
	 assign address_dmem = (m_store) ? m_o[11:0] : alu_out[11:0];
	 assign wren = m_store;
	 assign data = m_b;
	 
	 
	 
// M/W
	 
	 registerUnit MWInsn (clock, 1'b1, reset, m_insn, w_insn);
	 registerUnit MWOut (clock, 1'b1, reset, m_o, w_o);
	 registerUnit MWRead(clock, 1'b1, reset, q_dmem, w_read);		//?
	 dflipflopReg nam2(w_dataHazard, m_dataHazard, clock, 1'b1, reset);
	 register12bit PCDfX (w_pc, m_pc, clock, 1'b1, reset);
	 dflipflopReg nam2e2(w_exception, m_exception, clock, 1'b1, reset);
	 
	 
	 
// WRITEBACK
	 
	 assign pc_extend [31:12] = 20'b0;
	 assign pc_extend [11:0] = w_pc;
			
	 
	 getStatus statusGetter(status, w_exception, w_insn);
	 
	 	
	 assign ctrl_writeEnable = w_load | w_rtype | w_addi | w_jal | w_setx;
	 assign w_regTemp = (w_setx | w_exception) ? 5'b11110 : w_insn[26:22];
	 assign ctrl_writeReg = (w_jal) ? 5'b11111 : w_regTemp;
	 assign data_writeRegTemp1 = (w_load) ? w_read : w_o;
	 assign data_writeRegTemp2 = (w_setx | w_exception) ? status : data_writeRegTemp1;
	 assign data_writeReg = (w_jal) ? pc_extend : data_writeRegTemp2;
	 
	 
	 
// Here's where I hide the bodies
	 
	 assign d_btype 	= ~d_insn[31] & ~d_insn[30] & 					d_insn[28] & ~d_insn[27];// 00X10
	 assign d_store 	= ~d_insn[31] & ~d_insn[30] &  d_insn[29] &  d_insn[28] &  d_insn[27];// 00111
	 assign d_jr 		= ~d_insn[31] & ~d_insn[30] &  d_insn[29] & ~d_insn[28] & ~d_insn[27];// 00100
	 assign d_bex 		=  d_insn[31] & ~d_insn[30] &  d_insn[29] &  d_insn[28] & ~d_insn[27];// 10110
	 assign d_load 	= ~d_insn[31] &  d_insn[30] & ~d_insn[29] & ~d_insn[28] & ~d_insn[27];// 01000
	 
	 assign x_rtype 	= ~x_insn[31] & ~x_insn[30] & ~x_insn[29] & ~x_insn[28] & ~x_insn[27];// 00000
	 assign x_load 	= ~x_insn[31] &  x_insn[30] & ~x_insn[29] & ~x_insn[28] & ~x_insn[27];// 01000
	 assign x_addi 	= ~x_insn[31] & ~x_insn[30] &  x_insn[29] & ~x_insn[28] &  x_insn[27];// 00101
	 assign x_jal 		= ~x_insn[31] & ~x_insn[30] & ~x_insn[29] &  x_insn[28] &  x_insn[27];// 00011
	 assign x_setx 	=  x_insn[31] & ~x_insn[30] &  x_insn[29] & ~x_insn[28] &  x_insn[27];// 10101
	 assign x_store 	= ~x_insn[31] & ~x_insn[30] &  x_insn[29] &  x_insn[28] &  x_insn[27];// 00111
	 assign x_ttype 	= ~x_insn[31] & ~x_insn[30] & ~x_insn[29] &  				  x_insn[27];// 000X1
	 assign x_jr 		= ~x_insn[31] & ~x_insn[30] &  x_insn[29] & ~x_insn[28] & ~x_insn[27];// 00100
	 assign x_bex 		=  x_insn[31] & ~x_insn[30] &  x_insn[29] &  x_insn[28] & ~x_insn[27];// 10110
	 assign x_btype 	= ~x_insn[31] & ~x_insn[30] & 					x_insn[28] & ~x_insn[27];// 00X10
	 
	 assign x_itype 	=  x_btype | x_addi | x_load | x_store;
	 
	 assign m_store 	= ~m_insn[31] & ~m_insn[30] &  m_insn[29] &  m_insn[28] &  m_insn[27];// 00111 
	 assign m_load  	= ~m_insn[31] &  m_insn[30] & ~m_insn[29] & ~m_insn[28] & ~m_insn[27];// 01000 

	 assign w_load 	= ~w_insn[31] &  w_insn[30] & ~w_insn[29] & ~w_insn[28] & ~w_insn[27];// 01000
	 assign w_rtype	= ~w_insn[31] & ~w_insn[30] & ~w_insn[29] & ~w_insn[28] & ~w_insn[27];// 00000
	 assign w_addi 	= ~w_insn[31] & ~w_insn[30] &  w_insn[29] & ~w_insn[28] &  w_insn[27];// 00101
	 assign w_jal 		= ~w_insn[31] & ~w_insn[30] & ~w_insn[29] &  w_insn[28] &  w_insn[27];// 00011
	 assign w_setx 	=  w_insn[31] & ~w_insn[30] &  w_insn[29] & ~w_insn[28] &  w_insn[27];// 10101
	 

endmodule


module register12bit(q, d, clock, en, reset);
	input [11:0] d;
	input clock, en, reset;

	output [11:0] q;

	dflipflopReg flops [11:0] (q, d, clock, en, reset);

endmodule

module incrementBy1(out, in, en);
	input [11:0] in;
	output [11:0] out;
	input en;
	
	wire [15:0] tempin, tempout, bin;
	wire [1:0] G, P;
	
	assign tempin[15:12] = 4'b0;
	assign tempin[11:0] = in;
	
	assign bin = 16'b01;
	
	cla8Block cla8Block0(tempout[7:0], G[0], P[0], tempin[7:0], bin[7:0], 1'b0);
		
	cla8Block cla8Block1(tempout[15:8], G[1], P[1], tempin[15:8], bin[15:8], G[0]);
	
	assign out = (en) ? tempout[11:0] : tempin[11:0];

endmodule


module branchAdder(out, pc, n);
	input [16:0] n;
	input [11:0] pc;
	output [11:0] out;
	
		
	wire [15:0] tempin, tempout, bin;
	wire [1:0] G, P;
	
	assign tempin[15:12] = 4'b0;
	assign tempin[11:0] = pc;
	
	assign bin = n[15:0];
	
	cla8Block cla8Block0(tempout[7:0], G[0], P[0], tempin[7:0], bin[7:0], 1'b0);
		
	cla8Block cla8Block1(tempout[15:8], G[1], P[1], tempin[15:8], bin[15:8], G[0]);
	
	assign out = tempout[11:0];
	
endmodule


module addrCompare(out, inA, inB);
	output out;
	input[4:0] inA, inB;
	
	wire [4:0] temp;
	wire isZero;
	
	xnor xnors [4:0] (temp, inA, inB);
	
	assign isZero = ~inA[0] & ~inA[1] & ~inA[2] & ~inA[3] & ~inA[4];
	
	assign out = temp[0] & temp[1] & temp[2] & temp[3] & temp[4] & ~isZero;


endmodule

module getStatus(out, exc, insn);
	
	output [31:0] out;
	input [31:0] insn;
	input exc;
	wire [31:0] status, error0, error1, error2, error3, error4;
	wire is_addi, is_rtype, is_add, is_sub, is_mul, is_div;
	
	assign is_addi = ~insn[31] & ~insn[30] & insn[29] & ~insn[28] & insn[27];
	assign is_rtype = ~insn[31] & ~insn[30] & ~insn[29] & ~insn[28] & ~insn[27];
	assign is_add = is_rtype & (~insn[6] & ~insn[5] & ~insn[4] & ~insn[3] & ~insn[2]);
	assign is_sub = is_rtype & (~insn[6] & ~insn[5] & ~insn[4] & ~insn[3] & insn[2]);
	assign is_mul = is_rtype & (~insn[6] & ~insn[5] & insn[4] & insn[3] & ~insn[2]);
	assign is_div = is_rtype & (~insn[6] & ~insn[5] & insn[4] & insn[3] & insn[2]);
	
	assign error0 = (is_addi) ? 32'd2 : 32'd0;
	assign error1 = (is_add) ? 32'd1 : error0;
	assign error2 = (is_sub) ? 32'd3 : error1;
	assign error3 = (is_mul) ? 32'd4 : error2;
	assign error4 = (is_div) ? 32'd5 : error3;
	
	assign status [31:27] = insn[26];
	assign status [26:0] = insn[26:0];
	
	assign out = (exc) ? error4 : status;		// Prefs error statuses over setx ones

endmodule