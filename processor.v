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
);
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

	 
	 
	 wire stall;
	 wire flush;
	 
	 assign stall = 1'b0;
	 assign flush = 1'b0;
	 
	 
	 // FETCH
	 
	 wire [31:0] f_insn;
	 wire [11:0] pc_out, pc_in, pcPlus1;
	 
	 assign pc_in = (branch) ? branchedPC : pcPlus1;
	 
	 register12bit PC (pc_out, pc_in, ~clock, ~stall, reset);
	 
	 assign address_imem = pc_out;
	 assign f_insn = (flush) ? 32'h00000000 : q_imem;
	 
	 incrementBy1 incrementer(pcPlus1, pc_out, 1'b1);
	 
	 
	 // F/D
	 
	 wire [31:0] d_insn;
	 wire [11:0] d_pc;
	 
	 registerUnit FDInsn (~clock, ~stall, reset, f_insn, d_insn);
	 register12bit PCFD (d_pc, pc_in, ~clock, ~stall, reset);
	 
	 // D
	 
	 wire d_rtype;
	 nor isDRtype (d_rtype, d_insn[31], d_insn[30], d_insn[29], d_insn[28], d_insn[27]);
	 
	 assign ctrl_readRegA = d_insn[21:17];		//  rs
	 assign ctrl_readRegB = (d_rtype) ? d_insn[16:12] : d_insn[26:22];		// rt or rd
	 
	 //assign regB_actual = branch_indicator || sw_indicator || jr_indicator ? F_D_out[26:22] : F_D_out[16:12];
	 
	 // D/X
	 
	 wire [31:0] x_insn, x_a, x_b;
	 wire [11:0] x_pc;
	 
	 registerUnit DXInsn (~clock, ~stall, reset, d_insn, x_insn);
	 register12bit PCDX (x_pc, d_pc, ~clock, ~stall, reset);
	 registerUnit DXA (~clock, ~stall, reset, data_readRegA, x_a);
	 registerUnit DXB (~clock, ~stall, reset, data_readRegB, x_b);
	 
	 // X
	 
	 wire [31:0] alu_out, alu_inB, alu_inA, alu_inBTemp;
	 wire [4:0] alu_op, alu_shiftAmt, alu_opTemp;
	 wire alu_ine, alu_ilt, alu_ovf, neORlt;
	 
	 wire [31:0] immediate;
	 
	 assign immediate [31:17] = x_insn[16];
	 assign immediate [16:0] = x_insn[16:0];
	 
	 
	 assign alu_shiftAmt = (rtype) ? x_insn[11:7] : 5'b0;
	 
	 assign neORlt = ~x_insn[31] & ~x_insn[30] & x_insn[28] & ~x_insn[27]; // 00010 or 00110
	 assign alu_opTemp = (neORlt) ? 5'b00001 : 5'b0;	// Sub if bne or blt
	 assign alu_op = (rtype) ? x_insn[6:2] : alu_opTemp;
    
	 
	 wire rtype;
	 
	 nor isRtype (rtype, x_insn[31], x_insn[30], x_insn[29], x_insn[28], x_insn[27]);
	 
	 assign alu_inBTemp = (rtype) ? x_b : immediate;
	 assign alu_inB = (neORlt) ? x_a : alu_inBTemp;		// rs if R type, immediate if I, rd if bne or blt
	 assign alu_inA = (neORlt) ? x_b : x_a;
	 
	 alu aluModule(alu_inA, 				// I
					   alu_inB, 		// I
						alu_op,	 		// I
						alu_shiftAmt, 	// I
						alu_out, 		// O
						alu_ine, 		// O
						alu_ilt, 		// O
						alu_ovf);		// O
						
						
	 wire [31:0] mult_out; 		// For now, inA and inB are same as ALU
	 wire mult, div, mult_exc, mult_rdy;
						
	 multdiv mdModule(x_a, 			// I
							x_b, 			// I
							mult, 		// I
							div, 			// I
							clock, 		// I
							mult_out, 	// O
							mult_exc,	// O
							mult_rdy);	// O
	 
	 
	 
	 wire branch;
	 
	 assign branch = (neORlt & ~x_insn[29] & alu_ine) | (neORlt & x_insn[29] & alu_ilt);
	 
	 //assign flush = branch; // ASSUME NOT TAKEN
	 
	 wire [11:0] branchedPC;
	 
	 branchAdder bAdd (branchedPC, x_pc, x_insn[16:0]);
	 
	 
	 // X/M
	 
	 wire [31:0] m_o, m_insn, m_b;
	 
	 registerUnit XMInsn (~clock, ~stall, reset, x_insn, m_insn);
	 registerUnit XMOut (~clock, ~stall, reset, alu_out, m_o);
	 registerUnit XMB   (~clock, ~stall, reset, x_b, m_b);
	 
	 
	 // M
	 
	 // Something with dmem
	 
	 wire store;
	 
	 assign store = ~m_insn[31] & ~m_insn[30] & m_insn[29] &  m_insn[28] &  m_insn[27];// 00111
	 //address_dmem,                   
    //data,          
    //wren,            
    //q_dmem,           
	 
	 
	 assign address_dmem = m_o[11:0];
	 assign wren = store;
	 assign data = m_b;
	 
	 // M/W
	 
	 wire [31:0] w_o, w_insn, w_read;
	 
	 registerUnit MWInsn (~clock, ~stall, reset, m_insn, w_insn);
	 registerUnit MWOut (~clock, ~stall, reset, m_o, w_o);
	 registerUnit MWRead(~clock, ~stall, reset, q_dmem, w_read);
	 
	 // W
	 
			// ctrl_writeReg, ctrl_writeEn, data_writeReg
			
	 wire load, w_rtype, w_addi;
	 
	 assign load = ~w_insn[31] & w_insn[30] & ~w_insn[29] & ~w_insn[28] & ~w_insn[27];// 01000
	 assign w_rtype = ~w_insn[31] & ~w_insn[30] & ~w_insn[29] & ~w_insn[28] & ~w_insn[27];// 00000
	 assign w_addi = ~w_insn[31] & ~w_insn[30] & w_insn[29] & ~w_insn[28] & w_insn[27];// 00101
			
	 assign ctrl_writeEnable = load | w_rtype | w_addi;
	 assign ctrl_writeReg = w_insn[26:22];
	 assign data_writeReg = (load) ? w_read : w_o;
	 
	 

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
	
	assign bin = 16'b0000000000000001;
	
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