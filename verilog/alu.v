/*
	Authored 2018-2019, Ryan Voo.

	All rights reserved.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/



`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/rv32i-defines.v"
`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/sail-core-defines.v"
`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mods_to_use.v"
`ifdef USE_SUBSETTING
	`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/alu-subset-includes.v"
`else
	`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/full-isa-includes.v"
`endif


/*
 *	Description:
 *
 *		This module implements the ALU for the RV32I.
 */



/*
 *	Not all instructions are fed to the ALU. As a result, the ALUctl
 *	field is only unique across the instructions that are actually
 *	fed to the ALU.
 */
module alu(ALUctl, A, B, ALUOut, Branch_Enable);
	input [6:0]		ALUctl;
	input [31:0]		A;
	input [31:0]		B;
	output reg [31:0]	ALUOut;
	output reg		Branch_Enable;

	`ifdef USE_ALU_DSP_ADDERS	
		wire [31:0] DSPadd; //Wire to connect to DSP sum
		DSPadder alu_adder(
			.input1(A),
			.input2(B),
			.out(DSPadd)
		);
	`endif
	
	`ifdef USE_ALU_DSP_SUBTRACTOR
		wire [31:0] DSPsub; //Wire to connect to DSP subraction result
		wire carry;
		DSPsubtractor alu_subbtractor(
			.input1(A),
			.input2(B),
			.out(DSPsub),
			.carry(carry)
		);
	`elsif USE_ALU_DSP_COMPARATOR //Same code, verilog doesn't allow logic on ifdef statements :( 
		wire [31:0] DSPsub; //Wire to connect to DSP subraction result
		wire carry;
		DSPsubtractor alu_subbtractor(
			.input1(A),
			.input2(B),
			.out(DSPsub),
			.carry(carry)
		);
	`endif

	/*
	 *	This uses Yosys's support for nonzero initial values:
	 *
	 *		https://github.com/YosysHQ/yosys/commit/0793f1b196df536975a044a4ce53025c81d00c7f
	 *
	 *	Rather than using this simulation construct (`initial`),
	 *	the design should instead use a reset signal going to
	 *	modules in the design.
	 */
	initial begin
		ALUOut = 32'b0;
		Branch_Enable = 1'b0;
	end

	`ifdef USE_ALU_DSP_ADDERS
		`ifdef USE_ALU_DSP_SUBTRACTOR
			always @(ALUctl, A, B, DSPadd, DSPsub) begin
		`else
			always @(ALUctl, A, B, DSPadd) begin
		`endif  
	`elsif USE_ALU_DSP_SUBTRACTOR
			always @(ALUctl, A, B, DSPsub) begin
	`elsif USE_ALU_DSP_COMPARATOR
			always @(ALUctl, A, B, DSPsub) begin	
	`else
	always @(ALUctl, A, B) begin
	`endif

		case (ALUctl[3:0])
			/*
			 *	AND (the fields also match ANDI and LUI)
			 */
			`ifdef ALU_AND
				`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_AND:	ALUOut = A & B;
			`endif

			/*
			 *	OR (the fields also match ORI)
			 */
			`ifdef ALU_OR
				`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_OR:	ALUOut = A | B;
			`endif

			/*
			 *	ADD (the fields also match AUIPC, all loads, all stores, and ADDI)
			 */
			`ifdef ALU_ADD
				`ifdef USE_ALU_DSP_ADDERS
					`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_ADD:	ALUOut = DSPadd;
				`else
					`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_ADD:	ALUOut = A + B;
				`endif
			`endif 
			/*
			 *	SUBTRACT (the fields also matches all branches)
			 */

			`ifdef ALU_SUB
				`ifdef USE_ALU_DSP_SUBTRACTOR
					`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SUB:	ALUOut = DSPsub;
				`else
					`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SUB:	ALUOut = A - B;
				`endif
			`endif
			/*
			 *	SLT (the fields also matches all the other SLT variants)
			 */
			`ifdef ALU_SLT
				`ifdef USE_ALU_DSP_COMPARATOR //Need subtractor to use comparator, purpoefully gives you error for missconfiguration
					`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SLT:	ALUOut = DSPsub[31] ? 32'b1 : 32'b0;
				`else
					`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SLT:	ALUOut = $signed(A) < $signed(B) ? 32'b1 : 32'b0;
				`endif
			`endif
			/*
			 *	SRL (the fields also matches the other SRL variants)
			 */
			`ifdef ALU_SRL
				`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SRL:	ALUOut = A >> B[4:0];
			`endif

			/*
			 *	SRA (the fields also matches the other SRA variants)
			 */
			`ifdef ALU_SRA
				`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SRA:	ALUOut = A >>> B[4:0];
			`endif

			/*
			 *	SLL (the fields also match the other SLL variants)
			 */
			`ifdef ALU_SLL
				`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SLL:	ALUOut = A << B[4:0];
			`endif

			/*
			 *	XOR (the fields also match other XOR variants)
			 */
			`ifdef ALU_XOR
				`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_XOR:	ALUOut = A ^ B;
			`endif
			
			`ifdef CSR_REG
				/*
				*	CSRRW  only
				*/
				`ifdef ALU_CSRRW
					`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRW:	ALUOut = A;
				`endif

				/*
				*	CSRRS only
				*/
				`ifdef ALU_CSRRS
					`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRS:	ALUOut = A | B;
				`endif

				/*
				*	CSRRC only
				*/
				`ifdef ALU_CSRRC
					`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRC:	ALUOut = (~A) & B;
				`endif

			`endif
			/*
			 *	Should never happen.
			 */
			default:					ALUOut = 0;
		endcase
	end

	`ifdef USE_ALU_DSP_COMPARATOR

		always @(ALUctl, ALUOut, A, B, DSPsub, carry) begin
			case (ALUctl[6:4])
			`ifdef ALU_BEQ
				`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BEQ:	Branch_Enable = (ALUOut == 0);
			`endif
			`ifdef ALU_BNE
				`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BNE:	Branch_Enable = !(ALUOut == 0);
			`endif
			`ifdef ALU_BLT
				`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLT:	Branch_Enable = (DSPsub[31]);
			`endif
			`ifdef ALU_BGE
				`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGE:	Branch_Enable = !(DSPsub[31]);
			`endif
			`ifdef ALU_BLTU
				`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLTU:	Branch_Enable = !(carry);
			`endif
			`ifdef ALU_BGEU
				`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGEU:	Branch_Enable = (carry);
			`endif

				default:					Branch_Enable = 1'b0;
			endcase
		end

	`else

		always @(ALUctl, ALUOut, A, B) begin
			case (ALUctl[6:4])
				`ifdef ALU_BEQ
					`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BEQ:	Branch_Enable = (ALUOut == 0);
				`endif
				`ifdef ALU_BNE
					`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BNE:	Branch_Enable = !(ALUOut == 0);
				`endif
				`ifdef ALU_BLT
					`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLT:	Branch_Enable = ($signed(A) < $signed(B));
				`endif
				`ifdef ALU_BGE
					`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGE:	Branch_Enable = ($signed(A) >= $signed(B));
				`endif
				`ifdef ALU_BLTU
					`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLTU:	Branch_Enable = ($unsigned(A) < $unsigned(B));
				`endif
				`ifdef ALU_BGEU
					`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGEU:	Branch_Enable = ($unsigned(A) >= $unsigned(B));
				`endif

				default:					Branch_Enable = 1'b0;
			endcase
		end
		
	`endif
endmodule
