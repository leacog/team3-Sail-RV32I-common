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



`include "../include/rv32i-defines.v"
`include "../include/sail-core-defines.v"



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

	/*decoder*/
	wire[31:0] out;

	

	assign out=(ALUctl[3])? (
		(ALUctl[1])? (
			(ALUctl[0])? (~A) & B : A | B
		):(
			(ALUctl[0])? A : A ^ B
		)
	):(
		(ALUctl[2])?(
			(ALUctl[1])?(
				(ALUctl[0])? ($signed(A) < $signed(B) ? 32'b1 : 32'b0) : A - B
			):(
				(ALUctl[0])? A << B[4:0] : A >>> B[4:0]
			)
		):(
			(ALUctl[1])?(
				(ALUctl[0])? A >> B[4:0] : A + B
			):(
				(ALUctl[0])? A | B : A & B
			)
		)
	)

	
	

	

	
	always@(ALUctl, A, B)begin
		ALUOut <= out;
	end

	wire branch;

	assign branch = (ALUctl[6:4]==`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BEQ)? (ALUOut == 0):(
		(ALUctl[6:4]==`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BNE)? !(ALUOut == 0):(
			(ALUctl[6:4]==`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLT)? ($signed(A) < $signed(B)):(
				(ALUctl[6:4]==`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGE)? ($signed(A) >= $signed(B)):(
					(ALUctl[6:4]==`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLTU)? ($unsigned(A) < $unsigned(B)):(
						(ALUctl[6:4]==`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGEU)? ($unsigned(A) >= $unsigned(B)):1'b0
					)
				)
			)
		)
	);

	always@(ALUctl, A, B,ALUOut)begin
		Branch_Enable <= branch;
	end

endmodule
