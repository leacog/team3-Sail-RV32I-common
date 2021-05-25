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
	wire a;
	wire b;
	wire c;
	wire d;
	wire A3to0_AND;
	wire A3to0_OR;
	wire A3to0_ADD;
	wire A3to0_SUB;
	wire A3to0_SLT;
	wire A3to0_SRL;
	wire A3to0_SRA;
	wire A3to0_SLL;
	wire A3to0_XOR;
	wire A3to0_CSRRW;
	wire A3to0_CSRRS;
	wire A3to0_CSRRC;

	assign a=ALUctl[3];
	assign b=ALUctl[2];
	assign c=ALUctl[1];
	assign d=ALUctl[0];

	assign A3to0_AND= !( a | b | c | d);
	assign A3to0_OR= (!(a | b | c )) & d;
	assign A3to0_ADD= (!(a | b | d )) & c;
	assign A3to0_SUB= (!(a | d))&( b & c);
	assign A3to0_SLT= (!a)&(b & c & d);
	assign A3to0_SRL=(!(a | b))&( c & d);
	assign A3to0_SRA=(!b)&(!(a | c | d));
	assign A3to0_SLL=(!(a | c))&( b & d);
	assign A3to0_XOR= a &(!(b | c | d));
	assign A3to0_CSRRW=(!(b | c))&( a & d);
	assign A3to0_CSRRS=(!(b | d))&( a & c);
	assign A3to0_CSRRC=(!b)&(a & c & d);

	
	

	assign out = (A3to0_AND)? A & B:(
		((A3to0_OR)|(A3to0_CSRRS)) ? A | B :(
			(A3to0_ADD)?  A + B :(
				(A3to0_SUB)? A - B:(
					(A3to0_SLT)?	($signed(A) < $signed(B) ? 32'b1 : 32'b0):(
						(A3to0_SRL)? A >> B[4:0] :(
							(A3to0_SRA)? A >>> B[4:0]:(
								(A3to0_SLL)? A << B[4:0](
									(A3to0_XOR)?  A ^ B :(
										(A3to0_CSRRW)? A:(
											(A3to0_CSRRC)? (~A) & B : 32'b0
										)
									)
								)
							)
						)
					)		
				)
			)
		)
	);

	
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
