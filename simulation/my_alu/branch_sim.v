`include "../include/rv32i-defines.v"
`include "../include/sail-core-defines.v"

module top();
	reg clk = 0;

	reg[31:0] A, B;
	wire[31:0] ALUOut;
	wire Branch_Enable;

	//alu_control interface
	reg[3:0] FuncCode;
	reg[6:0] Opcode;

	//alu aluctl interface
	wire[6:0] AluCtl_wire;

	alu alu_inst(
		.ALUctl(AluCtl_wire),
		.A(A),
		.B(B),
		.ALUOut(ALUOut),
		.Branch_Enable(Branch_Enable)
	);

//simulation
always
 #0.5 clk = ~clk;

initial begin
	$dumpfile ("adder.vcd");
 	$dumpvars;

 	//reg[31:0] A, B;
 	//reg[3:0] FuncCode; //bit 32 + bit 14:12
	//reg[6:0] Opcode; //bits 6:0

 	A = 32'b0;
 	B = 32'b0;
 	AluCtl_wire= 7'b00;

 	#5

 	//simulate BEQ instruction
 	A = 32'b00001111;
 	B = 32'b01010101;
 	AluCtl_wire[6:4] =`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BEQ;

 	#5

 	//simulate BNE instruction
 	A = 32'b00001110;
 	B = 32'b01010101;
 	AluCtl_wire[6:4] =`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BNE;

 	#5

 	//simulate BLT instruction
 	A = 32'd10000;
 	B = 32'd0111;
 	AluCtl_wire[6:4] =`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLT;

 	#5

 	//simulate BGE instruction
 	A = 32'd10000;
 	B = 32'd0111;
 	AluCtl_wire[6:4] =`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGE;

	#5

 	//simulate BLTU instruction
 	A = 32'b0;
 	B = 32'b10;
 	AluCtl_wire[6:4] =`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLTU;


 	#5

 	//simulate BGEU instuction
 	A = 32'b10000;
 	B = 32'b10;
 	AluCtl_wire[6:4] =`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGEU;

 	#5

 	$finish;
end

endmodule
