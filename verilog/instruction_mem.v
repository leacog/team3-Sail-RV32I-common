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



/*
 *	RISC-V instruction memory
 */

`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mods_to_use.v"

`ifdef USE_INSTRUCTION_MEM_BRAM
/*
module instruction_memory_bram(addr, out, clk);
	input clk;
	reg[31:0] addr_buf;
	input [31:0]		addr;
	output reg [31:0]		out;

	reg [31:0]		insmem[0:1023];
	
	initial begin
		$readmemh("/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/programs/program.hex",insmem);
	end
	
	always @(addr) begin
		addr_buf <= addr;
	end

	always @(negedge clk) begin
		out <= insmem[addr_buf >> 2];
	end
endmodule
*/

module instruction_memory_bram(addr, out, clk);
	input clk;
	input [31:0]		addr;
	output reg [31:0]		out;

	reg [31:0]		insmem[0:1023];
	
	initial begin
		$readmemh("/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/programs/program.hex",insmem);
		out <= 32'h13;
	end
	
	always @(posedge clk) begin
		out <= insmem[addr >> 2];
	end
endmodule

`else

module instruction_memory(addr, out);
	input [31:0]		addr;
	output [31:0]		out;

	reg [31:0]		instruction_memory[0:1023];

	initial begin
		$readmemh("/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/programs/program.hex",instruction_memory);
	end

	assign out = instruction_memory[addr >> 2];
endmodule

`endif
