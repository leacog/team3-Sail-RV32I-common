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



//Data cache

module cache (clk, inst_addr,addr, write_data, memwrite, memread, sign_mask, read_data,inst_out,led, clk_stall);
	input			clk;
	input [31:0]    inst_addr;
	input [31:0]		addr;
	input [31:0]		write_data;
	input			memwrite;
	input			memread;
	input [3:0]		sign_mask;
	output reg [31:0]	read_data;
	output [31:0]        inst_out;
	output [7:0]		led;
	output reg		clk_stall;	//Sets the clock high

	/*
	 *	led register
	 */
	reg [31:0]		led_reg;

	/*
	 *	Current state
	 */
	integer			state = 0;

	/*
	 *	Possible states
	 */
	parameter		IDLE = 0;
	parameter		READ_BUFFER = 1;
	parameter		READ = 2;
	parameter		WRITE = 3;

	/*
	 *	Line buffer
	 */
	reg [31:0]		word_buf;

	/*
	 *	Read buffer
	 */
	reg [31:0]		read_buf;

	/*
	 *	Buffer to identify read or write operation
	 */
	reg			memread_buf;
	reg			memwrite_buf;

	/*
	 *	Buffers to store write data
	 */
	reg [31:0]		write_data_buffer;

	/*
	 *	Buffer to store address
	 */
	reg [31:0]		addr_buf;

	/*
	 *	Sign_mask buffer
	 */
	reg [3:0]		sign_mask_buf;

	/*
	 *	Block memory registers
	 *
	 *	(Bad practice: The constant for the size should be a `define).
	 */
	reg [31:0]		data_block[0:1023];
	reg [31:0]		instruction_memory[0:2**12-1];

	/*
	 *	wire assignments
	 */
	wire [9:0]		addr_buf_block_addr;
	wire [1:0]		addr_buf_byte_offset;
	
	reg [31:0]		replacement_word;

	assign			addr_buf_block_addr	= addr_buf[11:2];
	assign			addr_buf_byte_offset	= addr_buf[1:0];

	/*
	 *	Regs for multiplexer output
	 */
	wire [7:0]		buf0;
	wire [7:0]		buf1;
	wire [7:0]		buf2;
	wire [7:0]		buf3;

	assign 			buf0	= word_buf[7:0];
	assign 			buf1	= word_buf[15:8];
	assign 			buf2	= word_buf[23:16];
	assign 			buf3	= word_buf[31:24];

	wire[7:0]	byteword	
	
	assign		byteword	=write_data_buffer[7:0];
	
	wire[15:0]	halfword;

	assign		halfword	=write_data_buffer[15:0];

	/*
	 *	Constructing the word to be replaced for write byte
	 */

	always @(*) begin
		case(sign_mask_buf[2:0])
			3'b001: begin //Byte
				case(addr_buf_byte_offset)
					2'b00: begin
						 replacement_word = {byteword,buf1,buf2,buf3};
					end
					2'b01: begin
						 replacement_word = {buf0,byteword,buf2,buf3};
					end
					2'b10: begin
						 replacement_word = {buf0,buf1,byteword,buf3};
					end
					2'b11: begin
						 replacement_word = {buf0,buf1,buf3,byteword};
					end
				endcase
			end
			
			3'b011: begin //Halfword
				case(addr_buf_byte_offset[1])
					1'b0: begin
						 replacement_word = {buf3, buf2,halfword};
					end
					1'b1: begin
						 replacement_word = {halfword,buf1, buf0};
					end
				endcase
			end
			
			3'b111: begin //Word
				 replacement_word = 	write_data_buffer;
			end
			
			default: begin
				//do nothing
			end
		endcase
	end



	/*
	 *	Combinational logic for generating 32-bit read data
	 */
	

	/* a is sign_mask_buf[2], b is sign_mask_buf[1], c is sign_mask_buf[0]
	 * d is addr_buf_byte_offset[1], e is addr_buf_byte_offset[0]
	 */
	always @(*) begin
		case(sign_mask_buf[2:0])
			3'b001: begin //Byte
				case(addr_buf_byte_offset)
					2'b00: begin
						 read_buf = (sign_mask_buf[3]==1'b1)?{{24{buf0[7]}}, buf0}:{24'b0, buf0};
					end
					2'b01: begin
						 read_buf = (sign_mask_buf[3]==1'b1)?{{24{buf1[7]}}, buf1}:{24'b0, buf1};
					end
					2'b10: begin
						 read_buf = (sign_mask_buf[3]==1'b1)?{{24{buf2[7]}}, buf2}:{24'b0, buf2};
					end
					2'b11: begin
						 read_buf = (sign_mask_buf[3]==1'b1)?{{24{buf3[7]}}, buf3}:{24'b0, buf3};
					end
				endcase
			end
			
			3'b011: begin //Halfword
				case(addr_buf_byte_offset[1])
					1'b0: begin
						 read_buf = (sign_mask_buf[3]==1'b1)?{{16{buf1[7]}}, buf1, buf0}:{16'b0, buf1, buf0};
					end
					1'b1: begin
						 read_buf = (sign_mask_buf[3]==1'b1)?{{16{buf3[7]}}, buf3, buf2}:{16'b0, buf3, buf2};
					end
				endcase
			end
			
			3'b111: begin //Word
				 read_buf = {buf3, buf2, buf1, buf0};
			end
			
			default: begin
				//do nothing
			end
		endcase
	end
	
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
		//$readmemh("verilog/program.hex",instruction_memory);
		//$readmemh("verilog/data.hex", data_block);
		clk_stall = 0;
	end

	/*
	 *	LED register interfacing with I/O
	 */
	always @(posedge clk) begin
		if(memwrite == 1'b1 && addr == 32'h2000) begin
			led_reg <= write_data;
		end
	end

	/*
	 *	State machine
	 */
	always @(posedge clk) begin
		case (state)
			IDLE: begin
				clk_stall <= 0;
				memread_buf <= memread;
				memwrite_buf <= memwrite;
				write_data_buffer <= write_data;
				addr_buf <= addr;
				sign_mask_buf <= sign_mask;
				
				if(memwrite==1'b1 || memread==1'b1) begin
					state <= READ_BUFFER;
					clk_stall <= 1;
				end
			end

			READ_BUFFER: begin
				/*
				 *	Subtract out the size of the instruction memory.
				 *	(Bad practice: The constant should be a `define).
				 */
				word_buf <= data_block[addr_buf_block_addr - 32'h1000];
				if(memread_buf==1'b1) begin
					state <= READ;
				end
				else if(memwrite_buf == 1'b1) begin
					state <= WRITE;
				end
			end

			READ: begin
				clk_stall <= 0;
				read_data <= read_buf;
				state <= IDLE;
			end

			WRITE: begin
				clk_stall <= 0;

				/*
				 *	Subtract out the size of the instruction memory.
				 *	(Bad practice: The constant should be a `define).
				 */
				data_block[addr_buf_block_addr - 32'h1000] <= replacement_word;
				state <= IDLE;
			end

		endcase
	end

	assign inst_out = instruction_memory[inst_addr];

	/*
	 *	Test led
	 */
	assign led = led_reg[7:0];
endmodule
