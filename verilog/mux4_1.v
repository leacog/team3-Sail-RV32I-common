module m41 ( input a,  
input b,  
input c,  
input d,  
input s0, s1, 
output out);

assign out = s1 ? (s0 ? d : c) : (s0 ? b : a);

endmodule

module m41_32 ( input[31:0] a,  
input[31:0] b,  
input[31:0] c,  
input[31:0] d,  
input s0, s1, 
output[31:0] out);

genvar i;

generate
	for (i=0; i < 32; i = i+1)begin
		m41 m41_inst (a[i],b[i],c[i],d[i],s0,s1,out[i]);
	end
endgenerate

endmodule