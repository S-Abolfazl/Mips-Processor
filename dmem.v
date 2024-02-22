`timescale 1 ns/10 ps  // time-unit = 1 ns, precision = 10 ps
module dmem (clk, wr, addr, din, dout);
	input clk;
	input wr;
	input [5:0] addr;
	input [31:0] din;	
	output [31:0] dout; 
  reg [31:0] rb [0:63];
	always @(posedge clk)
    if(clk)
        if (wr)
        rb[addr] = din;
	
	assign dout = rb[addr];

endmodule