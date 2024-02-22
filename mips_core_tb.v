// `timescale 1 ns/10 ps  // time-unit = 1 ns, precision = 10 ps
// module mips_core_tb;

// 	reg clk;
// 	reg rst;
// 	integer i;
// 	wire [5:0]  ia;
// 	wire [31:0] id;
// 	wire [5:0]  cma;
// 	wire [31:0] cmd;
// 	wire [31:0] mcd;
// 	wire        cmw;

	
// 	mips_core CORE (.clk(clk), .rst(rst), .iaddr(ia), .idata(id), .daddr(cma), .dwr(cmw), .ddout(cmd), .ddin(mcd));
// 	imem IM (.addr(ia), .dout(id));
// 	dmem DM (.clk(clk), .wr(cmw), .addr(cma), .din(cmd), .dout(mcd));

// 		initial
// 		begin
// 			rst = 1'b1;
// 			#30
// 			rst = 1'b0;
// 		end

// 		initial
// 		begin
// 			clk = 1'b0;
// 			for(i=0;i<10000;i=i+1)
// 			begin	
// 				#5 
// 				clk = 1'b1;
// 				#5
// 				clk = 1'b0;
// 			end
// 		end
// endmodule


`resetall
`timescale 1ns/10ps

`define CLKP 10
`define CLKPDIV2 5

module tb;

    reg clock;
    reg reset;

    //clk, reset, pc, instr, aluout, writedata, memwrite, and readdata

    top uut (.clk(clock), .reset(reset));

    // generate clock
    always begin
        #`CLKPDIV2 clock = ~clock;
    end

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, uut);
    end

    initial begin
        // initialize all variables
        clock = 0; reset = 1;
        // wait for first negative edge before de-asserting reset
        @(negedge clock) reset = 0;
        #1000
        $finish;
    end

endmodule
