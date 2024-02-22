`timescale 1 ns/10 ps  // time-unit = 1 ns, precision = 10 ps

module dmem (clk, wr, rd, addr, din, dout);
	input clk;
	input wr;
	input rd;
	input [5:0] addr;
	input [31:0] din;	
	output reg [31:0] dout; 
    reg [31:0] rb [0:63];
    integer i;
	initial begin
		for (i = 0;i < 63 ;i = i + 1 ) begin
			rb[i] = 0;
		end
	end
	always @(posedge clk) begin
        if (wr) begin
			rb[addr] <= din;
		end 
        if (rd) begin
			dout <= rb[addr];
		end 
	end 
endmodule

module imem (addr, dout);
	input  [5:0] addr;
	output [31:0] dout; 
    reg [31:0] rb [0:63]; // 64 rows of 32-bits

	initial
			$readmemb("opcodes.bin", rb);

	assign dout = rb[addr];
endmodule

module control_unit(inst31_26, RegDst, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite);
	input [5:0] inst31_26;

	output reg RegDst;
	output reg Branch;
	output reg MemRead;
	output reg MemtoReg;
	output reg [1:0] ALUOp;
	output reg MemWrite;
	output reg ALUSrc;
	output reg RegWrite;

	always @(*) begin
		case(inst31_26)
			0: begin
				RegDst <= 1;
				ALUSrc <= 0;
				MemtoReg <= 0;
				RegWrite <= 1;
				MemRead	<= 0;
				MemWrite <= 0;
				Branch <= 0;
				ALUOp <= 2'b10;
			end
			35: begin
				RegDst <= 0;
				ALUSrc <= 1;
				MemtoReg <= 1;
				RegWrite <= 1;
				MemRead	<= 1;
				MemWrite <= 0;
				Branch <= 0;
				ALUOp <= 2'b00;
			end
			43: begin
				RegDst <= 0;
				ALUSrc <= 1;
				MemtoReg <= 0;
				RegWrite <= 0;
				MemRead	<= 0;
				MemWrite <= 1;
				Branch <= 0;
				ALUOp <= 2'b00;
			end
			4: begin
				RegDst <= 0;
				ALUSrc <= 0;
				MemtoReg <= 0;
				RegWrite <= 0;
				MemRead	<= 0;
				MemWrite <= 0;
				Branch <= 1;
				ALUOp <= 2'b01;
			end
		endcase
	end
endmodule

module SignExtend (inst15_0, Extend32);
	input [15:0] inst15_0;
	output reg [31:0] Extend32;

	always @(inst15_0) begin
		Extend32[31:0] <= inst15_0[15:0];
	end
endmodule

module RegBank(clk, RegWrite, ReadReg1, ReadReg2, WriteReg, WriteData, ReadData1, ReadData2);

	input clk;
	input RegWrite;
	
	input [4:0] ReadReg1, ReadReg2, WriteReg;
	input [31:0] WriteData;
		
	output [31:0] ReadData1, ReadData2;
	
	reg [31:0] reg_mem [0:31];

	assign ReadData1 = reg_mem[ReadReg1];
	assign ReadData2 = reg_mem[ReadReg2];
	
	always @(posedge clk) begin
		if (RegWrite == 1)
			reg_mem[WriteReg] = WriteData;
	end	
endmodule

module ALUControl (ALUOp, instruction5_0, ALUControl);
	input [1:0] ALUOp;
	input [5:0] instruction5_0;
	output reg [3:0] ALUControl;
	
	always @(ALUOp, instruction5_0) begin
	
	if(ALUOp == 0)
		ALUControl <= 2;    //LW and SW use add
	
	else if(ALUOp == 1)
		ALUControl <= 6;		// branch use subtract
	
	else
		case(instruction5_0)
			32: ALUControl <= 2; //add
			34: ALUControl <= 6; //subtract		
			36: ALUControl <= 0; //and	
			37: ALUControl <= 1; //or	
			39: ALUControl <= 12; //nor
			42: ALUControl <= 7; //slt
			default: ALUControl <= 15; //should not happen
		endcase
	end
endmodule

module ALU (ALUControl, A, B, ALUOut, Zero);

	input [3:0] ALUControl;
	input [31:0] A,B;
	
	output reg [31:0] ALUOut;
	output Zero;
	assign Zero = (ALUOut == 0);
	
	always @(ALUControl, A, B) begin
		case (ALUControl)
			0: ALUOut <= A & B;
			1: ALUOut <= A | B;
			2: ALUOut <= A + B;
			6: ALUOut <= A - B;
			7: ALUOut <= A < B ? 1:0;
			12: ALUOut <= ~(A | B);
			default: ALUOut <= 0;
		endcase
	end
endmodule

module Add_ALU(PC_add_out, ShiftOut, Add_ALUOut);
	input [31:0] PC_add_out;
	input [31:0] ShiftOut;
	
	output reg [31:0] Add_ALUOut;

	always @(*) begin
		Add_ALUOut <= PC_add_out + ShiftOut;
	end
endmodule

module AndGate(Branch, ALU_Zero, AndGateOut);
	input Branch;
	input ALU_Zero;
	output reg AndGateOut;
	always @(*) begin
		AndGateOut <= Branch && ALU_Zero;
	end
endmodule

module Mux_fetch_decode(inst20_16, inst15_11, RegDst, WriteReg);
	input [20:16] inst20_16;
	input [15:11] inst15_11;
	input RegDst;
	
	output reg [4:0] WriteReg;
	always @ (RegDst, inst20_16, inst15_11) begin
		case(RegDst) 
			0 : WriteReg <= inst20_16;
			1 : WriteReg <= inst15_11;
		endcase
	end
endmodule

module Mux_decode_exe(ALUSrc, ReadData2, Extend32, ALU_B);
	input ALUSrc;
	input [31:0] ReadData2,Extend32;	
	
	output reg [31:0] ALU_B;
	
	always @(ALUSrc, ReadData2, Extend32) begin
		case (ALUSrc)
			0: ALU_B <= ReadData2;
			1: ALU_B <= Extend32;
		endcase
	end
endmodule

module Mux_mem_writeback (ReadData, ALU_result, MemtoReg, WriteData_Reg);
	input [31:0] ReadData, ALU_result;
	input MemtoReg;	
	
	output reg [31:0] WriteData_Reg;
	
	always @(*) begin
		case (MemtoReg)
			0: WriteData_Reg <= ALU_result ;
			1: WriteData_Reg <= ReadData;
		endcase
	end
endmodule

module Mux_pc_counter (PCout, Add_ALUresult, AndGateOut, PCin);
	input [31:0] PCout, Add_ALUresult;
	input AndGateOut;	
	
	output reg [31:0] PCin;
	
	initial begin
		PCin <= 0;
	end
	
	always @(*) begin
		case (AndGateOut)
			0: PCin <= PCout ;
			1: PCin <= Add_ALUresult;
		endcase
	end
endmodule

module PC(clock, reset, PCin, PCout);

	input clock, reset;
	input [31:0] PCin;
	
	output reg [31:0] PCout;
	
	always @(posedge clock) begin
		if (reset == 1) 
			PCout <= 0;
		else 
			PCout <= PCin + 1; 
	end
endmodule


module mips_core (clk, rst, iaddr, idata, daddr, dwr, ddout, ddin);
	input  clk;
	input  rst;
	output [5:0]  iaddr;
	input  [31:0] idata;
	output [5:0]  daddr;
	output [31:0] ddout;
	input  [31:0] ddin;
	output        dwr;
	
	wire [5:0] pc;
	wire [31:0] inst;
	wire [31:0] reg_data1;
	wire [31:0] reg_data2;
	wire [4:0] wr_reg;
	wire [31:0] extend32;

	wire RegDest;
	wire Branch;
	wire MemRead;
	wire MemtoReg;
	wire [1:0] ALUop;
	wire MemWrite;
	wire ALUSrc;
	wire RegWrite;
	wire mux2_out;
	wire alu_ctrl;
	wire alu_out;
	wire alu_zero;
	wire mux3_out;
	wire and_out;
	wire add_out;


	imem i_mem(.addr(pc),.dout(idata));

	assign inst = idata;

	control_unit ctrl (.inst31_26(inst[31:26]), .RegDst(RegDest), 
					   .Branch(Branch), .MemRead(MemRead), .MemtoReg(MemToReg), 
					   .ALUOp(AluOp), .MemWrite(MemWrite), .ALUSrc(ALUSrc), .RegWrite(RegWrite));

	Mux_fetch_decode mux1(.inst20_16(inst[20:16]), .inst15_11(inst[15:11]), 
						  .RegDst(RegDest), .WriteReg(wr_reg));

	RegBank reg_mem(.clk(clk), .RegWrite(RegWrite),
					.ReadReg1(inst[25:21]), .ReadReg2(inst[20:16]),
					.WriteReg(wr_reg), .WriteData(mux3_out),
					.ReadData1(reg_data1), .ReadData2(reg_data2));

	SignExtend sign_extend (.inst15_0(inst[15:0]), .Extend32(extend32));

	Mux_decode_exe mux2 ( .ALUSrc(ALUSrc), .ReadData2(reg_data2), 
						  .Extend32(extend32), .ALU_B(mux2_out));

	ALUControl alu_control (.ALUOp(AluOp), .instruction5_0(inst[5:0]), 
							.ALUControl(alu_ctrl));

	ALU alu(.ALUControl(alu_ctrl), .A(reg_data1), .B(mux2_out), 	
			.ALUOut(alu_out), .Zero(alu_zero));

	dmem dmem (.clk(clk), .wr(MemWrite), .rd(MemRead), .addr(daddr), .din(ddin), .dout(ddout));
	

	Mux_mem_writeback mux3 (.ReadData(ddout), .ALU_result(alu_out), .MemtoReg(MemToReg), .WriteData_Reg(mux3_out));


	PC program_counter(.clock(clk), .reset(reset), .PCin(pc), .PCout(pc));


	AndGate annd(.Branch(Branch), .ALU_Zero(alu_zero), .AndGateOut(and_out));


	Add_ALU add_alu(.PC_add_out(pc), .ShiftOut(extend32), .Add_ALUOut(add_out));


	Mux_pc_counter mux_pc(.PCout(pc), .Add_ALUresult(add_out), .AndGateOut(and_out), .PCin(pc));


endmodule 

// module mips_core (clk, rst, iaddr, idata, daddr, dwr, ddout, ddin);
// 	input  clk;
// 	input  rst;
// 	output [5:0]  iaddr;
// 	input  [31:0] idata;
// 	output [5:0]  daddr;
// 	output [31:0] ddout;
// 	input  [31:0] ddin;
// 	output        dwr;
	
	
// 	//Registers
// 	reg  [31:0] FPC;
// 	reg  [31:0] DPC;
// 	wire [31:0] FIR;
// 	reg  [31:0] DIR;
	
// 	// Pipeline registers
// 	always @(posedge clk, posedge rst)
// 	begin
//     if(rst) 
// 		begin
// 			FPC = 0;
//     end
//     else 
// 		begin
// 			// Pipeline latches
// 			DPC = FPC;
// 			DIR = FIR;
// 			//x_alu_func = d_alu_func
// 			// Program counter
// 			FPC = FPC + 1;
// 		end
// 	end

// 	// Fetch
// 	assign iaddr = FPC[5:0];
// 	assign FIR = idata;
	
// 	// decode
// /*	always @(DIR)
// 	begin
// 		if (DIR[31:26] = 6'b000000)
// 		begin
// 			d_alu_func = 3'b000;
// 		end
// 	end
// */
// endmodule 