/*
 *  PicoSoC - A simple example SoC using PicoRV32
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

// 通过 PICORV32_REGS 开关实现外部可定义regfile，如使用 block sram
`ifndef PICORV32_REGS
`define PICORV32_REGS picosoc_regs
`endif

`ifndef PICOSOC_MEM
`define PICOSOC_MEM picosoc_mem
`endif

// `include "simpleuart.v"
// `include "spimemio.v"
// `include "../picorv32.sv"

// this macro can be used to check if the verilog files in your
// design are read in the correct order.
`define PICOSOC_V

// FPGA 无关，主要实现将 picorv32 内核同其它外设如 Memory、UART、SPI flash 等通过总线或内存地址映射连接起来
// 这里使用基于 valid/ready 的内存映射
module picosoc (
	input clk,
	input resetn,	// 高电平时电路达到稳定状态，否则应该使寄存器为 0

	// GPIO or memory mapped peripherals（address >= 0x02000000） 访问接口
	// hx8kdemo 示例中主要用来控制 led，uart 和 flash 不使用此处端口而是通过额外的接口访问
	// 这里的方向命名以 picorv32 为基准
	output        iomem_valid,
	input         iomem_ready,	// 用于告诉 picorv32，外设准备好接收数据
	output [31:0] iomem_addr,
	output [31:0] iomem_wdata,
	output [ 3:0] iomem_wstrb,
	input  [31:0] iomem_rdata,

	input  irq_5,
	input  irq_6,
	input  irq_7,

	output ser_tx,
	input  ser_rx,

	output flash_csb,
	output flash_clk,

	output flash_io0_oe,
	output flash_io1_oe,
	output flash_io2_oe,
	output flash_io3_oe,

	output flash_io0_do,
	output flash_io1_do,
	output flash_io2_do,
	output flash_io3_do,

	input  flash_io0_di,
	input  flash_io1_di,
	input  flash_io2_di,
	input  flash_io3_di
);
	parameter [0:0] BARREL_SHIFTER = 1;
	parameter [0:0] ENABLE_MULDIV = 1;
	parameter [0:0] ENABLE_COMPRESSED = 1;
	parameter [0:0] ENABLE_COUNTERS = 1;
	parameter [0:0] ENABLE_IRQ_QREGS = 0;

	// 声明sram大小、终止地址（1KB，栈起始地址）及程序起始地址（PC初始化地址：0x0010_0000=1048576=1MB，前面存放位流文件）
	parameter integer MEM_WORDS = 256;	// sram 大小(按字)，因为 sarm 地址从 0 开始，实际减 1
	// sram 和 flash 区域应当和 sections.lds 的设置一致
	parameter [31:0] STACKADDR = (4*MEM_WORDS);       // sarm 结束地址
	parameter [31:0] PROGADDR_RESET = 32'h 0010_0000; // 2的20次方 = 1 MB
	parameter [31:0] PROGADDR_IRQ = 32'h 0000_0000;

	// picorv32 支持 32 位的中断向量，因always语法规定声明为reg，实际为wire类型
	reg [31:0] irq;
	wire irq_stall = 0;
	wire irq_uart = 0;

	// 外部中断定义，其中前 3 个为 built-in
	// 0	Timer Interrupt
	// 1	EBREAK/ECALL or Illegal Instruction
	// 2	BUS Error (Unalign Memory Access)
	always @* begin
		irq = 0;
		irq[3] = irq_stall;
		irq[4] = irq_uart;	// 没有使用
		irq[5] = irq_5;
		irq[6] = irq_6;
		irq[7] = irq_7;
	end

	// 将 picorv32 对应的端口接出来
	wire mem_instr;	// picosoc 没有使用
	wire mem_valid;
	wire mem_ready;
	wire [31:0] mem_addr;
	wire [31:0] mem_wdata;
	wire [3:0] mem_wstrb;
	wire [31:0] mem_rdata;

	// spi flash ready和数据返回信号
	wire spimem_ready;
	wire [31:0] spimem_rdata;

	// sram ready和数据返回信号
	reg ram_ready;
	wire [31:0] ram_rdata;

	/** Memory map
		注意：实际 SRAM 和 flash 物理尺寸一般小于内存映射规定的最大值
		Address Range	Description
		0x00000000 .. 0x01FFFFFF	SRAM 和 Flash 共享，其中 SRAM < 4*MEM_WORDS
		0x02000000 .. 0x02000003	SPI Flash Controller Config Register
		0x02000004 .. 0x02000007	UART Clock Divider Register
		0x02000008 .. 0x0200000B	UART Send/Recv Data Register
		0x03000000 .. 0xFFFFFFFF	Memory mapped user peripherals
	*/
	// Reading from the addresses in the internal SRAM region beyond the end of the physical SRAM will
	// read from the corresponding addresses in serial flash.
	// picorv32 准备好写数据，且写地址大于 0x01FFFFFF
	assign iomem_valid = mem_valid && (mem_addr[31:24] > 8'h 01);
	assign iomem_addr = mem_addr;
	assign iomem_wdata = mem_wdata;
	assign iomem_wstrb = mem_wstrb;

	// SPI Flash Controller Config Register（32 bits）,see Memory map
	wire spimemio_cfgreg_sel = mem_valid && (mem_addr == 32'h 0200_0000);
	wire [31:0] spimemio_cfgreg_do;

	// UART Clock Divider Register（32 bits）,see Memory map
	wire        simpleuart_reg_div_sel = mem_valid && (mem_addr == 32'h 0200_0004);
	wire [31:0] simpleuart_reg_div_do;

	// UART Send/Recv Data Register（32 bits）,see Memory map
	wire        simpleuart_reg_dat_sel = mem_valid && (mem_addr == 32'h 0200_0008);
	wire [31:0] simpleuart_reg_dat_do;
	wire        simpleuart_reg_dat_wait;

	// 用于判断 sram、flash、spi、uart等外设之一是否准备好接收从 picorv32 发送过来的数据
	// 因picorv32同这些模块交互使用同一 memory 访问接口，因此需要根据不同情况获取ready信号，共6种情况
	assign mem_ready = (iomem_valid && iomem_ready)	// 访问非存储器且外设准备好接收数据
		|| spimem_ready // spi falsh 准备好接收数据
		|| ram_ready 	// sram 准备好接收数据
		|| spimemio_cfgreg_sel // 地址是 SPI Flash Controller Config Register 即可
		|| simpleuart_reg_div_sel
		|| (simpleuart_reg_dat_sel && !simpleuart_reg_dat_wait);

	// 用于从区分从 sram、flash、spi、uart等外设读取的数据
	// mem_valid 和 mem_addr 一起实现了类似片选的功能
	assign mem_rdata = (iomem_valid && iomem_ready) ? iomem_rdata :
			spimem_ready ? spimem_rdata :
			ram_ready ? ram_rdata :
			spimemio_cfgreg_sel ? spimemio_cfgreg_do :
			simpleuart_reg_div_sel ? simpleuart_reg_div_do :
			simpleuart_reg_dat_sel ? simpleuart_reg_dat_do : 32'h 0000_0000;

	// picorv32 通过 valid/ready 接口和 SRAM、SPI Flash、Uart、GPIO 交互
	picorv32 #(
		.STACKADDR(STACKADDR),
		.PROGADDR_RESET(PROGADDR_RESET),
		.PROGADDR_IRQ(PROGADDR_IRQ),
		.BARREL_SHIFTER(BARREL_SHIFTER),
		.COMPRESSED_ISA(ENABLE_COMPRESSED),
		.ENABLE_COUNTERS(ENABLE_COUNTERS),
		.ENABLE_MUL(ENABLE_MULDIV),
		.ENABLE_DIV(ENABLE_MULDIV),
		.ENABLE_IRQ(1),
		.ENABLE_IRQ_QREGS(ENABLE_IRQ_QREGS)
	) cpu (
		.clk         (clk        ),
		.resetn      (resetn     ),
		.mem_valid   (mem_valid  ),	// output
		.mem_instr   (mem_instr  ),	// output
		.mem_ready   (mem_ready  ),	// input
		.mem_addr    (mem_addr   ),	// output
		.mem_wdata   (mem_wdata  ),	// output
		.mem_wstrb   (mem_wstrb  ),	// output
		.mem_rdata   (mem_rdata  ), // input
		.irq         (irq        )  // input
	);

	// picorv32 通过 valid/ready 接口和 spimemio 交互进而访问 flash
	spimemio spimemio (
		.clk    (clk),
		.resetn (resetn),
		.valid  (mem_valid && mem_addr >= 4*MEM_WORDS && mem_addr < 32'h 0200_0000),
		.ready  (spimem_ready),
		// sram和flash地址重叠，且最大为 0x00FFFFFF，即实际只使用了24位地址空间
		.addr   (mem_addr[23:0]),
		.rdata  (spimem_rdata),

		.flash_csb    (flash_csb   ),
		.flash_clk    (flash_clk   ),

		.flash_io0_oe (flash_io0_oe),
		.flash_io1_oe (flash_io1_oe),
		.flash_io2_oe (flash_io2_oe),
		.flash_io3_oe (flash_io3_oe),

		.flash_io0_do (flash_io0_do),
		.flash_io1_do (flash_io1_do),
		.flash_io2_do (flash_io2_do),
		.flash_io3_do (flash_io3_do),

		.flash_io0_di (flash_io0_di),
		.flash_io1_di (flash_io1_di),
		.flash_io2_di (flash_io2_di),
		.flash_io3_di (flash_io3_di),

		.cfgreg_we(spimemio_cfgreg_sel ? mem_wstrb : 4'b 0000),
		.cfgreg_di(mem_wdata),
		.cfgreg_do(spimemio_cfgreg_do)
	);

	simpleuart simpleuart (
		.clk         (clk         ),
		.resetn      (resetn      ),

		.ser_tx      (ser_tx      ),
		.ser_rx      (ser_rx      ),

		.reg_div_we  (simpleuart_reg_div_sel ? mem_wstrb : 4'b 0000),
		.reg_div_di  (mem_wdata),
		.reg_div_do  (simpleuart_reg_div_do),

		.reg_dat_we  (simpleuart_reg_dat_sel ? mem_wstrb[0] : 1'b 0),
		.reg_dat_re  (simpleuart_reg_dat_sel && !mem_wstrb),
		.reg_dat_di  (mem_wdata),
		.reg_dat_do  (simpleuart_reg_dat_do),
		.reg_dat_wait(simpleuart_reg_dat_wait)
	);

	always @(posedge clk)
		ram_ready <= mem_valid && !mem_ready && mem_addr < 4*MEM_WORDS;

	`PICOSOC_MEM #(
		.WORDS(MEM_WORDS)
	) memory (
		.clk(clk),
		.wen((mem_valid && !mem_ready && mem_addr < 4*MEM_WORDS) ? mem_wstrb : 4'b0),
		.addr(mem_addr[23:2]), // 地址高8位为io访问（既不是访问sram也不是访问flash），见Memory map
		.wdata(mem_wdata),
		.rdata(ram_rdata)
	);
endmodule : picosoc

// Implementation note:
// Replace the following two modules with wrappers for your SRAM cells.
// 可通过显式的例化sram实现下述模块，否则由综合器自动推断

// 包含32个32位寄存器的3端口 register file
module picosoc_regs (
	input clk, wen,
	input [5:0] waddr,
	input [5:0] raddr1,
	input [5:0] raddr2,
	input [31:0] wdata,
	output [31:0] rdata1,
	output [31:0] rdata2
);
	// start.s 通常会初始化 register file，这里不需要硬件初始化
	reg [31:0] regs [0:31];

	always @(posedge clk)
		if (wen) regs[waddr[4:0]] <= wdata;

	assign rdata1 = regs[raddr1[4:0]];
	assign rdata2 = regs[raddr2[4:0]];
endmodule : picosoc_regs

// 默认 256*32 bits 即 1KB 大小的双端口sram，写使能（wen）为4位以支持按字节写入
module picosoc_mem #(
	parameter integer WORDS = 256
) (
	input clk,
	input [3:0] wen,
	// addr 必须 word 对齐，对应 mem_addr[23:2]，因为 SRAM 大小 WORDS 可自定义，所以这里不可严格限制地址位数，仅限制为小于 mem_addr[31:24]
	// 超过 SRAM 地址大小的地址对应的从 flash 访问
	input [21:0] addr,
	input [31:0] wdata,
	output reg [31:0] rdata
);
	reg [31:0] mem [0:WORDS-1];

	always @(posedge clk) begin
		// 每次读取一个字，即 4 字节，写支持 mask
		rdata <= mem[addr];
		if (wen[0]) mem[addr][ 7: 0] <= wdata[ 7: 0];
		if (wen[1]) mem[addr][15: 8] <= wdata[15: 8];
		if (wen[2]) mem[addr][23:16] <= wdata[23:16];
		if (wen[3]) mem[addr][31:24] <= wdata[31:24];
	end
endmodule : picosoc_mem

