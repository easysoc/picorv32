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

// 与特定 FPGA 相关的代码，主要实现将 picosoc 的端口和 FPGA 端口绑定
module hx8kdemo (
	// 12 MHz 晶振支持 FPGA和 FTDI
	input clk,	// J3

	// 简单uart 接口，板子其实还连接了其它端口，只是我们没有使用
	output ser_tx,	// B12	FTDI 39
	input ser_rx,	// B10	FTDI 38

	// 8个led，leds[0]-leds[7] C3 B3 C4 C5 A1 A2 B4 B5
	output [7:0] leds,

	// spi接口，CS(片选) CLK(时钟) MISO(主入从出) MOSI(主出从入)
	// 当 J6 连接为31和42时(默认)，FTDI 烧写 flash，FPGA从 flash 加载位流
	// 当 J6 连接为34和12时，FTDI 直接烧写 FPGA，如果J7断开，只烧写 FPGA，否则同时烧写 falsh
	// FTDI 烧写 flash, FPGA 从 flash 加载位流是自动的，这里是为了让 flash 作为 soc 的外设被访问
	output flash_csb,	// R12
	output flash_clk,	// R11
	inout  flash_io0,	// P12 MISO
	inout  flash_io1,	// P11 MOSI
	// picosoc 支持 QSPI模式接口，默认 flash 不支持，需要替换为支持 QSPI 的 flash
	inout  flash_io2,	// T9
	inout  flash_io3,	// P8

	// 将接口引出方便调试 J3 下面
	// output debug_ser_tx,	// T1
	// output debug_ser_rx,	// R3

	// J3 上面
	// output debug_flash_csb,	// T15
	// output debug_flash_clk,	// R16
	// output debug_flash_io0,	// N12
	// output debug_flash_io1,	// P13
	// output debug_flash_io2,	// T13
	// output debug_flash_io3	// T14
);
	// 一次性定时器，当寄存器全为1时保持不变，此时 resetn 为 1,之前为 0，用于寄存器初始化
	reg [5:0] reset_cnt = 0;
	wire resetn = &reset_cnt;

	always @(posedge clk) begin
		reset_cnt <= reset_cnt + !resetn;
	end

	// 对 picosoc 是 output/input，对 SB_IO 是 input/output
	// 这里的方向命名以 picosoc 为基准
	wire flash_io0_oe, flash_io0_do, flash_io0_di;
	wire flash_io1_oe, flash_io1_do, flash_io1_di;
	wire flash_io2_oe, flash_io2_do, flash_io2_di;
	wire flash_io3_oe, flash_io3_do, flash_io3_di;

	// 因为需要对端口进行控制，需要直接使用SB_IO，此处例化4个 SB_IO
	SB_IO #(
		.PIN_TYPE(6'b 1010_01),
		.PULLUP(1'b 0)
	) flash_io_buf [3:0] (
		.PACKAGE_PIN({flash_io3, flash_io2, flash_io1, flash_io0}),
		.OUTPUT_ENABLE({flash_io3_oe, flash_io2_oe, flash_io1_oe, flash_io0_oe}),
		.D_OUT_0({flash_io3_do, flash_io2_do, flash_io1_do, flash_io0_do}),
		.D_IN_0({flash_io3_di, flash_io2_di, flash_io1_di, flash_io0_di})
	);

	wire        iomem_valid;
	reg         iomem_ready;
	wire [31:0] iomem_addr;
	wire [31:0] iomem_wdata;
	wire [3:0]  iomem_wstrb;
	reg  [31:0] iomem_rdata;

	// 通过控制gpio寄存器(低8位)，实现控制led
	reg [31:0] gpio;
	assign leds = gpio;

	always @(posedge clk) begin
		// 确保电路处于稳定状态
		if (!resetn) begin
			gpio <= 0;
		end else begin
			iomem_ready <= 0;
			// 读取并向 gpio 写入数据
			if (iomem_valid && !iomem_ready && iomem_addr[31:24] == 8'h 03) begin
				iomem_ready <= 1;
				iomem_rdata <= gpio;
				if (iomem_wstrb[0]) gpio[ 7: 0] <= iomem_wdata[ 7: 0];
				// if (iomem_wstrb[1]) gpio[15: 8] <= iomem_wdata[15: 8];
				// if (iomem_wstrb[2]) gpio[23:16] <= iomem_wdata[23:16];
				// if (iomem_wstrb[3]) gpio[31:24] <= iomem_wdata[31:24];
			end
		end
	end

	picosoc soc (
		.clk          (clk         ),
		.resetn       (resetn      ),

		.ser_tx       (ser_tx      ),
		.ser_rx       (ser_rx      ),

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

		.irq_5        (1'b0        ),
		.irq_6        (1'b0        ),
		.irq_7        (1'b0        ),

		.iomem_valid  (iomem_valid ),
		.iomem_ready  (iomem_ready ),
		.iomem_addr   (iomem_addr  ),
		.iomem_wdata  (iomem_wdata ),
		.iomem_wstrb  (iomem_wstrb ),
		.iomem_rdata  (iomem_rdata )
	);

	// assign debug_ser_tx = ser_tx;
	// assign debug_ser_rx = ser_rx;
	//
	// assign debug_flash_csb = flash_csb;
	// assign debug_flash_clk = flash_clk;
	// assign debug_flash_io0 = flash_io0_di;
	// assign debug_flash_io1 = flash_io1_di;
	// assign debug_flash_io2 = flash_io2_di;
	// assign debug_flash_io3 = flash_io3_di;
endmodule : hx8kdemo
