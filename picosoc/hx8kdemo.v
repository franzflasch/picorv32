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

module clk_divn #(
    parameter WIDTH = 3,
    parameter N = 5)
    (clk,reset, clk_out);

    input clk;
    input reset;
    output clk_out;

    reg [WIDTH-1:0] pos_count, neg_count;
    wire [WIDTH-1:0] r_nxt;

    always @(posedge clk)
    if (reset)
    pos_count <=0;
    else if (pos_count ==N-1) pos_count <= 0;
    else pos_count<= pos_count +1;

    always @(negedge clk)
    if (reset)
    neg_count <=0;
    else  if (neg_count ==N-1) neg_count <= 0;
    else neg_count<= neg_count +1;

    assign clk_out = ((pos_count > (N>>1)) | (neg_count > (N>>1)));
endmodule


module sram (
    input wire reset,
    // 50ns max for data read/write. at 12MHz, each clock cycle is 83ns, so write in 1 cycle
	input wire clk,
    input wire write,
    input wire read,
    input wire [15:0] data_write,       // the data to write
    output wire [15:0] data_read,       // the data that's been read
    input wire [17:0] address,          // address to write to
    output wire ready,                  // high when ready for next operation
    output wire data_pins_out_en,       // when to switch between in and out on data pins

    // SRAM pins
    output wire [17:0] address_pins,    // address pins of the SRAM
    input  wire [15:0] data_pins_in,
    output wire [15:0] data_pins_out,
    output wire OE,                     // output enable - low to enable
    output wire WE,                     // write enable - low to enable
    output wire CS                      // chip select - low to enable
);

    localparam STATE_IDLE = 0;
    localparam STATE_WRITE = 1;
    localparam STATE_WRITE_SETUP = 2;
    localparam STATE_READ_SETUP = 3;
    localparam STATE_READ = 4;

    reg output_enable;
    reg write_enable;
    reg chip_select;

    reg [4:0] state;
    reg [15:0] data_read_reg;
    reg [15:0] data_write_reg;

    assign data_pins_out_en = (state == STATE_WRITE) ? 1 : 0; // turn on output pins when writing data
    assign address_pins = address;
    assign data_pins_out = data_write_reg;
    assign data_read = data_read_reg;
    assign OE = output_enable;
    assign WE = write_enable;
    assign CS = chip_select;

    assign ready = (!reset && state == STATE_IDLE) ? 1 : 0;

    initial begin
        state <= STATE_IDLE;
        output_enable <= 1;
        chip_select <= 1;
        write_enable <= 1;
    end

	always@(posedge clk) begin
        if( reset == 1 ) begin
            state <= STATE_IDLE;
            output_enable <= 1;
            chip_select <= 1;
            write_enable <= 1;
        end
        else begin
            case(state)
                STATE_IDLE: begin
                    output_enable <= 1;
                    chip_select <= 1;
                    write_enable <= 1;
                    if(write) state <= STATE_WRITE_SETUP;
                    else if(read) state <= STATE_READ_SETUP;
                end
                STATE_WRITE_SETUP: begin
                    chip_select <= 0;
                    data_write_reg <= data_write;
                    state <= STATE_WRITE;
                end
                STATE_WRITE: begin
                    write_enable <= 0;
                    state <= STATE_IDLE;
                end
                STATE_READ_SETUP: begin
                    output_enable <= 0;
                    chip_select <= 0;
                    state <= STATE_READ;
                end
                STATE_READ: begin
                    data_read_reg <= data_pins_in;
                    state <= STATE_IDLE;
                end
            endcase
        end
    end


endmodule

module hx8kdemo (
	input clk,

	output ser_tx,
	input ser_rx,

	output [1:0] leds,

	output flash_csb,
	output flash_clk,
	inout  flash_io0,
	inout  flash_io1,
	inout  flash_io2,
	inout  flash_io3,

  output [17:0] ADR,
  inout [15:0] DAT,
  output RAMOE,
  output RAMWE,
  output RAMCS
);

  reg system_clock;

  clk_divn #(.WIDTH(32), .N(8))
    clockdiv_slow(.clk(clk), .reset(0), .clk_out(system_clock));

	reg [5:0] reset_cnt = 0;
	wire resetn = &reset_cnt;

	always @(posedge system_clock) begin
		reset_cnt <= reset_cnt + !resetn;
	end

	wire flash_io0_oe, flash_io0_do, flash_io0_di;
	wire flash_io1_oe, flash_io1_do, flash_io1_di;
	wire flash_io2_oe, flash_io2_do, flash_io2_di;
	wire flash_io3_oe, flash_io3_do, flash_io3_di;

	SB_IO #(
		.PIN_TYPE(6'b 1010_01),
		.PULLUP(1'b 0)
	) flash_io_buf [3:0] (
		.PACKAGE_PIN({flash_io3, flash_io2, flash_io1, flash_io0}),
		.OUTPUT_ENABLE({flash_io3_oe, flash_io2_oe, flash_io1_oe, flash_io0_oe}),
		.D_OUT_0({flash_io3_do, flash_io2_do, flash_io1_do, flash_io0_do}),
		.D_IN_0({flash_io3_di, flash_io2_di, flash_io1_di, flash_io0_di})
	);

  wire [15:0] data_pins_in;
  wire [15:0] data_pins_out;
  wire data_pins_out_en;

  SB_IO #(
    .PIN_TYPE(6'b 1010_01),
  ) sram_data_pins [15:0] (
    .PACKAGE_PIN(DAT),
    .OUTPUT_ENABLE(data_pins_out_en),
    .D_OUT_0(data_pins_out),
    .D_IN_0(data_pins_in),
  );

  reg         extram_internal_ready;
  reg         extram_internal_write;
  reg         extram_internal_read;
  reg  [17:0] extram_internal_address;
  reg  [15:0] extram_internal_data_read;
  reg  [15:0] extram_internal_data_write;

	wire        extram_valid;
	reg         extram_ready;
	wire [3:0]  extram_wstrb;
	wire [31:0] extram_addr;
	wire [31:0] extram_wdata;
	reg  [31:0] extram_rdata;

  reg        extram_need_write;
  reg  [4:0] extram_state;
  reg [7:0]  extram_access_delay;
  reg [15:0] extram_tmp_read_val_1;
  reg [15:0] extram_tmp_read_val_2;

  localparam EXTRAM_STATE_IDLE = 0;
  localparam EXTRAM_STATE_READ = 1;
  localparam EXTRAM_STATE_WRITE = 2;

  sram sram_test(.clk(system_clock),
                 .address(extram_internal_address),
                 .data_read(extram_internal_data_read),
                 .data_write(extram_internal_data_write),
                 .write(extram_internal_write),
                 .read(extram_internal_read),
                 .reset(!resetn),
                 .ready(extram_internal_ready),

                 .data_pins_in(data_pins_in),
                 .data_pins_out(data_pins_out),
                 .data_pins_out_en(data_pins_out_en),
                 .address_pins(ADR),
                 .OE(RAMOE), .WE(RAMWE), .CS(RAMCS));

  always @(posedge system_clock)
  begin
    if(!resetn)
    begin
      extram_need_write <= 0;
      extram_ready <= 0;
      extram_internal_read <= 0;
      extram_internal_write <= 0;
      extram_access_delay <= 8'h 00;
      extram_tmp_read_val_1 <= 16'h 0000;
      extram_tmp_read_val_2 <= 16'h 0000;
      extram_state <= EXTRAM_STATE_IDLE;
    end
    else
    begin
      extram_ready <= 0;
      if(extram_internal_ready)
      begin
        case(extram_state)
          EXTRAM_STATE_IDLE:
          begin
            if (extram_valid)
            begin
              extram_tmp_read_val_1 <= 16'h 0000;
              extram_tmp_read_val_2 <= 16'h 0000;
              extram_internal_address[17:1] <= extram_addr[18:2];
              extram_internal_address[0] <= 0;
              if(extram_wstrb) extram_need_write <= 1;
              extram_state <= EXTRAM_STATE_READ;
              extram_internal_write <= 0;
              extram_internal_read <= 1;
            end
          end
          EXTRAM_STATE_READ:
          begin
            extram_access_delay <= extram_access_delay + 1;
            if (extram_access_delay == 2)
            begin
              extram_tmp_read_val_1[15:0] <= extram_internal_data_read[15:0];
              extram_internal_write <= 0;
              extram_internal_read <= 0;
            end
            if (extram_access_delay == 3)
            begin
               extram_internal_address[17:1] <= extram_addr[18:2];
               extram_internal_address[0] <= 1;
               extram_internal_write <= 0;
               extram_internal_read <= 1;
            end
            if (extram_access_delay == 5)
            begin
               extram_tmp_read_val_2[15:0] <= extram_internal_data_read[15:0];
               if(extram_need_write)
               begin
                 extram_access_delay <= 0;
                 extram_internal_write <= 0;
                 extram_internal_read <= 0;
                 extram_state <= EXTRAM_STATE_WRITE;
               end
            end
            if (extram_access_delay == 6)
            begin
               extram_access_delay <= 0;
               extram_rdata[15:0] <= extram_tmp_read_val_1[15:0];
               extram_rdata[31:16] <= extram_tmp_read_val_2[15:0];
               extram_ready <= 1;
               extram_internal_write <= 0;
               extram_internal_read <= 0;
               extram_state <= EXTRAM_STATE_IDLE;
            end
          end
          EXTRAM_STATE_WRITE:
          begin
             extram_access_delay <= extram_access_delay + 1;
             if (extram_access_delay == 2)
             begin
               if (extram_wstrb[0]) extram_tmp_read_val_1[ 7: 0] <= extram_wdata[ 7: 0];
               if (extram_wstrb[1]) extram_tmp_read_val_1[15: 8] <= extram_wdata[15: 8];
               if (extram_wstrb[2]) extram_tmp_read_val_2[ 7: 0] <= extram_wdata[23:16];
               if (extram_wstrb[3]) extram_tmp_read_val_2[15: 8] <= extram_wdata[31:24];
             end
             if (extram_access_delay == 3)
             begin
               extram_internal_address[17:1] <= extram_addr[18:2];
               extram_internal_address[0] <= 0;
               extram_internal_data_write[15:0] <= extram_tmp_read_val_1[15:0];
               extram_internal_read <= 0;
               extram_internal_write <= 1;
             end
             if (extram_access_delay == 4)
             begin
               extram_internal_read <= 0;
               extram_internal_write <= 0;
             end
             if (extram_access_delay == 5)
             begin
               extram_internal_address[17:1] <= extram_addr[18:2];
               extram_internal_address[0] <= 1;
               extram_internal_data_write[15:0] <= extram_tmp_read_val_2[15:0];
               extram_internal_read <= 0;
               extram_internal_write <= 1;
             end
             if (extram_access_delay == 6)
             begin
               extram_access_delay <= 0;
               extram_ready <= 1;
               extram_internal_write <= 0;
               extram_internal_read <= 0;
               extram_need_write <= 0;
               extram_state <= EXTRAM_STATE_IDLE;
             end
          end
        endcase
      end
    end
	end

	wire        iomem_valid;
	reg         iomem_ready;
	wire [3:0]  iomem_wstrb;
	wire [31:0] iomem_addr;
	wire [31:0] iomem_wdata;
	reg  [31:0] iomem_rdata;

	reg [31:0] gpio;
	assign leds[1:0] = gpio[7:6];

	always @(posedge system_clock) begin
		if (!resetn) begin
			gpio <= 0;
		end else begin
			iomem_ready <= 0;
			if (iomem_valid && !iomem_ready && iomem_addr[31:24] == 8'h 03) begin
				iomem_ready <= 1;
				iomem_rdata <= gpio;
				if (iomem_wstrb[0]) gpio[ 7: 0] <= iomem_wdata[ 7: 0];
				if (iomem_wstrb[1]) gpio[15: 8] <= iomem_wdata[15: 8];
				if (iomem_wstrb[2]) gpio[23:16] <= iomem_wdata[23:16];
				if (iomem_wstrb[3]) gpio[31:24] <= iomem_wdata[31:24];
			end
		end
	end

	picosoc soc (
		.clk          (system_clock),
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
		.iomem_wstrb  (iomem_wstrb ),
		.iomem_addr   (iomem_addr  ),
		.iomem_wdata  (iomem_wdata ),
		.iomem_rdata  (iomem_rdata ),

    .extram_valid (extram_valid),
    .extram_ready (extram_ready),
    .extram_wstrb (extram_wstrb),
    .extram_addr  (extram_addr),
    .extram_wdata (extram_wdata),
    .extram_rdata (extram_rdata)
	);

endmodule
