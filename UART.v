// ============================================================================
// Module Name: uart_tx
// Description: UART Transmitter (TX) FSM (8-N-1: 8 data bits, No parity, 1 stop bit)
// ============================================================================

`timescale 1ns / 1ps

module uart_tx #(
    parameter CLKS_PER_BIT = 434 // Clock frequency / Baud rate (e.g., 50MHz / 115200 ≈ 434)
)(
    input  wire       clk,        // System clock
    input  wire       rst_n,      // Active-low asynchronous reset
    input  wire       tx_start,   // Pulse high for 1 clock cycle to initiate transfer
    input  wire [7:0] tx_data,    // 8-bit parallel byte to transmit
    output reg        tx_line,    // Serial UART TX wire (idle high)
    output reg        tx_busy,    // High during transmission
    output reg        tx_done     // Asserted for 1 cycle when transmission finishes
);

    // FSM State Encoding
    localparam [2:0] S_IDLE  = 3'b000;
    localparam [2:0] S_START = 3'b001;
    localparam [2:0] S_DATA  = 3'b010;
    localparam [2:0] S_STOP  = 3'b011;
    localparam [2:0] S_CLEAN = 3'b100;

    reg [2:0]  state;
    reg [15:0] clk_count;
    reg [2:0]  bit_index;
    reg [7:0]  tx_shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            tx_line      <= 1'b1; // Idle line is high
            tx_busy      <= 1'b0;
            tx_done      <= 1'b0;
            clk_count    <= 16'd0;
            bit_index    <= 3'd0;
            tx_shift_reg <= 8'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    tx_line   <= 1'b1;
                    tx_done   <= 1'b0;
                    clk_count <= 16'd0;
                    bit_index <= 3'd0;

                    if (tx_start) begin
                        tx_shift_reg <= tx_data;
                        tx_busy      <= 1'b1;
                        state        <= S_START;
                    end else begin
                        tx_busy      <= 1'b0;
                    end
                end

                // Transmit Start Bit (Line pulled LOW)
                S_START: begin
                    tx_line <= 1'b0;

                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 16'd0;
                        state     <= S_DATA;
                    end
                end

                // Transmit 8 Data Bits (LSB first)
                S_DATA: begin
                    tx_line <= tx_shift_reg[bit_index];

                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 16'd0;
                        if (bit_index < 7) begin
                            bit_index <= bit_index + 1'b1;
                        end else begin
                            bit_index <= 3'd0;
                            state     <= S_STOP;
                        end
                    end
                end

                // Transmit Stop Bit (Line pushed HIGH)
                S_STOP: begin
                    tx_line <= 1'b1;

                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 16'd0;
                        state     <= S_CLEAN;
                    end
                end

                // Cleanup cycle: assert done flag
                S_CLEAN: begin
                    tx_done <= 1'b1;
                    tx_busy <= 1'b0;
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

// ============================================================================
// Module Name: uart_rx
// Description: UART Receiver (RX) FSM with mid-bit sampling & metastability protection
// ============================================================================

`timescale 1ns / 1ps

module uart_rx #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire       clk,        // System clock
    input  wire       rst_n,      // Active-low asynchronous reset
    input  wire       rx_line,    // Serial UART RX incoming wire
    output reg  [7:0] rx_data,    // Received byte
    output reg        rx_done     // Asserted for 1 cycle when valid byte is received
);

    // Double-flop synchronizer to avoid metastability from asynchronous pin
    reg rx_sync_1;
    reg rx_filtered;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync_1   <= 1'b1;
            rx_filtered <= 1'b1;
        end else begin
            rx_sync_1   <= rx_line;
            rx_filtered <= rx_sync_1;
        end
    end

    // FSM States
    localparam [2:0] S_IDLE       = 3'b000;
    localparam [2:0] S_START_WAIT = 3'b001;
    localparam [2:0] S_DATA       = 3'b010;
    localparam [2:0] S_STOP       = 3'b011;
    localparam [2:0] S_CLEAN      = 3'b100;

    reg [2:0]  state;
    reg [15:0] clk_count;
    reg [2:0]  bit_index;
    reg [7:0]  rx_shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            rx_done      <= 1'b0;
            rx_data      <= 8'd0;
            clk_count    <= 16'd0;
            bit_index    <= 3'd0;
            rx_shift_reg <= 8'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    rx_done   <= 1'b0;
                    clk_count <= 16'd0;
                    bit_index <= 3'd0;

                    // Detect falling edge (Start bit arrival)
                    if (rx_filtered == 1'b0) begin
                        state <= S_START_WAIT;
                    end
                end

                // Sample in the middle of start bit to reject glitches
                S_START_WAIT: begin
                    if (clk_count == (CLKS_PER_BIT - 1) / 2) begin
                        if (rx_filtered == 1'b0) begin
                            clk_count <= 16'd0;
                            state     <= S_DATA;
                        end else begin
                            state     <= S_IDLE; // False start glitch, abort
                        end
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                // Sample each of the 8 data bits at mid-bit period
                S_DATA: begin
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count               <= 16'd0;
                        rx_shift_reg[bit_index] <= rx_filtered; // LSB first

                        if (bit_index < 7) begin
                            bit_index <= bit_index + 1'b1;
                        end else begin
                            bit_index <= 3'd0;
                            state     <= S_STOP;
                        end
                    end
                end

                // Receive Stop Bit
                S_STOP: begin
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 16'd0;
                        // Verify valid stop bit
                        if (rx_filtered == 1'b1) begin
                            rx_data <= rx_shift_reg;
                            rx_done <= 1'b1;
                        end
                        state <= S_CLEAN;
                    end
                end

                S_CLEAN: begin
                    rx_done <= 1'b0;
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

// ============================================================================
// Module Name: uart_top
// Description: Full-duplex UART Transceiver Wrapper
// ============================================================================

`timescale 1ns / 1ps

module uart_top #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       rst_n,

    // Transmitter Interface
    input  wire       tx_start,
    input  wire [7:0] tx_data,
    output wire       tx_busy,
    output wire       tx_done,
    output wire       uart_tx_pin,

    // Receiver Interface
    input  wire       uart_rx_pin,
    output wire [7:0] rx_data,
    output wire       rx_done
);

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk     (clk),
        .rst_n   (rst_n),
        .tx_start(tx_start),
        .tx_data (tx_data),
        .tx_line (uart_tx_pin),
        .tx_busy (tx_busy),
        .tx_done (tx_done)
    );

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk    (clk),
        .rst_n  (rst_n),
        .rx_line(uart_rx_pin),
        .rx_data(rx_data),
        .rx_done(rx_done)
    );

endmodule


// ============================================================================
// Testbench: tb_uart
// Description: Loopback testbench verifying UART TX -> RX data integrity
// ============================================================================

`timescale 1ns / 1ps

module tb_uart;

    // Use smaller divider during simulation to speed up execution
    localparam CLKS_PER_BIT = 16;
    localparam CLK_PERIOD   = 10; // 100 MHz clock

    reg        clk;
    reg        rst_n;
    reg        tx_start;
    reg  [7:0] tx_data;
    wire       tx_busy;
    wire       tx_done;

    wire       uart_serial_wire; // Connects TX to RX directly
    wire [7:0] rx_data;
    wire       rx_done;

    integer    error_count = 0;

    // Instantiate Top-Level Device Under Test (DUT)
    uart_top #(.CLKS_PER_BIT(CLKS_PER_BIT)) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .tx_start   (tx_start),
        .tx_data    (tx_data),
        .tx_busy    (tx_busy),
        .tx_done    (tx_done),
        .uart_tx_pin(uart_serial_wire),
        .uart_rx_pin(uart_serial_wire), // Loopback connected
        .rx_data    (rx_data),
        .rx_done    (rx_done)
    );

    // Clock generator
    always #(CLK_PERIOD / 2) clk = ~clk;

    // Task to transmit a single byte and verify reception
    task send_and_verify;
        input [7:0] byte_to_send;
        begin
            @(posedge clk);
            tx_data  = byte_to_send;
            tx_start = 1'b1;
            @(posedge clk);
            tx_start = 1'b0;

            // Wait until receiver flags done
            @(posedge rx_done);

            if (rx_data === byte_to_send) begin
                $display("[PASS] Sent: 0x%02h ('%c') | Received: 0x%02h ('%c')", 
                          byte_to_send, byte_to_send, rx_data, rx_data);
            end else begin
                $error("[FAIL] Sent: 0x%02h | Received: 0x%02h", byte_to_send, rx_data);
                error_count = error_count + 1;
            end

            // Wait for transmitter to fully settle back into IDLE
            @(posedge tx_done);
            #(CLK_PERIOD * 5);
        end
    endtask

    initial begin
        $dumpfile("uart_waves.vcd");
        $dumpvars(0, tb_uart);

        clk      = 0;
        rst_n    = 0;
        tx_start = 0;
        tx_data  = 8'd0;

        $display("=======================================================");
        $display("   [TESTBENCH] Full-Duplex UART Loopback Test          ");
        $display("=======================================================");

        // Apply Reset
        #(CLK_PERIOD * 4);
        rst_n = 1;
        #(CLK_PERIOD * 4);

        // Test Cases: Hex values and ASCII characters
        send_and_verify(8'h55); // Alternating bit pattern: 01010101
        send_and_verify(8'hAA); // Alternating bit pattern: 10101010
        send_and_verify(8'h00); // All zeros
        send_and_verify(8'hFF); // All ones
        send_and_verify("V");
        send_and_verify("E");
        send_and_verify("R");
        send_and_verify("I");
        send_and_verify("L");
        send_and_verify("O");
        send_and_verify("G");

        $display("=======================================================");
        if (error_count == 0) begin
            $display("   ALL UART LOOPBACK TESTS PASSED!");
        end else begin
            $display("   TEST FAILED WITH %0d ERRORS.", error_count);
        end
        $display("=======================================================");

        $finish;
    end

endmodule
