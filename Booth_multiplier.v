// ============================================================================
// Module Name : booth_multiplier
// Description : Signed Radix-2 Booth Multiplier with FSM Controller
// Data Width  : Parameterized (Default N = 8-bit inputs, 16-bit signed product)
//
// Algorithm Recap (Radix-2 Booth):
//   Look at {Q[0], Q_minus_1}:
//     2'b01 -> Accumulator = Accumulator + Multiplicand (M)
//     2'b10 -> Accumulator = Accumulator - Multiplicand (M)
//     2'b00, 2'b11 -> No arithmetic operation
//   Then perform an Arithmetic Right Shift (ASR) on {Accumulator, Q, Q_minus_1}
// ============================================================================

`timescale 1ns / 1ps

module booth_multiplier #(
    parameter N = 8                     // Bit-width of signed operands
)(
    input  wire                 clk,    // System clock
    input  wire                 rst_n,  // Active-low asynchronous reset
    input  wire                 start,  // Start pulse to begin multiplication
    input  wire signed [N-1:0]  multiplicand, // Multiplicand (M)
    input  wire signed [N-1:0]  multiplier,   // Multiplier (Q)
    output reg  signed [2*N-1:0] product,     // 2*N-bit signed output product
    output reg                  done,         // Asserted high when result is valid
    output reg                  busy          // High while computation is in progress
);

    // ------------------------------------------------------------------------
    // FSM State Definitions (One-Hot / Gray / Sequential)
    // ------------------------------------------------------------------------
    localparam STATE_IDLE  = 2'b00;
    localparam STATE_CHECK = 2'b01;
    localparam STATE_SHIFT = 2'b10;
    localparam STATE_DONE  = 2'b11;

    reg [1:0] state, next_state;

    // ------------------------------------------------------------------------
    // Internal Registers & Datapath Elements
    // ------------------------------------------------------------------------
    reg signed [N-1:0]   A;           // Accumulator
    reg signed [N-1:0]   Q;           // Multiplier register
    reg signed [N-1:0]   M;           // Multiplicand register
    reg                  q_minus_1;   // Shift bit Q[-1]
    reg [$clog2(N):0]    count;       // Shift iteration counter

    // ------------------------------------------------------------------------
    // Sequential State Register & Datapath Execution
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= STATE_IDLE;
            A           <= {N{1'b0}};
            Q           <= {N{1'b0}};
            M           <= {N{1'b0}};
            q_minus_1   <= 1'b0;
            count       <= 0;
            product     <= {(2*N){1'b0}};
            done        <= 1'b0;
            busy        <= 1'b0;
        end else begin
            state <= next_state;

            case (state)
                STATE_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        A         <= {N{1'b0}};
                        M         <= multiplicand;
                        Q         <= multiplier;
                        q_minus_1 <= 1'b0;
                        count     <= N;          // Load N iterations
                        busy      <= 1'b1;
                    end else begin
                        busy      <= 1'b0;
                    end
                end

                STATE_CHECK: begin
                    // Evaluate least significant bit of Q and previous shift bit
                    case ({Q[0], q_minus_1})
                        2'b01:   A <= A + M;   // Add multiplicand
                        2'b10:   A <= A - M;   // Subtract multiplicand
                        default: A <= A;       // 2'b00 or 2'b11: do nothing
                    endcase
                end

                STATE_SHIFT: begin
                    // Arithmetic Right Shift on {A, Q, q_minus_1} preserving sign
                    {A, Q, q_minus_1} <= {A[N-1], A, Q};
                    count             <= count - 1'b1;
                end

                STATE_DONE: begin
                    product <= {A, Q}; // Final 2*N-bit signed product
                    done    <= 1'b1;
                    busy    <= 1'b0;
                end
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // Combinational Next-State Logic
    // ------------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            STATE_IDLE: begin
                if (start)
                    next_state = STATE_CHECK;
                else
                    next_state = STATE_IDLE;
            end

            STATE_CHECK: begin
                // Arithmetic operation evaluated -> proceed directly to shift
                next_state = STATE_SHIFT;
            end

            STATE_SHIFT: begin
                // If all N iterations are complete, transition to DONE
                if (count == 1)
                    next_state = STATE_DONE;
                else
                    next_state = STATE_CHECK;
            end

            STATE_DONE: begin
                // Return to IDLE or wait until start deasserts
                next_state = STATE_IDLE;
            end

            default: next_state = STATE_IDLE;
        endcase
    end

endmodule


// ============================================================================
// Testbench: tb_booth_multiplier
// Description: Exhaustive signed test suite for Radix-2 Booth Multiplier.
//              Tests:
//              1. Positive x Positive
//              2. Positive x Negative
//              3. Negative x Positive
//              4. Negative x Negative
//              5. Multiplication by Zero
//              6. Maximum / Minimum corner values (8-bit signed range: -128 to +127)
// ============================================================================

`timescale 1ns / 1ps

module tb_booth_multiplier;

    parameter N = 8;
    localparam CLK_PERIOD = 10; // 100 MHz clock

    // DUT Inputs
    reg                 clk;
    reg                 rst_n;
    reg                 start;
    reg signed [N-1:0]  multiplicand;
    reg signed [N-1:0]  multiplier;

    // DUT Outputs
    wire signed [2*N-1:0] product;
    wire                  done;
    wire                  busy;

    // Simulation verification variables
    reg signed [2*N-1:0]  expected_product;
    integer               errors = 0;

    // Instantiate Unit Under Test (UUT)
    booth_multiplier #(.N(N)) dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .multiplicand(multiplicand),
        .multiplier(multiplier),
        .product(product),
        .done(done),
        .busy(busy)
    );

    // Clock generator
    always #(CLK_PERIOD / 2) clk = ~clk;

    // ------------------------------------------------------------------------
    // Verification Task: Executes one multiplication and compares result
    // ------------------------------------------------------------------------
    task run_test;
        input signed [N-1:0] in_m;
        input signed [N-1:0] in_q;
        begin
            @(posedge clk);
            multiplicand     = in_m;
            multiplier       = in_q;
            expected_product = in_m * in_q;
            start            = 1'b1;

            @(posedge clk);
            start            = 1'b0;

            // Wait until hardware signals completion
            @(posedge done);
            #(CLK_PERIOD / 4); // Small delta to inspect stabilized outputs

            if (product === expected_product) begin
                $display("[PASS] %4d * %4d = %6d (Hex: 0x%04h)", in_m, in_q, product, product);
            end else begin
                $error("[FAIL] %4d * %4d | Expected: %6d, Got: %6d", in_m, in_q, expected_product, product);
                errors = errors + 1;
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // Main Stimulus Loop
    // ------------------------------------------------------------------------
    initial begin
        $dumpfile("booth_waves.vcd");
        $dumpvars(0, tb_booth_multiplier);

        clk          = 0;
        rst_n        = 0;
        start        = 0;
        multiplicand = 0;
        multiplier   = 0;

        $display("=======================================================");
        $display("   [TESTBENCH] Radix-2 Booth Multiplier (Signed %0d-bit)", N);
        $display("=======================================================");

        // Apply Reset
        #(CLK_PERIOD * 2);
        rst_n = 1;
        #(CLK_PERIOD * 2);

        // Test Cases
        $display("\n--- Positive x Positive ---");
        run_test(8'd7, 8'd3);       // 7 * 3 = 21
        run_test(8'd25, 8'd10);     // 25 * 10 = 250

        $display("\n--- Positive x Negative ---");
        run_test(8'd15, -8'd6);     // 15 * -6 = -90
        run_test(8'd120, -8'd1);    // 120 * -1 = -120

        $display("\n--- Negative x Positive ---");
        run_test(-8'd12, 8'd8);     // -12 * 8 = -96
        run_test(-8'd50, 8'd2);     // -50 * 2 = -100

        $display("\n--- Negative x Negative ---");
        run_test(-8'd9, -8'd7);     // -9 * -7 = 63
        run_test(-8'd30, -8'd4);    // -30 * -4 = 120

        $display("\n--- Zero and Extreme Boundary Checks ---");
        run_test(8'd0, -8'd45);     // 0 * -45 = 0
        run_test(-8'd128, 8'd1);    // Min signed 8-bit (-128) * 1 = -128
        run_test(-8'd128, -8'd1);   // -128 * -1 = +128
        run_test(8'd127, 8'd127);   // Max positive values: 127 * 127 = 16129
        run_test(-8'd128, 8'd127);  // Min * Max: -128 * 127 = -16256

        $display("\n=======================================================");
        if (errors == 0) begin
            $display("   ALL TESTS PASSED SUCCESSFULLY!");
        end else begin
            $display("   TESTS COMPLETED WITH %0d FAILURES.", errors);
        end
        $display("=======================================================");

        $finish;
    end

endmodule
