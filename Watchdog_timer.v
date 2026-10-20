// ============================================================================
// Module Name: watchdog_timer
// Description: Parameterized Hardware Watchdog Timer (WDT)
// 
// Features:
//  - Configurable counter width via parameter (WIDTH)
//  - Runtime programmable timeout threshold (timeout_val)
//  - Synchronous active-high clear/kick mechanism (kick)
//  - Module enable/disable control (enable)
//  - Pre-warning interrupt pulse before hard system reset
//  - Hard reset assertion when the counter expires
// ============================================================================

`timescale 1ns / 1ps

module watchdog_timer #(
    parameter WIDTH = 16  // Counter width (determines max timeout period)
)(
    input  wire              clk,          // System clock
    input  wire              rst_n,        // Active-low asynchronous reset
    input  wire              enable,       // Enable Watchdog Timer (1 = active, 0 = paused)
    input  wire              kick,         // Service/kick input (resets watchdog count to 0)
    input  wire [WIDTH-1:0]  timeout_val,  // Max timeout limit before reset occurs
    output reg               wdt_intr,     // Pre-warning interrupt (asserts at 75% threshold)
    output reg               wdt_reset     // System reset signal (asserts at 100% timeout)
);

    // Internal up-counter register
    reg [WIDTH-1:0] count;

    // Pre-warning threshold calculated at 75% (3/4) of timeout_val
    wire [WIDTH-1:0] warning_val = timeout_val - (timeout_val >> 2);

    // ------------------------------------------------------------------------
    // Watchdog Counter & Output Logic
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Asynchronous system reset state
            count     <= {WIDTH{1'b0}};
            wdt_intr  <= 1'b0;
            wdt_reset <= 1'b0;
        end else if (!enable) begin
            // When WDT is disabled: hold counter at zero and deassert flags
            count     <= {WIDTH{1'b0}};
            wdt_intr  <= 1'b0;
            wdt_reset <= 1'b0;
        end else if (kick) begin
            // Software serviced/kicked the watchdog in time: clear counter & alerts
            count     <= {WIDTH{1'b0}};
            wdt_intr  <= 1'b0;
            wdt_reset <= 1'b0;
        end else begin
            // Watchdog is running and hasn't been kicked this cycle
            if (count >= timeout_val) begin
                // Counter reached or exceeded timeout threshold: trigger hard reset
                wdt_reset <= 1'b1;
                wdt_intr  <= 1'b0;
            end else begin
                // Normal counting operation
                count     <= count + 1'b1;
                wdt_reset <= 1'b0;

                // Check for early pre-warning condition (>= 75% timeout)
                if (count >= warning_val) begin
                    wdt_intr <= 1'b1;
                end else begin
                    wdt_intr <= 1'b0;
                end
            end
        end
    end

endmodule



// ============================================================================
// Testbench: tb_watchdog_timer
// Description: Verification environment testing all operating modes:
//              1. Power-on reset behavior
//              2. Periodic kicking (healthy software operation)
//              3. Early warning interrupt & hard timeout assertion
//              4. Recovery after reset / service
// ============================================================================

`timescale 1ns / 1ps

module tb_watchdog_timer;

    // Parameters
    parameter WIDTH = 8;
    localparam CLK_PERIOD = 10; // 10ns clock period (100 MHz)

    // DUT Inputs
    reg             clk;
    reg             rst_n;
    reg             enable;
    reg             kick;
    reg [WIDTH-1:0] timeout_val;

    // DUT Outputs
    wire            wdt_intr;
    wire            wdt_reset;

    // Instantiate Design Under Test (DUT)
    watchdog_timer #(
        .WIDTH(WIDTH)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .enable     (enable),
        .kick       (kick),
        .timeout_val(timeout_val),
        .wdt_intr   (wdt_intr),
        .wdt_reset  (wdt_reset)
    );

    // ------------------------------------------------------------------------
    // Clock Generation (100 MHz)
    // ------------------------------------------------------------------------
    always #(CLK_PERIOD / 2) clk = ~clk;

    // ------------------------------------------------------------------------
    // Main Stimulus Procedure
    // ------------------------------------------------------------------------
    initial begin
        // Setup waveform dump for GTKWave / ModelSim / Vivado
        $dumpfile("wdt_waves.vcd");
        $dumpvars(0, tb_watchdog_timer);

        // Initialize signals
        clk         = 0;
        rst_n       = 0;
        enable      = 0;
        kick        = 0;
        timeout_val = 8'd20; // 20 clock cycles timeout (warning at cycle 15)

        $display("-------------------------------------------------------");
        $display("   [TESTBENCH START] Watchdog Timer Verification       ");
        $display("-------------------------------------------------------");

        // 1. Apply Reset
        #(CLK_PERIOD * 2);
        rst_n = 1;
        #(CLK_PERIOD);
        $display("[Time %0t] Test 1: Asynchronous Reset released", $time);

        // 2. Enable WDT & Test Periodic Kicking (Healthy CPU Scenario)
        enable = 1;
        $display("[Time %0t] Test 2: Watchdog enabled. Simulating healthy CPU periodic kicks...", $time);
        
        repeat (3) begin
            #(CLK_PERIOD * 8); // Wait 8 cycles (well below 20)
            @(posedge clk);
            kick = 1'b1;       // Service watchdog
            @(posedge clk);
            kick = 1'b0;
            $display("[Time %0t] -> CPU kicked WDT successfully. Counter cleared.", $time);
        end

        // Verify no false triggers occurred
        if (wdt_reset || wdt_intr) begin
            $error("[FAIL] False alert generated during periodic kicks!");
        end else begin
            $display("[PASS] No false resets occurred during healthy operation.");
        end

        // 3. Test CPU Freeze (Software Hang Scenario)
        $display("[Time %0t] Test 3: Simulating CPU lockup (kicks halted)...", $time);
        
        // Wait for interrupt pre-warning (at ~75% of 20 = cycle 15)
        @(posedge wdt_intr);
        $display("[Time %0t] -> Pre-warning interrupt asserted as expected!", $time);

        // Wait for hard timeout reset (at cycle 20)
        @(posedge wdt_reset);
        $display("[Time %0t] -> System Hard Reset asserted on timeout!", $time);

        // 4. Test Service / Recovery
        #(CLK_PERIOD * 3);
        @(posedge clk);
        kick = 1'b1; // Kick to recover/clear error
        @(posedge clk);
        kick = 1'b0;

        #(CLK_PERIOD);
        if (!wdt_reset && !wdt_intr) begin
            $display("[PASS] WDT successfully cleared and recovered after kick.");
        end else begin
            $error("[FAIL] WDT failed to clear outputs on service kick.");
        end

        $display("-------------------------------------------------------");
        $display("   [TESTBENCH FINISHED] All tests completed!          ");
        $display("-------------------------------------------------------");
        $finish;
    end

endmodule
