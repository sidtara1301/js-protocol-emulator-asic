`default_nettype none

// =============================================================================
// Package: isa_pkg
// Purpose: Defines the 16-bit Instruction Set Architecture (ISA), Opcodes,
//          Branch Conditions, Side-Set fields, and Host SPI Commands for the
//          Protocol Emulator ASIC.
// =============================================================================


// =============================================================================
// Module: sync_fifo
// Purpose: Area-efficient synchronous circular FIFO for buffering TX bytes
//          (Host -> Emulator OSR) and RX bytes (Emulator ISR -> Host).
//          Uses a combinational read-before-pop interface (rdata = mem_q[rptr_q])
//          and supports simultaneous push+pop when full.
// =============================================================================
module sync_fifo #(
    parameter int unsigned DATA_WIDTH = 8,
    parameter int unsigned DEPTH      = 4,
    parameter int unsigned PTR_WIDTH  = 2
) (
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic                  clear,
    input  logic                  push,
    input  logic [DATA_WIDTH-1:0] wdata,
    input  logic                  pop,
    output logic [DATA_WIDTH-1:0] rdata,
    output logic                  full,
    output logic                  empty,
    output logic                  overflow
);

    logic [DATA_WIDTH-1:0] mem_q [0:DEPTH-1];
    logic [PTR_WIDTH-1:0]  wptr_q;
    logic [PTR_WIDTH-1:0]  rptr_q;
    logic [PTR_WIDTH:0]    count_q;

    assign empty    = (count_q == '0);
    localparam logic [PTR_WIDTH:0] DEPTH_VAL = DEPTH[PTR_WIDTH:0];
    assign full     = (count_q == DEPTH_VAL);
    assign rdata    = mem_q[rptr_q];

    wire do_pop     = pop && !empty;
    wire do_push    = push && (!full || do_pop);
    assign overflow = push && full && !do_pop;

    integer i;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr_q  <= '0;
            rptr_q  <= '0;
            count_q <= '0;
            for (i = 0; i < DEPTH; i = i + 1) begin
                mem_q[i] <= '0;
            end
        end else if (clear) begin
            wptr_q  <= '0;
            rptr_q  <= '0;
            count_q <= '0;
        end else begin
            case ({do_push, do_pop})
                2'b10: begin
                    mem_q[wptr_q] <= wdata;
                    wptr_q        <= wptr_q + 1'b1;
                    count_q       <= count_q + 1'b1;
                end
                2'b01: begin
                    rptr_q  <= rptr_q + 1'b1;
                    count_q <= count_q - 1'b1;
                end
                2'b11: begin
                    mem_q[wptr_q] <= wdata;
                    wptr_q        <= wptr_q + 1'b1;
                    rptr_q        <= rptr_q + 1'b1;
                end
                default: ;
            endcase
        end
    end

endmodule : sync_fifo


// =============================================================================
// Module: sram_wrapper
// Purpose: Instantiates the IHP 130nm (SG13G2 / CMOS5L) 1024x32 Single-Port
//          SRAM Macro (RM_IHPSG13_1P_1024x32_c2_bm_bist).
//          Packs two 16-bit instructions per 32-bit SRAM word using addr[0]
//          and 16-bit bit-masks, providing 2,048 instructions with 0% waste.
// =============================================================================
module sram_wrapper (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ren,        // Read Enable
    input  logic        wen,        // Write Enable
    input  logic [10:0] addr,       // 11-bit instruction address (0..2047)
    input  logic [15:0] wdata,      // 16-bit instruction write data
    output logic [15:0] rdata       // 16-bit instruction read data
);

    logic [31:0] sram_dout;
    logic [31:0] bit_mask;
    logic        addr_lsb_q;

    // Select upper [31:16] or lower [15:0] half-word based on addr[0]
    assign bit_mask = addr[0] ? 32'hFFFF_0000 : 32'h0000_FFFF;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            addr_lsb_q <= 1'b0;
        end else if (ren && !wen) begin
            addr_lsb_q <= addr[0];
        end
    end

`ifdef SIM_BEHAVIORAL_SRAM
    // Behavioral 1024x32 model with bit-mask support for standalone RTL simulation
    logic [31:0] sim_mem [0:1023];
    logic [31:0] rdata_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rdata_q <= 32'h0000_0000;
        end else begin
            if (wen) begin
                sim_mem[addr[10:1]] <= (sim_mem[addr[10:1]] & ~bit_mask) |
                                       ({wdata, wdata} & bit_mask);
            end
            if (ren && !wen) begin
                rdata_q <= sim_mem[addr[10:1]];
            end
        end
    end
    assign sram_dout = rdata_q;
`else
    // Physical IHP 1024x32 Single-Port SRAM Macro Instantiation
    // Matches exact port list of IHP OpenPDK RM_IHPSG13_1P_1024x32_c2_bm_bist
    RM_IHPSG13_1P_1024x32_c2_bm_bist u_ihp_sram (
        .A_CLK        (clk),
        .A_MEN        (ren | wen),
        .A_WEN        (wen),
        .A_REN        (ren & ~wen),
        .A_ADDR       (addr[10:1]),
        .A_DIN        ({wdata, wdata}),
        .A_DLY        (1'b1),
        .A_DOUT       (sram_dout),
        .A_BM         (bit_mask),

        // Tie off BIST interface for normal ASIC operation
        .A_BIST_CLK   (1'b0),
        .A_BIST_EN    (1'b0),
        .A_BIST_MEN   (1'b0),
        .A_BIST_WEN   (1'b0),
        .A_BIST_REN   (1'b0),
        .A_BIST_ADDR  (10'd0),
        .A_BIST_DIN   (32'd0),
        .A_BIST_BM    (32'd0)
    );
`endif

assign rdata = addr_lsb_q ? sram_dout[31:16] : sram_dout[15:0];

endmodule : sram_wrapper


// =============================================================================
// Module: bootloader_host_if
// Purpose: Metastability-safe SPI Slave & Command Bridge (f_clk >= 8 * f_SPI).
//          Key Safeguards:
//          1. CMD_READ_RX only pops an RX FIFO byte upon completion of an 8-bit
//             data byte transfer (never on command reception).
//          2. CMD_WRITE_IMEM is strictly rejected if the CPU is currently running,
//             setting sticky status bit imem_err_sticky_q (status[5]).
//          3. Exiting load_mode does NOT auto-start the CPU; explicit CMD_CTRL
//             START is required.
// =============================================================================
module bootloader_host_if (
    input  logic        clk,
    input  logic        rst_n,

    // External Host SPI & Mode Pins
    input  logic        load_mode,      // 1 = Raw sequential IMEM load, 0 = Command SPI mode
    input  logic        spi_cs_n,       // Active-low Chip Select from host MCU
    input  logic        spi_sck,        // Serial Clock from host MCU (SPI Mode 0)
    input  logic        spi_mosi,       // Serial Data In from host MCU
    output logic        spi_miso,       // Serial Data Out to host MCU

    // SRAM Programming & CPU Control Interface
    output logic        sram_wen,
    output logic [10:0] sram_addr,
    output logic [15:0] sram_wdata,
    output logic        cpu_enable,
    output logic        cpu_reset,
    output logic        fifo_clear,
    input  logic        cpu_halted,

    // Runtime Host FIFO Interface
    output logic        tx_fifo_push,
    output logic [7:0]  tx_fifo_wdata,
    input  logic        tx_fifo_full,
    input  logic        tx_fifo_empty,
    input  logic        tx_fifo_ovf,
    output logic        rx_fifo_pop,
    input  logic [7:0]  rx_fifo_rdata,
    input  logic        rx_fifo_empty,
    input  logic        rx_fifo_full,
    input  logic        rx_fifo_ovf
);
    

    // 3-bit Opcode Definitions (8 core instructions)
    typedef enum logic [2:0] {
        OP_SET  = 3'b000, // Drive GPIO[3:0] pins using 4-bit mask and 4-bit value
        OP_WAIT = 3'b001, // Stall execution until selected GPIO pin matches polarity
        OP_OUT  = 3'b010, // Shift 1 bit from OSR to pin (supports LSB/MSB-first & Open-Drain)
        OP_IN   = 3'b011, // Sample 1 bit from selected GPIO pin into ISR (LSB/MSB-first)
        OP_JMP  = 3'b100, // Conditional or unconditional jump (supports X-- loop counter)
        OP_PULL = 3'b101, // Pull 8-bit byte from Host TX FIFO (or 7-bit imm) into OSR
        OP_PUSH = 3'b110, // Push 8-bit byte from ISR into Host RX FIFO
        OP_MOV  = 3'b111  // Configure Scratch X, Pin Directions, Clock Divider, Page/OD/Halt
    } opcode_e;

    // 2-bit Jump Condition Encoding (stored in operand[7:6] during OP_JMP)
    typedef enum logic [1:0] {
        JMP_ALWAYS   = 2'b00, // Unconditional branch to {pc_page_q, operand[5:0]}
        JMP_PIN_LOW  = 2'b01, // Branch if gpio_in[jmp_pin_sel_q] == 0
        JMP_PIN_HIGH = 2'b10, // Branch if gpio_in[jmp_pin_sel_q] == 1
        JMP_X_NZ_DEC = 2'b11  // Branch if scratch counter X != 0, and post-decrement X
    } jmp_cond_e;

    // 2-bit MOV Target Encoding (stored in operand[7:6] during OP_MOV)
    typedef enum logic [1:0] {
        MOV_DEST_X      = 2'b00, // Load 6-bit immediate into Scratch Loop Counter X
        MOV_DEST_PINDIR = 2'b01, // Load 6-bit immediate into GPIO Output Enable gpio_oe_q[5:0]
        MOV_DEST_CLKDIV = 2'b10, // Load 6-bit immediate into Baud Rate Clock Prescaler
        MOV_DEST_SYS    = 2'b11  // Sub-decoded by operand[5:3]:
                                 //   0xx: pc_page_q <= operand[4:0] (32 pages x 64 words = 2048 insts)
                                 //   100: jmp_pin_sel_q <= operand[2:0] (select pin 0..7 for OP_JMP)
                                 //   101: gpio_oe_q[7:6] <= operand[1:0] (configure upper 2 GPIO OEs)
                                 //   110: open_drain_mask_q[operand[2:0]] <= 1'b1 (lock pin as open-drain)
                                 //   111: state_q <= ST_HALTED (clean firmware HALT)
    } mov_dest_e;

    // Host SPI Command Opcodes (1st byte after CS_N asserts low when load_mode == 0)
    // Note on Host SPI Timing: Because SCK is synchronized into the ASIC clock domain,
    // the external host must satisfy f_clk >= 8 * f_SPI.
    typedef enum logic [7:0] {
        CMD_NOP        = 8'h00,
        CMD_WRITE_IMEM = 8'h01, // 16-bit start addr + 16-bit inst words (ONLY allowed when stopped)
        CMD_WRITE_TX   = 8'h02, // Stream 8-bit payload bytes into Host TX FIFO
        CMD_READ_RX    = 8'h03, // Stream 8-bit payload bytes out of Host RX FIFO
        CMD_CTRL       = 8'h04, // 1-byte control: bit[0]=run, bit[1]=reset_cpu, bit[2]=clear_fifo
        CMD_STATUS     = 8'h05  // Read 8-bit status: {halted, running, imem_err, fifo_ovf, rx_f, rx_e, tx_f, tx_e}
    } spi_cmd_e;

    // 16-bit Packed Instruction Format (Reference Layout)
    // +------------+-------+--------+-----------+--------------+
    // | Bits 15:13 | Bit12 | Bit 11 | Bits 10:8 |  Bits 7:0    |
    // |   opcode   | ss_en | ss_val |   delay   |   operand    |
    // +------------+-------+--------+-----------+--------------+
    // OP_OUT operand[7:0] layout:
    //   operand[7]   = 1: Open-Drain mode, 0: Push-Pull mode
    //   operand[6]   = 1: MSB-first (osr_q[7]), 0: LSB-first (osr_q[0])
    //   operand[5:3] = Reserved (0)
    //   operand[2:0] = Target GPIO pin index (0..7)
// Synchronizers for asynchronous Host pins (including load_mode)
    logic [1:0] load_mode_sync_q;
    logic [1:0] cs_n_sync_q;
    logic [2:0] sck_sync_q;
    logic [1:0] mosi_sync_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            load_mode_sync_q <= 2'b00;
            cs_n_sync_q      <= 2'b11;
            sck_sync_q       <= 3'b000;
            mosi_sync_q      <= 2'b00;
        end else begin
            load_mode_sync_q <= {load_mode_sync_q[0], load_mode};
            cs_n_sync_q      <= {cs_n_sync_q[0], spi_cs_n};
            sck_sync_q       <= {sck_sync_q[1:0], spi_sck};
            mosi_sync_q      <= {mosi_sync_q[0], spi_mosi};
        end
    end

    wire sync_load_mode = load_mode_sync_q[1];
    wire cs_active      = ~cs_n_sync_q[1];
    wire sck_rising     = (sck_sync_q[2:1] == 2'b01);
    wire sck_falling    = (sck_sync_q[2:1] == 2'b10);
    wire mosi_bit       = mosi_sync_q[1];

    typedef enum logic [2:0] {
        SPI_IDLE         = 3'd0,
        SPI_IMEM_ADDR    = 3'd1,
        SPI_IMEM_DATA    = 3'd2,
        SPI_WRITE_TX     = 3'd3,
        SPI_READ_RX      = 3'd4,
        SPI_CTRL_BYTE    = 3'd5,
        SPI_STATUS_OUT   = 3'd6,
        SPI_IGNORE_FRAME = 3'd7
    } spi_state_e;

    spi_state_e  spi_state_q;
    logic [10:0] boot_addr_q;
    logic [15:0] shift_in_q;
    wire         _unused_shift_in_msb = shift_in_q[15];
    logic [7:0]  shift_out_q;
    logic        load_miso_on_fall_q;
    logic [3:0]  bit_cnt_q;
    logic        cpu_run_reg_q;
    logic        prev_load_mode_q;
    logic        imem_err_sticky_q;
    logic        tx_ovf_sticky_q;
    logic        rx_ovf_sticky_q;

    // CPU is only actively running when enabled and not halted
    assign cpu_enable = cpu_run_reg_q &&
                        !cpu_halted &&
                        !sync_load_mode &&
                        !sram_wen &&
                        (spi_state_q != SPI_IMEM_ADDR) &&
                        (spi_state_q != SPI_IMEM_DATA);

    // 8-bit Status Word:
    //   [7] = HALTED      (1 when CPU reached ST_HALTED)
    //   [6] = RUNNING     (1 when CPU is actively executing; 0 when stopped or halted)
    //   [5] = IMEM_ERR    (1 if CMD_WRITE_IMEM was attempted while CPU was running)
    //   [4] = FIFO_OVF    (1 if TX or RX FIFO overflowed)
    //   [3] = RX_FULL
    //   [2] = RX_EMPTY
    //   [1] = TX_FULL
    //   [0] = TX_EMPTY
    wire [7:0] status_byte = {
        cpu_halted,
        cpu_enable,
        imem_err_sticky_q,
        (tx_ovf_sticky_q | rx_ovf_sticky_q),
        rx_fifo_full,
        rx_fifo_empty,
        tx_fifo_full,
        tx_fifo_empty
    };

    wire [7:0]  rx_byte_in = {shift_in_q[6:0], mosi_bit};
    wire [15:0] rx_word_in = {shift_in_q[14:0], mosi_bit};

    assign spi_miso = cs_active ? shift_out_q[7] : 1'b0;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_state_q         <= SPI_IDLE;
            boot_addr_q         <= 11'd0;
            shift_in_q          <= 16'd0;
            shift_out_q         <= 8'd0;
            load_miso_on_fall_q <= 1'b0;
            bit_cnt_q           <= 4'd0;
            sram_wen            <= 1'b0;
            sram_addr           <= 11'd0;
            sram_wdata          <= 16'd0;
            cpu_run_reg_q       <= 1'b0;
            prev_load_mode_q    <= 1'b0;
            cpu_reset           <= 1'b0;
            fifo_clear          <= 1'b0;
            tx_fifo_push        <= 1'b0;
            tx_fifo_wdata       <= 8'd0;
            rx_fifo_pop         <= 1'b0;
            imem_err_sticky_q   <= 1'b0;
            tx_ovf_sticky_q     <= 1'b0;
            rx_ovf_sticky_q     <= 1'b0;
        end else begin
            // Default single-cycle pulse resets
            sram_wen         <= 1'b0;
            tx_fifo_push     <= 1'b0;
            rx_fifo_pop      <= 1'b0;
            cpu_reset        <= 1'b0;
            fifo_clear       <= 1'b0;
            prev_load_mode_q <= sync_load_mode;

            // Automatically clear run request when CPU halts so HALTED and RUNNING
            // are mutually exclusive and CPU is in a stopped state after HALT
            if (cpu_halted) begin
                cpu_run_reg_q <= 1'b0;
            end

            // Latch sticky FIFO overflow flags
            if (tx_fifo_ovf) tx_ovf_sticky_q <= 1'b1;
            if (rx_fifo_ovf) rx_ovf_sticky_q <= 1'b1;

            // Entering load_mode stops and resets the CPU and resets boot_addr_q.
            // Exiting load_mode does NOT auto-start the CPU (requires explicit CMD_CTRL START).
            if (sync_load_mode && !prev_load_mode_q) begin
                boot_addr_q   <= 11'd0;
                cpu_run_reg_q <= 1'b0;
                cpu_reset     <= 1'b1;
                fifo_clear    <= 1'b1;
            end

            if (sync_load_mode) begin
                // Pin-Controlled Sequential 16-bit Bootloader Mode (CPU guaranteed stopped)
                spi_state_q <= SPI_IDLE;
                if (!cs_active) begin
                    bit_cnt_q <= 4'd0;
                end else if (sck_rising) begin
                    shift_in_q <= rx_word_in;
                    bit_cnt_q  <= bit_cnt_q + 4'd1;
                    if (bit_cnt_q == 4'd15) begin
                        sram_wdata  <= rx_word_in;
                        sram_addr   <= boot_addr_q;
                        sram_wen    <= 1'b1;
                        boot_addr_q <= boot_addr_q + 11'd1;
                        bit_cnt_q   <= 4'd0;
                    end
                end
            end else begin
                // Command-Based SPI Interface
                if (!cs_active) begin
                    spi_state_q         <= SPI_IDLE;
                    bit_cnt_q           <= 4'd0;
                    shift_out_q         <= status_byte;
                    load_miso_on_fall_q <= 1'b0;
                end else begin
                    if (sck_rising) begin
                        shift_in_q <= rx_word_in;
                        bit_cnt_q  <= bit_cnt_q + 4'd1;

                        case (spi_state_q)
                            SPI_IDLE: begin
                                // First 8 bits decode the SPI Command Header
                                if (bit_cnt_q == 4'd7) begin
                                    bit_cnt_q <= 4'd0;
                                    case (rx_byte_in)
                                        CMD_WRITE_IMEM: begin
                                            if (cpu_run_reg_q) begin
                                                // Reject IMEM write while CPU is running!
                                                imem_err_sticky_q <= 1'b1;
                                                spi_state_q       <= SPI_IGNORE_FRAME;
                                            end else begin
                                                spi_state_q       <= SPI_IMEM_ADDR;
                                            end
                                        end
                                        CMD_WRITE_TX: begin
                                            spi_state_q <= SPI_WRITE_TX;
                                        end
                                        CMD_READ_RX: begin
                                            // Stage head RX FIFO byte on next falling edge WITHOUT popping yet!
                                            spi_state_q         <= SPI_READ_RX;
                                            load_miso_on_fall_q <= 1'b1;
                                        end
                                        CMD_CTRL: begin
                                            spi_state_q <= SPI_CTRL_BYTE;
                                        end
                                        CMD_STATUS: begin
                                            spi_state_q         <= SPI_STATUS_OUT;
                                            load_miso_on_fall_q <= 1'b1;
                                        end
                                        default: begin
                                            spi_state_q <= SPI_IGNORE_FRAME;
                                        end
                                    endcase
                                end
                            end

                            SPI_IMEM_ADDR: begin
                                // 16-bit starting instruction address (0..2047)
                                if (bit_cnt_q == 4'd15) begin
                                    boot_addr_q <= rx_word_in[10:0];
                                    bit_cnt_q   <= 4'd0;
                                    spi_state_q <= SPI_IMEM_DATA;
                                end
                            end

                            SPI_IMEM_DATA: begin
                                // Stream 16-bit instructions sequentially into SRAM
                                if (bit_cnt_q == 4'd15) begin
                                    sram_wdata  <= rx_word_in;
                                    sram_addr   <= boot_addr_q;
                                    sram_wen    <= 1'b1;
                                    boot_addr_q <= boot_addr_q + 11'd1;
                                    bit_cnt_q   <= 4'd0;
                                end
                            end

                            SPI_WRITE_TX: begin
                                // Stream 8-bit bytes into TX FIFO without touching RX FIFO
                                if (bit_cnt_q == 4'd7) begin
                                    tx_fifo_wdata <= rx_byte_in;
                                    tx_fifo_push  <= 1'b1;
                                    bit_cnt_q     <= 4'd0;
                                end
                            end

                            SPI_READ_RX: begin
                                // Commit FIFO pop ONLY when an 8-bit data byte read completes!
                                if (bit_cnt_q == 4'd7) begin
                                    rx_fifo_pop         <= ~rx_fifo_empty;
                                    load_miso_on_fall_q <= 1'b1;
                                    bit_cnt_q           <= 4'd0;
                                end
                            end

                            SPI_CTRL_BYTE: begin
                                // Bit 0: START(1)/STOP(0), Bit 1: RESET_CPU, Bit 2: CLEAR_FIFO
                                if (bit_cnt_q == 4'd7) begin
                                    cpu_run_reg_q <= rx_byte_in[0];
                                    cpu_reset     <= rx_byte_in[1];
                                    fifo_clear    <= rx_byte_in[2];
                                    if (rx_byte_in[1]) begin
                                        imem_err_sticky_q <= 1'b0;
                                    end
                                    if (rx_byte_in[1] || rx_byte_in[2]) begin
                                        tx_ovf_sticky_q <= 1'b0;
                                        rx_ovf_sticky_q <= 1'b0;
                                    end
                                    bit_cnt_q <= 4'd0;
                                end
                            end

                            SPI_STATUS_OUT: begin
                                if (bit_cnt_q == 4'd7) begin
                                    load_miso_on_fall_q <= 1'b1;
                                    bit_cnt_q           <= 4'd0;
                                end
                            end

                            SPI_IGNORE_FRAME: begin
                                // Ignore trailing bits until CS_N deasserts
                                bit_cnt_q <= 4'd0;
                            end

                            default: spi_state_q <= SPI_IDLE;
                        endcase
                    end else if (sck_falling) begin
                        if (load_miso_on_fall_q) begin
                            if (spi_state_q == SPI_READ_RX) begin
                                shift_out_q <= rx_fifo_empty ? 8'h00 : rx_fifo_rdata;
                            end else begin
                                shift_out_q <= status_byte;
                            end
                            load_miso_on_fall_q <= 1'b0;
                        end else begin
                            shift_out_q <= {shift_out_q[6:0], 1'b0};
                        end
                    end
                end
            end
        end
    end

endmodule : bootloader_host_if


// =============================================================================
// Module: execution_core
// Purpose: Deterministic Protocol Engine with a Uniform 3-Phase Protocol-Tick
//          Timing Contract:
//            - 1 Protocol Tick = 3 * (clkdiv_max_q + 1) system clocks.
//            - Every non-stalled instruction with delay=0 takes 1 Protocol Tick.
//            - Every instruction with delay=D takes (1 + D) Protocol Ticks.
//            - Pin updates & Side-Set always occur on Sub-Cycle 2 (proto_tick_en).
//            - Supports LSB-first and MSB-first OP_OUT and persistent hardware
//              open-drain boundary protection (open_drain_mask_q).
// =============================================================================
module execution_core (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        enable,         // Run enable from host controller (gated by ena)
    input  logic        cpu_reset,      // Explicit synchronous architectural reset
    output logic        cpu_halted,     // Asserted when firmware executes HALT

    // SRAM Instruction Fetch Interface
    output logic        sram_ren,
    output logic [10:0] sram_addr,
    input  logic [15:0] sram_rdata,

    // Host Data FIFOs
    output logic        tx_fifo_pop,
    input  logic [7:0]  tx_fifo_rdata,
    input  logic        tx_fifo_empty,
    output logic        rx_fifo_push,
    output logic [7:0]  rx_fifo_wdata,
    input  logic        rx_fifo_full,

    // Physical Protocol GPIO & Side-Set Signals
    input  logic [7:0]  gpio_in,        // Synchronized input pins
    output logic [7:0]  gpio_out,       // Driven output levels (clamped by open_drain_mask_q)
    output logic [7:0]  gpio_oe,        // Output Enable (1 = Output, 0 = High-Z Input)
    output logic        side_set_pin    // Dedicated Side-Set clock/control pin (SCLK)
);
    

    // 3-bit Opcode Definitions (8 core instructions)
    typedef enum logic [2:0] {
        OP_SET  = 3'b000, // Drive GPIO[3:0] pins using 4-bit mask and 4-bit value
        OP_WAIT = 3'b001, // Stall execution until selected GPIO pin matches polarity
        OP_OUT  = 3'b010, // Shift 1 bit from OSR to pin (supports LSB/MSB-first & Open-Drain)
        OP_IN   = 3'b011, // Sample 1 bit from selected GPIO pin into ISR (LSB/MSB-first)
        OP_JMP  = 3'b100, // Conditional or unconditional jump (supports X-- loop counter)
        OP_PULL = 3'b101, // Pull 8-bit byte from Host TX FIFO (or 7-bit imm) into OSR
        OP_PUSH = 3'b110, // Push 8-bit byte from ISR into Host RX FIFO
        OP_MOV  = 3'b111  // Configure Scratch X, Pin Directions, Clock Divider, Page/OD/Halt
    } opcode_e;

    // 2-bit Jump Condition Encoding (stored in operand[7:6] during OP_JMP)
    typedef enum logic [1:0] {
        JMP_ALWAYS   = 2'b00, // Unconditional branch to {pc_page_q, operand[5:0]}
        JMP_PIN_LOW  = 2'b01, // Branch if gpio_in[jmp_pin_sel_q] == 0
        JMP_PIN_HIGH = 2'b10, // Branch if gpio_in[jmp_pin_sel_q] == 1
        JMP_X_NZ_DEC = 2'b11  // Branch if scratch counter X != 0, and post-decrement X
    } jmp_cond_e;

    // 2-bit MOV Target Encoding (stored in operand[7:6] during OP_MOV)
    typedef enum logic [1:0] {
        MOV_DEST_X      = 2'b00, // Load 6-bit immediate into Scratch Loop Counter X
        MOV_DEST_PINDIR = 2'b01, // Load 6-bit immediate into GPIO Output Enable gpio_oe_q[5:0]
        MOV_DEST_CLKDIV = 2'b10, // Load 6-bit immediate into Baud Rate Clock Prescaler
        MOV_DEST_SYS    = 2'b11  // Sub-decoded by operand[5:3]:
                                 //   0xx: pc_page_q <= operand[4:0] (32 pages x 64 words = 2048 insts)
                                 //   100: jmp_pin_sel_q <= operand[2:0] (select pin 0..7 for OP_JMP)
                                 //   101: gpio_oe_q[7:6] <= operand[1:0] (configure upper 2 GPIO OEs)
                                 //   110: open_drain_mask_q[operand[2:0]] <= 1'b1 (lock pin as open-drain)
                                 //   111: state_q <= ST_HALTED (clean firmware HALT)
    } mov_dest_e;

    // Host SPI Command Opcodes (1st byte after CS_N asserts low when load_mode == 0)
    // Note on Host SPI Timing: Because SCK is synchronized into the ASIC clock domain,
    // the external host must satisfy f_clk >= 8 * f_SPI.
    typedef enum logic [7:0] {
        CMD_NOP        = 8'h00,
        CMD_WRITE_IMEM = 8'h01, // 16-bit start addr + 16-bit inst words (ONLY allowed when stopped)
        CMD_WRITE_TX   = 8'h02, // Stream 8-bit payload bytes into Host TX FIFO
        CMD_READ_RX    = 8'h03, // Stream 8-bit payload bytes out of Host RX FIFO
        CMD_CTRL       = 8'h04, // 1-byte control: bit[0]=run, bit[1]=reset_cpu, bit[2]=clear_fifo
        CMD_STATUS     = 8'h05  // Read 8-bit status: {halted, running, imem_err, fifo_ovf, rx_f, rx_e, tx_f, tx_e}
    } spi_cmd_e;

    // 16-bit Packed Instruction Format (Reference Layout)
    // +------------+-------+--------+-----------+--------------+
    // | Bits 15:13 | Bit12 | Bit 11 | Bits 10:8 |  Bits 7:0    |
    // |   opcode   | ss_en | ss_val |   delay   |   operand    |
    // +------------+-------+--------+-----------+--------------+
    // OP_OUT operand[7:0] layout:
    //   operand[7]   = 1: Open-Drain mode, 0: Push-Pull mode
    //   operand[6]   = 1: MSB-first (osr_q[7]), 0: LSB-first (osr_q[0])
    //   operand[5:3] = Reserved (0)
    //   operand[2:0] = Target GPIO pin index (0..7)
typedef enum logic [1:0] {
        ST_EXECUTE   = 2'd0, // Normal 3-phase instruction fetch & execution
        ST_WAIT_COND = 2'd1, // Poll pin condition for OP_WAIT or FIFO stall
        ST_DELAY     = 2'd2, // Stall for inst_delay Uniform Protocol Ticks
        ST_HALTED    = 2'd3  // Processor halted via MOV_DEST_SYS HALT command
    } core_state_e;

    core_state_e  state_q;
    logic [10:0]  pc_q;              // 11-bit Program Counter (0..2047)
    logic [4:0]   pc_page_q;         // 5-bit Page Selector for 6-bit local jumps
    logic [2:0]   jmp_pin_sel_q;     // Selectable GPIO pin (0..7) for JMP_PIN_LOW/HIGH
    logic [15:0]  inst_latch_q;      // Latched 16-bit instruction for multi-tick WAIT/DELAY

    // Internal GPIO registers and persistent Open-Drain hardware safety mask
    logic [7:0]   gpio_out_q;
    logic [7:0]   gpio_oe_q;
    logic [7:0]   open_drain_mask_q; // 1 = Pin is open-drain (can NEVER actively drive HIGH)

    // Hardware-enforced open-drain boundary protection:
    // Any pin with open_drain_mask_q[i] == 1 has its output level clamped to 0,
    // and attempting to output a '1' automatically releases gpio_oe[i] to High-Z (0).
    assign gpio_out = gpio_out_q & ~open_drain_mask_q;
    assign gpio_oe  = gpio_oe_q  & (~open_drain_mask_q | ~gpio_out_q);

    // Unpack active 16-bit instruction word via explicit wire slices
    wire  [15:0]  active_word  = (state_q == ST_EXECUTE) ? sram_rdata : inst_latch_q;
    wire  [2:0]   inst_opcode  = active_word[15:13];
    wire          inst_ss_en   = active_word[12];
    wire          inst_ss_val  = active_word[11];
    wire  [2:0]   inst_delay   = active_word[10:8];
    wire  [7:0]   inst_operand = active_word[7:0];

    // Shift and Scratch Registers
    logic [7:0]   osr_q;         // Output Shift Register
    logic [7:0]   isr_q;         // Input Shift Register
    logic [5:0]   reg_x_q;       // Scratch Loop Counter Register X
    logic [2:0]   delay_cnt_q;   // Post-instruction delay counter (in Protocol Ticks)

    // Uniform 3-Phase Protocol-Tick Prescaler
    logic [5:0]   clkdiv_max_q;
    logic [5:0]   clkdiv_cnt_q;
    logic [1:0]   subcycle_q;
    logic         prescale_en;
    logic         proto_tick_en;

    assign prescale_en   = (clkdiv_cnt_q == 6'd0);
    assign proto_tick_en = prescale_en && (subcycle_q == 2'd2);
    assign sram_addr     = pc_q;
    assign cpu_halted    = (state_q == ST_HALTED);

    // Issue synchronous SRAM read during Sub-Cycle 0 so sram_rdata is stable by Sub-Cycle 1 & 2
    assign sram_ren      = enable && (state_q == ST_EXECUTE) && (subcycle_q == 2'd0) && prescale_en;

    // Helper signals for OP_JMP, OP_MOV, OP_OUT, and OP_WAIT
    wire [1:0] jmp_cond       = inst_operand[7:6];
    wire [1:0] mov_dest       = inst_operand[7:6];
    wire [2:0] target_pin_idx = inst_operand[2:0];
    wire       out_bit_val    = inst_operand[6] ? osr_q[7] : osr_q[0]; // [6]=1: MSB-first, [6]=0: LSB-first
    wire       wait_pin_match = (gpio_in[target_pin_idx] == inst_operand[7]);
    wire       is_halt_cmd    = (inst_opcode == OP_MOV) &&
                                (mov_dest == MOV_DEST_SYS) &&
                                (inst_operand[5:3] == 3'b111);

    logic branch_taken;
    always_comb begin
        branch_taken = 1'b0;
        case (jmp_cond)
            JMP_ALWAYS:   branch_taken = 1'b1;
            JMP_PIN_LOW:  branch_taken = (gpio_in[jmp_pin_sel_q] == 1'b0);
            JMP_PIN_HIGH: branch_taken = (gpio_in[jmp_pin_sel_q] == 1'b1);
            JMP_X_NZ_DEC: branch_taken = (reg_x_q != 6'd0);
            default:      branch_taken = 1'b0;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q           <= ST_EXECUTE;
            pc_q              <= 11'd0;
            pc_page_q         <= 5'd0;
            jmp_pin_sel_q     <= 3'd0;
            inst_latch_q      <= 16'd0;
            osr_q             <= 8'd0;
            isr_q             <= 8'd0;
            reg_x_q           <= 6'd0;
            delay_cnt_q       <= 3'd0;
            clkdiv_max_q      <= 6'd0;
            clkdiv_cnt_q      <= 6'd0;
            subcycle_q        <= 2'd0;
            gpio_out_q        <= 8'h00;
            gpio_oe_q         <= 8'h00; // Safe High-Z default on reset
            open_drain_mask_q <= 8'h00;
            side_set_pin      <= 1'b0;
            tx_fifo_pop       <= 1'b0;
            rx_fifo_push      <= 1'b0;
            rx_fifo_wdata     <= 8'd0;
        end else if (cpu_reset) begin
            // Full architectural reset without erasing SRAM
            state_q           <= ST_EXECUTE;
            pc_q              <= 11'd0;
            pc_page_q         <= 5'd0;
            jmp_pin_sel_q     <= 3'd0;
            inst_latch_q      <= 16'd0;
            osr_q             <= 8'd0;
            isr_q             <= 8'd0;
            reg_x_q           <= 6'd0;
            delay_cnt_q       <= 3'd0;
            clkdiv_max_q      <= 6'd0;
            clkdiv_cnt_q      <= 6'd0;
            subcycle_q        <= 2'd0;
            gpio_out_q        <= 8'h00;
            gpio_oe_q         <= 8'h00;
            open_drain_mask_q <= 8'h00;
            side_set_pin      <= 1'b0;
            tx_fifo_pop       <= 1'b0;
            rx_fifo_push      <= 1'b0;
            rx_fifo_wdata     <= 8'd0;
        end else if (!enable) begin
            // Pause execution cleanly when stopped or deselected
            tx_fifo_pop  <= 1'b0;
            rx_fifo_push <= 1'b0;
        end else begin
            // Clear single-cycle FIFO strobes
            tx_fifo_pop  <= 1'b0;
            rx_fifo_push <= 1'b0;

            // Advance Prescaler and 3-Phase Sub-Cycle Counter
            if (state_q != ST_HALTED) begin
                if (clkdiv_cnt_q == 6'd0) begin
                    if (subcycle_q == 2'd2) begin
                        subcycle_q <= 2'd0;
                        if (state_q == ST_EXECUTE &&
                            inst_opcode == OP_MOV &&
                            mov_dest == MOV_DEST_CLKDIV) begin
                            clkdiv_cnt_q <= inst_operand[5:0];
                        end else begin
                            clkdiv_cnt_q <= clkdiv_max_q;
                        end
                    end else begin
                        subcycle_q   <= subcycle_q + 2'd1;
                        clkdiv_cnt_q <= clkdiv_max_q;
                    end
                end else begin
                    clkdiv_cnt_q <= clkdiv_cnt_q - 6'd1;
                end
            end

            // Execute State Machine on Uniform Protocol-Tick Boundary (Sub-Cycle 2)
            if (proto_tick_en) begin
                case (state_q)
                    ST_EXECUTE: begin
                        inst_latch_q <= active_word;

                        // 1. CONCURRENT SIDE-SET EXECUTION
                        if (inst_ss_en) begin
                            side_set_pin <= inst_ss_val;
                        end

                        // 2. PRIMARY OPCODE EXECUTION
                        case (inst_opcode)
                            OP_SET: begin
                                // operand[7:4] = 4-bit pin mask, operand[3:0] = 4-bit pin values
                                gpio_out_q[3:0] <= (gpio_out_q[3:0] & ~inst_operand[7:4]) |
                                                   (inst_operand[3:0] & inst_operand[7:4]);
                            end

                            OP_WAIT: begin
                                // Evaluated below in stall & next-state logic
                            end

                            OP_OUT: begin
                                // operand[7] = Open-Drain mode, operand[6] = MSB-first (1) / LSB-first (0)
                                if (inst_operand[7] || open_drain_mask_q[target_pin_idx]) begin
                                    open_drain_mask_q[target_pin_idx] <= 1'b1;
                                    gpio_out_q[target_pin_idx]        <= 1'b0;
                                    gpio_oe_q[target_pin_idx]         <= ~out_bit_val;
                                end else begin
                                    gpio_out_q[target_pin_idx]        <= out_bit_val;
                                end

                                if (inst_operand[6]) begin
                                    // MSB-first shift (SPI / I2C)
                                    osr_q <= {osr_q[6:0], 1'b0};
                                end else begin
                                    // LSB-first shift (UART)
                                    osr_q <= {1'b0, osr_q[7:1]};
                                end
                            end

                            OP_IN: begin
                                if (inst_operand[7]) begin
                                    // MSB-first framing (SPI / I2C)
                                    isr_q <= {isr_q[6:0], gpio_in[target_pin_idx]};
                                end else begin
                                    // LSB-first framing (UART)
                                    isr_q <= {gpio_in[target_pin_idx], isr_q[7:1]};
                                end
                            end

                            OP_JMP: begin
                                if (jmp_cond == JMP_X_NZ_DEC && reg_x_q != 6'd0) begin
                                    reg_x_q <= reg_x_q - 6'd1;
                                end
                            end

                            OP_PULL: begin
                                if (inst_operand[7]) begin
                                    osr_q <= {1'b0, inst_operand[6:0]};
                                end else if (!tx_fifo_empty) begin
                                    osr_q       <= tx_fifo_rdata;
                                    tx_fifo_pop <= 1'b1;
                                end
                            end

                            OP_PUSH: begin
                                if (!rx_fifo_full) begin
                                    rx_fifo_wdata <= isr_q;
                                    rx_fifo_push  <= 1'b1;
                                    isr_q         <= 8'd0;
                                end
                            end

                            OP_MOV: begin
                                case (mov_dest)
                                    MOV_DEST_X:      reg_x_q        <= inst_operand[5:0];
                                    MOV_DEST_PINDIR: gpio_oe_q[5:0] <= inst_operand[5:0];
                                    MOV_DEST_CLKDIV: clkdiv_max_q   <= inst_operand[5:0];
                                    MOV_DEST_SYS: begin
                                        if (!inst_operand[5]) begin
                                            pc_page_q <= inst_operand[4:0];
                                        end else begin
                                            case (inst_operand[4:3])
                                                2'b00: jmp_pin_sel_q                        <= inst_operand[2:0];
                                                2'b01: gpio_oe_q[7:6]                       <= inst_operand[1:0];
                                                2'b10: open_drain_mask_q[inst_operand[2:0]] <= 1'b1;
                                                default: ;
                                            endcase
                                        end
                                    end
                                    default: ;
                                endcase
                            end

                            default: ;
                        endcase

                        // 3. STALL, HALT & NEXT-STATE DETERMINATION
                        if (is_halt_cmd) begin
                            state_q <= ST_HALTED;
                        end else if ((inst_opcode == OP_WAIT && !wait_pin_match) ||
                                     (inst_opcode == OP_PULL && !inst_operand[7] && tx_fifo_empty) ||
                                     (inst_opcode == OP_PUSH && rx_fifo_full)) begin
                            state_q <= ST_WAIT_COND;
                        end else begin
                            if (inst_opcode == OP_JMP && branch_taken) begin
                                pc_q <= {pc_page_q, inst_operand[5:0]};
                            end else begin
                                pc_q <= pc_q + 11'd1;
                            end

                            if (inst_delay > 3'd0) begin
                                delay_cnt_q <= inst_delay;
                                state_q     <= ST_DELAY;
                            end else begin
                                state_q     <= ST_EXECUTE;
                            end
                        end
                    end

                    ST_WAIT_COND: begin
                        if ((inst_opcode == OP_WAIT && wait_pin_match) ||
                            (inst_opcode == OP_PULL && !tx_fifo_empty) ||
                            (inst_opcode == OP_PUSH && !rx_fifo_full)) begin

                            if (inst_opcode == OP_PULL) begin
                                osr_q       <= tx_fifo_rdata;
                                tx_fifo_pop <= 1'b1;
                            end else if (inst_opcode == OP_PUSH) begin
                                rx_fifo_wdata <= isr_q;
                                rx_fifo_push  <= 1'b1;
                                isr_q         <= 8'd0;
                            end

                            pc_q <= pc_q + 11'd1;
                            if (inst_delay > 3'd0) begin
                                delay_cnt_q <= inst_delay;
                                state_q     <= ST_DELAY;
                            end else begin
                                state_q     <= ST_EXECUTE;
                            end
                        end
                    end

                    ST_DELAY: begin
                        if (delay_cnt_q == 3'd1) begin
                            state_q <= ST_EXECUTE;
                        end
                        delay_cnt_q <= delay_cnt_q - 3'd1;
                    end

                    ST_HALTED: begin
                        // Remain halted until cpu_reset is asserted
                        state_q <= ST_HALTED;
                    end

                    default: state_q <= ST_EXECUTE;
                endcase
            end
        end
    end

endmodule : execution_core


// =============================================================================
// Module: protocol_emulator_top
// Purpose: Integrates the Command-Based SPI Bootloader/Host Bridge, Synchronous
//          TX/RX FIFOs, Deterministic Execution Core, Input Synchronizers, and
//          the 2048-instruction IHP 1024x32 SRAM Macro Wrapper.
// =============================================================================
module protocol_emulator_top (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       ena,              // Tile enable from Tiny Tapeout wrapper

    // External Host Control & SPI Interface
    input  logic       host_load_mode,   // 1 = Raw sequential SRAM load, 0 = Command SPI mode
    input  logic       host_spi_cs_n,
    input  logic       host_spi_sck,
    input  logic       host_spi_mosi,
    output logic       host_spi_miso,

    // FIFO Status Flags for Host Interrupts / Polling
    output logic       tx_fifo_full,
    output logic       rx_fifo_empty,

    // Physical Protocol Interface (UART / SPI / I2C / Custom)
    input  logic [7:0] protocol_rx_pins,
    output logic [7:0] protocol_tx_pins,
    output logic [7:0] protocol_oe_pins,
    output logic       side_set_clk_pin
);
    

    // 3-bit Opcode Definitions (8 core instructions)
    typedef enum logic [2:0] {
        OP_SET  = 3'b000, // Drive GPIO[3:0] pins using 4-bit mask and 4-bit value
        OP_WAIT = 3'b001, // Stall execution until selected GPIO pin matches polarity
        OP_OUT  = 3'b010, // Shift 1 bit from OSR to pin (supports LSB/MSB-first & Open-Drain)
        OP_IN   = 3'b011, // Sample 1 bit from selected GPIO pin into ISR (LSB/MSB-first)
        OP_JMP  = 3'b100, // Conditional or unconditional jump (supports X-- loop counter)
        OP_PULL = 3'b101, // Pull 8-bit byte from Host TX FIFO (or 7-bit imm) into OSR
        OP_PUSH = 3'b110, // Push 8-bit byte from ISR into Host RX FIFO
        OP_MOV  = 3'b111  // Configure Scratch X, Pin Directions, Clock Divider, Page/OD/Halt
    } opcode_e;

    // 2-bit Jump Condition Encoding (stored in operand[7:6] during OP_JMP)
    typedef enum logic [1:0] {
        JMP_ALWAYS   = 2'b00, // Unconditional branch to {pc_page_q, operand[5:0]}
        JMP_PIN_LOW  = 2'b01, // Branch if gpio_in[jmp_pin_sel_q] == 0
        JMP_PIN_HIGH = 2'b10, // Branch if gpio_in[jmp_pin_sel_q] == 1
        JMP_X_NZ_DEC = 2'b11  // Branch if scratch counter X != 0, and post-decrement X
    } jmp_cond_e;

    // 2-bit MOV Target Encoding (stored in operand[7:6] during OP_MOV)
    typedef enum logic [1:0] {
        MOV_DEST_X      = 2'b00, // Load 6-bit immediate into Scratch Loop Counter X
        MOV_DEST_PINDIR = 2'b01, // Load 6-bit immediate into GPIO Output Enable gpio_oe_q[5:0]
        MOV_DEST_CLKDIV = 2'b10, // Load 6-bit immediate into Baud Rate Clock Prescaler
        MOV_DEST_SYS    = 2'b11  // Sub-decoded by operand[5:3]:
                                 //   0xx: pc_page_q <= operand[4:0] (32 pages x 64 words = 2048 insts)
                                 //   100: jmp_pin_sel_q <= operand[2:0] (select pin 0..7 for OP_JMP)
                                 //   101: gpio_oe_q[7:6] <= operand[1:0] (configure upper 2 GPIO OEs)
                                 //   110: open_drain_mask_q[operand[2:0]] <= 1'b1 (lock pin as open-drain)
                                 //   111: state_q <= ST_HALTED (clean firmware HALT)
    } mov_dest_e;

    // Host SPI Command Opcodes (1st byte after CS_N asserts low when load_mode == 0)
    // Note on Host SPI Timing: Because SCK is synchronized into the ASIC clock domain,
    // the external host must satisfy f_clk >= 8 * f_SPI.
    typedef enum logic [7:0] {
        CMD_NOP        = 8'h00,
        CMD_WRITE_IMEM = 8'h01, // 16-bit start addr + 16-bit inst words (ONLY allowed when stopped)
        CMD_WRITE_TX   = 8'h02, // Stream 8-bit payload bytes into Host TX FIFO
        CMD_READ_RX    = 8'h03, // Stream 8-bit payload bytes out of Host RX FIFO
        CMD_CTRL       = 8'h04, // 1-byte control: bit[0]=run, bit[1]=reset_cpu, bit[2]=clear_fifo
        CMD_STATUS     = 8'h05  // Read 8-bit status: {halted, running, imem_err, fifo_ovf, rx_f, rx_e, tx_f, tx_e}
    } spi_cmd_e;

    // 16-bit Packed Instruction Format (Reference Layout)
    // +------------+-------+--------+-----------+--------------+
    // | Bits 15:13 | Bit12 | Bit 11 | Bits 10:8 |  Bits 7:0    |
    // |   opcode   | ss_en | ss_val |   delay   |   operand    |
    // +------------+-------+--------+-----------+--------------+
    // OP_OUT operand[7:0] layout:
    //   operand[7]   = 1: Open-Drain mode, 0: Push-Pull mode
    //   operand[6]   = 1: MSB-first (osr_q[7]), 0: LSB-first (osr_q[0])
    //   operand[5:3] = Reserved (0)
    //   operand[2:0] = Target GPIO pin index (0..7)
// Internal SRAM Bus & Control Signals
    logic        sram_ren;
    logic        sram_wen;
    logic        cpu_sram_ren;
    logic [10:0] sram_addr;
    logic [10:0] boot_sram_addr;
    logic [10:0] cpu_sram_addr;
    logic [15:0] sram_wdata;
    logic [15:0] sram_rdata;
    logic        cpu_enable_raw;
    logic        cpu_enable;
    logic        cpu_reset;
    logic        cpu_halted;
    logic        fifo_clear;

    // Pause CPU execution when tile is deselected (ena == 0)
    assign cpu_enable = cpu_enable_raw && ena;

    // Internal FIFO Signals
    logic        tx_fifo_push, tx_fifo_pop, tx_fifo_empty, tx_fifo_ovf;
    logic [7:0]  tx_fifo_wdata, tx_fifo_rdata;
    logic        rx_fifo_push, rx_fifo_pop, rx_fifo_full, rx_fifo_ovf;
    logic [7:0]  rx_fifo_wdata, rx_fifo_rdata;

    // 2-Flop Synchronizer for Protocol RX Pins
    logic [7:0] rx_sync_0_q, rx_sync_1_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync_0_q <= 8'h00;
            rx_sync_1_q <= 8'h00;
        end else begin
            rx_sync_0_q <= protocol_rx_pins;
            rx_sync_1_q <= rx_sync_0_q;
        end
    end

    // Glitch-free SRAM bus multiplexing:
    // Whenever sram_wen is high or cpu_enable is low, boot_sram_addr owns the bus.
    assign sram_addr = (sram_wen || !cpu_enable) ? boot_sram_addr : cpu_sram_addr;
    assign sram_ren  = (sram_wen || !cpu_enable) ? 1'b0           : cpu_sram_ren;

    bootloader_host_if u_host_if (
        .clk           (clk),
        .rst_n         (rst_n),
        .load_mode     (host_load_mode),
        .spi_cs_n      (host_spi_cs_n),
        .spi_sck       (host_spi_sck),
        .spi_mosi      (host_spi_mosi),
        .spi_miso      (host_spi_miso),
        .sram_wen      (sram_wen),
        .sram_addr     (boot_sram_addr),
        .sram_wdata    (sram_wdata),
        .cpu_enable    (cpu_enable_raw),
        .cpu_reset     (cpu_reset),
        .fifo_clear    (fifo_clear),
        .cpu_halted    (cpu_halted),
        .tx_fifo_push  (tx_fifo_push),
        .tx_fifo_wdata (tx_fifo_wdata),
        .tx_fifo_full  (tx_fifo_full),
        .tx_fifo_empty (tx_fifo_empty),
        .tx_fifo_ovf   (tx_fifo_ovf),
        .rx_fifo_pop   (rx_fifo_pop),
        .rx_fifo_rdata (rx_fifo_rdata),
        .rx_fifo_empty (rx_fifo_empty),
        .rx_fifo_full  (rx_fifo_full),
        .rx_fifo_ovf   (rx_fifo_ovf)
    );

    sync_fifo #(
        .DATA_WIDTH (8),
        .DEPTH      (4),
        .PTR_WIDTH  (2)
    ) u_tx_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .clear    (fifo_clear),
        .push     (tx_fifo_push),
        .wdata    (tx_fifo_wdata),
        .pop      (tx_fifo_pop),
        .rdata    (tx_fifo_rdata),
        .full     (tx_fifo_full),
        .empty    (tx_fifo_empty),
        .overflow (tx_fifo_ovf)
    );

    sync_fifo #(
        .DATA_WIDTH (8),
        .DEPTH      (4),
        .PTR_WIDTH  (2)
    ) u_rx_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .clear    (fifo_clear),
        .push     (rx_fifo_push),
        .wdata    (rx_fifo_wdata),
        .pop      (rx_fifo_pop),
        .rdata    (rx_fifo_rdata),
        .full     (rx_fifo_full),
        .empty    (rx_fifo_empty),
        .overflow (rx_fifo_ovf)
    );

    execution_core u_cpu (
        .clk           (clk),
        .rst_n         (rst_n),
        .enable        (cpu_enable),
        .cpu_reset     (cpu_reset),
        .cpu_halted    (cpu_halted),
        .sram_ren      (cpu_sram_ren),
        .sram_addr     (cpu_sram_addr),
        .sram_rdata    (sram_rdata),
        .tx_fifo_pop   (tx_fifo_pop),
        .tx_fifo_rdata (tx_fifo_rdata),
        .tx_fifo_empty (tx_fifo_empty),
        .rx_fifo_push  (rx_fifo_push),
        .rx_fifo_wdata (rx_fifo_wdata),
        .rx_fifo_full  (rx_fifo_full),
        .gpio_in       (rx_sync_1_q),
        .gpio_out      (protocol_tx_pins),
        .gpio_oe       (protocol_oe_pins),
        .side_set_pin  (side_set_clk_pin)
    );

    sram_wrapper u_sram (
        .clk   (clk),
        .rst_n (rst_n),
        .ren   (sram_ren),
        .wen   (sram_wen),
        .addr  (sram_addr),
        .wdata (sram_wdata),
        .rdata (sram_rdata)
    );

endmodule : protocol_emulator_top


// =============================================================================
// Module: tt_um_protocol_emulator
// Purpose: Standard Tiny Tapeout CMOS5L top-level wrapper mapping the 24
//          digital pins (ui_in, uo_out, uio_in, uio_out, uio_oe) to the
//          Protocol Emulator Core with full output/enable gating on ena & rst_n.
// =============================================================================
module tt_um_protocol_emulator (
    input  wire [7:0] ui_in,    // Dedicated Inputs  (Host SPI & Control)
    output wire [7:0] uo_out,   // Dedicated Outputs (Side-Set Clock, MISO, Status)
    input  wire [7:0] uio_in,   // Bidirectional I/O Input path (Protocol Pins)
    output wire [7:0] uio_out,  // Bidirectional I/O Output path (Protocol Pins)
    output wire [7:0] uio_oe,   // Bidirectional I/O Output Enable (1=Out, 0=In)
    input  wire       ena,      // Tiny Tapeout design enable
    input  wire       clk,      // System Clock
    input  wire       rst_n     // Active-low Reset
);

    logic       spi_miso;
    logic       tx_full;
    logic       rx_empty;
    logic       side_set_clk;
    logic [7:0] proto_tx;
    logic [7:0] proto_oe;

    wire active_tile = ena && rst_n;

    protocol_emulator_top u_emulator (
        .clk              (clk),
        .rst_n            (rst_n),
        .ena              (ena),
        .host_load_mode   (ui_in[0]),
        .host_spi_cs_n    (ui_in[1]),
        .host_spi_sck     (ui_in[2]),
        .host_spi_mosi    (ui_in[3]),
        .host_spi_miso    (spi_miso),
        .tx_fifo_full     (tx_full),
        .rx_fifo_empty    (rx_empty),
        .protocol_rx_pins (uio_in),
        .protocol_tx_pins (proto_tx),
        .protocol_oe_pins (proto_oe),
        .side_set_clk_pin (side_set_clk)
    );

    // Gate all physical outputs and output-enables when tile is deselected or in reset
    assign uio_oe  = active_tile ? proto_oe : 8'h00;
    assign uio_out = active_tile ? proto_tx : 8'h00;

    wire [7:0] uo_raw = {
        proto_tx[3:0],
        ~rx_empty,
        tx_full,
        spi_miso,
        side_set_clk
    };
    assign uo_out = active_tile ? uo_raw : 8'h00;

    // Prevent synthesis warnings on unused inputs
    wire _unused_ok = &{ui_in[7:4], 1'b0};

endmodule : tt_um_protocol_emulator

`default_nettype wire