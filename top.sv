// ============================================================================
// SoC Option A (PULP AXI) — REAL, BIDIRECTIONAL CGRA↔Memory streaming
//  - CPU (riscV) unchanged, simple DMEM/IMEM ports
//  - Shared RAM via PULP axi_to_mem -> dual-port SRAM (CPU simple + AXI)
//  - CGRA packet bridge (AXIS_W=192) to MeshMultiCgra wrapper
//  - Duplex DMA:
//      * RX engine: Memory -> AXIS (-> CGRA recv_from_cpu)
//      * TX engine: AXIS (<- CGRA send_to_cpu) -> Memory
//    Uses proper AXI bursts (3 beats / 24B per packet) on a 64-bit AXI bus
//  - CPU MMIO @ 0x0000_0000_4000_0000 controls DMA (start/src/dst/len, status)
// ============================================================================
`timescale 1ns/1ps

// PULP AXI helpers (ensure +incdir+ points to pulp-platform/axi/include)
`include "axi/include/axi/typedef.svh"
`include "axi/include/axi/assign.svh"
import axi_pkg::*;






// ---------- Bit widths (compile-time checks) ----------
localparam int CGRA_DATA_W = 32+1+1+1; // 35
localparam int CGRA_CTRL_W = 7 + 4*3 + 8*3 + 8*2 + 3 + 1 + 4*2 + 4*4 + 4*1 + 4*4; // 107
localparam int MCGR_PAY_W = 5 + CGRA_DATA_W + 7 + CGRA_CTRL_W + 4; // 158
localparam int PKT_HDR_W = 5+5+2+2+1+1+1+1+8+1; // 27
localparam int CGRA_PKT_W = PKT_HDR_W + MCGR_PAY_W; // 185









function automatic logic [CGRA_PKT_W-1:0] pack_pkt (IntraCgraPacket_4_2x2_16_8_2_CgraPayload__432fde8bfb7da0ed p);
pack_pkt = {
// Header (MSB->LSB order)
p.src,
p.dst,
p.src_cgra_id,
p.dst_cgra_id,
p.src_cgra_x,
p.src_cgra_y,
p.dst_cgra_x,
p.dst_cgra_y,
p.opaque,
p.vc_id,
// Payload
p.payload.cmd,
p.payload.data.payload,
p.payload.data.predicate,
p.payload.data.bypass,
p.payload.data.delay,
p.payload.data_addr,
p.payload.ctrl.operation,
p.payload.ctrl.fu_in,
p.payload.ctrl.routing_xbar_outport,
p.payload.ctrl.fu_xbar_outport,
p.payload.ctrl.vector_factor_power,
p.payload.ctrl.is_last_ctrl,
p.payload.ctrl.write_reg_from,
p.payload.ctrl.write_reg_idx,
p.payload.ctrl.read_reg_from,
p.payload.ctrl.read_reg_idx,
p.payload.ctrl_addr
};
endfunction


function automatic IntraCgraPacket_4_2x2_16_8_2_CgraPayload__432fde8bfb7da0ed unpack_pkt (logic [CGRA_PKT_W-1:0] v);
IntraCgraPacket_4_2x2_16_8_2_CgraPayload__432fde8bfb7da0ed p;
// Use a running index from LSB upward for clarity
int i = 0;
// ctrl_addr (4)
p.payload.ctrl_addr = v[i +: 4]; i += 4;
// ctrl (107)
p.payload.ctrl.read_reg_idx = v[i +: 16]; i += 16;
p.payload.ctrl.read_reg_from = v[i +: 4]; i += 4;
p.payload.ctrl.write_reg_idx = v[i +: 16]; i += 16;
p.payload.ctrl.write_reg_from = v[i +: 8]; i += 8;
p.payload.ctrl.is_last_ctrl = v[i +: 1]; i += 1;
p.payload.ctrl.vector_factor_power = v[i +: 3]; i += 3;
p.payload.ctrl.fu_xbar_outport = v[i +: 16]; i += 16; // 8*2
p.payload.ctrl.routing_xbar_outport = v[i +: 24]; i += 24; // 8*3
p.payload.ctrl.fu_in = v[i +: 12]; i += 12; // 4*3
p.payload.ctrl.operation = v[i +: 7]; i += 7;
// data_addr (7)
p.payload.data_addr = v[i +: 7]; i += 7;
// data (35)
p.payload.data.delay = v[i +: 1]; i += 1;
p.payload.data.bypass = v[i +: 1]; i += 1;
p.payload.data.predicate = v[i +: 1]; i += 1;
p.payload.data.payload = v[i +: 32]; i += 32;
// cmd (5)
p.payload.cmd = v[i +: 5]; i += 5;
// header tail (27)
p.vc_id = v[i +: 1]; i += 1;
p.opaque = v[i +: 8]; i += 8;
p.dst_cgra_y = v[i +: 1]; i += 1;
p.dst_cgra_x = v[i +: 1]; i += 1;
p.src_cgra_y = v[i +: 1]; i += 1;
p.src_cgra_x = v[i +: 1]; i += 1;
p.dst_cgra_id = v[i +: 2]; i += 2;
p.src_cgra_id = v[i +: 2]; i += 2;
p.dst = v[i +: 5]; i += 5;
p.src = v[i +: 5]; i += 5;
// Sanity
if (i != CGRA_PKT_W) $error("unpack index mismatch: %0d != %0d", i, CGRA_PKT_W);
return p;
endfunction













  // ---------------- PULP AXI types ----------------
  localparam int AXI_ADDR_W = 32;
  localparam int AXI_DATA_W = 64;
  localparam int AXI_ID_W   = 4;
  localparam int AXI_USER_W = 1;
typedef logic [AXI_ADDR_W-1:0] td0;
typedef logic [AXI_DATA_W-1:0] td1;
typedef logic [AXI_ID_W-1:0] td2;
typedef logic [AXI_USER_W-1:0] td3;
  `AXI_TYPEDEF_ALL(axi,
  logic [AXI_ADDR_W-1:0],   // __addr_t
  logic [AXI_ID_W-1:0],     // __id_t
  logic [AXI_DATA_W-1:0],   // __data_t
  logic [(AXI_DATA_W/8)-1:0], // __strb_t
  logic [AXI_USER_W-1:0])   // __user_t









// MeshMultiCgraRTL__fifo
module MeshMultiCgraRTL__fifo
(
  input  logic [0:0] clk,
  input  logic [0:0] reset,
  input  IntraCgraPacket_4_2x2_16_8_2_CgraPayload__432fde8bfb7da0ed recv_from_cpu_pkt__msg,
  output logic [0:0] recv_from_cpu_pkt__rdy,
  input  logic [0:0] recv_from_cpu_pkt__val,
  output IntraCgraPacket_4_2x2_16_8_2_CgraPayload__432fde8bfb7da0ed send_to_cpu_pkt__msg,
  input  logic [0:0] send_to_cpu_pkt__rdy,
  output logic [0:0] send_to_cpu_pkt__val
);
  // Always ready to take a packet
  assign recv_from_cpu_pkt__rdy = 1'b1;

  // Simple 2-entry skid buffer (one-packet deep is usually enough)
  typedef IntraCgraPacket_4_2x2_16_8_2_CgraPayload__432fde8bfb7da0ed pkt_t;
  pkt_t q;
  logic full;

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      full <= 1'b0;
    end else begin
      // Capture when incoming valid and buffer empty
      if (recv_from_cpu_pkt__val && !full) begin
        q    <= recv_from_cpu_pkt__msg; // echo the exact packet
        full <= 1'b1;
      end
      // Dequeue when downstream ready
      if (send_to_cpu_pkt__rdy && full) begin
        full <= 1'b0;
      end
    end
  end

  assign send_to_cpu_pkt__msg = q;
  assign send_to_cpu_pkt__val = full;
endmodule





// -----------------------------------------------------------------------------
// Utility: 64-bit wide, dual-ported SRAM (CPU simple + PULP mem_req*)
// -----------------------------------------------------------------------------
module dp_sram_axi_cpu #(
  parameter longint DEPTH_WORD = 1024 * 64
)(
  input  logic        clk,
  input  logic        rstn,
  // Port A: CPU simple (byte addr)
  input  logic [63:0] cpu_addr,
  input  logic [63:0] cpu_wdata,
  input  logic        cpu_we,
  output logic [63:0] cpu_rdata,
  // Port B: PULP mem_req* (byte addr)
  input  logic        mem_req,
  output logic        mem_gnt,
  input  logic [31:0] mem_addr,
  input  logic [63:0] mem_wdata,
  input  logic [7:0]  mem_strb,
  input  logic        mem_we,
  output logic        mem_rvalid,
  output logic [63:0] mem_rdata
);
  // Storage
  logic [63:0] mem [0:DEPTH_WORD-1];

  logic [63:0] cpu_addr_aligned;
  logic [31:0] mem_addr_aligned;
  assign cpu_addr_aligned = cpu_addr >> 3;
  assign mem_addr_aligned = mem_addr >> 3;


  // Static loop indices (avoid automatic loop vars)
  int unsigned k_cpu_r, k_w, k_r;

  // ---------------- CPU read: purely combinational ----------------
  assign cpu_rdata = mem[cpu_addr_aligned];
  assign mem_rdata = mem[mem_addr_aligned];
  assign mem_rvalid = mem_req & ~mem_we;
  assign mem_gnt    = mem_req & ~cpu_we;
  // ---------------- Unified writer + AXI read model ----------------
  // One always_ff drives: mem_gnt, mem_rvalid, mem_rdata, and *all* writes to mem
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin;
      //mem_gnt    <= 1'b0;
      //mem_rvalid <= 1'b0;
      //mem_rdata  <= '0;
    end else begin
      // Default handshakes
      //mem_gnt    <= mem_req & ~cpu_we;
      //mem_rvalid <= mem_req & ~mem_we;

      // AXI read data (one-cycle model)
      //if (mem_req && !mem_we) begin
      //  mem_rdata <= mem[mem_addr_aligned];
      //end

      // ---- Writes (two-writer block) ----
      // Priority: 1) CPU Port-A 2) AXI Port-B.
      if (cpu_we) begin
        mem[cpu_addr_aligned] <= cpu_wdata;
      end
      else if (mem_req && mem_we) begin
        for (int b = 0; b < 8; b++) begin
          mem[mem_addr_aligned][8*b +: 8] <= {8{mem_strb[b]}} & mem_wdata[8*b +: 8];
        end
      end
    end
  end

endmodule

// -----------------------------------------------------------------------------
// IMEM ROM (32b words, addressed by PC[31:2])
// -----------------------------------------------------------------------------
module imem_rom #(parameter int DEPTH = 1024) (
  input  logic [31:0] pc,
  output logic [31:0] inst
);
  logic [31:0] mem [0:DEPTH-1];
  //initial for (int i=0;i<DEPTH;i++) mem[i] = 32'h00000013; // ADDI x0,x0,0
  initial $readmemh("prog.hex", mem);
  assign inst = mem[pc[31:2]];
endmodule

// -----------------------------------------------------------------------------
// Duplex AXI-stream DMA (192b packet = 3×64b beats) with AXI bursts
// Control via simple wires (connected to MMIO regs in top)
// -----------------------------------------------------------------------------
module axis_dma_duplex #(
  parameter int AXI_ADDR_W = 32,
  parameter int AXI_DATA_W = 64,
  parameter int AXI_ID_W   = 4,
  parameter int AXIS_W     = 192
)(
  input  logic clk,
  input  logic rstn,

  // Control/Status
  input  logic        start_rx,           // Memory->CGRA
  input  logic [63:0] src_addr_rx,
  input  logic [31:0] len_pkts_rx,        // number of 192b packets
  output logic        busy_rx,
  output logic        done_rx,

  input  logic        start_tx,           // CGRA->Memory
  input  logic [63:0] dst_addr_tx,
  input  logic [31:0] len_pkts_tx,
  output logic        busy_tx,
  output logic        done_tx,

  // AXI Master (PULP typed)
  input  axi_resp_t   axi_i,
  output axi_req_t    axi_o,

  // AXIS toward CGRA (ingress to CGRA)
  output logic [AXIS_W-1:0] s_axis_tdata,
  output logic              s_axis_tvalid,
  input  logic              s_axis_tready,

  // AXIS from CGRA (egress from CGRA)
  input  logic [AXIS_W-1:0] m_axis_tdata,
  input  logic              m_axis_tvalid,
  output logic              m_axis_tready
);
  localparam int BEATS_PER_PKT = (AXIS_W+AXI_DATA_W-1)/AXI_DATA_W; // 192/64=3

  // ---------------------------------------------------------------------------
  // AXI req defaulting
  // ---------------------------------------------------------------------------
  axi_req_t req_d, req_q;
  assign axi_o = req_q;
  always_comb begin
    req_d = '0;
    // defaults
    req_d.aw_valid = 1'b0; req_d.aw.id = '0; req_d.aw.len = BEATS_PER_PKT-1; req_d.aw.size = $clog2(AXI_DATA_W/8); req_d.aw.burst = BURST_INCR;
    req_d.w_valid  = 1'b0; req_d.w.strb = '1; req_d.w.last = 1'b0;
    req_d.b_ready  = 1'b1;
    req_d.ar_valid = 1'b0; req_d.ar.id = '0; req_d.ar.len = BEATS_PER_PKT-1; req_d.ar.size = $clog2(AXI_DATA_W/8); req_d.ar.burst = BURST_INCR;
    req_d.r_ready  = 1'b1;
  end

  // ---------------------------------------------------------------------------
  // RX engine: Memory -> AXIS
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {RX_IDLE, RX_AR, RX_R, RX_DONE} rx_state_e;
  rx_state_e rx_st; logic [31:0] rx_pkts_left; logic [AXI_ADDR_W-1:0] rx_addr;
  logic [1:0] rx_beat_cnt; logic [AXIS_W-1:0] rx_shift;

  assign busy_rx = (rx_st != RX_IDLE) && (rx_st != RX_DONE);
  assign done_rx = (rx_st == RX_DONE);

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      rx_st <= RX_IDLE; rx_pkts_left <= '0; rx_addr <= '0; rx_beat_cnt <= '0; rx_shift <= '0;
      req_q.ar_valid <= '0;
      req_q.ar <= '0;
      req_q.r_ready <= '0;
    end else begin
      unique case (rx_st)
        RX_IDLE: if (start_rx) begin
          rx_pkts_left <= len_pkts_rx;
          rx_addr      <= src_addr_rx[AXI_ADDR_W-1:0];
          rx_st        <= RX_AR;
          req_q.r_ready <= 1'b0;
          req_q.ar_valid <= 1'b1; req_q.ar.addr <= src_addr_rx[AXI_ADDR_W-1:0]; req_q.ar.len <= 'd2; req_q.ar.burst <= 'd1; req_q.ar.size <= 'd3;
        end
        RX_AR: begin
          // drive AR until accepted
          req_q.ar_valid <= 1'b1; req_q.ar.addr <= rx_addr; req_q.ar.len <= 'd2; req_q.ar.burst <= 'd1; req_q.ar.size <= 'd3;
          if (axi_i.ar_ready) begin
            req_q.ar_valid <= 1'b0;
            rx_beat_cnt <= '0;
            rx_st <= RX_R;
            req_q.r_ready <= 1'b1;
          end
        end
        RX_R: begin
          // Assemble BEATS_PER_PKT beats into rx_shift[AXIS_W-1:0]
          if (axi_i.r_valid) begin
            // place each 64-bit beat into increasing chunks [63:0], [127:64], [191:128]
            rx_shift[rx_beat_cnt*AXI_DATA_W +: AXI_DATA_W] <= axi_i.r.data;
            if (rx_beat_cnt == BEATS_PER_PKT-1) begin
              // Have a full packet; offer to AXIS sink
              if (s_axis_tready) begin
                // Commit packet
                rx_beat_cnt <= '0;
                // Next packet
                if (rx_pkts_left > 1) begin
                  rx_pkts_left <= rx_pkts_left - 1;
                  rx_addr      <= rx_addr + BEATS_PER_PKT*(AXI_DATA_W/8);
                  rx_st        <= RX_AR;
                  req_q.r_ready <= 1'b0;
                  req_q.ar_valid <= 1'b1; req_q.ar.addr <= rx_addr + BEATS_PER_PKT*(AXI_DATA_W/8); req_q.ar.len <= 'd2; req_q.ar.burst <= 'd1; req_q.ar.size <= 'd3;
                end else begin
                  rx_st <= RX_DONE;
                  req_q.r_ready <= 1'b0;
                end
              end
            end else begin
              rx_beat_cnt <= rx_beat_cnt + 1'b1;
            end
          end
        end
        RX_DONE: rx_st <= RX_IDLE;
      endcase
    end
  end

  // Present packet to AXIS sink (CGRA) when a full packet is buffered
  assign s_axis_tdata[(BEATS_PER_PKT-1)*AXI_DATA_W +: AXI_DATA_W] = axi_i.r.data;
  assign s_axis_tdata[0 +: (BEATS_PER_PKT-1)*AXI_DATA_W]          = rx_shift[0 +: (BEATS_PER_PKT-1)*AXI_DATA_W];
  assign s_axis_tvalid = (rx_st == RX_R) && (rx_beat_cnt == BEATS_PER_PKT-1) && axi_i.r_valid;

  // ---------------------------------------------------------------------------
  // TX engine: AXIS -> Memory
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {TX_IDLE, TX_CAPTURE, TX_AW, TX_W, TX_B, TX_DONE} tx_state_e;
  tx_state_e tx_st; logic [31:0] tx_pkts_left; logic [AXI_ADDR_W-1:0] tx_addr;
  logic [1:0] tx_beat_cnt; logic [AXIS_W-1:0] tx_buf;

  assign busy_tx = (tx_st != TX_IDLE) && (tx_st != TX_DONE);
  assign done_tx = (tx_st == TX_DONE);

  // Always ready to accept a packet when in CAPTURE
  assign m_axis_tready = (tx_st == TX_CAPTURE);

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      tx_st <= TX_IDLE; tx_pkts_left <= '0; tx_addr <= '0; tx_beat_cnt <= '0; tx_buf <= '0;
      req_q.aw_valid <= 1'b0;
      req_q.aw <= '0;
      req_q.w <= '0;
      req_q.w_valid <= 1'b0;
      req_q.b_ready <= 1'b0;
    end else begin
      unique case (tx_st)
        TX_IDLE: if (start_tx) begin
          tx_pkts_left <= len_pkts_tx;
          tx_addr      <= dst_addr_tx[AXI_ADDR_W-1:0];
          tx_st        <= TX_CAPTURE;
        end
        TX_CAPTURE: begin
          if (m_axis_tvalid) begin
            tx_buf <= m_axis_tdata;
            tx_st  <= TX_AW;
            //req_q.aw_valid <= 1'b1; req_q.aw.addr <= tx_addr; req_q.aw.len <= 'd2; req_q.aw.burst <= 'd1; req_q.aw.size <= 'd3;
            //req_q.w_valid <= 1'b1; req_q.w.data <= tx_buf[tx_beat_cnt*AXI_DATA_W +: AXI_DATA_W]; req_q.w.strb <= {8{1'b1}}; req_q.w.last <= (tx_beat_cnt == BEATS_PER_PKT-1);
          end
        end
        TX_AW: begin
          req_q.aw_valid <= 1'b1; req_q.aw.addr <= tx_addr; req_q.aw.len <= 'd2; req_q.aw.burst <= 'd1; req_q.aw.size <= 'd3;
          req_q.w_valid <= 1'b1; req_q.w.data <= tx_buf[tx_beat_cnt*AXI_DATA_W +: AXI_DATA_W]; req_q.w.strb <= {8{1'b1}}; req_q.w.last <= (tx_beat_cnt == BEATS_PER_PKT-1);
          req_q.b_ready <= 1'b1;
          if (axi_i.aw_ready && (tx_beat_cnt != BEATS_PER_PKT-1) ) begin // 1 != BEATS_PER_PKT
            req_q.aw_valid <= 1'b0;
            tx_beat_cnt <= 'b1;
            tx_st <= TX_W;
            req_q.w_valid <= 1'b1; req_q.w.data <= tx_buf[1*AXI_DATA_W +: AXI_DATA_W]; req_q.w.strb <= {8{1'b1}}; req_q.w.last <= (1 == BEATS_PER_PKT-1);
          end else if (axi_i.aw_ready && (tx_beat_cnt == BEATS_PER_PKT-1) ) begin // 1 == BEATS_PER_PKT
            tx_st <= TX_B;
          end
        end
        TX_W: begin
          //req_q.aw_valid <= 1'b1; req_q.aw.addr <= tx_addr; req_q.aw.len <= 'd2; req_q.aw.burst <= 'd1; req_q.aw.size <= 'd3;
          // drive each 64-bit chunk sequentially
          req_q.w_valid <= 1'b1; req_q.w.data <= tx_buf[tx_beat_cnt*AXI_DATA_W +: AXI_DATA_W]; req_q.w.strb <= {8{1'b1}}; req_q.w.last  <= (tx_beat_cnt == BEATS_PER_PKT-1);
          req_q.b_ready <= 1'b1;
          if (axi_i.w_ready) begin
            if (tx_beat_cnt == BEATS_PER_PKT-1) begin
              req_q.b_ready <= 1'b1;
              tx_st <= TX_B;
              req_q.w_valid <= 1'b0;
            end else begin
              tx_beat_cnt <= tx_beat_cnt + 1'b1;
              req_q.w_valid <= 1'b1; req_q.w.data <= tx_buf[(tx_beat_cnt+1'b1)*AXI_DATA_W +: AXI_DATA_W]; req_q.w.strb <= {8{1'b1}}; req_q.w.last  <= ((tx_beat_cnt+1'b1) == BEATS_PER_PKT-1);
            end
          end
        end
        TX_B: begin
          if (axi_i.b_valid) begin
            // Next packet or done
            if (tx_pkts_left > 1) begin
              tx_pkts_left <= tx_pkts_left - 1;
              tx_addr      <= tx_addr + BEATS_PER_PKT*(AXI_DATA_W/8);
              tx_st        <= TX_CAPTURE;
            end else begin
              tx_st <= TX_DONE;
            end
          end
        end
        TX_DONE: tx_st <= TX_IDLE;
      endcase
    end
  end
endmodule

// -----------------------------------------------------------------------------
// Import the CGRA packet bridge (pack/unpack) — assumed present in your tree
//   package cgra_pkt_pkg
//   module cgra_axis_bridge #(AXIS_W=192)
// -----------------------------------------------------------------------------
// ============================================================
// 1-beat AXI-Stream bridge to MeshMultiCgraRTL ports
// - s_axis_* : DMA -> CGRA (CPU->CGRA direction)
// - m_axis_* : CGRA -> DMA (CGRA->CPU direction)
// ============================================================
module cgra_axis_bridge #(
parameter int AXIS_W = 192 // must be >= cgra_pkt_pkg::CGRA_PKT_W (185)
)(
input logic clk,
input logic rstn,


// ------------------ AXI-Stream to CGRA (ingress) ------------------
input logic [AXIS_W-1:0] s_axis_tdata,
input logic s_axis_tvalid,
output logic s_axis_tready,


// ------------------ AXI-Stream from CGRA (egress) -----------------
output logic [AXIS_W-1:0] m_axis_tdata,
output logic m_axis_tvalid,
input logic m_axis_tready,


// ------------------ CGRA ports -----------------------------
output IntraCgraPacket_4_2x2_16_8_2_CgraPayload__432fde8bfb7da0ed recv_from_cpu_pkt__msg,
input logic [0:0] recv_from_cpu_pkt__rdy,
output logic [0:0] recv_from_cpu_pkt__val,


input IntraCgraPacket_4_2x2_16_8_2_CgraPayload__432fde8bfb7da0ed send_to_cpu_pkt__msg,
output logic [0:0] send_to_cpu_pkt__rdy,
input logic [0:0] send_to_cpu_pkt__val
);
//import cgra_pkt_pkg::*;
// Pad/trim helpers
localparam int PKT_W = CGRA_PKT_W; // 185
localparam int PAD_W = (AXIS_W >= PKT_W) ? (AXIS_W - PKT_W) : -1;
initial if (AXIS_W < PKT_W) $error("AXIS_W (%0d) < CGRA packet width (%0d)", AXIS_W, PKT_W);


// ---------- Ingress: AXIS -> CGRA ----------
// 1-beat transfer: when both s_axis_tvalid && recv_from_cpu_pkt__rdy
assign s_axis_tready = recv_from_cpu_pkt__rdy;
assign recv_from_cpu_pkt__val = s_axis_tvalid;
assign recv_from_cpu_pkt__msg = unpack_pkt(s_axis_tdata[PKT_W-1:0]);

assign send_to_cpu_pkt__rdy = m_axis_tready;
assign m_axis_tvalid = send_to_cpu_pkt__val;
assign m_axis_tdata[PKT_W-1:0] = pack_pkt(send_to_cpu_pkt__msg);
assign m_axis_tdata[AXIS_W-1:PKT_W] = '0;
/*
assign recv_from_cpu_pkt__val = s_axis_tvalid;
assign s_axis_tready          = recv_from_cpu_pkt__rdy[0];
assign recv_from_cpu_pkt__msg = unpack_pkt(s_axis_tdata[PKT_W-1:0]);

assign m_axis_tvalid          = send_to_cpu_pkt__val[0];
assign send_to_cpu_pkt__rdy   = { m_axis_tready };
assign m_axis_tdata[PKT_W-1:0]= pack_pkt(send_to_cpu_pkt__msg);
assign m_axis_tdata[AXIS_W-1:PKT_W] = '0;
*/
/*
// ============================================================================
  // Ingress path: AXIS -> (buffer) -> CGRA.recv_from_cpu_pkt
  // ============================================================================
  // One-entry skid buffer to avoid combinational loops and ease timing
  logic               in_full;
  logic [AXIS_W-1:0]  in_data_q;

  // Accept from AXIS when buffer not full
  assign s_axis_tready = ~in_full;

  // Buffer fill / drain
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      in_full   <= 1'b0;
      in_data_q <= '0;
    end else begin
      // Fill on handshake if empty
      if (!in_full && s_axis_tvalid && s_axis_tready) begin
        in_data_q <= s_axis_tdata;
        in_full   <= 1'b1;
      end
      // Drain to CGRA when it takes it
      if (in_full && recv_from_cpu_pkt__rdy[0]) begin
        in_full <= 1'b0;
      end
    end
  end

  // Drive CGRA typed message/val from buffer
  assign recv_from_cpu_pkt__val = in_full;
  assign recv_from_cpu_pkt__msg = unpack_pkt(in_data_q[PKT_W-1:0]);

  // ============================================================================
  // Egress path: CGRA.send_to_cpu_pkt -> (buffer) -> AXIS
  // ============================================================================
  logic               out_full;
  logic [AXIS_W-1:0]  out_data_q;

  // Backpressure toward CGRA (ready only when buffer has space)
  assign send_to_cpu_pkt__rdy = {~out_full};

  // Capture from CGRA when it produces and buffer empty
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      out_full   <= 1'b0;
      out_data_q <= '0;
    end else begin
      if (!out_full && send_to_cpu_pkt__val[0] && send_to_cpu_pkt__rdy[0]) begin
        out_data_q                <= '0;
        out_data_q[PKT_W-1:0]     <= pack_pkt(send_to_cpu_pkt__msg);
        out_full                  <= 1'b1;
      end
      // AXIS consumer handshake drains the buffer
      if (out_full && m_axis_tready) begin
        out_full <= 1'b0;
      end
    end
  end

  assign m_axis_tvalid = out_full;
  assign m_axis_tdata  = out_data_q;
*/
endmodule
// -----------------------------------------------------------------------------
// TOP: SoC with MMIO @ 0x0000_0000_4000_0000 controlling the duplex DMA
// -----------------------------------------------------------------------------
module soc_option_a_pulp_top_real_duplex (
  input  logic clk,
  input  logic rstn
);
  // ---------------- CPU instance ----------------
  logic [31:0] inst; logic [63:0] PC_IMEM;
  logic [63:0] readData_DMEM, addrData_DMEM, wrData_DMEM; 
  logic        MemWrite_DMEM, MemRead_DMEM;
  logic signed [63:0] readData1_RF, readData2_RF; // to be driven by RF model/testbench
  logic [4:0]  readAddr1_RF, readAddr2_RF, writeAddr_RF;
  logic [63:0] writeData_RF; logic RegWrite_RF;

  riscV u_cpu(
    .inst(inst), .PC_IMEM(PC_IMEM),
    .readData_DMEM(readData_DMEM), .addrData_DMEM(addrData_DMEM), .wrData_DMEM(wrData_DMEM),
    .MemWrite_DMEM(MemWrite_DMEM), .MemRead_DMEM(MemRead_DMEM),
    .readData1_RF(readData1_RF), .readData2_RF(readData2_RF),
    .readAddr1_RF(readAddr1_RF), .readAddr2_RF(readAddr2_RF),
    .writeData_RF(writeData_RF), .writeAddr_RF(writeAddr_RF), .RegWrite_RF(RegWrite_RF),
    .clk(clk), .rst(rstn)
  );

  // IMEM ROM
  imem_rom u_imem(.pc(PC_IMEM[31:0]), .inst(inst));

  // Register-file model
  logic [63:0] rf [0:31];

  // Drive reads combinationally
  assign readData1_RF = (readAddr1_RF == 0) ? 64'd0 : rf[readAddr1_RF];
  assign readData2_RF = (readAddr2_RF == 0) ? 64'd0 : rf[readAddr2_RF];

  // Capture writes
  always_ff @(posedge clk) begin
    if (RegWrite_RF && (writeAddr_RF != 0))
      rf[writeAddr_RF] <= writeData_RF;
  end



  // ---------------- MMIO decode @ 0x0000_0000_4000_0000 ----------------
  localparam logic [63:0] MMIO_BASE = 64'h0000_0000_4000_0000;
  localparam logic [63:0] MMIO_MASK = 64'hFFFF_FFFF_FFFF_F000; // 4KB window
  wire is_mmio = ((addrData_DMEM & MMIO_MASK) == (MMIO_BASE & MMIO_MASK));

  // Simple MMIO register map (all 64b unless noted):
  //  0x00 CONTROL   [0]=start_rx, [1]=start_tx, [8]=irq_en (optional)
  //  0x08 SRC_RX    64b byte addr (memory->CGRA)
  //  0x10 DST_TX    64b byte addr (CGRA->memory)
  //  0x18 LEN_RX    32b packet count
  //  0x1C LEN_TX    32b packet count
  //  0x20 STATUS    [0]=busy_rx, [1]=done_rx, [2]=busy_tx, [3]=done_tx



  logic        reg_start_rx, reg_start_tx, reg_irq_en;
  logic [63:0] reg_src_rx, reg_dst_tx;
  logic [31:0] reg_len_rx, reg_len_tx;
  logic        stat_busy_rx, stat_done_rx, stat_busy_tx, stat_done_tx;

  // Write
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      reg_start_rx<=0; reg_start_tx<=0; reg_irq_en<=0; reg_src_rx<='0; reg_dst_tx<='0; reg_len_rx<='0; reg_len_tx<='0;
    end else if (MemWrite_DMEM && is_mmio) begin
      unique case (addrData_DMEM[7:0])
        8'h00: begin reg_start_rx <= wrData_DMEM[0]; reg_start_tx <= wrData_DMEM[1]; reg_irq_en <= wrData_DMEM[8]; end
        8'h08: reg_src_rx <= wrData_DMEM;
        8'h10: reg_dst_tx <= wrData_DMEM;
        8'h18: reg_len_rx <= wrData_DMEM[31:0];
        8'h1C: reg_len_tx <= wrData_DMEM[31:0];
        default: ;
      endcase
    end else begin
      // auto-clear start strobes once DMA observes them (one-shot)
      if (reg_start_rx && stat_busy_rx) reg_start_rx <= 1'b0;
      if (reg_start_tx && stat_busy_tx) reg_start_tx <= 1'b0;
    end
  end

  // Read
  logic [63:0] mmio_rdata;
  always_comb begin
    unique case (addrData_DMEM[7:0])
      8'h00: mmio_rdata = {55'b0, reg_irq_en, 6'b0, reg_start_tx, reg_start_rx};
      8'h08: mmio_rdata = reg_src_rx;
      8'h10: mmio_rdata = reg_dst_tx;
      8'h18: mmio_rdata = {32'b0, reg_len_rx};
      8'h1C: mmio_rdata = {32'b0, reg_len_tx};
      8'h20: mmio_rdata = {60'b0, stat_done_tx, stat_busy_tx, stat_done_rx, stat_busy_rx};
      default: mmio_rdata = 64'b0;
    endcase
  end

  // Return path to CPU DMEM
  wire [63:0] ram_rdata;
  assign readData_DMEM = is_mmio ? mmio_rdata : ram_rdata;



  // AXI master from DMA -> AXI-to-mem -> SRAM
  axi_req_t  dma_axi_req; axi_resp_t dma_axi_rsp;
  logic       mem_req, mem_gnt, mem_we, mem_rvalid; logic [31:0] mem_addr; logic [63:0] mem_wdata, mem_rdata; logic [7:0] mem_strb;

  localparam int AXIS_W = 192; // 185 bits payload + 7 pad

  axi_to_mem #(
    .axi_req_t (axi_req_t),
    .axi_resp_t(axi_resp_t),
    .AddrWidth (AXI_ADDR_W),
    .DataWidth (AXI_DATA_W),
    .IdWidth   (AXI_ID_W),
    .NumBanks  (1),            // single bank -> simple SRAM
    .BufDepth( (AXIS_W+AXI_DATA_W-1)/AXI_DATA_W )
  ) u_axi2mem (
    .clk_i      (clk),
    .rst_ni     (rstn),
    .busy_o     (),
    .axi_req_i  (dma_axi_req),
    .axi_resp_o (dma_axi_rsp),
    .mem_req_o  (mem_req),
    .mem_gnt_i  (mem_gnt),
    .mem_addr_o (mem_addr),
    .mem_wdata_o(mem_wdata),
    .mem_strb_o (mem_strb),
    .mem_atop_o (),
    .mem_we_o   (mem_we),
    .mem_rvalid_i(mem_rvalid),
    .mem_rdata_i(mem_rdata)
  );

  dp_sram_axi_cpu u_sram (
    .clk(clk), .rstn(rstn),
    // CPU simple port (DMEM)
    .cpu_addr (addrData_DMEM),
    .cpu_wdata(wrData_DMEM),
    .cpu_we   (MemWrite_DMEM & ~is_mmio),
    .cpu_rdata(ram_rdata),
    // AXI side
    .mem_req  (mem_req),
    .mem_gnt  (mem_gnt),
    .mem_addr (mem_addr),
    .mem_wdata(mem_wdata),
    .mem_strb (mem_strb),
    .mem_we   (mem_we),
    .mem_rvalid(mem_rvalid),
    .mem_rdata(mem_rdata)
  );





  // ---------------- CGRA bridge (packet pack/unpack) + CGRA instantiation ----------------
  logic [AXIS_W-1:0] to_cgra_tdata; logic to_cgra_tvalid, to_cgra_tready;
  logic [AXIS_W-1:0] from_cgra_tdata; logic from_cgra_tvalid, from_cgra_tready;

  // Import packet types and declare bridge<->CGRA wires
  //import cgra_pkt_pkg::*;
  IntraCgraPacket_4_2x2_16_8_2_CgraPayload__432fde8bfb7da0ed recv_msg, send_msg;
  logic [0:0] recv_val, recv_rdy;
  logic [0:0] send_val, send_rdy;

  // Bridge between AXIS and CGRA packet ports
  cgra_axis_bridge #(.AXIS_W(AXIS_W)) u_cgra_bridge (
    .clk(clk), .rstn(rstn),
    .s_axis_tdata (to_cgra_tdata), .s_axis_tvalid(to_cgra_tvalid), .s_axis_tready(to_cgra_tready),
    .m_axis_tdata (from_cgra_tdata), .m_axis_tvalid(from_cgra_tvalid), .m_axis_tready(from_cgra_tready),
    .recv_from_cpu_pkt__msg(recv_msg), .recv_from_cpu_pkt__rdy(recv_rdy), .recv_from_cpu_pkt__val(recv_val),
    .send_to_cpu_pkt__msg  (send_msg), .send_to_cpu_pkt__rdy  (send_rdy), .send_to_cpu_pkt__val  (send_val)
  );

  // CGRA instance wired to the bridge
  //MeshMultiCgraRTL__3077cc8233e37d0f u_cgra (
  MeshMultiCgraRTL__fifo u_cgra (
    .clk                    (clk),
    .reset                  (~rstn),
    .recv_from_cpu_pkt__msg (recv_msg),
    .recv_from_cpu_pkt__rdy (recv_rdy),
    .recv_from_cpu_pkt__val (recv_val),
    .send_to_cpu_pkt__msg   (send_msg),
    .send_to_cpu_pkt__rdy   (send_rdy),
    .send_to_cpu_pkt__val   (send_val)
  );

  // ---------------- Duplex DMA instance ----------------
  axis_dma_duplex #(
    .AXI_ADDR_W(AXI_ADDR_W), .AXI_DATA_W(AXI_DATA_W), .AXI_ID_W(AXI_ID_W), .AXIS_W(AXIS_W)
  ) u_dma (
    .clk(clk), .rstn(rstn),
    .start_rx     (reg_start_rx),
    .src_addr_rx  (reg_src_rx),
    .len_pkts_rx  (reg_len_rx),
    .busy_rx      (stat_busy_rx),
    .done_rx      (stat_done_rx),
    .start_tx     (reg_start_tx),
    .dst_addr_tx  (reg_dst_tx),
    .len_pkts_tx  (reg_len_tx),
    .busy_tx      (stat_busy_tx),
    .done_tx      (stat_done_tx),
    .axi_i        (dma_axi_rsp),
    .axi_o        (dma_axi_req),
    .s_axis_tdata (to_cgra_tdata),
    .s_axis_tvalid(to_cgra_tvalid),
    .s_axis_tready(to_cgra_tready),
    .m_axis_tdata (from_cgra_tdata),
    .m_axis_tvalid(from_cgra_tvalid),
    .m_axis_tready(from_cgra_tready)
  );

endmodule

