`timescale 1ps/1ps

//`include "header.sv"

/*
vcs -sverilog -full64 -timescale=1ns/1ps erv_RISC-V/ALUCtrl.v erv_RISC-V/dff.v erv_RISC-V/immGen.v erv_RISC-V/mainCtrl.v erv_RISC-V/risc5ALU_core.v erv_RISC-V/riscV.v \
                                         common_cells/src/stream_mux.sv common_cells/src/stream_fork.sv common_cells/src/stream_fifo.sv \
                                         common_cells/src/stream_join.sv common_cells/src/stream_fork_dynamic.sv common_cells/src/fifo_v3.sv common_cells/src/stream_join_dynamic.sv \
                                         axi/src/axi_pkg.sv axi/src/axi_intf.sv axi/src/axi_to_detailed_mem.sv axi/src/axi_to_mem.sv \
                                         VectorCGRAfork0/multi_cgra/test/MeshMultiCgraRTL__3077cc8233e37d0f__pickled.v top.sv tb.v \
                                         -debug_access+all +incdir+common_cells/include +incdir+axi/include +incdir+erv_RISC-V -top tb
*/

module tb
(
);

  logic [0:0] clk;
  logic [0:0] reset;

  soc_option_a_pulp_top_real_duplex u_top ( .clk, .rstn(~reset) );

  initial
  begin
    // Static assertions
    if (CGRA_DATA_W != 35) $error("CGRA_DATA_W mismatch: %0d", CGRA_DATA_W);
    if (CGRA_CTRL_W != 107) $error("CGRA_CTRL_W mismatch: %0d", CGRA_CTRL_W);
    if (MCGR_PAY_W != 158) $error("MCGR_PAY_W mismatch: %0d", MCGR_PAY_W);
    if (PKT_HDR_W != 27) $error("PKT_HDR_W mismatch: %0d", PKT_HDR_W);
    if (CGRA_PKT_W != 185) $error("CGRA_PKT_W mismatch: %0d", CGRA_PKT_W);

    $display("\nTEST begin\n");

    clk = 1'b0;

    reset = 1'b0;
    #7
    reset = 1'b1;
    #50
    reset = 1'b0;
    #10


    $finish();
  end

  initial
    forever
    begin
      #5
      clk = ~clk;
    end

  always @ (posedge clk or negedge clk)
  begin
    #1
    /*$display("%t: clk %d reset %d recv_from_noc__rdy %d", $time(), clk, reset, recv_from_noc__rdy);
    $display("%t: e_recv_rdy[0] %d e_in_val[0] %d e_recv_rdy[1] %d e_in_val[1] %d", $time(), recv_data_on_boundary_east__rdy[0], recv_data_on_boundary_east__val[0], recv_data_on_boundary_east__rdy[1], recv_data_on_boundary_east__val[1]);
    $display("%t: recv_from_cpu_ctrl_pkt__rdy %d recv_from_cpu_ctrl_pkt__val %d val_rtl %d", $time(), recv_from_cpu_ctrl_pkt__rdy, recv_from_cpu_ctrl_pkt__val, CGRA.controller.recv_from_cpu_ctrl_pkt__val);
    $display("%t: send_to_noc__val %d\n", $time(), send_to_noc__val);
    $display("%t: CGRA.tile__0.const_mem.reg_file.regs[0] %d CGRA.tile__0.const_mem.reg_file.regs[1] %d CGRA.tile__0.ctrl_mem.reg_file.regs[0] %d CGRA.tile__0.ctrl_mem.reg_file.regs[1] %d", $time(), CGRA.tile__0.const_mem.reg_file.regs[0], CGRA.tile__0.const_mem.reg_file.regs[1], CGRA.tile__0.ctrl_mem.reg_file.regs[0], CGRA.tile__0.ctrl_mem.reg_file.regs[1]);
    */
    //$display("%t: %d %d %d %d", $time(), CGRA.controller.recv_ctrl_pkt_queue.recv__val, CGRA.controller.recv_ctrl_pkt_queue.recv__msg, CGRA.controller.recv_ctrl_pkt_queue.send__val, CGRA.controller.recv_ctrl_pkt_queue.send__msg);
    //$display("%t: %d %d %d %d", $time(), CGRA.ctrl_ring.recv__val[0], CGRA.ctrl_ring.recv__msg[0], CGRA.ctrl_ring.send__val[0], CGRA.ctrl_ring.send__msg[0]);
    //$display("%t: tile00 cnst val  %d tile00 cnst rdy %d msg %d", $time, MultiCGRA.cgra__0.tile__0.const_mem.recv_const__val, MultiCGRA.cgra__0.tile__0.const_mem.recv_const__rdy, MultiCGRA.cgra__0.tile__0.const_mem.recv_const__msg);
    //$display("%t: recv_from_cpu_pkt__rdy %d recv_from_cpu_pkt__val %d", $time(), recv_from_cpu_pkt__rdy, recv_from_cpu_pkt__val);
    //$display("%t: %d %d", $time, CGRA.tile__0.const_mem.__tmpvar__load_const_not_full, CGRA.tile__0.const_mem.wr_cur);
    //$display("%t: %d %d %d", $time, CGRA.tile__1.const_mem.recv_const__val, CGRA.tile__1.const_mem.recv_const__rdy, CGRA.tile__1.const_mem.recv_const__msg);
    //$display("%t: %d %d %d", $time, CGRA.tile__2.const_mem.recv_const__val, CGRA.tile__2.const_mem.recv_const__rdy, CGRA.tile__2.const_mem.recv_const__msg);
    //$display("%t: %d %d %d", $time, CGRA.tile__3.const_mem.recv_const__val, CGRA.tile__3.const_mem.recv_const__rdy, CGRA.tile__3.const_mem.recv_const__msg);
    /*$display("%t: cgra0datamem noc rd ready %d val %d", $time, MultiCGRA.cgra__0.data_mem.recv_from_noc_rdata__rdy, MultiCGRA.cgra__0.data_mem.recv_from_noc_rdata__val);
    $display("%t: cgra0datamem waddr0 ready %d val %d", $time, MultiCGRA.cgra__0.data_mem.recv_waddr__rdy[0], MultiCGRA.cgra__0.data_mem.recv_waddr__val[0]);
    $display("%t: cgra0datamem wdata0 ready %d val %d", $time, MultiCGRA.cgra__0.data_mem.recv_wdata__rdy[0], MultiCGRA.cgra__0.data_mem.recv_wdata__val[0]);
    $display("%t: cgra0datamem waddr1 ready %d val %d", $time, MultiCGRA.cgra__0.data_mem.recv_waddr__rdy[1], MultiCGRA.cgra__0.data_mem.recv_waddr__val[1]);
    $display("%t: cgra0datamem wdata1 ready %d val %d", $time, MultiCGRA.cgra__0.data_mem.recv_wdata__rdy[1], MultiCGRA.cgra__0.data_mem.recv_wdata__val[1]);
    $display("%t: cgra0datamem waddr2 ready %d val %d", $time, MultiCGRA.cgra__0.data_mem.recv_waddr__rdy[2], MultiCGRA.cgra__0.data_mem.recv_waddr__val[2]);
    $display("%t: cgra0datamem wdata2 ready %d val %d", $time, MultiCGRA.cgra__0.data_mem.recv_wdata__rdy[2], MultiCGRA.cgra__0.data_mem.recv_wdata__val[2]);
    $display("%t: cgra0datamem waddr3 ready %d val %d", $time, MultiCGRA.cgra__0.data_mem.recv_waddr__rdy[3], MultiCGRA.cgra__0.data_mem.recv_waddr__val[3]);
    $display("%t: cgra0datamem wdata3 ready %d val %d", $time, MultiCGRA.cgra__0.data_mem.recv_wdata__rdy[3], MultiCGRA.cgra__0.data_mem.recv_wdata__val[3]);
    */
    //$display("%t: init_mem_done %d", $time, MultiCGRA.cgra__0.data_mem.init_mem_done);
    $display("%t: ", $time);
    /*$display("%t: cgra0datamem wdata0 ready %d val %d", $time, MultiCGRA.cgra__0.data_mem.recv_wdata__rdy[0], MultiCGRA.cgra__0.data_mem.recv_wdata__val[0]);
    //$display("%t: cgra0rf0 wen %d", $time, MultiCGRA.cgra__0.data_mem.reg_file__wen[0][0]);
    //$display("%t: cgra0rf1 wen %d", $time, MultiCGRA.cgra__0.data_mem.reg_file__wen[1][0]);
    $display("%t: c0t1 ctrl mem 101 reg %d c0t1 ctrl mem 102 reg %d", $time, MultiCGRA.cgra__0.tile__1.ctrl_mem.ctrl_count_per_iter_val, MultiCGRA.cgra__0.tile__1.ctrl_mem.total_ctrl_steps_val);
    //$display("%t: 102 val %d c_ac %d addr %d msg %d", $time,
                                                      //MultiCGRA.cgra__0.tile__1.ctrl_mem.recv_pkt_queue__send__val, 
                                                      //MultiCGRA.cgra__0.tile__1.ctrl_mem.recv_pkt_queue__send__msg.ctrl_action,
                                                      //MultiCGRA.cgra__0.tile__1.ctrl_mem.recv_pkt_queue__send__msg.addr,
                                                      //MultiCGRA.cgra__0.tile__1.ctrl_mem.recv_pkt_queue__send__msg.data);
    $display("!!!!!!!!!!!! %d", recv_from_cpu_pkt__msg.payload.ctrl.fu_in[1]);
    $display("!!!!!!!!!!!! %d %d %d %d", send_to_cpu_pkt__val, send_to_cpu_pkt__msg.src, send_to_cpu_pkt__msg.dst, send_to_cpu_pkt__msg.payload.data.payload);
    //$display("!!!!!!!!!!!. %d %d", MultiCGRA.cgra__0.tile__1.send_data__val[0], MultiCGRA.cgra__0.tile__1.send_data__msg[0].payload);
    //$display("!!!!!!!!!!!. %d %d", MultiCGRA.cgra__0.tile__5.send_data__val[0], MultiCGRA.cgra__0.tile__5.send_data__msg[0].payload);
    //$display("!!!!!!!!!!!. %d %d", MultiCGRA.cgra__0.tile__1.send_data__val[1], MultiCGRA.cgra__0.tile__1.send_data__msg[1].payload);
    $display("!!!!!!!!!!!. %d %d %d", MultiCGRA.cgra__0.tile__5.send_data__val[1], MultiCGRA.cgra__0.tile__5.send_data__msg[1].payload, MultiCGRA.cgra__0.tile__5.send_data__msg[1].predicate);
    //$display("!!!!!!!!!!!. %d %d", MultiCGRA.cgra__0.tile__1.send_data__val[2], MultiCGRA.cgra__0.tile__1.send_data__msg[2].payload);
    //$display("!!!!!!!!!!!. %d %d", MultiCGRA.cgra__0.tile__5.send_data__val[2], MultiCGRA.cgra__0.tile__5.send_data__msg[2].payload);
    //$display("!!!!!!!!!!!. %d %d", MultiCGRA.cgra__0.tile__1.send_data__val[3], MultiCGRA.cgra__0.tile__1.send_data__msg[3].payload);
    //$display("!!!!!!!!!!!. %d %d", MultiCGRA.cgra__0.tile__5.send_data__val[3], MultiCGRA.cgra__0.tile__5.send_data__msg[3].payload);*/
  end

  /*initial
  begin  
    $dumpfile("./output.vcd");
    $dumpvars (0, cgra_test);
  end*/
  initial
  begin
    $fsdbDumpfile("./output.fsdb");
    //$fsdbDumpvars (0, "cgra_test.MultiCGRA.cgra__0.tile__5.send_data__msg");
    $fsdbDumpvars ("+all", "tb");
    $fsdbDumpMDA;
    $fsdbDumpSVA;
  end


endmodule

