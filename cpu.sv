module cpu
import rv32im_types::*;
(
    input   logic               clk,
    input   logic               rst,

    output  logic   [31:0]      dram_addr,
    output  logic               dram_read,
    output  logic               dram_write,
    output  logic   [63:0]      dram_wdata,
    input   logic               dram_ready,

    input   logic   [31:0]      dram_raddr,
    input   logic   [63:0]      dram_rdata,
    input   logic               dram_rvalid
);

    // ========================================================================
    // Internal wires
    // ========================================================================

    // Fetch outputs → IQ → decode → dispatch (2-wide)
    iq_entry_t      inst_data [2];
    logic [1:0]     inst_valid;
    logic [1:0]     inst_ready;

    // I-Cache DFP (fetch ↔ mem)
    logic [31:0]    icache_dfp_addr;
    logic           icache_dfp_read;
    logic           icache_dfp_write;
    logic [255:0]   icache_dfp_rdata;
    logic [255:0]   icache_dfp_wdata;
    logic           icache_dfp_resp;

    // D-Cache UFP (commit <-> dcache)
    logic [31:0]    dcache_ufp_addr;
    logic [3:0]     dcache_ufp_rmask;
    logic [3:0]     dcache_ufp_wmask;
    logic [31:0]    dcache_ufp_rdata;
    logic [31:0]    dcache_ufp_wdata;
    logic           dcache_ufp_resp;

    // Step C: commit-side dcache port (now drives stores only) and
    // load_issuer-side dcache port. dcache_arb muxes between them onto
    // the single physical UFP above.
    logic [31:0]    commit_dmem_addr;
    logic [3:0]     commit_dmem_rmask;
    logic [3:0]     commit_dmem_wmask;
    logic [31:0]    commit_dmem_wdata;
    logic           commit_dmem_resp;
    logic [31:0]    commit_dmem_rdata;

    logic           ld_dmem_req;
    logic [31:0]    ld_dmem_addr;
    logic [3:0]     ld_dmem_rmask;
    logic           ld_dmem_grant;
    logic           ld_dmem_resp;
    logic [31:0]    ld_dmem_rdata;

    // D-Cache DFP (dcache <-> mem)
    logic [31:0]    dcache_dfp_addr;
    logic           dcache_dfp_read;
    logic           dcache_dfp_write;
    logic [255:0]   dcache_dfp_wdata;
    logic [255:0]   dcache_dfp_rdata;
    logic           dcache_dfp_resp;

    // Branch redirect (unused in CP2)
    logic           flush;
    logic [31:0]    flush_target;
    logic           rob_head_mispredict;
    logic [31:0]    rob_head_correct_target;
    logic           rob_head_is_control;

    logic                  ctrl_resolve_valid;
    logic [ROB_IDX_W-1:0]  ctrl_resolve_rob_idx;
    logic [31:0]           ctrl_resolve_target;
    logic                  ctrl_resolve_valid_c;
    logic [ROB_IDX_W-1:0]  ctrl_resolve_rob_idx_c;
    logic [31:0]           ctrl_resolve_target_c;

    logic alu_is_branch, alu_is_jal, alu_is_jalr, alu_is_control, branch_taken;
    logic ctrl_commit_valid;
    logic [31:0] fallthrough_pc;
    logic [31:0] ctrl_actual_target;

    // IQ full signals (from fetch, for perf counters)
    logic perf_iq_full, perf_iq_full_ge2;

    // Branch predictor wires
    logic [31:0] bp_pred_pc;
    logic        bp_pred_taken;
    logic        bp_upd_valid;
    logic [31:0] bp_upd_pc;
    logic        bp_upd_taken;
    logic [BP_GHR_BITS-1:0] bp_upd_ghr;
    logic [1:0]  bp_spec_update;
    logic [1:0]  bp_spec_taken;
    logic [BP_GHR_BITS-1:0] bp_pred_ghr_out;

    // Lane-1 BP wires
    logic [31:0] bp_pred_pc_1;
    logic [BP_GHR_BITS-1:0] bp_pred_ghr_1;
    logic        bp_pred_taken_1;

    // RAS wires
    logic [31:0] ras_top;
    logic [31:0] ras_second;
    logic        ras_spec_push;
    logic [31:0] ras_spec_push_addr;
    logic [1:0]  ras_spec_pop;

    // BTB wires
    logic [31:0] btb_target;
    logic        btb_hit;

    // Perf-only frontend signal
    logic        fetch_perf_linebuf_miss;
    logic        perf_iq_empty;
    logic        rob_head_valid;
    // Step C: ROB <-> load_issuer prefetch interface + load-done flag.
    logic                       rob_commit_load_done;
    logic                       pf_req_valid;
    logic [ROB_IDX_W-1:0]       pf_req_idx;
    logic [31:0]                pf_req_addr;
    logic [2:0]                 pf_req_funct3;
    logic [PHYS_REG_W-1:0]      pf_req_phys_rd;
    logic                       pf_req_ready;

    // Perf-only RS signals (only alu_rs's is consumed; others tied off)
    logic        alu_rs_perf_ready_ge3;
    logic        mul_rs_perf_ready_ge3_unused;
    logic        div_rs_perf_ready_ge3_unused;
    logic        mem_rs_perf_ready_ge3_unused;


    // ========================================================================
    // Fetch Stage
    // ========================================================================
    fetch fetch_i (
        .clk            (clk),
        .rst            (rst),
        .inst_data      (inst_data),
        .inst_valid     (inst_valid),
        .inst_ready     (inst_ready),
        .dfp_addr       (icache_dfp_addr),
        .dfp_read       (icache_dfp_read),
        .dfp_write      (icache_dfp_write),
        .dfp_rdata      (icache_dfp_rdata),
        .dfp_wdata      (icache_dfp_wdata),
        .dfp_resp       (icache_dfp_resp),
        .branch_redirect(flush),
        .branch_target  (flush_target),
        .bp_pred_pc     (bp_pred_pc),
        .bp_pred_taken  (bp_pred_taken),
        .bp_spec_update (bp_spec_update),
        .bp_spec_taken  (bp_spec_taken),
        .bp_pred_ghr_out(bp_pred_ghr_out),
        .bp_pred_pc_1   (bp_pred_pc_1),
        .bp_pred_ghr_1  (bp_pred_ghr_1),
        .bp_pred_taken_1(bp_pred_taken_1),
        .ras_top        (ras_top),
        .ras_second     (ras_second),
        .ras_spec_push  (ras_spec_push),
        .ras_spec_push_addr(ras_spec_push_addr),
        .ras_spec_pop   (ras_spec_pop),
        .btb_target     (btb_target),
        .btb_hit        (btb_hit),
        .iq_full_out    (perf_iq_full),
        .iq_full_ge2_out(perf_iq_full_ge2),
        .perf_linebuf_miss(fetch_perf_linebuf_miss),
        .iq_empty_out   (perf_iq_empty)
    );

    // ========================================================================
    // Branch Predictor (gshare + bimodal tournament)
    // ========================================================================
    bp_tournament #(.INDEX_BITS(8), .GHR_BITS(14), .LHT_BITS(10), .LHIST_BITS(10)) bp_i (
        .clk                (clk),
        .rst                (rst),
        .pred_pc            (bp_pred_pc),
        .pred_taken         (bp_pred_taken),
        .pred_bim_taken     (),
        .pred_gsh_taken     (),
        .pred_choose_gshare (),
        .upd_valid          (bp_upd_valid),
        .upd_pc             (bp_upd_pc),
        .upd_taken          (bp_upd_taken),
        .upd_ghr            (bp_upd_ghr),
        .spec_update        (bp_spec_update),
        .spec_taken         (bp_spec_taken),
        .spec_flush         (flush),
        .pred_ghr_out       (bp_pred_ghr_out),
        .pred_pc_1          (bp_pred_pc_1),
        .pred_ghr_1         (bp_pred_ghr_1),
        .pred_taken_1       (bp_pred_taken_1)
    );

    // ========================================================================
    // Data Cache + Memory Stage
    // ========================================================================
    cache d_cache (
        .clk        (clk),
        .rst        (rst),
        .ufp_addr   (dcache_ufp_addr),
        .ufp_rmask  (dcache_ufp_rmask),
        .ufp_wmask  (dcache_ufp_wmask),
        .ufp_rdata  (dcache_ufp_rdata),
        .ufp_cl_data(),
        .ufp_wdata  (dcache_ufp_wdata),
        .ufp_resp   (dcache_ufp_resp),
        .dfp_addr   (dcache_dfp_addr),
        .dfp_read   (dcache_dfp_read),
        .dfp_write  (dcache_dfp_write),
        .dfp_rdata  (dcache_dfp_rdata),
        .dfp_wdata  (dcache_dfp_wdata),
        .dfp_resp   (dcache_dfp_resp)
    );

    // Step C: arbiter between commit (stores) and load_issuer (loads).
    dcache_arb dcache_arb_i (
        .clk          (clk),
        .rst          (rst),
        .commit_addr  (commit_dmem_addr),
        .commit_rmask (commit_dmem_rmask),
        .commit_wmask (commit_dmem_wmask),
        .commit_wdata (commit_dmem_wdata),
        .commit_resp  (commit_dmem_resp),
        .commit_rdata (commit_dmem_rdata),
        .load_req     (ld_dmem_req),
        .load_addr    (ld_dmem_addr),
        .load_rmask   (ld_dmem_rmask),
        .load_grant   (ld_dmem_grant),
        .load_resp    (ld_dmem_resp),
        .load_rdata   (ld_dmem_rdata),
        .ufp_addr     (dcache_ufp_addr),
        .ufp_rmask    (dcache_ufp_rmask),
        .ufp_wmask    (dcache_ufp_wmask),
        .ufp_wdata    (dcache_ufp_wdata),
        .ufp_resp     (dcache_ufp_resp),
        .ufp_rdata    (dcache_ufp_rdata)
    );

    // Step C: load_issuer drives the load slot of the CDB.
    // (Decls moved up here so port connections see proper widths.)
    logic                   cdb_load_valid;
    logic [PHYS_REG_W-1:0]  cdb_load_phys_rd;
    logic [31:0]            cdb_load_rd_data;
    logic [ROB_IDX_W-1:0]   cdb_load_rob_idx;
    logic [31:0]            load_raw_rdata;
    logic [31:0]            load_raw_addr;

    // Step D: SB hazard wires (decl before ROB because the SB module is
    // instantiated after load_issuer below).
    localparam integer unsigned SB_DEPTH_P = 4;
    logic [SB_DEPTH_P-1:0]  sb_hz_valid;
    logic [15:0]            sb_hz_word_mask;
    logic                   sb_hz_inflight_valid;

    load_issuer load_issuer_i (
        .clk             (clk),
        .rst             (rst),
        .flush           (flush),
        .pf_req_valid    (pf_req_valid),
        .pf_req_idx      (pf_req_idx),
        .pf_req_addr     (pf_req_addr),
        .pf_req_funct3   (pf_req_funct3),
        .pf_req_phys_rd  (pf_req_phys_rd),
        .pf_req_ready    (pf_req_ready),
        .ld_dmem_req     (ld_dmem_req),
        .ld_dmem_addr    (ld_dmem_addr),
        .ld_dmem_rmask   (ld_dmem_rmask),
        .ld_dmem_resp    (ld_dmem_resp),
        .ld_dmem_rdata   (ld_dmem_rdata),
        .load_cdb_valid  (cdb_load_valid),
        .load_cdb_phys_rd(cdb_load_phys_rd),
        .load_cdb_rd_data(cdb_load_rd_data),
        .load_cdb_rob_idx(cdb_load_rob_idx),
        .load_raw_rdata  (load_raw_rdata),
        .load_raw_addr   (load_raw_addr)
    );

    // Step D: store buffer. Decouples commit from dcache for stores.
    logic        sb_push;
    logic [31:0] sb_push_addr;
    logic [3:0]  sb_push_wmask;
    logic [31:0] sb_push_wdata;
    logic        sb_full;

    store_buffer #(.DEPTH(SB_DEPTH_P)) store_buffer_i (
        .clk         (clk),
        .rst         (rst),
        .push        (sb_push),
        .push_addr   (sb_push_addr),
        .push_wmask  (sb_push_wmask),
        .push_wdata  (sb_push_wdata),
        .full        (sb_full),
        .drain_req   (/* unused -- arb infers from |wmask */),
        .drain_addr  (commit_dmem_addr),
        .drain_wmask (commit_dmem_wmask),
        .drain_wdata (commit_dmem_wdata),
        .drain_resp  (commit_dmem_resp),
        .hz_addr            (),
        .hz_valid           (sb_hz_valid),
        .hz_word_mask       (sb_hz_word_mask),
        .hz_inflight_valid  (sb_hz_inflight_valid),
        .hz_inflight_addr   ()
    );

    // SB has no read path; tie rmask to 0 on the commit-side arb port.
    assign commit_dmem_rmask = 4'b0000;

    mem mem_i (
        .clk            (clk),
        .rst            (rst),
        .icache_dfp_addr(icache_dfp_addr),
        .icache_dfp_read(icache_dfp_read),
        .icache_dfp_write(icache_dfp_write),
        .icache_dfp_wdata(icache_dfp_wdata),
        .icache_dfp_rdata(icache_dfp_rdata),
        .icache_dfp_resp(icache_dfp_resp),
        .dcache_dfp_addr(dcache_dfp_addr),
        .dcache_dfp_read(dcache_dfp_read),
        .dcache_dfp_write(dcache_dfp_write),
        .dcache_dfp_wdata(dcache_dfp_wdata),
        .dcache_dfp_rdata(dcache_dfp_rdata),
        .dcache_dfp_resp(dcache_dfp_resp),
        .dram_addr      (dram_addr),
        .dram_read      (dram_read),
        .dram_write     (dram_write),
        .dram_wdata     (dram_wdata),
        .dram_ready     (dram_ready),
        .dram_raddr     (dram_raddr),
        .dram_rdata     (dram_rdata),
        .dram_rvalid    (dram_rvalid)
    );

    // ========================================================================
    // Decode Stage (purely combinational, 2-wide)
    // ========================================================================
    logic [4:0]     dec_rs1_s   [2], dec_rs2_s   [2], dec_rd_s [2];
    logic [2:0]     dec_funct3  [2];
    logic [6:0]     dec_funct7  [2];
    logic [31:0]    dec_imm     [2];
    alu_ops         dec_aluop   [2];
    alu_m1_sel_t    dec_alu_m1_sel [2];
    alu_m2_sel_t    dec_alu_m2_sel [2];
    logic [1:0]     dec_regf_we;
    logic [1:0]     dec_dmem_read, dec_dmem_write;
    logic [1:0]     dec_valid;

    decode decode0_i (
        .inst       (inst_data[0].inst),
        .valid_in   (inst_valid[0]),
        .rs1_s      (dec_rs1_s[0]),
        .rs2_s      (dec_rs2_s[0]),
        .rd_s       (dec_rd_s[0]),
        .funct3     (dec_funct3[0]),
        .funct7     (dec_funct7[0]),
        .imm        (dec_imm[0]),
        .aluop      (dec_aluop[0]),
        .alu_m1_sel (dec_alu_m1_sel[0]),
        .alu_m2_sel (dec_alu_m2_sel[0]),
        .regf_we    (dec_regf_we[0]),
        .dmem_read  (dec_dmem_read[0]),
        .dmem_write (dec_dmem_write[0]),
        .valid_out  (dec_valid[0])
    );

    decode decode1_i (
        .inst       (inst_data[1].inst),
        .valid_in   (inst_valid[1]),
        .rs1_s      (dec_rs1_s[1]),
        .rs2_s      (dec_rs2_s[1]),
        .rd_s       (dec_rd_s[1]),
        .funct3     (dec_funct3[1]),
        .funct7     (dec_funct7[1]),
        .imm        (dec_imm[1]),
        .aluop      (dec_aluop[1]),
        .alu_m1_sel (dec_alu_m1_sel[1]),
        .alu_m2_sel (dec_alu_m2_sel[1]),
        .regf_we    (dec_regf_we[1]),
        .dmem_read  (dec_dmem_read[1]),
        .dmem_write (dec_dmem_write[1]),
        .valid_out  (dec_valid[1])
    );

    // ========================================================================
    // Backend wires (2-wide dispatch)
    // ========================================================================

    // RAT (dispatch <-> RAT, 2-wide arrays directly)
    logic [4:0]             rat_rs1_arch [2], rat_rs2_arch [2], rat_rd_arch [2];
    logic [PHYS_REG_W-1:0] rat_rs1_phys [2], rat_rs2_phys [2], rat_rd_phys [2], rat_rd_old_phys [2];
    logic [1:0]             rat_wr_en;

    // Free list (dispatch alloc 2-wide; commit free 2-wide)
    logic [1:0]             fl_alloc_en;
    logic [PHYS_REG_W-1:0] fl_alloc_phys [2];
    logic                   fl_empty, fl_empty_ge2;
    logic [1:0]             fl_free_en;
    logic [PHYS_REG_W-1:0] fl_free_phys [2];

    // ROB alloc (2-wide from dispatch) + commit (1-wide)
    logic [1:0]             rob_alloc_en;
    logic [31:0]            rob_alloc_pc        [2];
    logic [31:0]            rob_alloc_inst      [2];
    logic [31:0]            rob_alloc_pred_target [2];
    logic [BP_GHR_BITS-1:0]  rob_alloc_pred_ghr [2];
    logic [4:0]             rob_alloc_rd_arch   [2];
    logic [PHYS_REG_W-1:0] rob_alloc_rd_phys   [2];
    logic [PHYS_REG_W-1:0] rob_alloc_rd_old_phys [2];
    logic [4:0]             rob_alloc_rs1_addr  [2];
    logic [4:0]             rob_alloc_rs2_addr  [2];
    logic [ROB_IDX_W-1:0]  rob_alloc_idx       [2];
    logic                   rob_full, rob_full_ge2;

    // ROB commit (2-wide)
    logic [1:0]             rob_commit_en;
    logic                   rob_commit_ready;
    logic [BP_GHR_BITS-1:0] rob_commit_pred_ghr;
    logic [31:0]            rob_commit_pc, rob_commit_inst;
    logic [4:0]             rob_commit_rd_arch;
    logic [PHYS_REG_W-1:0] rob_commit_rd_phys, rob_commit_rd_old_phys;
    logic [31:0]            rob_commit_rd_wdata;
    logic [4:0]             rob_commit_rs1_addr, rob_commit_rs2_addr;
    logic [ROB_IDX_W-1:0]  rob_commit_idx;
    logic                   rob_commit_is_load, rob_commit_is_store;
    logic [31:0]            rob_commit_mem_addr, rob_commit_mem_wdata;
    logic [3:0]             rob_commit_mem_wmask;
    logic [31:0]            rob_commit_mem_rdata;

    // ROB commit slot 1 (head+1)
    logic                   rob_commit_1_ready;
    logic [31:0]            rob_commit_1_pc, rob_commit_1_inst;
    logic [4:0]             rob_commit_1_rd_arch;
    logic [PHYS_REG_W-1:0] rob_commit_1_rd_phys, rob_commit_1_rd_old_phys;
    logic [31:0]            rob_commit_1_rd_wdata;
    logic [4:0]             rob_commit_1_rs1_addr, rob_commit_1_rs2_addr;
    logic                   rob_commit_1_is_load, rob_commit_1_is_store, rob_commit_1_is_control;
    logic                   rob_commit_1_load_done;
    logic [31:0]            rob_commit_1_mem_addr, rob_commit_1_mem_rdata;

    // PRF (2-wide dispatch ports)
    logic [PHYS_REG_W-1:0] prf_ready_q1_idx [2], prf_ready_q2_idx [2];
    logic [1:0]             prf_ready_q1, prf_ready_q2;
    logic [1:0]             prf_set_not_ready;
    logic [PHYS_REG_W-1:0] prf_set_not_ready_idx [2];
    // RS (2-wide insert per FU)
    logic [1:0]             rs_alu_insert_en, rs_mul_insert_en, rs_div_insert_en, rs_mem_insert_en;
    rs_entry_t              rs_insert_data [2];
    logic                   rs_alu_full, rs_mul_full, rs_div_full, rs_mem_full;
    logic                   rs_alu_full_ge2, rs_mul_full_ge2, rs_div_full_ge2, rs_mem_full_ge2;

    // RS issue (2-wide per RS; only port [0] used for non-ALU FUs)
    logic [1:0]             alu_issue_valid, mul_issue_valid, div_issue_valid, mem_issue_valid;
    rs_entry_t              alu_issue_data [2], mul_issue_data [2], div_issue_data [2], mem_issue_data [2];

    // CDB
    cdb_t                   cdb_out [2];
    logic                   cdb_mul_stall, cdb_div_stall;
    // cdb_load_* declared earlier (before load_issuer_i instance)

    // FU outputs
    logic                   alu_fu_valid, alu1_fu_valid, mul_fu_valid, div_fu_valid;
    logic [PHYS_REG_W-1:0] alu_fu_phys_rd, alu1_fu_phys_rd, mul_fu_phys_rd, div_fu_phys_rd;
    logic [31:0]            alu_fu_rd_data, alu1_fu_rd_data, mul_fu_rd_data, div_fu_rd_data;
    logic [ROB_IDX_W-1:0]  alu_fu_rob_idx, alu1_fu_rob_idx, mul_fu_rob_idx, div_fu_rob_idx;
    logic                   mul_fu_ready, div_fu_ready;
    logic                   alu0_issue_ready_c, alu1_issue_ready_c;

    // FU regfile reads
    logic [PHYS_REG_W-1:0] alu_prf_rs1_idx, alu_prf_rs2_idx;
    logic [31:0]            alu_prf_rs1_data, alu_prf_rs2_data;
    logic [PHYS_REG_W-1:0] alu1_prf_rs1_idx, alu1_prf_rs2_idx;
    logic [31:0]            alu1_prf_rs1_data, alu1_prf_rs2_data;
    logic [PHYS_REG_W-1:0] mul_prf_rs1_idx, mul_prf_rs2_idx;
    logic [31:0]            mul_prf_rs1_data, mul_prf_rs2_data;
    logic [PHYS_REG_W-1:0] div_prf_rs1_idx, div_prf_rs2_idx;
    logic [31:0]            div_prf_rs1_data, div_prf_rs2_data;
    logic [PHYS_REG_W-1:0] mem_prf_rs1_idx, mem_prf_rs2_idx;
    logic [31:0]            mem_prf_rs1_data, mem_prf_rs2_data;

    // AGU metadata to ROB
    mem_result_t            mem_result;

    // Commit PRF reads (2-wide)
    logic [PHYS_REG_W-1:0] commit_prf_0_rs1_idx, commit_prf_0_rs2_idx;
    logic [31:0]            commit_prf_0_rs1_data, commit_prf_0_rs2_data;
    logic [PHYS_REG_W-1:0] commit_prf_1_rs1_idx, commit_prf_1_rs2_idx;
    logic [31:0]            commit_prf_1_rs1_data, commit_prf_1_rs2_data;
    // Power: gated commit PRF indices (hold at 0 when slot not committing)
    logic [PHYS_REG_W-1:0] commit_prf_0_rs1_idx_g, commit_prf_0_rs2_idx_g;
    logic [PHYS_REG_W-1:0] commit_prf_1_rs1_idx_g, commit_prf_1_rs2_idx_g;
    logic [PHYS_REG_W-1:0] rrf_table [32];

    // Dispatch → IQ backpressure (2-wide)
    logic [1:0]             dispatch_ready;
    assign inst_ready = dispatch_ready;

    // PRF safety on flush cycle
    logic [1:0] cdb_out_valid;
    assign cdb_out_valid[0] = cdb_out[0].valid && !flush;
    assign cdb_out_valid[1] = cdb_out[1].valid && !flush;

    // Power: gate CDB phys_rd to 0 when invalid. Prevents RS/ROB comparators
    // and PRF write-decode trees from switching on idle CDB slots.
    cdb_t cdb_gated [2];
    always_comb begin
        cdb_gated[0].valid   = cdb_out[0].valid && !flush;
        cdb_gated[0].phys_rd = cdb_out_valid[0] ? cdb_out[0].phys_rd : '0;
        cdb_gated[0].rd_data = cdb_out[0].rd_data;
        cdb_gated[0].rob_idx = cdb_out[0].rob_idx;
        cdb_gated[1].valid   = cdb_out[1].valid && !flush;
        cdb_gated[1].phys_rd = cdb_out_valid[1] ? cdb_out[1].phys_rd : '0;
        cdb_gated[1].rd_data = cdb_out[1].rd_data;
        cdb_gated[1].rob_idx = cdb_out[1].rob_idx;
    end

    // ========================================================================
    // Dispatch (2-wide)
    // ========================================================================
    logic [31:0] disp_inst       [2];
    logic [31:0] disp_pc         [2];
    logic [31:0] disp_pred_target[2];
    logic        disp_pred_taken [2];
    logic [BP_GHR_BITS-1:0] disp_pred_ghr [2];
    assign disp_inst[0]        = inst_data[0].inst;
    assign disp_inst[1]        = inst_data[1].inst;
    assign disp_pc[0]          = inst_data[0].pc;
    assign disp_pc[1]          = inst_data[1].pc;
    assign disp_pred_target[0] = inst_data[0].pred_target;
    assign disp_pred_target[1] = inst_data[1].pred_target;
    assign disp_pred_taken[0]  = inst_data[0].pred_taken;
    assign disp_pred_taken[1]  = inst_data[1].pred_taken;
    assign disp_pred_ghr[0]    = inst_data[0].pred_ghr;
    assign disp_pred_ghr[1]    = inst_data[1].pred_ghr;

    dispatch dispatch_i (
        .inst_valid         (inst_valid),
        .inst               (disp_inst),
        .pc                 (disp_pc),
        .pred_target        (disp_pred_target),
        .pred_taken         (disp_pred_taken),
        .pred_ghr           (disp_pred_ghr),
        .dispatch_ready     (dispatch_ready),
        .dec_rs1_s          (dec_rs1_s),
        .dec_rs2_s          (dec_rs2_s),
        .dec_rd_s           (dec_rd_s),
        .dec_funct3         (dec_funct3),
        .dec_funct7         (dec_funct7),
        .dec_imm            (dec_imm),
        .dec_aluop          (dec_aluop),
        .dec_alu_m1_sel     (dec_alu_m1_sel),
        .dec_alu_m2_sel     (dec_alu_m2_sel),
        .dec_regf_we        (dec_regf_we),
        .dec_dmem_read      (dec_dmem_read),
        .dec_dmem_write     (dec_dmem_write),
        .rat_rs1_arch       (rat_rs1_arch),
        .rat_rs2_arch       (rat_rs2_arch),
        .rat_rs1_phys       (rat_rs1_phys),
        .rat_rs2_phys       (rat_rs2_phys),
        .rat_wr_en          (rat_wr_en),
        .rat_rd_arch        (rat_rd_arch),
        .rat_rd_phys        (rat_rd_phys),
        .rat_rd_old_phys    (rat_rd_old_phys),
        .fl_alloc_en        (fl_alloc_en),
        .fl_alloc_phys      (fl_alloc_phys),
        .fl_empty           (fl_empty),
        .fl_empty_ge2       (fl_empty_ge2),
        .rob_alloc_en       (rob_alloc_en),
        .rob_alloc_pc       (rob_alloc_pc),
        .rob_alloc_inst     (rob_alloc_inst),
        .rob_alloc_pred_target(rob_alloc_pred_target),
        .rob_alloc_pred_ghr (rob_alloc_pred_ghr),
        .rob_alloc_rd_arch  (rob_alloc_rd_arch),
        .rob_alloc_rd_phys  (rob_alloc_rd_phys),
        .rob_alloc_rd_old_phys(rob_alloc_rd_old_phys),
        .rob_alloc_rs1_addr (rob_alloc_rs1_addr),
        .rob_alloc_rs2_addr (rob_alloc_rs2_addr),
        .rob_alloc_idx      (rob_alloc_idx),
        .rob_full           (rob_full),
        .rob_full_ge2       (rob_full_ge2),
        .prf_ready_q1_idx   (prf_ready_q1_idx),
        .prf_ready_q2_idx   (prf_ready_q2_idx),
        .prf_ready_q1       (prf_ready_q1),
        .prf_ready_q2       (prf_ready_q2),
        .prf_set_not_ready  (prf_set_not_ready),
        .prf_set_not_ready_idx(prf_set_not_ready_idx),
        .rs_alu_insert_en   (rs_alu_insert_en),
        .rs_mul_insert_en   (rs_mul_insert_en),
        .rs_div_insert_en   (rs_div_insert_en),
        .rs_mem_insert_en   (rs_mem_insert_en),
        .rs_insert_data     (rs_insert_data),
        .rs_alu_full        (rs_alu_full),
        .rs_mul_full        (rs_mul_full),
        .rs_div_full        (rs_div_full),
        .rs_mem_full        (rs_mem_full),
        .rs_alu_full_ge2    (rs_alu_full_ge2),
        .rs_mul_full_ge2    (rs_mul_full_ge2),
        .rs_div_full_ge2    (rs_div_full_ge2),
        .rs_mem_full_ge2    (rs_mem_full_ge2),
        .cdb_in             (cdb_gated),
        .flush              (flush)
    );

    // ========================================================================
    // RAT
    // ========================================================================
    rat rat_i (
        .clk        (clk),
        .rst        (rst),
        .rs1_arch   (rat_rs1_arch),
        .rs2_arch   (rat_rs2_arch),
        .rs1_phys   (rat_rs1_phys),
        .rs2_phys   (rat_rs2_phys),
        .wr_en      (rat_wr_en),
        .rd_arch    (rat_rd_arch),
        .rd_phys    (rat_rd_phys),
        .rd_old_phys(rat_rd_old_phys),
        .flush      (flush),
        .rrf_table  (rrf_table)
    );

    // ========================================================================
    // Free List
    // ========================================================================
    free_list fl_i (
        .clk        (clk),
        .rst        (rst),
        .alloc_en   (fl_alloc_en),
        .alloc_phys (fl_alloc_phys),
        .empty      (fl_empty),
        .empty_ge2  (fl_empty_ge2),
        .free_en    (fl_free_en),
        .free_phys  (fl_free_phys),
        .flush      (flush),
        .rrf_table  (rrf_table)
    );

    // ========================================================================
    // ROB
    // ========================================================================
    rob rob_i (
        .clk                (clk),
        .rst                (rst),
        .alloc_en           (rob_alloc_en),
        .alloc_pc           (rob_alloc_pc),
        .alloc_inst         (rob_alloc_inst),
        .alloc_pred_target  (rob_alloc_pred_target),
        .alloc_pred_ghr     (rob_alloc_pred_ghr),
        .alloc_rd_arch      (rob_alloc_rd_arch),
        .alloc_rd_phys      (rob_alloc_rd_phys),
        .alloc_rd_old_phys  (rob_alloc_rd_old_phys),
        .alloc_rs1_addr     (rob_alloc_rs1_addr),
        .alloc_rs2_addr     (rob_alloc_rs2_addr),
        .alloc_idx          (rob_alloc_idx),
        .full               (rob_full),
        .full_ge2           (rob_full_ge2),
        .cdb_in             (cdb_gated),
        .mem_result_in      (mem_result),
        .commit_ready       (rob_commit_ready),
        .commit_pc          (rob_commit_pc),
        .commit_inst        (rob_commit_inst),
        .commit_rd_arch     (rob_commit_rd_arch),
        .commit_rd_phys     (rob_commit_rd_phys),
        .commit_rd_old_phys (rob_commit_rd_old_phys),
        .commit_rd_wdata    (rob_commit_rd_wdata),
        .commit_rs1_addr    (rob_commit_rs1_addr),
        .commit_rs2_addr    (rob_commit_rs2_addr),
        .commit_idx         (rob_commit_idx),
        .commit_is_load     (rob_commit_is_load),
        .commit_is_store    (rob_commit_is_store),
        .commit_mem_addr    (rob_commit_mem_addr),
        .commit_mem_wdata   (rob_commit_mem_wdata),
        .commit_mem_wmask   (rob_commit_mem_wmask),
        .commit_mem_rdata   (rob_commit_mem_rdata),
        // Commit slot 1 (head+1)
        .commit_1_ready       (rob_commit_1_ready),
        .commit_1_pc          (rob_commit_1_pc),
        .commit_1_inst        (rob_commit_1_inst),
        .commit_1_rd_arch     (rob_commit_1_rd_arch),
        .commit_1_rd_phys     (rob_commit_1_rd_phys),
        .commit_1_rd_old_phys (rob_commit_1_rd_old_phys),
        .commit_1_rd_wdata    (rob_commit_1_rd_wdata),
        .commit_1_rs1_addr    (rob_commit_1_rs1_addr),
        .commit_1_rs2_addr    (rob_commit_1_rs2_addr),
        .commit_1_idx         (),
        .commit_1_is_load     (rob_commit_1_is_load),
        .commit_1_is_store    (rob_commit_1_is_store),
        .commit_1_is_control  (rob_commit_1_is_control),
        .commit_1_load_done   (rob_commit_1_load_done),
        .commit_1_mem_addr    (rob_commit_1_mem_addr),
        .commit_1_mem_rdata   (rob_commit_1_mem_rdata),
        .commit_en            (rob_commit_en),

        .flush                   (flush),
        .head_is_control         (rob_head_is_control),
        .head_mispredict         (rob_head_mispredict),
        .head_correct_target     (rob_head_correct_target),
        .commit_pred_ghr         (rob_commit_pred_ghr),
        .ctrl_resolve_valid      (ctrl_resolve_valid),
        .ctrl_resolve_rob_idx    (ctrl_resolve_rob_idx),
        .ctrl_resolve_target     (ctrl_resolve_target),
        .head_valid              (rob_head_valid),
        // Step C: prefetch / load-done plumbing
        .commit_load_done        (rob_commit_load_done),
        .pf_req_valid            (pf_req_valid),
        .pf_req_idx              (pf_req_idx),
        .pf_req_addr             (pf_req_addr),
        .pf_req_funct3           (pf_req_funct3),
        .pf_req_phys_rd          (pf_req_phys_rd),
        .pf_req_ready            (pf_req_ready),
        .sb_hz_word_mask         (sb_hz_word_mask),
        // Step C: raw rdata side-band into mem_rdata_q
        .load_raw_valid          (cdb_load_valid),
        .load_raw_rob_idx        (cdb_load_rob_idx),
        .load_raw_rdata          (load_raw_rdata)
    );

    // ========================================================================
    // Physical Register File
    // ========================================================================
    // Power: gate commit RVFI read indices when slot not committing
    assign commit_prf_0_rs1_idx_g = rob_commit_en[0] ? commit_prf_0_rs1_idx : '0;
    assign commit_prf_0_rs2_idx_g = rob_commit_en[0] ? commit_prf_0_rs2_idx : '0;
    assign commit_prf_1_rs1_idx_g = rob_commit_en[1] ? commit_prf_1_rs1_idx : '0;
    assign commit_prf_1_rs2_idx_g = rob_commit_en[1] ? commit_prf_1_rs2_idx : '0;

    phys_regfile prf_i (
        .clk                (clk),
        .rst                (rst),
        // CDB write (2-wide) — indices gated via cdb_gated
        .wr_en              (cdb_out_valid),
        .wr_idx_0           (cdb_gated[0].phys_rd),
        .wr_data_0          (cdb_out[0].rd_data),
        .wr_idx_1           (cdb_gated[1].phys_rd),
        .wr_data_1          (cdb_out[1].rd_data),
        // ALU issue reads
        .alu_rs1_idx        (alu_prf_rs1_idx),
        .alu_rs1_data       (alu_prf_rs1_data),
        .alu_rs2_idx        (alu_prf_rs2_idx),
        .alu_rs2_data       (alu_prf_rs2_data),
        // ALU1 issue reads
        .alu1_rs1_idx       (alu1_prf_rs1_idx),
        .alu1_rs1_data      (alu1_prf_rs1_data),
        .alu1_rs2_idx       (alu1_prf_rs2_idx),
        .alu1_rs2_data      (alu1_prf_rs2_data),
        // MUL issue reads
        .mul_rs1_idx        (mul_prf_rs1_idx),
        .mul_rs1_data       (mul_prf_rs1_data),
        .mul_rs2_idx        (mul_prf_rs2_idx),
        .mul_rs2_data       (mul_prf_rs2_data),
        // DIV issue reads
        .div_rs1_idx        (div_prf_rs1_idx),
        .div_rs1_data       (div_prf_rs1_data),
        .div_rs2_idx        (div_prf_rs2_idx),
        .div_rs2_data       (div_prf_rs2_data),
        // MEM issue reads
        .mem_rs1_idx        (mem_prf_rs1_idx),
        .mem_rs1_data       (mem_prf_rs1_data),
        .mem_rs2_idx        (mem_prf_rs2_idx),
        .mem_rs2_data       (mem_prf_rs2_data),
        // Commit RVFI reads (slot 0) — gated
        .commit0_rs1_idx    (commit_prf_0_rs1_idx_g),
        .commit0_rs1_data   (commit_prf_0_rs1_data),
        .commit0_rs2_idx    (commit_prf_0_rs2_idx_g),
        .commit0_rs2_data   (commit_prf_0_rs2_data),
        // Commit RVFI reads (slot 1) — gated
        .commit1_rs1_idx    (commit_prf_1_rs1_idx_g),
        .commit1_rs1_data   (commit_prf_1_rs1_data),
        .commit1_rs2_idx    (commit_prf_1_rs2_idx_g),
        .commit1_rs2_data   (commit_prf_1_rs2_data),
        // Scoreboard
        .set_not_ready      (prf_set_not_ready),
        .set_not_ready_idx  (prf_set_not_ready_idx),
        .ready_q1_idx       (prf_ready_q1_idx),
        .ready_q2_idx       (prf_ready_q2_idx),
        .ready_q1           (prf_ready_q1),
        .ready_q2           (prf_ready_q2)
    );

    // ========================================================================
    // Reservation Stations
    // ========================================================================
    rs #(.DEPTH(8), .CONTROL_PORT0_ONLY(1'b1)) alu_rs (
        .clk        (clk),
        .rst        (rst),
        .insert_en  (rs_alu_insert_en),
        .insert_data(rs_insert_data),
        .full       (rs_alu_full),
        .full_ge2   (rs_alu_full_ge2),
        .cdb_in     (cdb_gated),
        .issue_valid(alu_issue_valid),
        .issue_data (alu_issue_data),
        .fu_ready   ({alu1_issue_ready_c, alu0_issue_ready_c}),
        .flush      (flush),
        .perf_ready_ge3(alu_rs_perf_ready_ge3)
    );

    rs #(.DEPTH(1)) mul_rs (
        .clk        (clk),
        .rst        (rst),
        .insert_en  (rs_mul_insert_en),
        .insert_data(rs_insert_data),
        .full       (rs_mul_full),
        .full_ge2   (rs_mul_full_ge2),
        .cdb_in     (cdb_gated),
        .issue_valid(mul_issue_valid),
        .issue_data (mul_issue_data),
        .fu_ready   ({1'b0, mul_fu_ready}),
        .flush      (flush),
        .perf_ready_ge3(mul_rs_perf_ready_ge3_unused)
    );

    rs #(.DEPTH(1)) div_rs (
        .clk        (clk),
        .rst        (rst),
        .insert_en  (rs_div_insert_en),
        .insert_data(rs_insert_data),
        .full       (rs_div_full),
        .full_ge2   (rs_div_full_ge2),
        .cdb_in     (cdb_gated),
        .issue_valid(div_issue_valid),
        .issue_data (div_issue_data),
        .fu_ready   ({1'b0, div_fu_ready}),
        .flush      (flush),
        .perf_ready_ge3(div_rs_perf_ready_ge3_unused)
    );

    // mem_rs DEPTH 4 -> 16: cyc_rs_mem_full_ge2 was 16.9% with depth 4 and still
    // ~28% with depth 8 once ROB widened. Memory-heavy bursts (mergesort, image)
    // commit-rate-limited; bigger window lets dispatch keep streaming while AGU
    // drains in-order. ALU RS also bumped 8->12 for symmetric back-pressure.
    rs #(.DEPTH(10)) mem_rs (
        .clk        (clk),
        .rst        (rst),
        .insert_en  (rs_mem_insert_en),
        .insert_data(rs_insert_data),
        .full       (rs_mem_full),
        .full_ge2   (rs_mem_full_ge2),
        .cdb_in     (cdb_gated),
        .issue_valid(mem_issue_valid),
        .issue_data (mem_issue_data),
        .fu_ready   (2'b01),
        .flush      (flush),
        .perf_ready_ge3(mem_rs_perf_ready_ge3_unused)
    );

    // ========================================================================
    // Regfile read index wiring (from RS issue to PRF)
    // ========================================================================
    assign alu0_issue_ready_c = !(cdb_load_valid && alu1_fu_valid);
    assign alu1_issue_ready_c = !cdb_load_valid;

    // Power: gate PRF read indices to 0 when FU isn't issuing.
    // Prevents 48:1×32-bit mux trees from switching on idle ports.
    // NOTE: ALU ports are NOT gated — they are on the critical timing path
    // (RS issue → PRF read → ALU → PRF write).
    assign alu_prf_rs1_idx = alu_issue_data[0].rs1_phys;
    assign alu_prf_rs2_idx = alu_issue_data[0].rs2_phys;
    assign mul_prf_rs1_idx = mul_issue_valid[0] ? mul_issue_data[0].rs1_phys : '0;
    assign mul_prf_rs2_idx = mul_issue_valid[0] ? mul_issue_data[0].rs2_phys : '0;
    assign div_prf_rs1_idx = div_issue_valid[0] ? div_issue_data[0].rs1_phys : '0;
    assign div_prf_rs2_idx = div_issue_valid[0] ? div_issue_data[0].rs2_phys : '0;

    // ========================================================================
    // Control Resolve Logic
    // ========================================================================
    assign ctrl_commit_valid = rob_commit_ready && rob_head_is_control;
    assign flush = ctrl_commit_valid && rob_head_mispredict;
    assign flush_target = rob_head_correct_target;
    assign fallthrough_pc = alu_issue_data[0].pc_p4; // precomputed at dispatch

    // Branch predictor update at commit (conditional branches only).
    // Train direction = (resolved target != static fallthrough) so true taken
    // branches train to "taken" and not-taken/fallthrough train to "not-taken".
    assign bp_upd_valid = ctrl_commit_valid &&
                          (rob_commit_inst[6:0] == 7'b1100011);
    assign bp_upd_pc    = rob_commit_pc;
    assign bp_upd_taken = (rob_head_correct_target != (rob_commit_pc + 32'd4));
    assign bp_upd_ghr   = rob_commit_pred_ghr;

    // ========================================================================
    // Return Address Stack
    // ========================================================================
    // Commit-time call / return detection.
    logic commit_is_jal_c, commit_is_jalr_c;
    logic commit_rd_link_c, commit_rs1_link_c;
    logic commit_is_call_c, commit_is_return_c;

    assign commit_is_jal_c    = (rob_commit_inst[6:0] == 7'b1101111);
    assign commit_is_jalr_c   = (rob_commit_inst[6:0] == 7'b1100111);
    assign commit_rd_link_c   = (rob_commit_inst[11:7] == 5'd1) || (rob_commit_inst[11:7] == 5'd5);
    assign commit_rs1_link_c  = (rob_commit_inst[19:15] == 5'd1) || (rob_commit_inst[19:15] == 5'd5);
    assign commit_is_call_c   = (commit_is_jal_c || commit_is_jalr_c) && commit_rd_link_c;
    assign commit_is_return_c = commit_is_jalr_c && commit_rs1_link_c && !commit_rd_link_c;

    ras #(.DEPTH(4)) ras_i (
        .clk            (clk),
        .rst            (rst),
        .ras_top        (ras_top),
        .ras_second     (ras_second),
        .spec_push      (ras_spec_push),
        .spec_push_addr (ras_spec_push_addr),
        .spec_pop       (ras_spec_pop),
        .arch_push      (ctrl_commit_valid && commit_is_call_c),
        .arch_push_addr (rob_commit_pc + 32'd4),
        .arch_pop       (ctrl_commit_valid && commit_is_return_c),
        .flush          (flush)
    );

    // ========================================================================
    // Branch Target Buffer (indirect-jump targets, excludes returns)
    // ========================================================================
    btb #(.NUM_ENTRIES(8)) btb_i (
        .clk        (clk),
        .rst        (rst),
        .lookup_pc  (bp_pred_pc),
        .btb_target (btb_target),
        .btb_hit    (btb_hit),
        .upd_valid  (ctrl_commit_valid && commit_is_jalr_c && !commit_is_return_c),
        .upd_pc     (rob_commit_pc),
        .upd_target (rob_head_correct_target)
    );

    // Control resolve signal computations.
    assign alu_is_branch  = (alu_issue_data[0].opcode == op_b_br);
    assign alu_is_jal     = (alu_issue_data[0].opcode == op_b_jal);
    assign alu_is_jalr    = (alu_issue_data[0].opcode == op_b_jalr) && (alu_issue_data[0].funct3 == 3'b000);
    assign alu_is_control = alu_is_branch || alu_is_jal || alu_is_jalr;
    
    assign ctrl_resolve_valid_c   = alu_issue_valid[0] && alu_is_control;
    assign ctrl_resolve_rob_idx_c = alu_issue_data[0].rob_idx;
    assign ctrl_resolve_target_c  = ctrl_actual_target;

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            ctrl_resolve_valid   <= 1'b0;
            ctrl_resolve_rob_idx <= '0;
            ctrl_resolve_target  <= '0;
        end else begin
            ctrl_resolve_valid   <= ctrl_resolve_valid_c;
            ctrl_resolve_rob_idx <= ctrl_resolve_rob_idx_c;
            ctrl_resolve_target  <= ctrl_resolve_target_c;
        end
    end

    always_comb begin
        branch_taken = 1'b0;
        ctrl_actual_target = 32'b0;

        unique case (alu_issue_data[0].funct3)
            branch_f3_beq:  branch_taken = (alu_prf_rs1_data == alu_prf_rs2_data);
            branch_f3_bne:  branch_taken = (alu_prf_rs1_data != alu_prf_rs2_data);
            branch_f3_blt:  branch_taken = ($signed(alu_prf_rs1_data) <  $signed(alu_prf_rs2_data));
            branch_f3_bge:  branch_taken = ($signed(alu_prf_rs1_data) >= $signed(alu_prf_rs2_data));
            branch_f3_bltu: branch_taken = (alu_prf_rs1_data <  alu_prf_rs2_data);
            branch_f3_bgeu: branch_taken = (alu_prf_rs1_data >= alu_prf_rs2_data);
            default:        branch_taken = 1'b0;
        endcase

        if (alu_is_branch) begin
            ctrl_actual_target = branch_taken ? alu_issue_data[0].imm_target : fallthrough_pc;
        end else if (alu_is_jal) begin
            ctrl_actual_target = alu_issue_data[0].imm_target;
        end else if (alu_is_jalr) begin
            ctrl_actual_target = (alu_prf_rs1_data + alu_issue_data[0].imm) & 32'hffff_fffe; // handles JALR edge case
        end else begin
            ctrl_actual_target = 32'b0;
        end
    end
    
    assign mem_prf_rs1_idx = mem_issue_valid[0] ? mem_issue_data[0].rs1_phys : '0;
    assign mem_prf_rs2_idx = mem_issue_valid[0] ? mem_issue_data[0].rs2_phys : '0;

    agu agu_i (
        .issue_valid (mem_issue_valid[0]),
        .issue_data  (mem_issue_data[0]),
        .rs1_data    (mem_prf_rs1_data),
        .rs2_data    (mem_prf_rs2_data),
        .mem_result  (mem_result)
    );

    // ========================================================================
    // ALU Functional Unit (combinational path: RS issue → mux → ALU → CDB)
    // ========================================================================
    logic [31:0] alu_op_a, alu_op_b, alu_result;

    // Operand isolation: gate ALU0 inputs to zero when no issue (power).
    assign alu_op_a = alu_issue_valid[0]
                    ? ((alu_issue_data[0].alu_m1_sel == pc_out) ? alu_issue_data[0].pc  : alu_prf_rs1_data)
                    : '0;
    assign alu_op_b = alu_issue_valid[0]
                    ? ((alu_issue_data[0].alu_m2_sel == imm_out) ? alu_issue_data[0].imm : alu_prf_rs2_data)
                    : '0;

    alu alu_i (
        .aluop  (alu_issue_data[0].aluop),
        .op_a   (alu_op_a),
        .op_b   (alu_op_b),
        .alu_out(alu_result)
    );

    assign alu_fu_valid   = alu_issue_valid[0];
    assign alu_fu_phys_rd = alu_issue_data[0].rd_phys;
    assign alu_fu_rd_data = (alu_is_jal || alu_is_jalr) ? (alu_issue_data[0].pc + 32'd4) : alu_result;
    assign alu_fu_rob_idx = alu_issue_data[0].rob_idx;

    // ========================================================================
    // ALU1 Functional Unit (2nd ALU — no branch/control, pure ALU ops)
    // ========================================================================
    logic [31:0] alu1_op_a, alu1_op_b, alu1_result;
    logic                   alu1_fu_valid_q;
    logic [PHYS_REG_W-1:0]  alu1_fu_phys_rd_q;
    logic [31:0]            alu1_fu_rd_data_q;
    logic [ROB_IDX_W-1:0]   alu1_fu_rob_idx_q;

    assign alu1_prf_rs1_idx = alu_issue_valid[1] ? alu_issue_data[1].rs1_phys : '0;
    assign alu1_prf_rs2_idx = alu_issue_valid[1] ? alu_issue_data[1].rs2_phys : '0;

    // Operand isolation: gate ALU1 inputs to zero when no issue (power).
    assign alu1_op_a = alu_issue_valid[1]
                     ? ((alu_issue_data[1].alu_m1_sel == pc_out) ? alu_issue_data[1].pc  : alu1_prf_rs1_data)
                     : '0;
    assign alu1_op_b = alu_issue_valid[1]
                     ? ((alu_issue_data[1].alu_m2_sel == imm_out) ? alu_issue_data[1].imm : alu1_prf_rs2_data)
                     : '0;

    alu alu1_i (
        .aluop  (alu_issue_data[1].aluop),
        .op_a   (alu1_op_a),
        .op_b   (alu1_op_b),
        .alu_out(alu1_result)
    );

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            alu1_fu_valid_q   <= 1'b0;
            alu1_fu_phys_rd_q <= '0;
            alu1_fu_rd_data_q <= '0;
            alu1_fu_rob_idx_q <= '0;
        end else begin
            alu1_fu_valid_q   <= alu_issue_valid[1];
            alu1_fu_phys_rd_q <= alu_issue_data[1].rd_phys;
            alu1_fu_rd_data_q <= alu1_result;
            alu1_fu_rob_idx_q <= alu_issue_data[1].rob_idx;
        end
    end

    assign alu1_fu_valid   = alu1_fu_valid_q;
    assign alu1_fu_phys_rd = alu1_fu_phys_rd_q;
    assign alu1_fu_rd_data = alu1_fu_rd_data_q;
    assign alu1_fu_rob_idx = alu1_fu_rob_idx_q;

    // ========================================================================
    // MUL Functional Unit (pipelined, 4 stages)
    // ========================================================================
    logic [31:0] mul_rs1_val, mul_rs2_val;
    // Operand isolation: gate MUL inputs to zero when no issue (power).
    assign mul_rs1_val = mul_issue_valid[0]
                       ? ((mul_issue_data[0].alu_m1_sel == pc_out) ? mul_issue_data[0].pc : mul_prf_rs1_data)
                       : '0;
    assign mul_rs2_val = mul_issue_valid[0] ? mul_prf_rs2_data : '0;

    mul_fu #(.NUM_STAGES(4)) mul_fu_i (
        .clk        (clk),
        .rst        (rst),
        .start_valid(mul_issue_valid[0]),
        .funct3     (mul_issue_data[0].funct3),
        .rs1_data   (mul_rs1_val),
        .rs2_data   (mul_rs2_val),
        .rob_idx    (mul_issue_data[0].rob_idx),
        .rd_phys    (mul_issue_data[0].rd_phys),
        .mul_valid  (mul_fu_valid),
        .mul_rd_data(mul_fu_rd_data),
        .mul_rob_idx(mul_fu_rob_idx),
        .mul_phys_rd(mul_fu_phys_rd),
        .mul_stall  (cdb_mul_stall),
        .fu_ready   (mul_fu_ready),
        .flush      (flush)
    );

    // ========================================================================
    // DIV Functional Unit (multi-cycle state machine)
    // ========================================================================
    logic [31:0] div_rs1_val, div_rs2_val;
    // Operand isolation: gate DIV inputs to zero when no issue (power).
    assign div_rs1_val = div_issue_valid[0] ? div_prf_rs1_data : '0;
    assign div_rs2_val = div_issue_valid[0] ? div_prf_rs2_data : '0;

    div_fu #(.DIV_LATENCY(32)) div_fu_i (
        .clk        (clk),
        .rst        (rst),
        .start_valid(div_issue_valid[0]),
        .funct3     (div_issue_data[0].funct3),
        .rs1_data   (div_rs1_val),
        .rs2_data   (div_rs2_val),
        .rob_idx    (div_issue_data[0].rob_idx),
        .rd_phys    (div_issue_data[0].rd_phys),
        .div_valid  (div_fu_valid),
        .div_rd_data(div_fu_rd_data),
        .div_rob_idx(div_fu_rob_idx),
        .div_phys_rd(div_fu_phys_rd),
        .div_stall  (cdb_div_stall),
        .fu_ready   (div_fu_ready),
        .flush      (flush)
    );

    // ========================================================================
    // CDB Arbiter (2-slot, priority: load > ALU0 > ALU1 > MUL > DIV)
    // ========================================================================
    cdb cdb_i (
        .load_valid (cdb_load_valid),
        .load_phys_rd(cdb_load_phys_rd),
        .load_rd_data(cdb_load_rd_data),
        .load_rob_idx(cdb_load_rob_idx),
        .alu_valid  (alu_fu_valid),
        .alu_phys_rd(alu_fu_phys_rd),
        .alu_rd_data(alu_fu_rd_data),
        .alu_rob_idx(alu_fu_rob_idx),
        .alu1_valid (alu1_fu_valid),
        .alu1_phys_rd(alu1_fu_phys_rd),
        .alu1_rd_data(alu1_fu_rd_data),
        .alu1_rob_idx(alu1_fu_rob_idx),
        .mul_valid  (mul_fu_valid),
        .mul_phys_rd(mul_fu_phys_rd),
        .mul_rd_data(mul_fu_rd_data),
        .mul_rob_idx(mul_fu_rob_idx),
        .div_valid  (div_fu_valid),
        .div_phys_rd(div_fu_phys_rd),
        .div_rd_data(div_fu_rd_data),
        .div_rob_idx(div_fu_rob_idx),
        .cdb_out    (cdb_out),
        .mul_stall  (cdb_mul_stall),
        .div_stall  (cdb_div_stall)
    );

    // ========================================================================
    // Commit (retire from ROB head, update RRF, drive RVFI)
    // ========================================================================
    // RVFI channel 0
    logic           rvfi_valid;
    logic [63:0]    rvfi_order;
    logic [31:0]    rvfi_inst;
    logic [4:0]     rvfi_rs1_addr;
    logic [4:0]     rvfi_rs2_addr;
    logic [31:0]    rvfi_rs1_rdata;
    logic [31:0]    rvfi_rs2_rdata;
    logic [4:0]     rvfi_rd_addr;
    logic [31:0]    rvfi_rd_wdata;
    logic [31:0]    rvfi_pc_rdata;
    logic [31:0]    rvfi_pc_wdata;
    logic [31:0]    rvfi_mem_addr;
    logic [3:0]     rvfi_mem_rmask;
    logic [3:0]     rvfi_mem_wmask;
    logic [31:0]    rvfi_mem_rdata;
    logic [31:0]    rvfi_mem_wdata;

    // RVFI channel 1
    logic           rvfi_valid_1;
    logic [63:0]    rvfi_order_1;
    logic [31:0]    rvfi_inst_1;
    logic [4:0]     rvfi_rs1_addr_1;
    logic [4:0]     rvfi_rs2_addr_1;
    logic [31:0]    rvfi_rs1_rdata_1;
    logic [31:0]    rvfi_rs2_rdata_1;
    logic [4:0]     rvfi_rd_addr_1;
    logic [31:0]    rvfi_rd_wdata_1;
    logic [31:0]    rvfi_pc_rdata_1;
    logic [31:0]    rvfi_pc_wdata_1;
    logic [31:0]    rvfi_mem_addr_1;
    logic [3:0]     rvfi_mem_rmask_1;
    logic [3:0]     rvfi_mem_wmask_1;
    logic [31:0]    rvfi_mem_rdata_1;
    logic [31:0]    rvfi_mem_wdata_1;

    commit commit_i (
        .clk                (clk),
        .rst                (rst),
        // ROB slot 0
        .rob_commit_ready   (rob_commit_ready),
        .rob_commit_pc      (rob_commit_pc),
        .rob_commit_inst    (rob_commit_inst),
        .rob_commit_rd_arch (rob_commit_rd_arch),
        .rob_commit_rd_phys (rob_commit_rd_phys),
        .rob_commit_rd_old_phys(rob_commit_rd_old_phys),
        .rob_commit_rd_wdata(rob_commit_rd_wdata),
        .rob_commit_rs1_addr(rob_commit_rs1_addr),
        .rob_commit_rs2_addr(rob_commit_rs2_addr),
        .rob_commit_idx     (rob_commit_idx),
        .rob_commit_is_load (rob_commit_is_load),
        .rob_commit_is_store(rob_commit_is_store),
        .rob_commit_mem_addr(rob_commit_mem_addr),
        .rob_commit_mem_wdata(rob_commit_mem_wdata),
        .rob_commit_mem_wmask(rob_commit_mem_wmask),
        .rob_commit_mem_rdata(rob_commit_mem_rdata),
        // ROB slot 1
        .rob_commit_1_ready      (rob_commit_1_ready),
        .rob_commit_1_pc         (rob_commit_1_pc),
        .rob_commit_1_inst       (rob_commit_1_inst),
        .rob_commit_1_rd_arch    (rob_commit_1_rd_arch),
        .rob_commit_1_rd_phys    (rob_commit_1_rd_phys),
        .rob_commit_1_rd_old_phys(rob_commit_1_rd_old_phys),
        .rob_commit_1_rd_wdata   (rob_commit_1_rd_wdata),
        .rob_commit_1_rs1_addr   (rob_commit_1_rs1_addr),
        .rob_commit_1_rs2_addr   (rob_commit_1_rs2_addr),
        .rob_commit_1_is_load    (rob_commit_1_is_load),
        .rob_commit_1_is_store   (rob_commit_1_is_store),
        .rob_commit_1_is_control (rob_commit_1_is_control),
        .rob_commit_1_load_done  (rob_commit_1_load_done),
        .rob_commit_1_mem_addr   (rob_commit_1_mem_addr),
        .rob_commit_1_mem_rdata  (rob_commit_1_mem_rdata),
        // Commit enable
        .rob_commit_en      (rob_commit_en),
        // Step C: load_done from ROB; same-cycle CDB bypass from load_issuer
        .rob_commit_load_done(rob_commit_load_done),
        .load_cdb_in_valid   (cdb_load_valid),
        .load_cdb_in_rob_idx (cdb_load_rob_idx),
        .load_cdb_in_rd_data (cdb_load_rd_data),
        .load_cdb_in_raw_rdata (load_raw_rdata),
        // Load CDB (Step C: tied to 0 inside commit; load_issuer drives the
        // real load slot in the CDB)
        .load_cdb_valid     (),
        .load_cdb_phys_rd   (),
        .load_cdb_rd_data   (),
        .load_cdb_rob_idx   (),
        // D-cache (Step D: removed -- stores go to store buffer)
        .sb_push            (sb_push),
        .sb_push_addr       (sb_push_addr),
        .sb_push_wmask      (sb_push_wmask),
        .sb_push_wdata      (sb_push_wdata),
        .sb_full            (sb_full),
        // Free list (2-wide)
        .fl_free_en         (fl_free_en),
        .fl_free_phys       (fl_free_phys),
        // PRF RVFI reads — slot 0
        .rvfi_prf_0_rs1_idx (commit_prf_0_rs1_idx),
        .rvfi_prf_0_rs2_idx (commit_prf_0_rs2_idx),
        .rvfi_prf_0_rs1_data(commit_prf_0_rs1_data),
        .rvfi_prf_0_rs2_data(commit_prf_0_rs2_data),
        // PRF RVFI reads — slot 1
        .rvfi_prf_1_rs1_idx (commit_prf_1_rs1_idx),
        .rvfi_prf_1_rs2_idx (commit_prf_1_rs2_idx),
        .rvfi_prf_1_rs1_data(commit_prf_1_rs1_data),
        .rvfi_prf_1_rs2_data(commit_prf_1_rs2_data),
        // RRF
        .rrf_table          (rrf_table),
        // RVFI channel 0
        .rvfi_valid         (rvfi_valid),
        .rvfi_order         (rvfi_order),
        .rvfi_inst          (rvfi_inst),
        .rvfi_rs1_addr      (rvfi_rs1_addr),
        .rvfi_rs2_addr      (rvfi_rs2_addr),
        .rvfi_rs1_rdata     (rvfi_rs1_rdata),
        .rvfi_rs2_rdata     (rvfi_rs2_rdata),
        .rvfi_rd_addr       (rvfi_rd_addr),
        .rvfi_rd_wdata      (rvfi_rd_wdata),
        .rvfi_pc_rdata      (rvfi_pc_rdata),
        .rvfi_pc_wdata      (rvfi_pc_wdata),
        .rvfi_mem_addr      (rvfi_mem_addr),
        .rvfi_mem_rmask     (rvfi_mem_rmask),
        .rvfi_mem_wmask     (rvfi_mem_wmask),
        .rvfi_mem_rdata     (rvfi_mem_rdata),
        .rvfi_mem_wdata     (rvfi_mem_wdata),
        // RVFI channel 1
        .rvfi_valid_1       (rvfi_valid_1),
        .rvfi_order_1       (rvfi_order_1),
        .rvfi_inst_1        (rvfi_inst_1),
        .rvfi_rs1_addr_1    (rvfi_rs1_addr_1),
        .rvfi_rs2_addr_1    (rvfi_rs2_addr_1),
        .rvfi_rs1_rdata_1   (rvfi_rs1_rdata_1),
        .rvfi_rs2_rdata_1   (rvfi_rs2_rdata_1),
        .rvfi_rd_addr_1     (rvfi_rd_addr_1),
        .rvfi_rd_wdata_1    (rvfi_rd_wdata_1),
        .rvfi_pc_rdata_1    (rvfi_pc_rdata_1),
        .rvfi_pc_wdata_1    (rvfi_pc_wdata_1),
        .rvfi_mem_addr_1    (rvfi_mem_addr_1),
        .rvfi_mem_rmask_1   (rvfi_mem_rmask_1),
        .rvfi_mem_wmask_1   (rvfi_mem_wmask_1),
        .rvfi_mem_rdata_1   (rvfi_mem_rdata_1),
        .rvfi_mem_wdata_1   (rvfi_mem_wdata_1),
        // Control
        .rob_head_is_control(rob_head_is_control),
        .rob_head_mispredict(rob_head_mispredict),
        .rob_head_correct_target(rob_head_correct_target)
    );

    // ========================================================================
    // Performance Counters (simulation-only instrumentation)
    // ========================================================================
    perf_counters perf_i (
        .clk                (clk),
        .rst                (rst),
        .rs_alu_full        (rs_alu_full),
        .rs_alu_full_ge2    (rs_alu_full_ge2),
        .rs_mul_full        (rs_mul_full),
        .rs_mul_full_ge2    (rs_mul_full_ge2),
        .rs_div_full        (rs_div_full),
        .rs_div_full_ge2    (rs_div_full_ge2),
        .rs_mem_full        (rs_mem_full),
        .rs_mem_full_ge2    (rs_mem_full_ge2),
        .rob_full           (rob_full),
        .rob_full_ge2       (rob_full_ge2),
        .fl_empty           (fl_empty),
        .fl_empty_ge2       (fl_empty_ge2),
        .iq_full            (perf_iq_full),
        .iq_full_ge2        (perf_iq_full_ge2),
        .dispatch_ready     (dispatch_ready),
        .inst_valid         (inst_valid),
        .rs_alu_insert_en   (rs_alu_insert_en),
        .rs_mul_insert_en   (rs_mul_insert_en),
        .rs_div_insert_en   (rs_div_insert_en),
        .rs_mem_insert_en   (rs_mem_insert_en),
        .rob_commit_en      (rob_commit_en),
        .cdb_mul_stall      (cdb_mul_stall),
        .cdb_div_stall      (cdb_div_stall),
        .flush              (flush),
        .icache_miss        (icache_dfp_read),
        .dcache_miss        (dcache_dfp_read || dcache_dfp_write),
        // Branch resolution counters: drive from commit so each branch is
        // counted exactly once and compared against the actual prediction
        // (pred_target stored in ROB), not the static PC+4 fallthrough.
        .ctrl_resolve_valid     (rob_commit_en[0] && rob_head_is_control),
        .ctrl_resolve_mispredict(rob_commit_en[0] && rob_head_is_control && rob_head_mispredict),
        .rob_commit_ready   (rob_commit_ready),
        .rob_commit_is_load (rob_commit_is_load),
        .rob_commit_is_store(rob_commit_is_store),
        .rob_commit_op      (rob_commit_inst[6:0]),
        .fetch_linebuf_miss (fetch_perf_linebuf_miss),
        .alu_rs_ready_ge3   (alu_rs_perf_ready_ge3),
        .rob_head_valid     (rob_head_valid),
        .iq_empty           (perf_iq_empty),
        .rob_head_is_control     (rob_head_is_control),
        .rob_commit_1_ready      (rob_commit_1_ready),
        .rob_commit_1_is_load    (rob_commit_1_is_load),
        .rob_commit_1_is_store   (rob_commit_1_is_store),
        .rob_commit_1_is_control (rob_commit_1_is_control),
        .fetch_valid        (inst_valid),
        .perf_dummy_out     ()
    );

endmodule : cpu