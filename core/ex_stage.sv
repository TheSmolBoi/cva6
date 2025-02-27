
// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 19.04.2017
// Description: Instantiation of all functional units residing in the execute stage


module ex_stage
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter int unsigned ASID_WIDTH = 1,
    parameter int unsigned VMID_WIDTH = 1
) (
    // Subsystem Clock - SUBSYSTEM
    input  logic                                                        clk_i,
    // Asynchronous reset active low - SUBSYSTEM
    input  logic                                                        rst_ni,
    // Fetch flush request - CONTROLLER
    input  logic                                                        flush_i,
    // Debug mode is enabled - CSR_REGFILE
    input  logic                                                        debug_mode_i,
    // rs1 forwarding - ISSUE_STAGE
    input  logic                   [  riscv::VLEN-1:0]                  rs1_forwarding_i,
    // rs2 forwarding - ISSUE_STAGE
    input  logic                   [  riscv::VLEN-1:0]                  rs2_forwarding_i,
    // FU data useful to execute instruction - ISSUE_STAGE
    input  fu_data_t                                                    fu_data_i,
    // PC of the current instruction - ISSUE_STAGE
    input  logic                   [  riscv::VLEN-1:0]                  pc_i,
    // Report whether isntruction is compressed - ISSUE_STAGE
    input  logic                                                        is_compressed_instr_i,
    // TO_BE_COMPLETED - ISSUE_STAGE
    input  riscv::xlen_t                                                tinst_i,
    // Fixed Latency Unit result - ISSUE_STAGE
    output riscv::xlen_t                                                flu_result_o,
    // ID of the scoreboard entry at which a=to write back - ISSUE_STAGE
    output logic                   [TRANS_ID_BITS-1:0]                  flu_trans_id_o,
    // Fixed Latency Unit exception - ISSUE_STAGE
    output exception_t                                                  flu_exception_o,
    // FLU is ready - ISSUE_STAGE
    output logic                                                        flu_ready_o,
    // FLU result is valid - ISSUE_STAGE
    output logic                                                        flu_valid_o,
    // ALU instruction is valid - ISSUE_STAGE
    input  logic                                                        alu_valid_i,
    // Branch unit instruction is valid - ISSUE_STAGE
    input  logic                                                        branch_valid_i,
    // Information of branch prediction - ISSUE_STAGE
    input  branchpredict_sbe_t                                          branch_predict_i,
    // The branch engine uses the write back from the ALU - several_modules
    output bp_resolve_t                                                 resolved_branch_o,
    // Signaling that we resolved the branch - ISSUE_STAGE
    output logic                                                        resolve_branch_o,
    // CSR instruction is valid - ISSUE_STAGE
    input  logic                                                        csr_valid_i,
    // CSR address to write - COMMIT_STAGE
    output logic                   [             11:0]                  csr_addr_o,
    // CSR commit - COMMIT_STAGE
    input  logic                                                        csr_commit_i,
    // CMO
    output logic                                   cmo_ready_o,        // FU is ready
    input  logic                                   cmo_valid_i,        // Input is valid
    output logic [TRANS_ID_BITS-1:0]               cmo_trans_id_o,
    output logic [riscv::XLEN-1:0]                 cmo_result_o,
    output logic                                   cmo_valid_o,
    output exception_t                             cmo_exception_o,
    // interface to caches for CMOs
    output cmo_req_t                               cmo_dc_req_o,          // CMO request to D$
    input  cmo_resp_t                              cmo_dc_resp_i,         // CMO response from D$
    output cmo_req_t                               cmo_ic_req_o,          // CMO request to I$
    input  cmo_resp_t                              cmo_ic_resp_i,         // CMO response from I$
    // MULT instruction is valid - ISSUE_STAGE
    input  logic                                                        mult_valid_i,
    // LSU is ready - ISSUE_STAGE
    output logic                                                        lsu_ready_o,
    // LSU instruction is valid - ISSUE_STAGE
    input  logic                                                        lsu_valid_i,
    // Load result is valid - ISSUE_STAGE
    output logic                                                        load_valid_o,
    // Load result valid - ISSUE_STAGE
    output riscv::xlen_t                                                load_result_o,
    // Load instruction ID - ISSUE_STAGE
    output logic                   [TRANS_ID_BITS-1:0]                  load_trans_id_o,
    // Exception generated by load instruction - ISSUE_STAGE
    output exception_t                                                  load_exception_o,
    // Store result is valid - ISSUe_STAGE
    output logic                                                        store_valid_o,
    // Store result - ISSUE_STAGE
    output riscv::xlen_t                                                store_result_o,
    // Store instruction ID - ISSUE_STAGE
    output logic                   [TRANS_ID_BITS-1:0]                  store_trans_id_o,
    // Exception generated by store instruction - ISSUE_STAGE
    output exception_t                                                  store_exception_o,
    // LSU commit - COMMIT_STAGE
    input  logic                                                        lsu_commit_i,
    // Commit queue ready to accept another commit request - COMMIT_STAGE
    output logic                                                        lsu_commit_ready_o,
    // Commit transaction ID - COMMIT_STAGE
    input  logic                   [TRANS_ID_BITS-1:0]                  commit_tran_id_i,
    // TO_BE_COMPLETED - ACC_DISPATCHER
    input  logic                                                        stall_st_pending_i,
    // TO_BE_COMPLETED - COMMIT_STAGE
    output logic                                                        no_st_pending_o,
    // Atomic result is valid - COMMIT_STAGE
    input  logic                                                        amo_valid_commit_i,
    // FU is ready - ISSUE_STAGE
    output logic                                                        fpu_ready_o,
    // FPU instruction is ready - ISSUE_STAGE
    input  logic                                                        fpu_valid_i,
    // FPU format - ISSUE_STAGE
    input  logic                   [              1:0]                  fpu_fmt_i,
    // FPU rm - ISSUE_STAGE
    input  logic                   [              2:0]                  fpu_rm_i,
    // FPU frm - ISSUE_STAGE
    input  logic                   [              2:0]                  fpu_frm_i,
    // FPU precision control - CSR_REGFILE
    input  logic                   [              6:0]                  fpu_prec_i,
    // FPU transaction ID - ISSUE_STAGE
    output logic                   [TRANS_ID_BITS-1:0]                  fpu_trans_id_o,
    // FPU result - ISSUE_STAGE
    output riscv::xlen_t                                                fpu_result_o,
    // FPU valid - ISSUE_STAGE
    output logic                                                        fpu_valid_o,
    // FPU exception - ISSUE_STAGE
    output exception_t                                                  fpu_exception_o,
    // CVXIF instruction is valid - ISSUE_STAGE
    input  logic                                                        x_valid_i,
    // CVXIF is ready - ISSUE_STAGE
    output logic                                                        x_ready_o,
    // undecoded instruction - ISSUE_STAGE
    input  logic                   [             31:0]                  x_off_instr_i,
    // CVXIF transaction ID - ISSUE_STAGE
    output logic                   [TRANS_ID_BITS-1:0]                  x_trans_id_o,
    // CVXIF exception - ISSUE_STAGE
    output exception_t                                                  x_exception_o,
    // CVXIF result - ISSUE_STAGE
    output riscv::xlen_t                                                x_result_o,
    // CVXIF result valid - ISSUE_STAGE
    output logic                                                        x_valid_o,
    // CVXIF write enable - ISSUE_STAGE
    output logic                                                        x_we_o,
    // CVXIF request - SUBSYSTEM
    output cvxif_pkg::cvxif_req_t                                       cvxif_req_o,
    // CVXIF response - SUBSYSTEM
    input  cvxif_pkg::cvxif_resp_t                                      cvxif_resp_i,
    // accelerate port result is valid - ACC_DISPATCHER
    input  logic                                                        acc_valid_i,
    // Enable virtual memory translation - CSR_REGFILE
    input  logic                                                        enable_translation_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input  logic                                                        enable_g_translation_i,
    // Enable virtual memory translation for load/stores - CSR_REGFILE
    input  logic                                                        en_ld_st_translation_i,
    // TO_BE_COMPLETED - CONTROLLER
    input  logic                                                        en_ld_st_g_translation_i,
    // Flush TLB - CONTROLLER
    input  logic                                                        flush_tlb_i,
    // TO_BE_COMPLETED - CONTROLLER
    input  logic                                                        flush_tlb_vvma_i,
    // TO_BE_COMPLETED - CONTROLLER
    input  logic                                                        flush_tlb_gvma_i,
    // Privilege mode - CSR_REGFILE
    input  riscv::priv_lvl_t                                            priv_lvl_i,
    // Virtualization mode - CSR_REGFILE
    input  logic                                                        v_i,
    // Privilege level at which load and stores should happen - CSR_REGFILE
    input  riscv::priv_lvl_t                                            ld_st_priv_lvl_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input  logic                                                        ld_st_v_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    output logic                                                        csr_hs_ld_st_inst_o,
    // Supervisor user memory - CSR_REGFILE
    input  logic                                                        sum_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input  logic                                                        vs_sum_i,
    // Make executable readable - CSR_REGFILE
    input  logic                                                        mxr_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input  logic                                                        vmxr_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input  logic                   [  riscv::PPNW-1:0]                  satp_ppn_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input  logic                   [   ASID_WIDTH-1:0]                  asid_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input  logic                   [  riscv::PPNW-1:0]                  vsatp_ppn_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input  logic                   [   ASID_WIDTH-1:0]                  vs_asid_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input  logic                   [  riscv::PPNW-1:0]                  hgatp_ppn_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input  logic                   [   VMID_WIDTH-1:0]                  vmid_i,
    // icache translation response - CACHE
    input  icache_arsp_t                                                icache_areq_i,
    // icache translation request - CACHE
    output icache_areq_t                                                icache_areq_o,
    // Data cache request ouput - CACHE
    input  dcache_req_o_t          [              2:0]                  dcache_req_ports_i,
    // Data cache request input - CACHE
    output dcache_req_i_t          [              2:0]                  dcache_req_ports_o,
    // Write buffer is empty - CACHE
    input  logic                                                        dcache_wbuffer_empty_i,
    // TO_BE_COMPLETED - CACHE
    input  logic                                                        dcache_wbuffer_not_ni_i,
    // AMO request - CACHE
    output amo_req_t                                                    amo_req_o,
    // AMO response - CACHE
    input  amo_resp_t                                                   amo_resp_i,
    // To count the instruction TLB misses - PERF_COUNTERS
    output logic                                                        itlb_miss_o,
    // To count the data TLB misses - PERF_COUNTERS
    output logic                                                        dtlb_miss_o,
    // Report the PMP configuration - CSR_REGFILE
    input  riscv::pmpcfg_t         [             15:0]                  pmpcfg_i,
    // Report the PMP addresses - CSR_REGFILE
    input  logic                   [             15:0][riscv::PLEN-3:0] pmpaddr_i,
    // Information dedicated to RVFI - RVFI
    output lsu_ctrl_t                                                   rvfi_lsu_ctrl_o,
    // Information dedicated to RVFI - RVFI
    output                         [  riscv::PLEN-1:0]                  rvfi_mem_paddr_o
);

  // -------------------------
  // Fixed Latency Units
  // -------------------------
  // all fixed latency units share a single issue port and a sing write
  // port into the scoreboard. At the moment those are:
  // 1. ALU - all operations are single cycle
  // 2. Branch unit: operation is single cycle, the ALU is needed
  //    for comparison
  // 3. CSR: This is a small buffer which saves the address of the CSR.
  //    The value is then re-fetched once the instruction retires. The buffer
  //    is only a single entry deep, hence this operation will block all
  //    other operations once this buffer is full. This should not be a major
  //    concern though as CSRs are infrequent.
  // 4. Multiplier/Divider: The multiplier has a fixed latency of 1 cycle.
  //                        The issue logic will take care of not issuing
  //                        another instruction if it will collide on the
  //                        output port. Divisions are arbitrary in length
  //                        they will simply block the issue of all other
  //                        instructions.


  logic current_instruction_is_sfence_vma;
  logic current_instruction_is_hfence_vvma;
  logic current_instruction_is_hfence_gvma;
  // These two register store the rs1 and rs2 parameters in case of `SFENCE_VMA`
  // instruction to be used for TLB flush in the next clock cycle.
  logic [VMID_WIDTH-1:0] vmid_to_be_flushed;
  logic [ASID_WIDTH-1:0] asid_to_be_flushed;
  logic [riscv::VLEN-1:0] vaddr_to_be_flushed;
  logic [riscv::GPLEN-1:0] gpaddr_to_be_flushed;

  // from ALU to branch unit
  logic alu_branch_res;  // branch comparison result
  riscv::xlen_t alu_result, csr_result, mult_result;
  logic [riscv::VLEN-1:0] branch_result;
  logic csr_ready, mult_ready;
  logic [TRANS_ID_BITS-1:0] mult_trans_id;
  logic mult_valid;

  // 1. ALU (combinatorial)
  // data silence operation
  fu_data_t alu_data;
  assign alu_data = (alu_valid_i | branch_valid_i) ? fu_data_i : '0;

  alu #(
      .CVA6Cfg(CVA6Cfg)
  ) alu_i (
      .clk_i,
      .rst_ni,
      .fu_data_i       (alu_data),
      .result_o        (alu_result),
      .alu_branch_res_o(alu_branch_res)
  );

  // 2. Branch Unit (combinatorial)
  // we don't silence the branch unit as this is already critical and we do
  // not want to add another layer of logic
  branch_unit #(
      .CVA6Cfg(CVA6Cfg)
  ) branch_unit_i (
      .clk_i,
      .rst_ni,
      .v_i,
      .debug_mode_i,
      .fu_data_i,
      .pc_i,
      .is_compressed_instr_i,
      // any functional unit is valid, check that there is no accidental mis-predict
      .fu_valid_i ( alu_valid_i || lsu_valid_i || csr_valid_i || mult_valid_i || fpu_valid_i || acc_valid_i ) ,
      .branch_valid_i,
      .branch_comp_res_i(alu_branch_res),
      .branch_result_o(branch_result),
      .branch_predict_i,
      .resolved_branch_o,
      .resolve_branch_o,
      .branch_exception_o(flu_exception_o)
  );

  // 3. CSR (sequential)
  csr_buffer #(
      .CVA6Cfg(CVA6Cfg)
  ) csr_buffer_i (
      .clk_i,
      .rst_ni,
      .flush_i,
      .fu_data_i,
      .csr_valid_i,
      .csr_ready_o (csr_ready),
      .csr_result_o(csr_result),
      .csr_commit_i,
      .csr_addr_o
  );

  assign flu_valid_o = alu_valid_i | branch_valid_i | csr_valid_i | mult_valid;

  // result MUX
  always_comb begin
    // Branch result as default case
    flu_result_o   = {{riscv::XLEN - riscv::VLEN{1'b0}}, branch_result};
    flu_trans_id_o = fu_data_i.trans_id;
    // ALU result
    if (alu_valid_i) begin
      flu_result_o = alu_result;
      // CSR result
    end else if (csr_valid_i) begin
      flu_result_o = csr_result;
    end else if (mult_valid) begin
      flu_result_o   = mult_result;
      flu_trans_id_o = mult_trans_id;
    end
  end

  // ready flags for FLU
  always_comb begin
    flu_ready_o = csr_ready & mult_ready;
  end

  // 4. Multiplication (Sequential)
  fu_data_t mult_data;
  // input silencing of multiplier
  assign mult_data = mult_valid_i ? fu_data_i : '0;

  mult #(
      .CVA6Cfg(CVA6Cfg)
  ) i_mult (
      .clk_i,
      .rst_ni,
      .flush_i,
      .mult_valid_i,
      .fu_data_i      (mult_data),
      .result_o       (mult_result),
      .mult_valid_o   (mult_valid),
      .mult_ready_o   (mult_ready),
      .mult_trans_id_o(mult_trans_id)
  );

  // ----------------
  // FPU
  // ----------------
  generate
    if (CVA6Cfg.FpPresent) begin : fpu_gen
      fu_data_t fpu_data;
      assign fpu_data = fpu_valid_i ? fu_data_i : '0;

      fpu_wrap #(
          .CVA6Cfg(CVA6Cfg)
      ) fpu_i (
          .clk_i,
          .rst_ni,
          .flush_i,
          .fpu_valid_i,
          .fpu_ready_o,
          .fu_data_i(fpu_data),
          .fpu_fmt_i,
          .fpu_rm_i,
          .fpu_frm_i,
          .fpu_prec_i,
          .fpu_trans_id_o,
          .result_o (fpu_result_o),
          .fpu_valid_o,
          .fpu_exception_o
      );
    end else begin : no_fpu_gen
      assign fpu_ready_o     = '0;
      assign fpu_trans_id_o  = '0;
      assign fpu_result_o    = '0;
      assign fpu_valid_o     = '0;
      assign fpu_exception_o = '0;
    end
  endgenerate

  // ----------------
  // Load-Store Unit
  // ----------------
  fu_data_t lsu_data;

  assign lsu_data = lsu_valid_i ? fu_data_i : '0;

  load_store_unit #(
      .CVA6Cfg   (CVA6Cfg),
      .ASID_WIDTH(ASID_WIDTH),
      .VMID_WIDTH(VMID_WIDTH)
  ) lsu_i (
      .clk_i,
      .rst_ni,
      .flush_i,
      .stall_st_pending_i,
      .no_st_pending_o,
      .fu_data_i             (lsu_data),
      .lsu_ready_o,
      .lsu_valid_i,
      .load_trans_id_o,
      .load_result_o,
      .load_valid_o,
      .load_exception_o,
      .store_trans_id_o,
      .store_result_o,
      .store_valid_o,
      .store_exception_o,
      .commit_i              (lsu_commit_i),
      .commit_ready_o        (lsu_commit_ready_o),
      .commit_tran_id_i,
      .enable_translation_i,
      .enable_g_translation_i,
      .en_ld_st_translation_i,
      .en_ld_st_g_translation_i,
      .icache_areq_i,
      .icache_areq_o,
      .priv_lvl_i,
      .v_i,
      .ld_st_priv_lvl_i,
      .ld_st_v_i,
      .csr_hs_ld_st_inst_o,
      .sum_i,
      .vs_sum_i,
      .mxr_i,
      .vmxr_i,
      .satp_ppn_i,
      .vsatp_ppn_i,
      .hgatp_ppn_i,
      .asid_i,
      .vs_asid_i,
      .asid_to_be_flushed_i  (asid_to_be_flushed),
      .vmid_i,
      .vmid_to_be_flushed_i  (vmid_to_be_flushed),
      .vaddr_to_be_flushed_i (vaddr_to_be_flushed),
      .gpaddr_to_be_flushed_i(gpaddr_to_be_flushed),
      .flush_tlb_i,
      .flush_tlb_vvma_i,
      .flush_tlb_gvma_i,
      .itlb_miss_o,
      .dtlb_miss_o,
      .dcache_req_ports_i,
      .dcache_req_ports_o,
      .dcache_wbuffer_empty_i,
      .dcache_wbuffer_not_ni_i,
      .amo_valid_commit_i,
      .amo_req_o,
      .amo_resp_i,
      .tinst_i,
      .pmpcfg_i,
      .pmpaddr_i,
      .rvfi_lsu_ctrl_o,
      .rvfi_mem_paddr_o
  );

  if (CVA6Cfg.CvxifEn) begin : gen_cvxif
    fu_data_t cvxif_data;
    assign cvxif_data = x_valid_i ? fu_data_i : '0;
    cvxif_fu #(
        .CVA6Cfg(CVA6Cfg)
    ) cvxif_fu_i (
        .clk_i,
        .rst_ni,
        .fu_data_i,
        .priv_lvl_i(ld_st_priv_lvl_i),
        .x_valid_i,
        .x_ready_o,
        .x_off_instr_i,
        .x_trans_id_o,
        .x_exception_o,
        .x_result_o,
        .x_valid_o,
        .x_we_o,
        .cvxif_req_o,
        .cvxif_resp_i
    );
  end else begin : gen_no_cvxif
    assign cvxif_req_o   = '0;
    assign x_trans_id_o  = '0;
    assign x_exception_o = '0;
    assign x_result_o    = '0;
    assign x_valid_o     = '0;
  end
  
  if (CVA6Cfg.CMOEn) begin : gen_cmo
    // FIXME
    // Relocate into the load_store_unit and perform address translation.
    // Currently, target addresses must be already physical
    fu_data_t cmo_data;
    assign cmo_data = cmo_valid_i ? fu_data_i : '0;
    cmo_fu cmo_fu_i (
        .clk_i,
        .rst_ni,
        .fu_data_i(cmo_data),
        .cmo_valid_i,
        .cmo_ready_o,
        .cmo_trans_id_o,
        .cmo_exception_o,
        .cmo_result_o,
        .cmo_valid_o,
        .cmo_ic_req_o,
        .cmo_ic_resp_i,
        .cmo_dc_req_o,
        .cmo_dc_resp_i
    );
  end else begin : gen_no_cmo
    assign cmo_trans_id_o  = '0,
           cmo_exception_o = '0,
           cmo_result_o    = '0,
           cmo_valid_o     = '0,
           cmo_ic_req_o    = '0,
           cmo_dc_req_o    = '0;
  end
        

  if (CVA6Cfg.RVS) begin
    if (CVA6Cfg.RVH) begin
      always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
          current_instruction_is_sfence_vma  <= 1'b0;
          current_instruction_is_hfence_vvma <= 1'b0;
          current_instruction_is_hfence_gvma <= 1'b0;
        end else begin
          if (flush_i) begin
            current_instruction_is_sfence_vma  <= 1'b0;
            current_instruction_is_hfence_vvma <= 1'b0;
            current_instruction_is_hfence_gvma <= 1'b0;
          end else if ((fu_data_i.operation == SFENCE_VMA && !v_i) && csr_valid_i) begin
            current_instruction_is_sfence_vma <= 1'b1;
          end else if (((fu_data_i.operation == SFENCE_VMA && v_i) || fu_data_i.operation == HFENCE_VVMA) && csr_valid_i) begin
            current_instruction_is_hfence_vvma <= 1'b1;
          end else if ((fu_data_i.operation == HFENCE_GVMA) && csr_valid_i) begin
            current_instruction_is_hfence_gvma <= 1'b1;
          end
        end
      end
    end else begin
      assign current_instruction_is_hfence_vvma = 1'b0;
      assign current_instruction_is_hfence_gvma = 1'b0;
      always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
          current_instruction_is_sfence_vma <= 1'b0;
        end else begin
          if (flush_i) begin
            current_instruction_is_sfence_vma <= 1'b0;
          end else if (fu_data_i.operation == SFENCE_VMA && csr_valid_i) begin
            current_instruction_is_sfence_vma <= 1'b1;
          end
        end
      end
    end
    if (CVA6Cfg.RVH) begin
      // This process stores the rs1 and rs2 parameters of a SFENCE_VMA instruction.
      always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
          vmid_to_be_flushed   <= '0;
          asid_to_be_flushed   <= '0;
          vaddr_to_be_flushed  <= '0;
          gpaddr_to_be_flushed <= '0;
          // if the current instruction in EX_STAGE is a sfence.vma, in the next cycle no writes will happen
        end else if ((~(current_instruction_is_sfence_vma || current_instruction_is_hfence_vvma || current_instruction_is_hfence_gvma)) && (~((fu_data_i.operation == SFENCE_VMA || fu_data_i.operation == HFENCE_VVMA || fu_data_i.operation == HFENCE_GVMA ) && csr_valid_i))) begin
          vaddr_to_be_flushed  <= rs1_forwarding_i;
          gpaddr_to_be_flushed <= rs1_forwarding_i >> 2;
          asid_to_be_flushed   <= rs2_forwarding_i[ASID_WIDTH-1:0];
          vmid_to_be_flushed   <= rs2_forwarding_i[VMID_WIDTH-1:0];
        end
      end
    end else begin
      assign vmid_to_be_flushed   = '0;
      assign gpaddr_to_be_flushed = '0;
      // This process stores the rs1 and rs2 parameters of a SFENCE_VMA instruction.
      always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
          asid_to_be_flushed  <= '0;
          vaddr_to_be_flushed <= '0;
          // if the current instruction in EX_STAGE is a sfence.vma, in the next cycle no writes will happen
        end else if ((~current_instruction_is_sfence_vma) && (~((fu_data_i.operation == SFENCE_VMA) && csr_valid_i))) begin
          vaddr_to_be_flushed <= rs1_forwarding_i;
          asid_to_be_flushed  <= rs2_forwarding_i[ASID_WIDTH-1:0];
        end
      end
    end
  end else begin
    assign current_instruction_is_sfence_vma  = 1'b0;
    assign current_instruction_is_hfence_vvma = 1'b0;
    assign current_instruction_is_hfence_gvma = 1'b0;
    assign asid_to_be_flushed                 = '0;
    assign vaddr_to_be_flushed                = '0;
    assign vmid_to_be_flushed                 = '0;
    assign gpaddr_to_be_flushed               = '0;
  end

endmodule
