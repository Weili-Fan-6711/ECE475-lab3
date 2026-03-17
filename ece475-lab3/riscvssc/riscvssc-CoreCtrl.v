//=========================================================================
// 7-Stage RISCV Control Unit
//=========================================================================

`ifndef RISCV_CORE_CTRL_V
`define RISCV_CORE_CTRL_V

`include "riscvssc-InstMsg.v"

module riscv_CoreCtrl
(
  input clk,
  input reset,

  // Instruction Memory Port
  output        imemreq0_val,
  input         imemreq0_rdy,
  input  [31:0] imemresp0_msg_data,
  input         imemresp0_val,

  // Instruction Memory Port
  output        imemreq1_val,
  input         imemreq1_rdy,
  input  [31:0] imemresp1_msg_data,
  input         imemresp1_val,

  // Data Memory Port

  output        dmemreq_msg_rw,
  output  [1:0] dmemreq_msg_len,
  output        dmemreq_val,
  input         dmemreq_rdy,
  input         dmemresp_val,

  // Controls Signals (ctrl->dpath)

  output  [1:0] pc_mux_sel_Phl,
  output        steering_mux_sel_Dhl,
  output  [3:0] opA0_byp_mux_sel_Dhl,
  output  [1:0] opA0_mux_sel_Dhl,
  output  [3:0] opA1_byp_mux_sel_Dhl,
  output  [2:0] opA1_mux_sel_Dhl,
  output  [3:0] opB0_byp_mux_sel_Dhl,
  output  [1:0] opB0_mux_sel_Dhl,
  output  [3:0] opB1_byp_mux_sel_Dhl,
  output  [2:0] opB1_mux_sel_Dhl,
  output [31:0] instA_Dhl,
  output [31:0] instB_Dhl,
  output  [3:0] aluA_fn_X0hl,
  output  [3:0] aluB_fn_X0hl,
  output  [2:0] muldivreq_msg_fn_Dhl,
  output        muldivreq_val,
  input         muldivreq_rdy,
  input         muldivresp_val,
  output        muldivresp_rdy,
  output        muldiv_stall_mult1,
  output reg [2:0] dmemresp_mux_sel_X1hl,
  output        dmemresp_queue_en_X1hl,
  output reg       dmemresp_queue_val_X1hl,
  output reg       muldiv_mux_sel_X3hl,
  output reg       execute_mux_sel_X3hl,
  output reg       memex_mux_sel_X1hl,
  output        rfA_wen_Whl,
  output  [4:0] rfA_waddr_Whl,
  output        rfB_wen_Whl,
  output  [4:0] rfB_waddr_Whl,
  output        stall_Fhl,
  output        stall_Dhl,
  output        stall_X0hl,
  output        stall_X1hl,
  output        stall_X2hl,
  output        stall_X3hl,
  output        stall_Whl,
  output        ir0_valid_issue,
  output        ir1_valid_issue,

  // Control Signals (dpath->ctrl)

  input         branch_cond_eq_X0hl,
  input         branch_cond_ne_X0hl,
  input         branch_cond_lt_X0hl,
  input         branch_cond_ltu_X0hl,
  input         branch_cond_ge_X0hl,
  input         branch_cond_geu_X0hl,
  input  [31:0] proc2csr_data_Whl,

  // CSR Status

  output reg [31:0] csr_status
);

  //----------------------------------------------------------------------
  // PC Stage: Instruction Memory Request
  //----------------------------------------------------------------------

  // PC Mux Select

  assign pc_mux_sel_Phl
    = (brj_taken_X0hl === 1'b1) ? pm_b
    : (brj_taken_Dhl  === 1'b1) ? pc_mux_sel_Dhl
    :                             pm_p;

  // Only send a valid imem request if not stalled

  // We always request two instructions if not in reset.
  wire   imemreq_val_Phl  = !reset;
  assign imemreq0_val     = imemreq_val_Phl;
  assign imemreq1_val     = imemreq_val_Phl;

  // Dummy Squash Signal

  wire squash_Phl = 1'b0;

  // Stall in PC if F is stalled

  wire stall_Phl = stall_Fhl;

  // Next bubble bit

  wire bubble_next_Phl = ( squash_Phl || stall_Phl );

  //----------------------------------------------------------------------
  // F <- P
  //----------------------------------------------------------------------

  reg imemreq_val_Fhl;

  reg bubble_Fhl;

  always @ ( posedge clk ) begin
    // Only pipeline the bubble bit if the next stage is not stalled
    if ( reset ) begin
      imemreq_val_Fhl <= 1'b0;

      bubble_Fhl <= 1'b0;
    end
    else if( !stall_Fhl ) begin 
      imemreq_val_Fhl <= imemreq_val_Phl;

      bubble_Fhl <= bubble_next_Phl;
    end
    else begin 
      imemreq_val_Fhl <= imemreq_val_Phl;
    end
  end

  //----------------------------------------------------------------------
  // Fetch Stage: Instruction Memory Response
  //----------------------------------------------------------------------

  // Is the current stage valid?

  wire inst_val_Fhl = ( !bubble_Fhl && !squash_Fhl );

  // We squash F and D if a jump resolves in D (PC selection)
  // or a branch/jump resolves in X0.
  // We use !bubble_Dhl instead of inst_val_Dhl to avoid a combinatorial feedback loop.
  wire squash_Fhl = ( inst_val_X0hl && brj_taken_X0hl )
                      || ( inst_val_Dhl && brj_taken_Dhl );

  wire squash_Dhl = squash_Fhl;

  // Stall in F if D is stalled, EXCEPT when squash fires (let the PC redirect happen).
  // Use strict binary comparison to prevent x-propagation.
  assign stall_Fhl = (stall_Dhl === 1'b1) && !squash_Fhl;

  // Next bubble bit

  wire bubble_sel_Fhl  = ( squash_Fhl || stall_Fhl );
  wire bubble_next_Fhl = ( !bubble_sel_Fhl ) ? bubble_Fhl
                       : ( bubble_sel_Fhl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // Queue for instruction memory response
  //----------------------------------------------------------------------

  wire imemresp0_queue_en_Fhl = ( stall_Dhl && imemresp0_val );
  wire imemresp0_queue_val_next_Fhl
    = stall_Dhl && ( imemresp0_val || imemresp0_queue_val_Fhl );

  wire imemresp1_queue_en_Fhl = ( stall_Dhl && imemresp1_val );
  wire imemresp1_queue_val_next_Fhl
    = stall_Dhl && ( imemresp1_val || imemresp1_queue_val_Fhl );

  reg [31:0] imemresp0_queue_reg_Fhl;
  reg        imemresp0_queue_val_Fhl;

  reg [31:0] imemresp1_queue_reg_Fhl;
  reg        imemresp1_queue_val_Fhl;

  always @ ( posedge clk ) begin
    if ( imemresp0_queue_en_Fhl ) begin
      imemresp0_queue_reg_Fhl <= imemresp0_msg_data;
    end
    if ( imemresp1_queue_en_Fhl ) begin
      imemresp1_queue_reg_Fhl <= imemresp1_msg_data;
    end
    imemresp0_queue_val_Fhl <= imemresp0_queue_val_next_Fhl;
    imemresp1_queue_val_Fhl <= imemresp1_queue_val_next_Fhl;
  end

  //----------------------------------------------------------------------
  // Instruction memory queue mux
  //----------------------------------------------------------------------

  wire [31:0] imemresp0_queue_mux_out_Fhl
    = ( !imemresp0_queue_val_Fhl ) ? imemresp0_msg_data
    : ( imemresp0_queue_val_Fhl )  ? imemresp0_queue_reg_Fhl
    :                               32'bx;

  wire [31:0] imemresp1_queue_mux_out_Fhl
    = ( !imemresp1_queue_val_Fhl ) ? imemresp1_msg_data
    : ( imemresp1_queue_val_Fhl )  ? imemresp1_queue_reg_Fhl
    :                               32'bx;

  //----------------------------------------------------------------------
  // D <- F
  //----------------------------------------------------------------------

  reg [31:0] ir0_Dhl;
  reg [31:0] ir1_Dhl;
  reg        bubble_Dhl;

  wire squash_first_D_inst =
    (inst_val_Dhl && !stall_0_Dhl && stall_1_Dhl);

  // In Part 1, Pipeline A executes both instructions. stall_0_Dhl dynamically evaluates 
  // whichever instruction is steered down Pipeline A (because rs10_addr etc are multiplexed).
  // stall_1_Dhl evaluates Pipeline B, which we explicitly disabled.
  wire active_stall_Dhl = stall_0_Dhl;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_Dhl <= 1'b1;
      ir0_Dhl    <= 32'b0;
      ir1_Dhl    <= 32'b0;
    end
    else if( !stall_Dhl ) begin
      ir0_Dhl    <= imemresp0_queue_mux_out_Fhl;
      ir1_Dhl    <= imemresp1_queue_mux_out_Fhl;
      bubble_Dhl <= bubble_next_Fhl;
    end
    // Partial issue: ir0 issued but ir1 stalled.
    // We freeze D (to hold ir1 for next cycle) and zero ir0 so it won't re-issue.
    else if( !stall_0_Dhl && stall_1_hazard_Dhl && cs1_valid ) begin
      ir0_Dhl <= 32'b0;  // NOP out ir0 — it already issued
      // If that ir0 was a branch/jump that squashed, also kill ir1
      if ( cs0_valid && (cs0[`RISCV_INST_MSG_J_EN] || squash_Dhl) ) begin
        ir1_Dhl    <= 32'b0;
        bubble_Dhl <= 1'b1;
      end
    end
    // Global squash (even if stalled)
    else if( squash_Dhl ) begin
      ir0_Dhl <= 32'b0;
      ir1_Dhl <= 32'b0;
      bubble_Dhl <= 1'b1;
    end
  end

  //----------------------------------------------------------------------
  // Decode Stage: Constants
  //----------------------------------------------------------------------

  // Generic Parameters

  localparam n = 1'd0;
  localparam y = 1'd1;

  // Register specifiers

  localparam rx = 5'bx;
  localparam r0 = 5'd0;

  // Branch Type

  localparam br_x    = 3'bx;
  localparam br_none = 3'd0;
  localparam br_beq  = 3'd1;
  localparam br_bne  = 3'd2;
  localparam br_blt  = 3'd3;
  localparam br_bltu = 3'd4;
  localparam br_bge  = 3'd5;
  localparam br_bgeu = 3'd6;

  // PC Mux Select

  localparam pm_x   = 2'bx;  // Don't care
  localparam pm_p   = 2'd0;  // Use pc+4
  localparam pm_b   = 2'd1;  // Use branch address
  localparam pm_j   = 2'd2;  // Use jump address
  localparam pm_r   = 2'd3;  // Use jump register

  // Operand 0 Bypass Mux Select

  localparam am_r0    = 4'd0; // Use rdata0
  localparam am_AX0_byp = 4'd1; // Bypass from X0
  localparam am_AX1_byp = 4'd2; // Bypass from X1
  localparam am_AX2_byp = 4'd3; // Bypass from X2
  localparam am_AX3_byp = 4'd4; // Bypass from X3
  localparam am_AW_byp = 4'd5; // Bypass from W
  localparam am_BX0_byp = 4'd6; // Bypass from X0
  localparam am_BX1_byp = 4'd7; // Bypass from X1
  localparam am_BX2_byp = 4'd8; // Bypass from X2
  localparam am_BX3_byp = 4'd9; // Bypass from X3
  localparam am_BW_byp = 4'd10; // Bypass from W

  // Operand 0 Mux Select

  localparam am_x     = 2'bx;
  localparam am_rdat  = 2'd0; // Use output of bypass mux for rs1
  localparam am_pc    = 2'd1; // Use current PC
  localparam am_pc4   = 2'd2; // Use PC + 4
  localparam am_0     = 2'd3; // Use constant 0

  // Operand 1 Bypass Mux Select

  localparam bm_r1    = 4'd0; // Use rdata1
  localparam bm_AX0_byp = 4'd1; // Bypass from X0
  localparam bm_AX1_byp = 4'd2; // Bypass from X1
  localparam bm_AX2_byp = 4'd3; // Bypass from X2
  localparam bm_AX3_byp = 4'd4; // Bypass from X3
  localparam bm_AW_byp = 4'd5; // Bypass from W
  localparam bm_BX0_byp = 4'd6; // Bypass from X0
  localparam bm_BX1_byp = 4'd7; // Bypass from X1
  localparam bm_BX2_byp = 4'd8; // Bypass from X2
  localparam bm_BX3_byp = 4'd9; // Bypass from X3
  localparam bm_BW_byp = 4'd10; // Bypass from W

  // Operand 1 Mux Select

  localparam bm_x      = 3'bx; // Don't care
  localparam bm_rdat   = 3'd0; // Use output of bypass mux for rs2
  localparam bm_shamt  = 3'd1; // Use shift amount
  localparam bm_imm_u  = 3'd2; // Use U-type immediate
  localparam bm_imm_sb = 3'd3; // Use SB-type immediate
  localparam bm_imm_i  = 3'd4; // Use I-type immediate
  localparam bm_imm_s  = 3'd5; // Use S-type immediate
  localparam bm_0      = 3'd6; // Use constant 0

  // ALU Function

  localparam alu_x    = 4'bx;
  localparam alu_add  = 4'd0;
  localparam alu_sub  = 4'd1;
  localparam alu_sll  = 4'd2;
  localparam alu_or   = 4'd3;
  localparam alu_lt   = 4'd4;
  localparam alu_ltu  = 4'd5;
  localparam alu_and  = 4'd6;
  localparam alu_xor  = 4'd7;
  localparam alu_nor  = 4'd8;
  localparam alu_srl  = 4'd9;
  localparam alu_sra  = 4'd10;

  // Muldiv Function

  localparam md_x    = 3'bx;
  localparam md_mul  = 3'd0;
  localparam md_div  = 3'd1;
  localparam md_divu = 3'd2;
  localparam md_rem  = 3'd3;
  localparam md_remu = 3'd4;

  // MulDiv Mux Select

  localparam mdm_x = 1'bx; // Don't Care
  localparam mdm_l = 1'd0; // Take lower half of 64-bit result, mul/div/divu
  localparam mdm_u = 1'd1; // Take upper half of 64-bit result, rem/remu

  // Execute Mux Select

  localparam em_x   = 1'bx; // Don't Care
  localparam em_alu = 1'd0; // Use ALU output
  localparam em_md  = 1'd1; // Use muldiv output

  // Memory Request Type

  localparam nr = 2'b0; // No request
  localparam ld = 2'd1; // Load
  localparam st = 2'd2; // Store

  // Subword Memop Length

  localparam ml_x  = 2'bx;
  localparam ml_w  = 2'd0;
  localparam ml_b  = 2'd1;
  localparam ml_h  = 2'd2;

  // Memory Response Mux Select

  localparam dmm_x  = 3'bx;
  localparam dmm_w  = 3'd0;
  localparam dmm_b  = 3'd1;
  localparam dmm_bu = 3'd2;
  localparam dmm_h  = 3'd3;
  localparam dmm_hu = 3'd4;

  // Writeback Mux 1

  localparam wm_x   = 1'bx; // Don't care
  localparam wm_alu = 1'd0; // Use ALU output
  localparam wm_mem = 1'd1; // Use data memory response

  //----------------------------------------------------------------------
  // Decode Stage: Logic
  //----------------------------------------------------------------------

  // Is the current stage valid?

  wire inst_val_Dhl = ( !bubble_Dhl && !squash_Dhl );

  // Parse instruction fields

  wire   [4:0] inst0_rs1_Dhl;
  wire   [4:0] inst0_rs2_Dhl;
  wire   [4:0] inst0_rd_Dhl;

  riscv_InstMsgFromBits inst0_msg_from_bits
  (
    .msg      (ir0_Dhl),
    .opcode   (),
    .rs1      (inst0_rs1_Dhl),
    .rs2      (inst0_rs2_Dhl),
    .rd       (inst0_rd_Dhl),
    .funct3   (),
    .funct7   (),
    .shamt    (),
    .imm_i    (),
    .imm_s    (),
    .imm_sb   (),
    .imm_u    (),
    .imm_uj   ()
  );

  wire   [4:0] inst1_rs1_Dhl;
  wire   [4:0] inst1_rs2_Dhl;
  wire   [4:0] inst1_rd_Dhl;

  riscv_InstMsgFromBits inst1_msg_from_bits
  (
    .msg      (ir1_Dhl),
    .opcode   (),
    .rs1      (inst1_rs1_Dhl),
    .rs2      (inst1_rs2_Dhl),
    .rd       (inst1_rd_Dhl),
    .funct3   (),
    .funct7   (),
    .shamt    (),
    .imm_i    (),
    .imm_s    (),
    .imm_sb   (),
    .imm_u    (),
    .imm_uj   ()
  );

  // Shorten register specifier name for table

  wire [4:0] rs10 = inst0_rs1_Dhl;
  wire [4:0] rs20 = inst0_rs2_Dhl;
  wire [4:0] rd0 = inst0_rd_Dhl;

  wire [4:0] rs11 = inst1_rs1_Dhl;
  wire [4:0] rs21 = inst1_rs2_Dhl;
  wire [4:0] rd1 = inst1_rd_Dhl;

  // Instruction Decode

  localparam cs_sz = 39;
  reg [cs_sz-1:0] cs0;
  reg [cs_sz-1:0] cs1;

  always @ (*) begin

    cs0 = {cs_sz{1'bx}}; // Default to invalid instruction

    casez ( ir0_Dhl )

      //                                j     br       pc      op0      rs1 op1       rs2 alu       md       md md     ex      mem  mem   memresp wb      rf      csr
      //                            val taken type     muxsel  muxsel   en  muxsel    en  fn        fn       en muxsel muxsel  rq   len   muxsel  muxsel  wen wa  wen
      `RISCV_INST_MSG_LUI     :cs0={ y,  n,    br_none, pm_p,   am_0,    n,  bm_imm_u, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_AUIPC   :cs0={ y,  n,    br_none, pm_p,   am_pc,   n,  bm_imm_u, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };

      `RISCV_INST_MSG_ADDI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_ORI     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLTI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLTIU   :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_XORI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_ANDI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLLI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SRLI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SRAI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };

      `RISCV_INST_MSG_ADD     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SUB     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLL     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLT     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLTU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_XOR     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SRL     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SRA     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_OR      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_AND     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };

      `RISCV_INST_MSG_LW      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_w, dmm_w,  wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_LB      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_b,  wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_LH      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_h,  wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_LBU     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_bu, wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_LHU     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_hu, wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_SW      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_w, dmm_w,  wm_mem, n,  rx, n   };
      `RISCV_INST_MSG_SB      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_b, dmm_b,  wm_mem, n,  rx, n   };
      `RISCV_INST_MSG_SH      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_h, dmm_h,  wm_mem, n,  rx, n   };

      `RISCV_INST_MSG_JAL     :cs0={ y,  y,    br_none, pm_j,   am_pc4,  n,  bm_0,     n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_JALR    :cs0={ y,  y,    br_none, pm_r,   am_pc4,  y,  bm_0,     n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };

      `RISCV_INST_MSG_BNE     :cs0={ y,  n,    br_bne,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BEQ     :cs0={ y,  n,    br_beq,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BLT     :cs0={ y,  n,    br_blt,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BGE     :cs0={ y,  n,    br_bge,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BLTU    :cs0={ y,  n,    br_bltu, pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BGEU    :cs0={ y,  n,    br_bgeu, pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };

      `RISCV_INST_MSG_MUL     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_mul,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_DIV     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_div,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_REM     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_rem,  y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_DIVU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_divu, y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_REMU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_remu, y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };

      `RISCV_INST_MSG_CSRW    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_0,     y,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, n,  rx, y   };

    endcase

  end

  always @ (*) begin

    cs1 = {cs_sz{1'bx}}; // Default to invalid instruction

    casez ( ir1_Dhl )

      //                                j     br       pc      op0      rs1 op1       rs2 alu       md       md md     ex      mem  mem   memresp wb      rf      csr
      //                            val taken type     muxsel  muxsel   en  muxsel    en  fn        fn       en muxsel muxsel  rq   len   muxsel  muxsel  wen wa  wen
      `RISCV_INST_MSG_LUI     :cs1={ y,  n,    br_none, pm_p,   am_0,    n,  bm_imm_u, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_AUIPC   :cs1={ y,  n,    br_none, pm_p,   am_pc,   n,  bm_imm_u, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `RISCV_INST_MSG_ADDI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_ORI     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLTI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLTIU   :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_XORI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_ANDI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLLI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SRLI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SRAI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `RISCV_INST_MSG_ADD     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SUB     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLL     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLT     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLTU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_XOR     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SRL     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SRA     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_OR      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_AND     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `RISCV_INST_MSG_LW      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_w, dmm_w,  wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_LB      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_b,  wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_LH      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_h,  wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_LBU     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_bu, wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_LHU     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_hu, wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_SW      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_w, dmm_w,  wm_mem, n,  rx, n   };
      `RISCV_INST_MSG_SB      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_b, dmm_b,  wm_mem, n,  rx, n   };
      `RISCV_INST_MSG_SH      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_h, dmm_h,  wm_mem, n,  rx, n   };

      `RISCV_INST_MSG_JAL     :cs1={ y,  y,    br_none, pm_j,   am_pc4,  n,  bm_0,     n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_JALR    :cs1={ y,  y,    br_none, pm_r,   am_pc4,  y,  bm_0,     n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `RISCV_INST_MSG_BNE     :cs1={ y,  n,    br_bne,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BEQ     :cs1={ y,  n,    br_beq,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BLT     :cs1={ y,  n,    br_blt,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BGE     :cs1={ y,  n,    br_bge,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BLTU    :cs1={ y,  n,    br_bltu, pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BGEU    :cs1={ y,  n,    br_bgeu, pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };

      `RISCV_INST_MSG_MUL     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_mul,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_DIV     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_div,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_REM     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_rem,  y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_DIVU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_divu, y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_REMU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_remu, y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `RISCV_INST_MSG_CSRW    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_0,     y,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, n,  rx, y   };

    endcase

  end

  //----------------------------------------------------------------------
  // Scoreboard Structure
  //----------------------------------------------------------------------

  // 32 architectural registers
  reg [31:0] sb_pending;
  reg [31:0] sb_pipeline; // 0 for Pipeline A, 1 for Pipeline B
  reg [31:0] sb_stage_0;  // True if instruction is currently in X0
  reg [31:0] sb_stage_1;  // True if instruction is currently in X1
  reg [31:0] sb_stage_2;  // True if instruction is currently in X2
  reg [31:0] sb_stage_3;  // True if instruction is currently in X3
  reg [31:0] sb_stage_4;  // True if instruction is currently in W
  reg [31:0] sb_is_load;  // True if instruction is a load
  reg [31:0] sb_is_muldiv; // True if instruction uses the mul/div unit

  // Scoreboard allocation signals from Decode stage
  wire sb_alloc_0 = ir0_valid_issue && inst0_wen_Dhl && (inst0_rd_Dhl != 5'd0);
  wire sb_alloc_1 = ir1_valid_issue && inst1_wen_Dhl && (inst1_rd_Dhl != 5'd0);

  integer i;
  always @(posedge clk) begin
    if (reset) begin
      sb_pending   <= 32'b0;
      sb_pipeline  <= 32'b0;
      sb_stage_0   <= 32'b0;
      sb_stage_1   <= 32'b0;
      sb_stage_2   <= 32'b0;
      sb_stage_3   <= 32'b0;
      sb_stage_4   <= 32'b0;
      sb_is_load   <= 32'b0;
      sb_is_muldiv <= 32'b0;
    end else begin
      for (i = 0; i < 32; i = i + 1) begin
        // The scoreboard must always track the YOUNGEST producer of a register.
        // We ensure this by checking from the oldest stage (W) to the youngest (Issue-1).
        // Each successive check for architectural register 'i' will overwrite the previous ones if valid.
        
        // Default: not pending.
        sb_pending[i]   <= 1'b0;
        sb_pipeline[i]  <= 1'b0;
        sb_stage_0[i]   <= 1'b0;
        sb_stage_1[i]   <= 1'b0;
        sb_stage_2[i]   <= 1'b0;
        sb_stage_3[i]   <= 1'b0;
        sb_stage_4[i]   <= 1'b0;
        sb_is_load[i]   <= 1'b0;
        sb_is_muldiv[i] <= 1'b0;

        // 1. Existing pipeline stages
        if (inst_val_Whl && rf0_wen_Whl && (rf0_waddr_Whl == i) && (i != 0)) begin
            sb_pending[i]   <= 1'b1;
            sb_pipeline[i]  <= 1'b0;
            sb_stage_4[i]   <= 1'b1;
            sb_is_load[i]   <= 1'b0; // Metadata irrelevant in W, but clear it
            sb_is_muldiv[i] <= 1'b0;
        end
        if (inst_val_Whl && rf1_wen_Whl && (rf1_waddr_Whl == i) && (i != 0)) begin
            sb_pending[i]   <= 1'b1;
            sb_pipeline[i]  <= 1'b1;
            sb_stage_4[i]   <= 1'b1;
        end
        if (inst_val_X3hl && rf0_wen_X3hl && (rf0_waddr_X3hl == i) && (i != 0)) begin
            sb_pending[i]   <= 1'b1;
            sb_pipeline[i]  <= 1'b0;
            sb_stage_3[i]   <= 1'b1;
            sb_is_load[i]   <= is_load_X3hl;
            sb_is_muldiv[i] <= is_muldiv_X3hl;
        end
        if (inst_val_X3hl && rf1_wen_X3hl && (rf1_waddr_X3hl == i) && (i != 0)) begin
            sb_pending[i]   <= 1'b1;
            sb_pipeline[i]  <= 1'b1;
            sb_stage_3[i]   <= 1'b1;
        end
        if (inst_val_X2hl && rf0_wen_X2hl && (rf0_waddr_X2hl == i) && (i != 0)) begin
            sb_pending[i]   <= 1'b1;
            sb_pipeline[i]  <= 1'b0;
            sb_stage_2[i]   <= 1'b1;
            sb_is_load[i]   <= is_load_X2hl;
            sb_is_muldiv[i] <= is_muldiv_X2hl;
        end
        if (inst_val_X2hl && rf1_wen_X2hl && (rf1_waddr_X2hl == i) && (i != 0)) begin
            sb_pending[i]   <= 1'b1;
            sb_pipeline[i]  <= 1'b1;
            sb_stage_2[i]   <= 1'b1;
        end
        if (inst_val_X1hl && rf0_wen_X1hl && (rf0_waddr_X1hl == i) && (i != 0)) begin
            sb_pending[i]   <= 1'b1;
            sb_pipeline[i]  <= 1'b0;
            sb_stage_1[i]   <= 1'b1;
            sb_is_load[i]   <= is_load_X1hl;
            sb_is_muldiv[i] <= is_muldiv_X1hl;
        end
        if (inst_val_X1hl && rf1_wen_X1hl && (rf1_waddr_X1hl == i) && (i != 0)) begin
            sb_pending[i]   <= 1'b1;
            sb_pipeline[i]  <= 1'b1;
            sb_stage_1[i]   <= 1'b1;
        end
        if (inst_val_X0hl && rf0_wen_X0hl && (rf0_waddr_X0hl == i) && (i != 0)) begin
            sb_pending[i]   <= 1'b1;
            sb_pipeline[i]  <= 1'b0;
            sb_stage_0[i]   <= 1'b1;
            sb_is_load[i]   <= is_load_X0hl;
            sb_is_muldiv[i] <= is_muldiv_X0hl;
        end
        if (inst_val_X0hl && rf1_wen_X0hl && (rf1_waddr_X0hl == i) && (i != 0)) begin
            sb_pending[i]   <= 1'b1;
            sb_pipeline[i]  <= 1'b1;
            sb_stage_0[i]   <= 1'b1;
            // sb_is_load/muldiv default to 0 for Pipe B
        end

        // 2. Newly issued instructions (the YOUNGEST)
        if (sb_alloc_0 && !stall_0_Dhl && (inst0_rd_Dhl == i)) begin
            sb_pending[i]   <= 1'b1;
            sb_pipeline[i]  <= steer_inst0_to_B_Dhl;
            sb_stage_0[i]   <= 1'b0; // Reaching X0 next cycle
            sb_is_load[i]   <= (cs0[`RISCV_INST_MSG_MEM_REQ] == ld);
            sb_is_muldiv[i] <= (cs0[`RISCV_INST_MSG_MULDIV_EN] == 1'b1);
        end
        if (sb_alloc_1 && !stall_1_hazard_Dhl && (inst1_rd_Dhl == i)) begin
            sb_pending[i]   <= 1'b1;
            sb_pipeline[i]  <= !steer_inst0_to_B_Dhl; 
            sb_stage_0[i]   <= 1'b0;
            sb_is_load[i]   <= (cs1[`RISCV_INST_MSG_MEM_REQ] == ld);
            sb_is_muldiv[i] <= (cs1[`RISCV_INST_MSG_MULDIV_EN] == 1'b1);
        end

        // Special case: if no instruction in the entire pipeline is currently writing to 'i', 
        // clear the pending bit. This handles squashes automatically because inst_val_X* will be 0.
        // Wait, if it's already pending from the previous cycle, but not in any stage now (e.g. was in W last cycle),
        // then the default must be 0? Yes, sb_pending is updated by the assignments above.
      end
    end
  end

  // Safe validity flags - prevent x propagation from unrecognized instructions
  wire cs0_valid = (cs0[`RISCV_INST_MSG_INST_VAL] === 1'b1);
  wire cs1_valid = (cs1[`RISCV_INST_MSG_INST_VAL] === 1'b1);

  // Instruction Categorization (ALU vs NON-ALU)
  wire inst0_is_alu_Dhl = inst_val_Dhl && cs0_valid
                       && (cs0[`RISCV_INST_MSG_MULDIV_EN] == 1'b0)
                       && (cs0[`RISCV_INST_MSG_BR_SEL] == br_none)
                       && (cs0[`RISCV_INST_MSG_J_EN] == 1'b0)
                       && (cs0[`RISCV_INST_MSG_MEM_REQ] == nr)
                       && (cs0[`RISCV_INST_MSG_CSR_WEN] == 1'b0);

  wire inst1_is_alu_Dhl = inst_val_Dhl && cs1_valid
                       && (cs1[`RISCV_INST_MSG_MULDIV_EN] == 1'b0)
                       && (cs1[`RISCV_INST_MSG_BR_SEL] == br_none)
                       && (cs1[`RISCV_INST_MSG_J_EN] == 1'b0)
                       && (cs1[`RISCV_INST_MSG_MEM_REQ] == nr)
                       && (cs1[`RISCV_INST_MSG_CSR_WEN] == 1'b0);

  // Destination Registers Write-Enable
  // Destination Registers Write-Enable (gated by valid)
  wire inst0_wen_Dhl = cs0_valid && cs0[`RISCV_INST_MSG_RF_WEN];
  wire inst1_wen_Dhl = cs1_valid && cs1[`RISCV_INST_MSG_RF_WEN];

  // WAW Hazard Detection (Intra-pair)
  wire waw_hazard_Dhl = inst_val_Dhl && cs0_valid && cs1_valid && inst0_wen_Dhl && inst1_wen_Dhl 
                     && (inst0_rd_Dhl == inst1_rd_Dhl) && (inst0_rd_Dhl != 5'd0);

  // RAW Hazard Detection (Intra-pair)
  wire inst1_rs1_en_Dhl = cs1_valid && cs1[`RISCV_INST_MSG_RS1_EN];
  wire inst1_rs2_en_Dhl = cs1_valid && cs1[`RISCV_INST_MSG_RS2_EN];
  // In our superscalar implementation, a stage is valid if it's not a bubble.
  // We consume physical slots (4 bytes each) even if they contain NOPs.
  wire raw_hazard_Dhl   = inst_val_Dhl && cs0_valid && cs1_valid && inst0_wen_Dhl && (inst0_rd_Dhl != 5'd0)
                       && ( (inst1_rs1_en_Dhl && (inst1_rs1_Dhl == inst0_rd_Dhl))
                         || (inst1_rs2_en_Dhl && (inst1_rs2_Dhl == inst0_rd_Dhl)) );

  // Structural Hazard Detection
  wire structural_hazard_Dhl = inst_val_Dhl && cs0_valid && cs1_valid && !inst0_is_alu_Dhl && !inst1_is_alu_Dhl;

  // Steering Logic Table
  // Steers ir0 to B and ir1 to A ONLY if ir0 is ALU and ir1 is Non-ALU. Otherwise ir0 goes A, ir1 goes B.
  // DISABLED until Pipeline B writeback is enabled (rfB_wen_Whl is currently tied to 0)
  wire steer_inst0_to_B_Dhl = inst0_is_alu_Dhl && !inst1_is_alu_Dhl && cs0_valid && cs1_valid;

  // // Debugging part
  // `ifndef SYNTHESIS
  // always @(negedge clk) begin
  //   if (!reset && (ir0_valid_issue === 1'bx || ir1_valid_issue === 1'bx || (inst_val_Dhl && cs0_valid))) begin
  //     $display("DEBUG D: cyc=%0d ir0=%h ir1=%h", $time/10, ir0_Dhl, ir1_Dhl);
  //     $display("  val=%b cs0v=%b cs1v=%b steer=%b", inst_val_Dhl, cs0_valid, cs1_valid, steer_inst0_to_B_Dhl);
  //     $display("  hazards: raw=%b waw=%b struct=%b sb0=%b sb1=%b", raw_hazard_Dhl, waw_hazard_Dhl, structural_hazard_Dhl, stall_0_sb_Dhl, stall_1_sb_Dhl);
  //     $display("  issue: ir0_v=%b ir1_v=%b", ir0_valid_issue, ir1_valid_issue);
  //   end
  // end
  // `endif
  
  // Scoreboard RAW Hazards
  // We query the scoreboard for Rs1 and Rs2 of each instruction.
  // We MUST STALL if the producer is in a stage where the value isn't ready to be bypassed yet.
  //   - ALU/Branch/Jump (not load, not muldiv): result ready at end of X0. So if currently IN X0 (stage_0 is true), stall.
  //   - Load: result ready at end of X1. So if currently IN X0 or X1, stall.
  //   - Mul/Div: result ready at end of X3. So if currently IN X0, X1, X2, or X3, stall.
  
  wire rs10_sb_stall = rs10_en_Dhl && sb_pending[inst0_rs1_Dhl] && (
                          (sb_is_muldiv[inst0_rs1_Dhl] && (sb_stage_0[inst0_rs1_Dhl] || sb_stage_1[inst0_rs1_Dhl] || sb_stage_2[inst0_rs1_Dhl] || sb_stage_3[inst0_rs1_Dhl])) ||
                          (sb_is_load[inst0_rs1_Dhl] && (sb_stage_0[inst0_rs1_Dhl] || sb_stage_1[inst0_rs1_Dhl])) );
                          
  wire rs20_sb_stall = rs20_en_Dhl && sb_pending[inst0_rs2_Dhl] && (
                          (sb_is_muldiv[inst0_rs2_Dhl] && (sb_stage_0[inst0_rs2_Dhl] || sb_stage_1[inst0_rs2_Dhl] || sb_stage_2[inst0_rs2_Dhl] || sb_stage_3[inst0_rs2_Dhl])) ||
                          (sb_is_load[inst0_rs2_Dhl] && (sb_stage_0[inst0_rs2_Dhl] || sb_stage_1[inst0_rs2_Dhl])) );

  // We must also check the scoreboard for Instruction 1! 
  // Wait, inst1_rs1_Dhl uses the unsteered rs. rs11_addr_Dhl holds the ACTUAL rs1 for Pipeline 1.
  wire [4:0] inst1_actual_rs1 = inst1_rs1_Dhl; // Unsteered is fine, we just need to know if the instruction itself stalls.
  wire [4:0] inst1_actual_rs2 = inst1_rs2_Dhl;
  
  wire rs11_sb_stall = inst1_rs1_en_Dhl && sb_pending[inst1_actual_rs1] && (
                          (sb_is_muldiv[inst1_actual_rs1] && (sb_stage_0[inst1_actual_rs1] || sb_stage_1[inst1_actual_rs1] || sb_stage_2[inst1_actual_rs1] || sb_stage_3[inst1_actual_rs1])) ||
                          (sb_is_load[inst1_actual_rs1] && (sb_stage_0[inst1_actual_rs1] || sb_stage_1[inst1_actual_rs1])) );
                          
  wire rs21_sb_stall = inst1_rs2_en_Dhl && sb_pending[inst1_actual_rs2] && (
                          (sb_is_muldiv[inst1_actual_rs2] && (sb_stage_0[inst1_actual_rs2] || sb_stage_1[inst1_actual_rs2] || sb_stage_2[inst1_actual_rs2] || sb_stage_3[inst1_actual_rs2])) ||
                          (sb_is_load[inst1_actual_rs2] && (sb_stage_0[inst1_actual_rs2] || sb_stage_1[inst1_actual_rs2])) );

  wire stall_0_sb_Dhl = rs10_sb_stall || rs20_sb_stall;
  wire stall_1_sb_Dhl = rs11_sb_stall || rs21_sb_stall;

  // ir1 stalls if there is a RAW, WAW, or Structural Hazard (or if ir0 stalls because of scoreboard)
  wire stall_1_hazard_Dhl = cs1_valid && (waw_hazard_Dhl || raw_hazard_Dhl || structural_hazard_Dhl || stall_1_sb_Dhl || stall_0_sb_Dhl);

  assign ir0_valid_issue = (inst_val_Dhl === 1'b1) && cs0_valid && !stall_0_sb_Dhl;
  assign ir1_valid_issue = (inst_val_Dhl === 1'b1) && cs1_valid && !stall_1_hazard_Dhl;

  assign instA_Dhl = (steer_inst0_to_B_Dhl) ? (ir1_valid_issue ? ir1_Dhl : 32'b0)
                                            : (ir0_valid_issue ? ir0_Dhl : 32'b0);

  assign instB_Dhl = (steer_inst0_to_B_Dhl) ? (ir0_valid_issue ? ir0_Dhl : 32'b0)
                                            : (ir1_valid_issue ? ir1_Dhl : 32'b0);

  // Compatability with the rest of Part 1 code so things keep wiring correctly right now.
  wire steering_mux_sel_Dhl = steer_inst0_to_B_Dhl;

  // Jump and Branch Controls

  // A jump or branch in the D stage is taken if EITHER validly issuing instruction is a jump.
  // Use raw validity (not including squash) to handle Jumps in D-stage 
  // to avoid combinatorial feedback loops.
  wire       ir0_is_jump_Dhl = ( cs0_valid && ( cs0[`RISCV_INST_MSG_J_EN] ) );
  wire       ir1_is_jump_Dhl = ( cs1_valid && ( cs1[`RISCV_INST_MSG_J_EN] ) );
  wire       brj_taken_Dhl   = (ir0_is_jump_Dhl && ir0_valid_issue) || (ir1_is_jump_Dhl && ir1_valid_issue);

  // Since we only have one branch target Datapath input, we take it from the active jump/branch.
  // We prioritize ir0 over ir1 (if both were branches, the first taken one would kill the second anyway).
  // AND gate the branch select by validity so stalled branches don't masquerade as valid.
  wire [2:0] br_sel_Dhl    = (ir0_is_jump_Dhl && ir0_valid_issue) ? cs0[`RISCV_INST_MSG_BR_SEL] : 
                             (ir1_is_jump_Dhl && ir1_valid_issue) ? cs1[`RISCV_INST_MSG_BR_SEL] : 
                             (steering_mux_sel_Dhl == 1'b0) ? 
                               (ir0_valid_issue ? cs0[`RISCV_INST_MSG_BR_SEL] : 3'd0) : 
                               (ir1_valid_issue ? cs1[`RISCV_INST_MSG_BR_SEL] : 3'd0);

  // PC Mux Select
  wire [1:0] pc_mux_sel_Dhl = (ir0_is_jump_Dhl && ir0_valid_issue) ? cs0[`RISCV_INST_MSG_PC_SEL] : cs1[`RISCV_INST_MSG_PC_SEL];

  // Operand Bypassing Logic

  wire [4:0] rs10_addr_Dhl  = (steering_mux_sel_Dhl == 1'b0) ? inst0_rs1_Dhl : inst1_rs1_Dhl;
  wire [4:0] rs20_addr_Dhl  = (steering_mux_sel_Dhl == 1'b0) ? inst0_rs2_Dhl : inst1_rs2_Dhl;

  wire [4:0] rs11_addr_Dhl  = inst1_rs1_Dhl;
  wire [4:0] rs21_addr_Dhl  = inst1_rs2_Dhl;

  wire       rs10_en_Dhl    = (steering_mux_sel_Dhl == 1'b0) ? (cs0_valid && cs0[`RISCV_INST_MSG_RS1_EN]) : (cs1_valid && cs1[`RISCV_INST_MSG_RS1_EN]);
  wire       rs20_en_Dhl    = (steering_mux_sel_Dhl == 1'b0) ? (cs0_valid && cs0[`RISCV_INST_MSG_RS2_EN]) : (cs1_valid && cs1[`RISCV_INST_MSG_RS2_EN]);

  wire       rs11_en_Dhl    = (steering_mux_sel_Dhl == 1'b0) ? (cs1_valid && cs1[`RISCV_INST_MSG_RS1_EN]) : (cs0_valid && cs0[`RISCV_INST_MSG_RS1_EN]);
  wire       rs21_en_Dhl    = (steering_mux_sel_Dhl == 1'b0) ? (cs1_valid && cs1[`RISCV_INST_MSG_RS2_EN]) : (cs0_valid && cs0[`RISCV_INST_MSG_RS2_EN]);

  // ----------------------------------------------------------------------
  // Pipeline Mapping and Grounding Logic
  // ----------------------------------------------------------------------

  // Map Pipeline B write signals to the internal rf1 registers
  // X0 is the stage where the squash happens, so we use the gated version!
  wire       rfB_wen_X0hl   = rfB_wen_X0hl_real;
  wire [4:0] rfB_waddr_X0hl = rf1_waddr_X0hl;
  wire       rfB_wen_X1hl   = rf1_wen_X1hl;
  wire [4:0] rfB_waddr_X1hl = rf1_waddr_X1hl;
  wire       rfB_wen_X2hl   = rf1_wen_X2hl;
  wire [4:0] rfB_waddr_X2hl = rf1_waddr_X2hl;
  wire       rfB_wen_X3hl   = rf1_wen_X3hl;
  wire [4:0] rfB_waddr_X3hl = rf1_waddr_X3hl;
  
  // Map Pipeline A write signals to the internal rf0 registers for Part 1 bypass logic
  wire       rfA_wen_X0hl   = rf0_wen_X0hl;
  wire [4:0] rfA_waddr_X0hl = rf0_waddr_X0hl;
  wire       rfA_wen_X1hl   = rf0_wen_X1hl;
  wire [4:0] rfA_waddr_X1hl = rf0_waddr_X1hl;
  wire       rfA_wen_X2hl   = rf0_wen_X2hl;
  wire [4:0] rfA_waddr_X2hl = rf0_waddr_X2hl;
  wire       rfA_wen_X3hl   = rf0_wen_X3hl;
  wire [4:0] rfA_waddr_X3hl = rf0_waddr_X3hl;

  // Drive outputs for Pipeline A writeback
  wire rf0_wen_out_Whl        = ( inst_val_Whl && !stall_Whl && rf0_wen_Whl );
  assign rfA_wen_Whl          = rf0_wen_out_Whl;
  assign rfA_waddr_Whl        = rf0_waddr_Whl;

  // Drive outputs for Pipeline A operand and ALU execution
  assign opA0_mux_sel_Dhl     = op00_mux_sel_Dhl;
  assign opA1_mux_sel_Dhl     = op01_mux_sel_Dhl;
  assign aluA_fn_X0hl         = alu0_fn_X0hl;

  // Drive outputs for Pipeline B operand and ALU execution
  wire rf1_wen_out_Whl        = ( inst_val_Whl && !stall_Whl && rf1_wen_Whl );
  assign rfB_wen_Whl          = rf1_wen_out_Whl;
  assign rfB_waddr_Whl        = rf1_waddr_Whl;
  assign aluB_fn_X0hl         = alu1_fn_X0hl;
  assign opB0_mux_sel_Dhl     = op10_mux_sel_Dhl;
  assign opB1_mux_sel_Dhl     = op11_mux_sel_Dhl;

  // Scoreboard-based bypass mux selects for Instruction 0's operands
  wire inst0_rs1_en_Dhl = cs0_valid && cs0[`RISCV_INST_MSG_RS1_EN];
  wire inst0_rs2_en_Dhl = cs0_valid && cs0[`RISCV_INST_MSG_RS2_EN];

  wire [3:0] op00_byp_mux_sel_Dhl = 
    (!inst0_rs1_en_Dhl || inst0_rs1_Dhl == 5'd0 || !sb_pending[inst0_rs1_Dhl]) ? 4'd0 :
    (sb_pipeline[inst0_rs1_Dhl] == 1'b0) ? // Pipeline A producer
        (sb_stage_0[inst0_rs1_Dhl] ? 4'd1 :
         sb_stage_1[inst0_rs1_Dhl] ? 4'd2 :
         sb_stage_2[inst0_rs1_Dhl] ? 4'd3 :
         sb_stage_3[inst0_rs1_Dhl] ? 4'd4 :
         sb_stage_4[inst0_rs1_Dhl] ? 4'd5 : 4'd0) :
    // Pipeline B producer
        (sb_stage_0[inst0_rs1_Dhl] ? 4'd6 :
         sb_stage_1[inst0_rs1_Dhl] ? 4'd7 :
         sb_stage_2[inst0_rs1_Dhl] ? 4'd8 :
         sb_stage_3[inst0_rs1_Dhl] ? 4'd9 :
         sb_stage_4[inst0_rs1_Dhl] ? 4'd10 : 4'd0);

  wire [3:0] op01_byp_mux_sel_Dhl = 
    (!inst0_rs2_en_Dhl || inst0_rs2_Dhl == 5'd0 || !sb_pending[inst0_rs2_Dhl]) ? 4'd0 :
    (sb_pipeline[inst0_rs2_Dhl] == 1'b0) ? // Pipeline A producer
        (sb_stage_0[inst0_rs2_Dhl] ? 4'd1 :
         sb_stage_1[inst0_rs2_Dhl] ? 4'd2 :
         sb_stage_2[inst0_rs2_Dhl] ? 4'd3 :
         sb_stage_3[inst0_rs2_Dhl] ? 4'd4 :
         sb_stage_4[inst0_rs2_Dhl] ? 4'd5 : 4'd0) :
    // Pipeline B producer
        (sb_stage_0[inst0_rs2_Dhl] ? 4'd6 :
         sb_stage_1[inst0_rs2_Dhl] ? 4'd7 :
         sb_stage_2[inst0_rs2_Dhl] ? 4'd8 :
         sb_stage_3[inst0_rs2_Dhl] ? 4'd9 :
         sb_stage_4[inst0_rs2_Dhl] ? 4'd10 : 4'd0);

  // Scoreboard-based bypass mux selects for Instruction 1's operands
  wire [3:0] op10_byp_mux_sel_Dhl = 
    (!inst1_rs1_en_Dhl || inst1_rs1_Dhl == 5'd0 || !sb_pending[inst1_rs1_Dhl]) ? 4'd0 :
    (sb_pipeline[inst1_rs1_Dhl] == 1'b0) ? // Pipeline A producer
        (sb_stage_0[inst1_rs1_Dhl] ? 4'd1 :
         sb_stage_1[inst1_rs1_Dhl] ? 4'd2 :
         sb_stage_2[inst1_rs1_Dhl] ? 4'd3 :
         sb_stage_3[inst1_rs1_Dhl] ? 4'd4 :
         sb_stage_4[inst1_rs1_Dhl] ? 4'd5 : 4'd0) :
    // Pipeline B producer
        (sb_stage_0[inst1_rs1_Dhl] ? 4'd6 :
         sb_stage_1[inst1_rs1_Dhl] ? 4'd7 :
         sb_stage_2[inst1_rs1_Dhl] ? 4'd8 :
         sb_stage_3[inst1_rs1_Dhl] ? 4'd9 :
         sb_stage_4[inst1_rs1_Dhl] ? 4'd10 : 4'd0);
         
  wire [3:0] op11_byp_mux_sel_Dhl = 
    (!inst1_rs2_en_Dhl || inst1_rs2_Dhl == 5'd0 || !sb_pending[inst1_rs2_Dhl]) ? 4'd0 :
    (sb_pipeline[inst1_rs2_Dhl] == 1'b0) ? // Pipeline A producer
        (sb_stage_0[inst1_rs2_Dhl] ? 4'd1 :
         sb_stage_1[inst1_rs2_Dhl] ? 4'd2 :
         sb_stage_2[inst1_rs2_Dhl] ? 4'd3 :
         sb_stage_3[inst1_rs2_Dhl] ? 4'd4 :
         sb_stage_4[inst1_rs2_Dhl] ? 4'd5 : 4'd0) :
    // Pipeline B producer
        (sb_stage_0[inst1_rs2_Dhl] ? 4'd6 :
         sb_stage_1[inst1_rs2_Dhl] ? 4'd7 :
         sb_stage_2[inst1_rs2_Dhl] ? 4'd8 :
         sb_stage_3[inst1_rs2_Dhl] ? 4'd9 :
         sb_stage_4[inst1_rs2_Dhl] ? 4'd10 : 4'd0);

  // Drive outputs for Pipeline A operand bypasses (steered from inst0 or inst1)
  assign opA0_byp_mux_sel_Dhl = (steer_inst0_to_B_Dhl) ? op10_byp_mux_sel_Dhl : op00_byp_mux_sel_Dhl;
  assign opA1_byp_mux_sel_Dhl = (steer_inst0_to_B_Dhl) ? op11_byp_mux_sel_Dhl : op01_byp_mux_sel_Dhl;

  // Drive outputs for Pipeline B operand bypasses
  assign opB0_byp_mux_sel_Dhl = (steer_inst0_to_B_Dhl) ? op00_byp_mux_sel_Dhl : op10_byp_mux_sel_Dhl;
  assign opB1_byp_mux_sel_Dhl = (steer_inst0_to_B_Dhl) ? op01_byp_mux_sel_Dhl : op11_byp_mux_sel_Dhl;

  // Operand Mux Select

  wire [1:0] op00_mux_sel_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs0[`RISCV_INST_MSG_OP0_SEL] : cs1[`RISCV_INST_MSG_OP0_SEL];
  wire [2:0] op01_mux_sel_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs0[`RISCV_INST_MSG_OP1_SEL] : cs1[`RISCV_INST_MSG_OP1_SEL];

  wire [1:0] op10_mux_sel_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs1[`RISCV_INST_MSG_OP0_SEL] : cs0[`RISCV_INST_MSG_OP0_SEL];
  wire [2:0] op11_mux_sel_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs1[`RISCV_INST_MSG_OP1_SEL] : cs0[`RISCV_INST_MSG_OP1_SEL];

  // ALU Function

  wire [3:0] alu0_fn_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs0[`RISCV_INST_MSG_ALU_FN] : cs1[`RISCV_INST_MSG_ALU_FN];
  wire [3:0] alu1_fn_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs1[`RISCV_INST_MSG_ALU_FN] : cs0[`RISCV_INST_MSG_ALU_FN];

  // Muldiv Function

  assign muldivreq_msg_fn_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs0[`RISCV_INST_MSG_MULDIV_FN] : cs1[`RISCV_INST_MSG_MULDIV_FN];

  // Muldiv Controls

  wire muldivreq_val_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs0[`RISCV_INST_MSG_MULDIV_EN] : cs1[`RISCV_INST_MSG_MULDIV_EN];

  // Muldiv Mux Select

  wire muldiv_mux_sel_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs0[`RISCV_INST_MSG_MULDIV_SEL] : cs1[`RISCV_INST_MSG_MULDIV_SEL];

  // Pipeline Validity Signals for Instruction A and B
  wire instA_val_Dhl = (steering_mux_sel_Dhl == 1'b0) ? ir0_valid_issue : ir1_valid_issue;
  wire instB_val_Dhl = (steering_mux_sel_Dhl == 1'b0) ? ir1_valid_issue : ir0_valid_issue;

  // Execute Mux Select
  wire execute_mux_sel_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs0[`RISCV_INST_MSG_MULDIV_EN] : cs1[`RISCV_INST_MSG_MULDIV_EN];

  wire       is_load_Dhl         = instA_val_Dhl && ((steering_mux_sel_Dhl == 1'b0) ? ( cs0[`RISCV_INST_MSG_MEM_REQ] == ld ) : ( cs1[`RISCV_INST_MSG_MEM_REQ] == ld ));
  wire       dmemreq_msg_rw_Dhl  = (steering_mux_sel_Dhl == 1'b0) ? ( cs0[`RISCV_INST_MSG_MEM_REQ] == st ) : ( cs1[`RISCV_INST_MSG_MEM_REQ] == st );
  wire [1:0] dmemreq_msg_len_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs0[`RISCV_INST_MSG_MEM_LEN] : cs1[`RISCV_INST_MSG_MEM_LEN];
  wire       dmemreq_val_Dhl     = instA_val_Dhl && ((steering_mux_sel_Dhl == 1'b0) ? ( cs0[`RISCV_INST_MSG_MEM_REQ] != nr ) : ( cs1[`RISCV_INST_MSG_MEM_REQ] != nr ));

  // Memory response mux select
  wire [2:0] dmemresp_mux_sel_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs0[`RISCV_INST_MSG_MEM_SEL] : cs1[`RISCV_INST_MSG_MEM_SEL];

  // Writeback Mux Select
  wire memex_mux_sel_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs0[`RISCV_INST_MSG_WB_SEL] : cs1[`RISCV_INST_MSG_WB_SEL];

  // Register Writeback Controls
  wire rf0_wen_Dhl         = instA_val_Dhl && ((steering_mux_sel_Dhl == 1'b0) ? cs0[`RISCV_INST_MSG_RF_WEN] : cs1[`RISCV_INST_MSG_RF_WEN]);
  wire [4:0] rf0_waddr_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs0[`RISCV_INST_MSG_RF_WADDR] : cs1[`RISCV_INST_MSG_RF_WADDR];
  wire rf1_wen_Dhl         = instB_val_Dhl && ((steering_mux_sel_Dhl == 1'b0) ? cs1[`RISCV_INST_MSG_RF_WEN] : cs0[`RISCV_INST_MSG_RF_WEN]);
  wire [4:0] rf1_waddr_Dhl = (steering_mux_sel_Dhl == 1'b0) ? cs1[`RISCV_INST_MSG_RF_WADDR] : cs0[`RISCV_INST_MSG_RF_WADDR];

  // CSR register write enable
  wire csr_wen_Dhl = instA_val_Dhl && ((steering_mux_sel_Dhl == 1'b0) ? cs0[`RISCV_INST_MSG_CSR_WEN] : cs1[`RISCV_INST_MSG_CSR_WEN]);

  // CSR register address

  wire [11:0] csr_addr_Dhl  = (steering_mux_sel_Dhl == 1'b0) ? ir0_Dhl[31:20] : ir1_Dhl[31:20];

  //----------------------------------------------------------------------
  // Squash and Stall Logic
  //----------------------------------------------------------------------

  // Squash instruction in D if a valid branch in X is taken

  // Squash logic moved to top of module to avoid circular dependencies

  // For Part 2 of this lab, replace the multdiv and ld stall logic with a scoreboard based stall logic

  // Stall in D if muldiv unit is not ready and there is a valid request
  
  wire stall_0_muldiv_use_Dhl = inst_val_Dhl && (
                              ( inst_val_X0hl && rs10_en_Dhl && rfA_wen_X0hl
                                && ( rs10_addr_Dhl == rfA_waddr_X0hl )
                                && ( rfA_waddr_X0hl != 5'd0 ) && is_muldiv_X0hl )
                           || ( inst_val_X1hl && rs10_en_Dhl && rfA_wen_X1hl
                                && ( rs10_addr_Dhl == rfA_waddr_X1hl )
                                && ( rfA_waddr_X1hl != 5'd0 ) && is_muldiv_X1hl )
                           || ( inst_val_X2hl && rs10_en_Dhl && rfA_wen_X2hl
                                && ( rs10_addr_Dhl == rfA_waddr_X2hl )
                                && ( rfA_waddr_X2hl != 5'd0 ) && is_muldiv_X2hl )
                           || ( inst_val_X3hl && rs10_en_Dhl && rfA_wen_X3hl
                                && ( rs10_addr_Dhl == rfA_waddr_X3hl )
                                && ( rfA_waddr_X3hl != 5'd0 ) && is_muldiv_X3hl )
                           || ( inst_val_X0hl && rs20_en_Dhl && rfA_wen_X0hl
                                && ( rs20_addr_Dhl == rfA_waddr_X0hl )
                                && ( rfA_waddr_X0hl != 5'd0 ) && is_muldiv_X0hl )
                           || ( inst_val_X1hl && rs20_en_Dhl && rfA_wen_X1hl
                                && ( rs20_addr_Dhl == rfA_waddr_X1hl )
                                && ( rfA_waddr_X1hl != 5'd0 ) && is_muldiv_X1hl )
                           || ( inst_val_X2hl && rs20_en_Dhl && rfA_wen_X2hl
                                && ( rs20_addr_Dhl == rfA_waddr_X2hl )
                                && ( rfA_waddr_X2hl != 5'd0 ) && is_muldiv_X2hl )
                           || ( inst_val_X3hl && rs20_en_Dhl && rfA_wen_X3hl
                                && ( rs20_addr_Dhl == rfA_waddr_X3hl )
                                && ( rfA_waddr_X3hl != 5'd0 ) && is_muldiv_X3hl ));
  wire stall_1_muldiv_use_Dhl = inst_val_Dhl && (
                              ( inst_val_X0hl && rs11_en_Dhl && rfA_wen_X0hl
                                && ( rs11_addr_Dhl == rfA_waddr_X0hl )
                                && ( rfA_waddr_X0hl != 5'd0 ) && is_muldiv_X0hl )
                           || ( inst_val_X1hl && rs11_en_Dhl && rfA_wen_X1hl
                                && ( rs11_addr_Dhl == rfA_waddr_X1hl )
                                && ( rfA_waddr_X1hl != 5'd0 ) && is_muldiv_X1hl )
                           || ( inst_val_X2hl && rs11_en_Dhl && rfA_wen_X2hl
                                && ( rs11_addr_Dhl == rfA_waddr_X2hl )
                                && ( rfA_waddr_X2hl != 5'd0 ) && is_muldiv_X2hl )
                           || ( inst_val_X3hl && rs11_en_Dhl && rfA_wen_X3hl
                                && ( rs11_addr_Dhl == rfA_waddr_X3hl )
                                && ( rfA_waddr_X3hl != 5'd0 ) && is_muldiv_X3hl )
                           || ( inst_val_X0hl && rs21_en_Dhl && rfA_wen_X0hl
                                && ( rs21_addr_Dhl == rfA_waddr_X0hl )
                                && ( rfA_waddr_X0hl != 5'd0 ) && is_muldiv_X0hl )
                           || ( inst_val_X1hl && rs21_en_Dhl && rfA_wen_X1hl
                                && ( rs21_addr_Dhl == rfA_waddr_X1hl )
                                && ( rfA_waddr_X1hl != 5'd0 ) && is_muldiv_X1hl )
                           || ( inst_val_X2hl && rs21_en_Dhl && rfA_wen_X2hl
                                && ( rs21_addr_Dhl == rfA_waddr_X2hl )
                                && ( rfA_waddr_X2hl != 5'd0 ) && is_muldiv_X2hl )
                           || ( inst_val_X3hl && rs21_en_Dhl && rfA_wen_X3hl
                                && ( rs21_addr_Dhl == rfA_waddr_X3hl )
                                && ( rfA_waddr_X3hl != 5'd0 ) && is_muldiv_X3hl ));

  // Stall for load-use only if instruction in D is valid and either of
  // the source registers match the destination register of of a valid
  // instruction in a later stage.

  wire stall_0_load_use_Dhl = inst_val_Dhl && (
                            ( inst_val_X0hl && rs10_en_Dhl && rfA_wen_X0hl
                              && ( rs10_addr_Dhl == rfA_waddr_X0hl )
                              && ( rfA_waddr_X0hl != 5'd0 ) && is_load_X0hl )
                         || ( inst_val_X1hl && rs10_en_Dhl && rfA_wen_X1hl
                              && ( rs10_addr_Dhl == rfA_waddr_X1hl )
                              && ( rfA_waddr_X1hl != 5'd0 ) && is_load_X1hl )
                         || ( inst_val_X0hl && rs20_en_Dhl && rfA_wen_X0hl
                              && ( rs20_addr_Dhl == rfA_waddr_X0hl )
                              && ( rfA_waddr_X0hl != 5'd0 ) && is_load_X0hl )
                         || ( inst_val_X1hl && rs20_en_Dhl && rfA_wen_X1hl
                              && ( rs20_addr_Dhl == rfA_waddr_X1hl )
                              && ( rfA_waddr_X1hl != 5'd0 ) && is_load_X1hl ) );

  wire stall_1_load_use_Dhl = inst_val_Dhl && (
                            ( inst_val_X0hl && rs11_en_Dhl && rfA_wen_X0hl
                              && ( rs11_addr_Dhl == rfA_waddr_X0hl )
                              && ( rfA_waddr_X0hl != 5'd0 ) && is_load_X0hl )
                         || ( inst_val_X1hl && rs11_en_Dhl && rfA_wen_X1hl
                              && ( rs11_addr_Dhl == rfA_waddr_X1hl )
                              && ( rfA_waddr_X1hl != 5'd0 ) && is_load_X1hl )
                         || ( inst_val_X0hl && rs21_en_Dhl && rfA_wen_X0hl
                              && ( rs21_addr_Dhl == rfA_waddr_X0hl )
                              && ( rfA_waddr_X0hl != 5'd0 ) && is_load_X0hl )
                         || ( inst_val_X1hl && rs21_en_Dhl && rfA_wen_X1hl
                              && ( rs21_addr_Dhl == rfA_waddr_X1hl )
                              && ( rfA_waddr_X1hl != 5'd0 ) && is_load_X1hl ) );

  // Aggregate Stall Signal

  wire stall_0_Dhl = (stall_X0hl || stall_0_muldiv_use_Dhl || stall_0_load_use_Dhl || stall_0_sb_Dhl);
  wire stall_1_Dhl = (stall_X0hl || stall_1_muldiv_use_Dhl || stall_1_load_use_Dhl || stall_1_sb_Dhl);

  // Freeze-and-NOP: stall entire D if EITHER instruction cannot issue.
  // When ir1 stalls but ir0 issues, D stays frozen and we NOP out ir0 above.
  // Use stall_1_hazard_Dhl (not stall_1_Dhl) because it's correctly gated by cs1_valid
  // and won't false-stall when ir1_Dhl is a NOP (after being zeroed).
  assign stall_Dhl = (stall_0_Dhl === 1'b1) || (stall_1_hazard_Dhl === 1'b1 && cs1_valid);

  // Next bubble bit
  // The D stage injects a bubble into X0 if the active instruction is squash or stalled.
  wire bubble_sel_Dhl  = ( squash_Dhl || active_stall_Dhl );
  wire bubble_next_Dhl = ( !bubble_sel_Dhl ) ? bubble_Dhl
                       : ( bubble_sel_Dhl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // X0 <- D
  //----------------------------------------------------------------------

  reg [31:0] ir0_X0hl;
  reg [31:0] ir1_X0hl;
  wire [31:0] irA_X0hl = ir0_X0hl;
  wire [31:0] irB_X0hl = ir1_X0hl;
  reg  [2:0] br_sel_X0hl;
  reg  [3:0] alu0_fn_X0hl;
  reg  [3:0] alu1_fn_X0hl;
  reg        muldivreq_val_X0hl;
  reg  [2:0] muldivreq_msg_fn_X0hl;
  reg        muldiv_mux_sel_X0hl;
  reg        execute_mux_sel_X0hl;
  reg        is_load_X0hl;
  reg        is_muldiv_X0hl;
  reg        dmemreq_msg_rw_X0hl;
  reg  [1:0] dmemreq_msg_len_X0hl;
  reg        dmemreq_val_X0hl;
  reg  [2:0] dmemresp_mux_sel_X0hl;
  reg        memex_mux_sel_X0hl;
  reg        rf0_wen_X0hl;
  reg  [4:0] rf0_waddr_X0hl;
  reg        rf1_wen_X0hl;
  reg  [4:0] rf1_waddr_X0hl;
  reg        csr_wen_X0hl;
  reg [11:0] csr_addr_X0hl;

  reg        bubble_X0hl;
  reg        instA_was_ir0_X0hl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      rf0_wen_X0hl        <= 1'b0;
      rf1_wen_X0hl        <= 1'b0;
      dmemreq_val_X0hl    <= 1'b0;
      muldivreq_val_X0hl  <= 1'b0;
      csr_wen_X0hl        <= 1'b0;
      bubble_X0hl         <= 1'b1;
    end
    else if( !stall_X0hl ) begin
      ir0_X0hl              <= instA_Dhl;
      ir1_X0hl              <= instB_Dhl;
      br_sel_X0hl           <= br_sel_Dhl;
      alu0_fn_X0hl          <= alu0_fn_Dhl;
      alu1_fn_X0hl          <= alu1_fn_Dhl;
      muldivreq_val_X0hl    <= muldivreq_val_Dhl;
      muldivreq_msg_fn_X0hl <= muldivreq_msg_fn_Dhl;
      muldiv_mux_sel_X0hl   <= muldiv_mux_sel_Dhl;
      execute_mux_sel_X0hl  <= execute_mux_sel_Dhl;
      is_load_X0hl          <= is_load_Dhl;
      is_muldiv_X0hl        <= muldivreq_val_Dhl;
      dmemreq_msg_rw_X0hl   <= dmemreq_msg_rw_Dhl;
      dmemreq_msg_len_X0hl  <= dmemreq_msg_len_Dhl;
      dmemreq_val_X0hl      <= dmemreq_val_Dhl;
      dmemresp_mux_sel_X0hl <= dmemresp_mux_sel_Dhl;
      memex_mux_sel_X0hl    <= memex_mux_sel_Dhl;
      rf0_wen_X0hl          <= rf0_wen_Dhl;
      rf0_waddr_X0hl        <= rf0_waddr_Dhl;
      rf1_wen_X0hl          <= rf1_wen_Dhl;
      rf1_waddr_X0hl        <= rf1_waddr_Dhl;
      csr_wen_X0hl          <= csr_wen_Dhl;
      csr_addr_X0hl         <= csr_addr_Dhl;
      instA_was_ir0_X0hl    <= !steer_inst0_to_B_Dhl;

      bubble_X0hl           <= bubble_next_Dhl;
    end

  end

  //----------------------------------------------------------------------
  // Execute Stage
  //----------------------------------------------------------------------

  // Is the current stage valid?

  wire inst_val_X0hl = ( !bubble_X0hl && !squash_X0hl );

  // Muldiv request

  assign muldivreq_val = muldivreq_val_Dhl && inst_val_Dhl && (!bubble_next_Dhl);
  assign muldivresp_rdy = 1'b1;
  assign muldiv_stall_mult1 = stall_X1hl;

  // Only send a valid dmem request if not stalled

  assign dmemreq_msg_rw  = dmemreq_msg_rw_X0hl;
  assign dmemreq_msg_len = dmemreq_msg_len_X0hl;
  assign dmemreq_val     = ( inst_val_X0hl && !stall_X1hl && dmemreq_val_X0hl );

  // Resolve Branch

  wire bne_taken_X0hl  = ( ( br_sel_X0hl == br_bne ) && branch_cond_ne_X0hl );
  wire beq_taken_X0hl  = ( ( br_sel_X0hl == br_beq ) && branch_cond_eq_X0hl );
  wire blt_taken_X0hl  = ( ( br_sel_X0hl == br_blt ) && branch_cond_lt_X0hl );
  wire bltu_taken_X0hl = ( ( br_sel_X0hl == br_bltu) && branch_cond_ltu_X0hl);
  wire bge_taken_X0hl  = ( ( br_sel_X0hl == br_bge ) && branch_cond_ge_X0hl );
  wire bgeu_taken_X0hl = ( ( br_sel_X0hl == br_bgeu) && branch_cond_geu_X0hl);


  wire any_br_taken_X0hl
    = ( beq_taken_X0hl
   ||   bne_taken_X0hl
   ||   blt_taken_X0hl
   ||   bltu_taken_X0hl
   ||   bge_taken_X0hl
   ||   bgeu_taken_X0hl );

  wire brj_taken_X0hl = ( inst_val_X0hl && any_br_taken_X0hl );

  // Squash Signals

  // If a branch is taken, Pipeline A is always squashed for the NEXT cycle (affecting F and D).
  // But wait, what about the instruction CURRENTLY in Pipeline B alongside the branch in Pipeline A?
  
  // Scenario 1: ir0 went to B (ALU), ir1 went to A (Branch). 
  //   - Pipeline A branch resolves. Since ir0 is before ir1, ir0 in Pipeline B is VALID.
  //   - We DO NOT squash Pipeline B's current instruction.
  
  // Scenario 2: ir0 went to A (Branch), ir1 went to B (ALU).
  //   - Pipeline A branch resolves. Since ir1 is after ir0, ir1 in Pipeline B is INVALID (wrong path).
  //   - We MUST squash Pipeline B's current instruction.
  
  wire squash_B_X0hl = brj_taken_X0hl && instA_was_ir0_X0hl;

  // The pipeline register itself handles F and D stalls.
  // squash_X0hl is a dummy signal here because if X0 squashes F/D, F/D send bubbles into X0 next cycle.
  wire squash_X0hl = 1'b0;

  // We actually need to clear out Pipeline B's write enables if it gets squashed!
  wire rfB_wen_X0hl_real = rf1_wen_X0hl && !squash_B_X0hl;

  // Stall in X if muldiv reponse is not valid and there was a valid request

  wire stall_muldiv_X0hl = 1'b0; //( muldivreq_val_X0hl && inst_val_X0hl && !muldivresp_val );

  // Stall in X if imem is not ready

  wire stall_imem_X0hl = !imemreq0_rdy || !imemreq1_rdy;

  // Stall in X if dmem is not ready and there was a valid request

  wire stall_dmem_X0hl = ( dmemreq_val_X0hl && inst_val_X0hl && !dmemreq_rdy );

  // Aggregate Stall Signal

  assign stall_X0hl = ( stall_X1hl || stall_muldiv_X0hl || stall_imem_X0hl || stall_dmem_X0hl );

  // Next bubble bit

  wire bubble_sel_X0hl  = ( squash_X0hl || stall_X0hl );
  wire bubble_next_X0hl = ( !bubble_sel_X0hl ) ? bubble_X0hl
                       : ( bubble_sel_X0hl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // X1 <- X0
  //----------------------------------------------------------------------

  reg [31:0] ir0_X1hl;
  reg [31:0] ir1_X1hl;
  wire [31:0] irA_X1hl = ir0_X1hl;
  wire [31:0] irB_X1hl = ir1_X1hl;
  reg        is_load_X1hl;
  reg        is_muldiv_X1hl;
  reg        dmemreq_val_X1hl;
  reg        execute_mux_sel_X1hl;
  reg        muldiv_mux_sel_X1hl;
  reg        rf0_wen_X1hl;
  reg  [4:0] rf0_waddr_X1hl;
  reg        rf1_wen_X1hl;
  reg  [4:0] rf1_waddr_X1hl;
  reg        csr_wen_X1hl;
  reg  [4:0] csr_addr_X1hl;

  reg        bubble_X1hl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      dmemreq_val_X1hl      <= 1'b0;
      rf0_wen_X1hl          <= 1'b0;
      rf1_wen_X1hl          <= 1'b0;
      csr_wen_X1hl          <= 1'b0;
      dmemresp_queue_val_X1hl <= 1'b0;
      bubble_X1hl           <= 1'b1;
    end
    else if( !stall_X1hl ) begin
      ir0_X1hl              <= ir0_X0hl;
      ir1_X1hl              <= ir1_X0hl;
      is_load_X1hl          <= is_load_X0hl;
      is_muldiv_X1hl        <= is_muldiv_X0hl;
      dmemreq_val_X1hl      <= dmemreq_val;
      dmemresp_mux_sel_X1hl <= dmemresp_mux_sel_X0hl;
      memex_mux_sel_X1hl    <= memex_mux_sel_X0hl;
      execute_mux_sel_X1hl  <= execute_mux_sel_X0hl;
      muldiv_mux_sel_X1hl   <= muldiv_mux_sel_X0hl;
      rf0_wen_X1hl          <= rf0_wen_X0hl;
      rf0_waddr_X1hl        <= rf0_waddr_X0hl;
      rf1_wen_X1hl          <= rfB_wen_X0hl_real;
      rf1_waddr_X1hl        <= rf1_waddr_X0hl;
      csr_wen_X1hl          <= csr_wen_X0hl;
      csr_addr_X1hl         <= csr_addr_X0hl;

      bubble_X1hl           <= bubble_next_X0hl;
    end
  end

  //----------------------------------------------------------------------
  // X1 Stage
  //----------------------------------------------------------------------

  // Is current stage valid?

  wire inst_val_X1hl = ( !bubble_X1hl && !squash_X1hl );

  // Data memory queue control signals

  assign dmemresp_queue_en_X1hl = ( stall_X1hl && dmemresp_val );
  wire   dmemresp_queue_val_next_X1hl
    = stall_X1hl && ( dmemresp_val || dmemresp_queue_val_X1hl );

  // Dummy Squash Signal

  wire squash_X1hl = 1'b0;

  // Stall in X1 if memory response is not returned for a valid request

  wire stall_dmem_X1hl
    = ( !reset && dmemreq_val_X1hl && inst_val_X1hl && !dmemresp_val && !dmemresp_queue_val_X1hl );
  wire stall_imem_X1hl
    = ( !reset && imemreq_val_Fhl && inst_val_Fhl && !imemresp0_val && !imemresp0_queue_val_Fhl )
   || ( !reset && imemreq_val_Fhl && inst_val_Fhl && !imemresp1_val && !imemresp1_queue_val_Fhl );

  // Aggregate Stall Signal

  assign stall_X1hl = ( stall_imem_X1hl || stall_dmem_X1hl );

  // Next bubble bit

  wire bubble_sel_X1hl  = ( squash_X1hl || stall_X1hl );
  wire bubble_next_X1hl = ( !bubble_sel_X1hl ) ? bubble_X1hl
                       : ( bubble_sel_X1hl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // X2 <- X1
  //----------------------------------------------------------------------

  reg [31:0] ir0_X2hl;
  reg [31:0] ir1_X2hl;
  wire [31:0] irA_X2hl = ir0_X2hl;
  wire [31:0] irB_X2hl = ir1_X2hl;
  reg        is_load_X2hl;
  reg        is_muldiv_X2hl;
  reg        rf0_wen_X2hl;
  reg  [4:0] rf0_waddr_X2hl;
  reg        rf1_wen_X2hl;
  reg  [4:0] rf1_waddr_X2hl;
  reg        csr_wen_X2hl;
  reg  [4:0] csr_addr_X2hl;
  reg        execute_mux_sel_X2hl;
  reg        muldiv_mux_sel_X2hl;

  reg        bubble_X2hl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      rf0_wen_X2hl  <= 1'b0;
      rf1_wen_X2hl  <= 1'b0;
      csr_wen_X2hl  <= 1'b0;
      bubble_X2hl   <= 1'b1;
    end
    else if( !stall_X2hl ) begin
      ir0_X2hl              <= ir0_X1hl;
      ir1_X2hl              <= ir1_X1hl;
      is_load_X2hl          <= is_load_X1hl;
      is_muldiv_X2hl        <= is_muldiv_X1hl;
      muldiv_mux_sel_X2hl   <= muldiv_mux_sel_X1hl;
      rf0_wen_X2hl          <= rf0_wen_X1hl;
      rf0_waddr_X2hl        <= rf0_waddr_X1hl;
      rf1_wen_X2hl          <= rf1_wen_X1hl;
      rf1_waddr_X2hl        <= rf1_waddr_X1hl;
      csr_wen_X2hl          <= csr_wen_X1hl;
      csr_addr_X2hl         <= csr_addr_X1hl;
      execute_mux_sel_X2hl  <= execute_mux_sel_X1hl;

      bubble_X2hl           <= bubble_next_X1hl;
    end
    dmemresp_queue_val_X1hl <= dmemresp_queue_val_next_X1hl;
  end
  //----------------------------------------------------------------------
  // X2 Stage
  //----------------------------------------------------------------------

  // Is current stage valid?

  wire inst_val_X2hl = ( !bubble_X2hl && !squash_X2hl );

  // Dummy Squash Signal

  wire squash_X2hl = 1'b0;

  // Dummy Stall Signal

  assign stall_X2hl = 1'b0;

  // Next bubble bit

  wire bubble_sel_X2hl  = ( squash_X2hl || stall_X2hl );
  wire bubble_next_X2hl = ( !bubble_sel_X2hl ) ? bubble_X2hl
                       : ( bubble_sel_X2hl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // X3 <- X2
  //----------------------------------------------------------------------

  reg [31:0] ir0_X3hl;
  reg [31:0] ir1_X3hl;
  wire [31:0] irA_X3hl = ir0_X3hl;
  wire [31:0] irB_X3hl = ir1_X3hl;
  reg        is_load_X3hl;
  reg        is_muldiv_X3hl;
  reg        rf0_wen_X3hl;
  reg  [4:0] rf0_waddr_X3hl;
  reg        rf1_wen_X3hl;
  reg  [4:0] rf1_waddr_X3hl;
  reg        csr_wen_X3hl;
  reg  [4:0] csr_addr_X3hl;

  reg        bubble_X3hl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      rf0_wen_X3hl  <= 1'b0;
      rf1_wen_X3hl  <= 1'b0;
      csr_wen_X3hl  <= 1'b0;
      bubble_X3hl   <= 1'b1;
    end
    else if( !stall_X3hl ) begin
      ir0_X3hl              <= ir0_X2hl;
      ir1_X3hl              <= ir1_X2hl;
      is_load_X3hl          <= is_load_X2hl;
      is_muldiv_X3hl        <= is_muldiv_X2hl;
      muldiv_mux_sel_X3hl   <= muldiv_mux_sel_X2hl;
      rf0_wen_X3hl          <= rf0_wen_X2hl;
      rf0_waddr_X3hl        <= rf0_waddr_X2hl;
      rf1_wen_X3hl          <= rf1_wen_X2hl;
      rf1_waddr_X3hl        <= rf1_waddr_X2hl;
      csr_wen_X3hl          <= csr_wen_X2hl;
      csr_addr_X3hl         <= csr_addr_X2hl;
      execute_mux_sel_X3hl  <= execute_mux_sel_X2hl;

      bubble_X3hl           <= bubble_next_X2hl;
    end
  end

  //----------------------------------------------------------------------
  // X3 Stage
  //----------------------------------------------------------------------

  // Is current stage valid?

  wire inst_val_X3hl = ( !bubble_X3hl && !squash_X3hl );

  // Dummy Squash Signal

  wire squash_X3hl = 1'b0;

  // Dummy Stall Signal

  assign stall_X3hl = 1'b0;

  // Next bubble bit

  wire bubble_sel_X3hl  = ( squash_X3hl || stall_X3hl );
  wire bubble_next_X3hl = ( !bubble_sel_X3hl ) ? bubble_X3hl
                       : ( bubble_sel_X3hl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // W <- X3
  //----------------------------------------------------------------------

  reg [31:0] ir0_Whl;
  reg [31:0] ir1_Whl;
  wire [31:0] irA_Whl = ir0_Whl;
  wire [31:0] irB_Whl = ir1_Whl;
  reg        rf0_wen_Whl;
  reg  [4:0] rf0_waddr_Whl;
  reg        rf1_wen_Whl;
  reg  [4:0] rf1_waddr_Whl;
  reg        csr_wen_Whl;
  reg  [4:0] csr_addr_Whl;

  reg        bubble_Whl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      rf0_wen_Whl  <= 1'b0;
      rf1_wen_Whl  <= 1'b0;
      csr_wen_Whl  <= 1'b0;
      bubble_Whl   <= 1'b1;
    end
    else if( !stall_Whl ) begin
      ir0_Whl          <= ir0_X3hl;
      ir1_Whl          <= ir1_X3hl;
      rf0_wen_Whl      <= rf0_wen_X3hl;
      rf0_waddr_Whl    <= rf0_waddr_X3hl;
      rf1_wen_Whl      <= rf1_wen_X3hl;
      rf1_waddr_Whl    <= rf1_waddr_X3hl;
      csr_wen_Whl      <= csr_wen_X3hl;
      csr_addr_Whl     <= csr_addr_X3hl;

      bubble_Whl       <= bubble_next_X3hl;
    end
  end

  //----------------------------------------------------------------------
  // Writeback Stage
  //----------------------------------------------------------------------

  // Is current stage valid?
  wire squash_Whl = 1'b0;
  wire stall_Whl  = 1'b0;
  wire inst_val_Whl = ( !bubble_Whl && !squash_Whl );

  // Writeback Stage Logic is already driven above by gated rfA/rfB signals.

  //----------------------------------------------------------------------
  // Debug registers for instruction disassembly
  //----------------------------------------------------------------------

  reg [31:0] irA_debug;
  reg [31:0] irB_debug;
  reg        inst_val_debug;

  always @ ( posedge clk ) begin
    irA_debug      <= irA_Whl;
    irB_debug      <= irB_Whl;
    inst_val_debug <= inst_val_Whl;
  end

  //----------------------------------------------------------------------
  // CSR register
  //----------------------------------------------------------------------

  reg         csr_stats;

  always @ ( posedge clk ) begin
    if ( csr_wen_Whl && inst_val_Whl ) begin
      case ( csr_addr_Whl )
        12'd10 : csr_stats  <= proc2csr_data_Whl[0];
        12'd21 : csr_status <= proc2csr_data_Whl;
      endcase
    end
  end

//========================================================================
// Disassemble instructions
//========================================================================

  `ifndef SYNTHESIS

  riscv_InstMsgDisasm inst0_msg_disasm_D
  (
    .msg ( ir0_Dhl )
  );

  riscv_InstMsgDisasm instA_msg_disasm_X0
  (
    .msg ( irA_X0hl )
  );

  riscv_InstMsgDisasm instA_msg_disasm_X1
  (
    .msg ( irA_X1hl )
  );

  riscv_InstMsgDisasm instA_msg_disasm_X2
  (
    .msg ( irA_X2hl )
  );

  riscv_InstMsgDisasm instA_msg_disasm_X3
  (
    .msg ( irA_X3hl )
  );

  riscv_InstMsgDisasm instA_msg_disasm_W
  (
    .msg ( irA_Whl )
  );

  riscv_InstMsgDisasm instA_msg_disasm_debug
  (
    .msg ( irA_debug )
  );

  riscv_InstMsgDisasm inst1_msg_disasm_D
  (
    .msg ( ir1_Dhl )
  );

  riscv_InstMsgDisasm instB_msg_disasm_X0
  (
    .msg ( irB_X0hl )
  );

  riscv_InstMsgDisasm instB_msg_disasm_X1
  (
    .msg ( irB_X1hl )
  );

  riscv_InstMsgDisasm instB_msg_disasm_X2
  (
    .msg ( irB_X2hl )
  );

  riscv_InstMsgDisasm instB_msg_disasm_X3
  (
    .msg ( irB_X3hl )
  );

  riscv_InstMsgDisasm instB_msg_disasm_W
  (
    .msg ( irB_Whl )
  );

  riscv_InstMsgDisasm instB_msg_disasm_debug
  (
    .msg ( irB_debug )
  );

  `endif

//========================================================================
// Assertions
//========================================================================
// Detect illegal instructions and terminate the simulation if multiple
// illegal instructions are detected in succession.

  `ifndef SYNTHESIS

  reg overload = 1'b0;

  always @ ( posedge clk ) begin
    if (( (steering_mux_sel_Dhl == 1'b0) ? !cs0[`RISCV_INST_MSG_INST_VAL] : !cs1[`RISCV_INST_MSG_INST_VAL] ) && !reset ) begin
      $display(" RTL-ERROR : %m : Illegal instruction!");

      if ( overload == 1'b1 ) begin
        $finish;
      end

      overload = 1'b1;
    end
    else begin
      overload = 1'b0;
    end
  end

  `endif

//========================================================================
// Stats
//========================================================================

  `ifndef SYNTHESIS

  reg [31:0] num_inst    = 32'b0;
  reg [31:0] num_cycles  = 32'b0;
  reg        stats_en    = 1'b0; // Used for enabling stats on asm tests

  always @( posedge clk ) begin
    if ( !reset ) begin

      // Count cycles if stats are enabled

      if ( stats_en || csr_stats ) begin
        num_cycles = num_cycles + 1;

        // Count instructions for every cycle not squashed or stalled
        // In superscalar, we count how many valid instructions issued this cycle.
        if ( !stall_Dhl && !squash_Dhl) begin
            if (ir0_valid_issue && ir1_valid_issue) begin
                num_inst = num_inst + 2;
            end else if (ir0_valid_issue || ir1_valid_issue) begin // Handled squash cases where one might be valid but killed
                num_inst = num_inst + 1;
            end
        end

      end

    end
  end

  `endif

endmodule

`endif

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
