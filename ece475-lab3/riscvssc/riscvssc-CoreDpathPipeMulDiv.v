//========================================================================
// Functional Pipelined Mul/Div Unit
//========================================================================

`ifndef RISCV_PIPE_MULDIV_ITERATIVE_V
`define RISCV_PIPE_MULDIV_ITERATIVE_V

`include "imuldiv-MulDivReqMsg.v"

module riscv_CoreDpathPipeMulDiv
(
  input         clk,
  input         reset,

  input   [2:0] muldivreq_msg_fn,
  input  [31:0] muldivreq_msg_a,
  input  [31:0] muldivreq_msg_b,
  input         muldivreq_val,
  output        muldivreq_rdy,

  output [63:0] muldivresp_msg_result,
  output        muldivresp_val,
  input         muldivresp_rdy,
  input         stall_Xhl,
  input         stall_Mhl,
  input         stall_X2hl,
  input         stall_X3hl
);

  wire stall = val4_reg && !muldivresp_rdy;

  assign muldivreq_rdy = !stall;
  wire   muldivreq_go  = muldivreq_val && muldivreq_rdy;

  reg  [2:0] fn_reg;
  reg [31:0] a_reg;
  reg [31:0] b_reg;
  reg [63:0] result1_reg;
  reg [63:0] result2_reg;
  reg [63:0] result3_reg;
  reg [63:0] result4_reg;

  reg        val0_reg;
  reg        val1_reg;
  reg        val2_reg;
  reg        val3_reg;
  reg        val4_reg;

  wire [2:0]  fn_stage0 = muldivreq_go ? muldivreq_msg_fn : fn_reg;
  wire [31:0] a_stage0  = muldivreq_go ? muldivreq_msg_a  : a_reg;
  wire [31:0] b_stage0  = muldivreq_go ? muldivreq_msg_b  : b_reg;

  wire        sign = ( a_stage0[31] ^ b_stage0[31] );

  wire [31:0] a_unsign = ( a_stage0[31] == 1'b1 ) ? ( ~a_stage0 + 1'b1 )
                                                    :   a_stage0;
  wire [31:0] b_unsign = ( b_stage0[31] == 1'b1 ) ? ( ~b_stage0 + 1'b1 )
                                                    :   b_stage0;

  wire        divisor_zero = ( b_stage0 == 32'b0 );

  wire [31:0] quotientu  = divisor_zero ? 32'hffffffff : ( a_stage0 / b_stage0 );
  wire [31:0] remainderu = divisor_zero ? a_stage0     : ( a_stage0 % b_stage0 );
  wire [63:0] productu   = a_stage0 * b_stage0;
  wire [63:0] productsu_raw = a_unsign * b_stage0;

  wire [63:0] product_raw   = a_unsign * b_unsign;
  wire [31:0] quotient_raw  = divisor_zero ? 32'b0 : ( a_unsign / b_unsign );
  wire [31:0] remainder_raw = divisor_zero ? 32'b0 : ( a_unsign % b_unsign );

  wire [63:0] product = sign ? ( ~product_raw + 1'b1 )
                             :   product_raw;

  wire [63:0] productsu = a_stage0[31] ? ( ~productsu_raw + 1'b1 )
                                       :   productsu_raw;

  wire [31:0] quotient = divisor_zero ? 32'hffffffff
                         : sign ? ( ~quotient_raw + 1'b1 )
                                :   quotient_raw;

  wire [31:0] remainder = divisor_zero ? a_stage0
                          : a_stage0[31] ? ( ~remainder_raw + 1'b1 )
                                         :   remainder_raw;

  wire [63:0] result0
    = ( fn_stage0 == `IMULDIV_MULDIVREQ_MSG_FUNC_MUL   ) ? product
    : ( fn_stage0 == `IMULDIV_MULDIVREQ_MSG_FUNC_MULU  ) ? productu
    : ( fn_stage0 == `IMULDIV_MULDIVREQ_MSG_FUNC_MULSU ) ? productsu
    : ( fn_stage0 == `IMULDIV_MULDIVREQ_MSG_FUNC_DIV   ) ? { remainder,  quotient  }
    : ( fn_stage0 == `IMULDIV_MULDIVREQ_MSG_FUNC_DIVU  ) ? { remainderu, quotientu }
    : ( fn_stage0 == `IMULDIV_MULDIVREQ_MSG_FUNC_REM   ) ? { remainder,  quotient  }
    : ( fn_stage0 == `IMULDIV_MULDIVREQ_MSG_FUNC_REMU  ) ? { remainderu, quotientu }
    :                                                      64'bx;

  wire val1_next = muldivreq_go;
  wire val2_next = stall_Mhl  ? val2_reg : val1_reg;
  wire val3_next = stall_X2hl ? val3_reg : val2_reg;
  wire val4_next = stall_X3hl ? val4_reg : val3_reg;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      fn_reg      <= 3'b0;
      a_reg       <= 32'b0;
      b_reg       <= 32'b0;
      result1_reg <= 64'b0;
      result2_reg <= 64'b0;
      result3_reg <= 64'b0;
      result4_reg <= 64'b0;
      val0_reg    <= 1'b0;
      val1_reg    <= 1'b0;
      val2_reg    <= 1'b0;
      val3_reg    <= 1'b0;
      val4_reg    <= 1'b0;
    end
    else begin
      if ( muldivreq_go ) begin
        fn_reg   <= muldivreq_msg_fn;
        a_reg    <= muldivreq_msg_a;
        b_reg    <= muldivreq_msg_b;
        val0_reg <= 1'b1;
      end
      else if ( !stall_Xhl ) begin
        val0_reg <= 1'b0;
      end

      if ( !stall_Mhl ) begin
        result1_reg <= result0;
        val1_reg    <= val1_next;
      end

      if ( !stall_X2hl ) begin
        result2_reg <= result1_reg;
        val2_reg    <= val2_next;
      end

      if ( !stall_X3hl ) begin
        result3_reg <= result2_reg;
        val3_reg    <= val3_next;
      end

      if ( !stall ) begin
        result4_reg <= result3_reg;
        val4_reg    <= val4_next;
      end
    end
  end

  assign muldivresp_msg_result = result4_reg;
  assign muldivresp_val        = val4_reg;

endmodule

`endif
