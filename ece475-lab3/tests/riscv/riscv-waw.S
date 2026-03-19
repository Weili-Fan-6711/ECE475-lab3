//=========================================================================
// riscv-waw.S
//=========================================================================

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        //-----------------------------------------------------------------
        // Back-to-back ALU writes to the same destination
        //-----------------------------------------------------------------
        // Both instructions target x4, so the second must not issue in the
        // same cycle as the first. The architectural result must still be
        // the later write.

        li    x2, 5
        li    x3, 9
        addi  x4, x2, 7
        addi  x4, x3, 4
        TEST_CHECK_EQ( x4, 13 )

        //-----------------------------------------------------------------
        // WAW plus same-register source dependency
        //-----------------------------------------------------------------
        // The second instruction overwrites x5 but also needs the value
        // produced by the first one, so replay must preserve program order.

        lui   x5, 0x8
        addi  x5, x5, -256
        TEST_CHECK_EQ( x5, 0x00007f00 )

        //-----------------------------------------------------------------
        // Long-latency producer overwritten by a later ALU write
        //-----------------------------------------------------------------
        // The load to x6 must issue first, and the later addi must still be
        // the final architectural write to x6.

        la    x7, tdata
        lw    x6, 0(x7)
        addi  x6, x0, 99
        TEST_CHECK_EQ( x6, 99 )

        TEST_RISCV_END

        .data
        .align 4
tdata:
        .word 0x12345678
