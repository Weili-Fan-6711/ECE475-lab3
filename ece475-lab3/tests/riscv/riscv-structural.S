//=========================================================================
// riscv-structural.S
//=========================================================================

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        //-----------------------------------------------------------------
        // Two memory ops in one fetch pair
        //-----------------------------------------------------------------
        // Only one non-ALU instruction can issue in a cycle, so the second
        // load must wait and still execute correctly.

        la    x2, tdata
        lw    x4, 0(x2)
        lw    x5, 4(x2)
        TEST_CHECK_EQ( x4, 0x00000011 )
        TEST_CHECK_EQ( x5, 0x00000022 )

        //-----------------------------------------------------------------
        // Load followed by mul in the same fetch pair
        //-----------------------------------------------------------------
        // The load and mul both need the single non-ALU issue slot.

        li    x6, 3
        li    x7, 7
        lw    x8, 8(x2)
        mul   x9, x6, x7
        TEST_CHECK_EQ( x8, 0x00000033 )
        TEST_CHECK_EQ( x9, 21 )

        //-----------------------------------------------------------------
        // Two mul/div ops in one fetch pair
        //-----------------------------------------------------------------
        // Mul/div instructions also serialize on the shared non-ALU path.

        li    x10, 100
        li    x11, 9
        mul   x12, x6, x11
        div   x13, x10, x11
        TEST_CHECK_EQ( x12, 27 )
        TEST_CHECK_EQ( x13, 11 )

        TEST_RISCV_END

        .data
        .align 4
tdata:
        .word 0x00000011
        .word 0x00000022
        .word 0x00000033
