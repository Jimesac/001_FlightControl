/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 2014 - 2022 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* - Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File      : HPM6xxx_Startup.s
Purpose   : Startup and exception handlers for HPM6xxx devices.
*/

#include "hpm_csr_regs.h"
#include "vectors.h"

/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/
#define NUM_CALLER_SAVE_REGISTERS (1+3+8+4)     // save caller integer registers ra, t0-t2, a0-a7, t3-t6

/*********************************************************************
*
*       Macros
*
**********************************************************************
*/
//
// Declare a regular function.
// Functions from the startup are placed in the init section.
//
.macro START_FUNC Name
        .section .init.\Name, "ax"
        .global \Name
#if __riscv_compressed
        .balign 2
#else
        .balign 4
#endif
        .type \Name, function
\Name:
.endm

//
// Declare a regular weak function, such as default C/C++ library helpers
//
.macro WEAK_FUNC Name
        .section .text.\Name, "ax", %progbits
        .weak \Name
        .type \Name,@function
#if __riscv_compressed
        .balign 2
#else
        .balign 4
#endif
\Name:
.endm

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/
/*********************************************************************
*
*       Reset_Handler
*
*  Function description
*    Exception handler for reset.
*/
START_FUNC Reset_Handler
        .option push
        .option norelax
        lui     gp,     %hi(__global_pointer$)
        addi    gp, gp, %lo(__global_pointer$)
        lui     tp,     %hi(__thread_pointer$)
        addi    tp, tp, %lo(__thread_pointer$)
        lui     t0,     %hi(__stack_end__)
        addi    sp, t0, %lo(__stack_end__)
        .option pop
        la      a0, irq_handler_trap
        csrw    mtvec, a0
        csrw    mcause, zero

#ifdef __riscv_flen
        // Enable FPU
        li      t0, CSR_MSTATUS_FS_MASK
        csrrs   t0, mstatus, t0

        // Initialize FCSR
        fscsr   zero
#endif

#ifdef INIT_EXT_RAM_FOR_DATA
        la      t0, _stack_in_dlm
        mv      sp, t0
        call    _init_ext_ram
#endif

#ifdef CONFIG_NOT_ENABLE_ICACHE
        call    l1c_ic_disable
#else
        call    l1c_ic_enable
#endif
#ifdef CONFIG_NOT_ENABLE_DCACHE
        call    l1c_dc_invalidate_all
        call    l1c_dc_disable
#else
        call    l1c_dc_enable
        call    l1c_dc_invalidate_all
#endif

#ifndef __NO_SYSTEM_INIT
        //
        // Call system_init
        //
        //call    system_init
        //
        // Clean up
        //
        call    _clean_up
#endif

#ifndef CONFIG_FREERTOS
  #define HANDLER_TRAP irq_handler_trap
#else
  #define HANDLER_TRAP freertos_risc_v_trap_handler
#endif

#ifndef USE_NONVECTOR_MODE
        // Initial machine trap-vector Base
        la      t0, __vector_table
        csrw    mtvec, t0

        // Enable vectored external PLIC interrupt
        csrsi   CSR_MMISC_CTL, 2
#else
        // Initial machine trap-vector Base
        la      t0, HANDLER_TRAP
        csrw    mtvec, t0
#endif

        //
        // Jump to program start
        //
        la      t1, _start
        jalr    t1

/*********************************************************************
*
*       HELPER functions
*
**********************************************************************
*/
WEAK_FUNC irq_handler_trap
        addi    sp, sp, -NUM_CALLER_SAVE_REGISTERS*4
        sw      ra, 0*4(sp)
        sw      t0, 1*4(sp)
        sw      t1, 2*4(sp)
        sw      t2, 3*4(sp)
        sw      a0, 4*4(sp)
        sw      a1, 5*4(sp)
        sw      a2, 6*4(sp)
        sw      a3, 7*4(sp)
        sw      a4, 8*4(sp)
        sw      a5, 9*4(sp)
#ifndef __riscv_abi_rve
        sw      a6, 10*4(sp)
        sw      a7, 11*4(sp)
        sw      t3, 12*4(sp)
        sw      t4, 13*4(sp)
        sw      t5, 14*4(sp)
        sw      t6, 15*4(sp)
#endif
        //
        csrr    a0, mcause
        csrr    a1, mepc
        //
        // Call handle_trap
        //
        la      t1, handle_trap
        jalr    t1
        csrw    mepc, a0
        //
        lw      ra, 0*4(sp)
        lw      t0, 1*4(sp)
        lw      t1, 2*4(sp)
        lw      t2, 3*4(sp)
        lw      a0, 4*4(sp)
        lw      a1, 5*4(sp)
        lw      a2, 6*4(sp)
        lw      a3, 7*4(sp)
        lw      a4, 8*4(sp)
        lw      a5, 9*4(sp)
#ifndef __riscv_abi_rve
        lw      a6, 10*4(sp)
        lw      a7, 11*4(sp)
        lw      t3, 12*4(sp)
        lw      t4, 13*4(sp)
        lw      t5, 14*4(sp)
        lw      t6, 15*4(sp)
#endif
        //
        addi    sp, sp, NUM_CALLER_SAVE_REGISTERS*4
        mret

WEAK_FUNC handle_trap
        j       .

WEAK_FUNC system_init
        ret

WEAK_FUNC _fini
        ret

/*************************** End of file ****************************/
