#![no_std]
#![no_main]
#![feature(riscv_ext_intrinsics)]
#![feature(core_intrinsics)]

use core::{arch::{asm, global_asm}, panic::PanicInfo, ptr};
use mik32v2_pac::{spi_0::delay, Peripherals};
use riscv::{self as _};

fn scale(percent: u32) -> u32 {
    assert!(percent <= 100);
    return 40 * percent + 4000
}

fn timer_init(p: &Peripherals) {
    
    p.pm.clk_apb_p_set().modify({|_, w| w
        .timer32_1().enable()
    });

    p.pad_config.pad0_cfg().modify(|_, w| w.port_0_0().func3_interface_or_timer());
    p.timer32_1.ch1_cntr().modify(|_, w| w
        .mode().pwm()
        .en().set_bit()
    );
    p.timer32_1.top().modify(|_, w| unsafe { w.tim_top().bits(8000) });
    p.timer32_1.enable().modify(|_, w| w.tim_clr().set_bit());
    p.timer32_1.enable().modify(|_, w| w.tim_en().enable());
}

fn uart_init(p: &Peripherals) {
    p.pm.clk_apb_p_set().modify({|_, w| w
        .uart_0().enable()
    });

    p.pad_config.pad0_cfg().modify(|_, w| w.port_0_6().func2_interface());
    p.pad_config.pad0_cfg().modify(|_, w| w.port_0_5().func2_interface());
    p.usart_0.divider().modify(|_, w| unsafe { w.brr().bits(0xd05) });
    p.usart_0.control1().modify(|_, w| w
        .te().enable()
        .re().enable()
        .ue().enable()
    );

    while p.usart_0.flags().read().teack().bit_is_clear() {};
}

fn uart_send_bytes(p: &Peripherals, bytes: &[u8]) {
    for &byte in bytes {
        p.usart_0.txdata().modify(|_, w| unsafe { w.tdr().bits(byte.into()) });
        while p.usart_0.flags().read().tc().bit_is_clear() {};
    }
    p.usart_0.txdata().modify(|_, w| unsafe { w.tdr().bits(b'\n'.into()) });
    while p.usart_0.flags().read().tc().bit_is_clear() {};
}

fn uart_recieve_bytes(p: &Peripherals, buf: &mut [u8]) -> usize {
    let mut i = 0;

    loop {
        while p.usart_0.flags().read().rxne().bit_is_clear() {}
        let byte = p.usart_0.rxdata().read().rdr().bits() as u8;

        if byte == b'\n' {
            break;
        }

        if i < buf.len() {
            buf[i] = byte;
            i += 1;
        }
    }

    i
}

fn bytes_to_number(arr: &[u8]) -> u32 {
    let mut result = 0;
    for &digit in arr {
        if digit < b'0' || digit > b'9' { 
            continue; // Пропускаем некорректные символы
        }
        result = result * 10 + (digit - b'0') as u32;
    }
    result
}

#[unsafe(no_mangle)]
pub extern "C" fn __start_rust() -> ! {

    let p = Peripherals::take().unwrap();

    timer_init(&p);
    uart_init(&p);

    let bytes = "start".as_bytes();
    uart_send_bytes(&p, bytes);

    let mut pulse_width = scale(0);
    p.timer32_1.ch1_ocr().modify(|_, w| unsafe { w.ocr().bits(pulse_width) }); 

    let mut buf = [0; 10];

    loop {
        let len = uart_recieve_bytes(&p, &mut buf); 
        uart_send_bytes(&p, &buf[..len]);
        let num = bytes_to_number(&buf[..len]);

        let delay = 1000;

        let new_pulse_width = scale(num);
        if new_pulse_width > pulse_width {
            while pulse_width < new_pulse_width {
                pulse_width += 10;
                p.timer32_1.ch1_ocr().modify(|_, w| unsafe { w.ocr().bits(pulse_width) }); 
                riscv::asm::delay(delay);
            }
            riscv::asm::delay(delay);
            pulse_width = new_pulse_width;
            p.timer32_1.ch1_ocr().modify(|_, w| unsafe { w.ocr().bits(pulse_width) }); 
        } else {
            while pulse_width > new_pulse_width {
                pulse_width -= 10;
                p.timer32_1.ch1_ocr().modify(|_, w| unsafe { w.ocr().bits(pulse_width) }); 
                riscv::asm::delay(delay);
            }
            riscv::asm::delay(delay);
            pulse_width = new_pulse_width;
            p.timer32_1.ch1_ocr().modify(|_, w| unsafe { w.ocr().bits(pulse_width) }); 
        }
    }
}


#[unsafe(export_name = "trap_handler")]
fn trap() {
    loop {
        
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}



global_asm!(
    r#"
    #define EXCEPTION_STACK_SPACE 128
    #define EXCEPTION_SAVED_REGISTERS 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31

    .globl _start, __start_rust
    .weak SmallSystemInit, SystemInit

    .globl trap_entry
    .globl trap_handler
    .globl _start_trap

    .altmacro
    .macro memcpy src_beg, src_end, dst, tmp_reg
        j    memcpy_2\@
    memcpy_1\@:
        lw   \tmp_reg, (\src_beg)
        sw   \tmp_reg, (\dst)
        add  \src_beg, \src_beg, 4
        add  \dst, \dst, 4
    memcpy_2\@:
        bltu \src_beg, \src_end, memcpy_1\@
    .endm

    .macro memset dst_beg, dst_end, val_reg
        j    memset_2\@
    memset_1\@:
        sw   \val_reg, (\dst_beg)
        add  \dst_beg, \dst_beg, 4
    memset_2\@:
        bltu \dst_beg, \dst_end, memset_1\@
    .endm
    
    .macro la_abs reg, address
        .option push
        .option norelax
        lui \reg, %hi(\address)
        addi \reg, \reg, %lo(\address)
        .option pop
    .endm

    .macro jalr_abs return_reg, address
        .option push
        .option norelax
        lui \return_reg, %hi(\address)
        jalr \return_reg, %lo(\address)(\return_reg)
        .option pop
    .endm

    .section .init, "ax"

    _start:

        li t0, 128000
        start_loop_delay:
        nop
        addi t0, t0, -1
        bnez t0, start_loop_delay

        la_abs  sp, __sstack
        la_abs  gp, _gp

        la_abs  a1, __sidata
        la_abs  a2, __eidata
        la_abs  a3, __sdata
        memcpy  a1, a2, a3, t0

        la_abs  a1, __siram_text
        la_abs  a2, __eiram_text
        la_abs  a3, __sram_text
        memcpy  a1, a2, a3, t0

        la_abs  a1, __sbss
        la_abs  a2, __ebss
        memset a1, a2, zero

        jalr_abs ra, SmallSystemInit
        jalr_abs ra, SystemInit
        jalr_abs ra, __start_rust
    1:  wfi
        j 1b

    SmallSystemInit:
    SystemInit:
        ret

    .section .init.trap, "ax"
    trap_entry:
        j _start_trap

    .section .trap, "ax"
    _start_trap:
        addi    sp, sp, -(128)
        .irp index, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
            sw      x\index, 4*index(sp)
        .endr

        la      ra, trap_handler
        jalr    ra

        .irp index, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
            lw      x\index, 4*index(sp)
        .endr

        addi sp, sp, 128
        mret

    "#
);
