MEMORY
{
    RAM : ORIGIN = 0x02000000, LENGTH = 16K
    FLASH : ORIGIN = 0x01000000, LENGTH = 8K
}

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);
REGION_ALIAS("REGION_RAM", RAM);

/* # Developer notes

- Symbols that start with a double underscore (__) are considered "private"

- Symbols that start with a single underscore (_) are considered "semi-public"; they can be
  overridden in a user linker script, but should not be referred from user code (e.g. `extern "C" {
  static mut _heap_size }`).

- `EXTERN` forces the linker to keep a symbol in the final binary. We use this to make sure a
  symbol is not dropped if it appears in or near the front of the linker arguments and "it's not
  needed" by any of the preceding objects (linker arguments)

- `PROVIDE` is used to provide default values that can be overridden by a user linker script

- In this linker script, you may find symbols that look like `${...}` (e.g., `4`).
  These are wildcards used by the `build.rs` script to adapt to different target particularities.
  Check `build.rs` for more details about these symbols.

- On alignment: it's important for correctness that the VMA boundaries of both .bss and .data *and*
  the LMA of .data are all `4`-byte aligned. These alignments are assumed by the RAM
  initialization routine. There's also a second benefit: `4`-byte aligned boundaries
  means that you won't see "Address (..) is out of bounds" in the disassembly produced by `objdump`.
*/

PROVIDE(_stext = ORIGIN(REGION_TEXT));
PROVIDE(_stack_start = ORIGIN(REGION_STACK) + LENGTH(REGION_STACK));
PROVIDE(_max_hart_id = 0);
PROVIDE(_hart_stack_size = 1K);
PROVIDE(_heap_size = 0);

/** TRAP ENTRY POINTS **/

/* Default trap entry point. The riscv-rt crate provides a weak alias of this function,
   which saves caller saved registers, calls _start_trap_rust, restores caller saved registers
   and then returns. Users can override this alias by defining the symbol themselves */
EXTERN(_start_trap);

/* Default interrupt trap entry point. When vectored trap mode is enabled,
   the riscv-rt crate provides an implementation of this function, which saves caller saved
   registers, calls the the DefaultHandler ISR, restores caller saved registers and returns. */
PROVIDE(_start_DefaultHandler_trap = _start_trap);

SECTIONS
{
  .text.dummy (NOLOAD) :
  {
    /* This section is intended to make _stext address work */
    . = ABSOLUTE(_stext);
  } > REGION_TEXT
  
  .text _stext :
  {
    __stext = .;

    /* Put reset handler first in .text section so it ends up as the entry */
    /* point of the program. */
    KEEP(*(.init));
    . = ALIGN(4);
    . = ORIGIN(REGION_TEXT) + 0xC0;
    KEEP(*(.init.trap));
    . = ALIGN(4);
    *(.trap);
    *(.trap.rust);
    *(.text.abort);
    *(.text .text.*);

    . = ALIGN(4);
    __etext = .;
  } > REGION_TEXT

  .rodata : ALIGN(4)
  {
    *(.srodata .srodata.*);
    *(.rodata .rodata.*);

    /* 4-byte align the end (VMA) of this section.
       This is required by LLD to ensure the LMA of the following .data
       section will have the correct alignment. */
    . = ALIGN(4);
  } > REGION_RODATA

  .data : ALIGN(4)
  {
    . = ALIGN(4);
    __sdata = .;

    /* Must be called __global_pointer$ for linker relaxations to work. */
    PROVIDE(__global_pointer$ = .);
    _gp = .;
    *(.sdata .sdata.* .sdata2 .sdata2.*);
    *(.data .data.*);

  } > REGION_DATA AT > REGION_RODATA

  __sidata = LOADADDR(.data);
  __eidata = LOADADDR(.data) + SIZEOF(.data);
  
  /* Allow sections from user `memory.x` injected using `INSERT AFTER .data` to
   * use the .data loading mechanism by pushing __edata. Note: do not change
   * output region or load region in those user sections! */
  . = ALIGN(4);
  __edata = .;
  
  /* LMA of .data */
  __sidata = LOADADDR(.data);

  .bss (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    __sbss = .;

    *(.sbss .sbss.* .bss .bss.*);
  } > REGION_BSS

  /* Allow sections from user `memory.x` injected using `INSERT AFTER .bss` to
   * use the .bss zeroing mechanism by pushing __ebss. Note: do not change
   * output region or load region in those user sections! */
  . = ALIGN(4);
  __ebss = .;

  .ram_text : ALIGN(4)
  {
    . = ALIGN(4);
    __sram_text = .;
    *(.ram_text)
  } > REGION_RAM AT > REGION_TEXT
  . = ALIGN(4);
  __eram_text = .;

  __siram_text = LOADADDR(.ram_text);
  __eiram_text = LOADADDR(.ram_text) + SIZEOF(.ram_text);

  ASSERT(__eiram_text < ORIGIN(REGION_TEXT) + LENGTH(REGION_TEXT), "REGION_TEXT segment overflows")
  /* ASSERT(__eram_text < ORIGIN(REGION_RAM) + LENGTH(REGION_RAM) - STACK_SIZE, "REGION_RAM section overflows") */


  /* fictitious region that represents the memory available for the heap */
  .heap (NOLOAD) : ALIGN(4)
  {
    __sheap = .;
    . += _heap_size;
    . = ALIGN(4);
    __eheap = .;
  } > REGION_HEAP

  /* fictitious region that represents the memory available for the stack */
  .stack (NOLOAD) :
  {
    __estack = .;
    . = ABSOLUTE(_stack_start);
    __sstack = .;
  } > REGION_STACK

  /* fake output .got section */
  /* Dynamic relocations are unsupported. This section is only used to detect
     relocatable code in the input files and raise an error if relocatable code
     is found */
  .got (INFO) :
  {
    KEEP(*(.got .got.*));
  }
}

/* Do not exceed this mark in the error messages above                                    | */
ASSERT(ORIGIN(REGION_TEXT) % 4 == 0, "
ERROR(riscv-rt): the start of the REGION_TEXT must be 4-byte aligned");

ASSERT(ORIGIN(REGION_RODATA) % 4 == 0, "
ERROR(riscv-rt): the start of the REGION_RODATA must be 4-byte aligned");

ASSERT(ORIGIN(REGION_DATA) % 4 == 0, "
ERROR(riscv-rt): the start of the REGION_DATA must be 4-byte aligned");

ASSERT(ORIGIN(REGION_HEAP) % 4 == 0, "
ERROR(riscv-rt): the start of the REGION_HEAP must be 4-byte aligned");

ASSERT(ORIGIN(REGION_STACK) % 4 == 0, "
ERROR(riscv-rt): the start of the REGION_STACK must be 4-byte aligned");

ASSERT(_stext % 4 == 0, "
ERROR(riscv-rt): `_stext` must be 4-byte aligned");

ASSERT(__sdata % 4 == 0 && __edata % 4 == 0, "
BUG(riscv-rt): .data is not 4-byte aligned");

ASSERT(__sidata % 4 == 0, "
BUG(riscv-rt): the LMA of .data is not 4-byte aligned");

ASSERT(__sbss % 4 == 0 && __ebss % 4 == 0, "
BUG(riscv-rt): .bss is not 4-byte aligned");

ASSERT(__sheap % 4 == 0, "
BUG(riscv-rt): start of .heap is not 4-byte aligned");

ASSERT(_stext + SIZEOF(.text) < ORIGIN(REGION_TEXT) + LENGTH(REGION_TEXT), "
ERROR(riscv-rt): The .text section must be placed inside the REGION_TEXT region.
Set _stext to an address smaller than 'ORIGIN(REGION_TEXT) + LENGTH(REGION_TEXT)'");

ASSERT(SIZEOF(.stack) > (_max_hart_id + 1) * _hart_stack_size, "
ERROR(riscv-rt): .stack section is too small for allocating stacks for all the harts.
Consider changing `_max_hart_id` or `_hart_stack_size`.");

/* # Other checks */
ASSERT(SIZEOF(.got) == 0, "
ERROR(riscv-rt): .got section detected in the input files. Dynamic relocations are not
supported. If you are linking to C code compiled using the `cc` crate then modify your
build script to compile the C code _without_ the -fPIC flag. See the documentation of
the `cc::Build.pic` method for details.");

/* Do not exceed this mark in the error messages above                                    | */