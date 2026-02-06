//! Build script for uart_logger — RP2350A memory layout

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    let memory_x = "/* Memory layout for RP2350A (Spotpear RP2350-Core-A) */\n\
            MEMORY {\n\
            \x20   FLASH : ORIGIN = 0x10000000, LENGTH = 4096K\n\
            \x20   RAM   : ORIGIN = 0x20000000, LENGTH = 512K\n\
            \x20   SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K\n\
            \x20   SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K\n\
            }\n\
            \n\
            _stack_start = ORIGIN(RAM) + LENGTH(RAM);\n\
            \n\
            SECTIONS {\n\
            \x20   .start_block : ALIGN(4)\n\
            \x20   {\n\
            \x20       __start_block_addr = .;\n\
            \x20       KEEP(*(.start_block));\n\
            \x20       KEEP(*(.boot_info));\n\
            \x20   } > FLASH\n\
            } INSERT AFTER .vector_table;\n\
            \n\
            _stext = ADDR(.start_block) + SIZEOF(.start_block);\n\
            \n\
            SECTIONS {\n\
            \x20   .bi_entries : ALIGN(4)\n\
            \x20   {\n\
            \x20       __bi_entries_start = .;\n\
            \x20       KEEP(*(.bi_entries));\n\
            \x20       . = ALIGN(4);\n\
            \x20       __bi_entries_end = .;\n\
            \x20   } > FLASH\n\
            } INSERT AFTER .text;\n\
            \n\
            SECTIONS {\n\
            \x20   .end_block : ALIGN(4)\n\
            \x20   {\n\
            \x20       __end_block_addr = .;\n\
            \x20       KEEP(*(.end_block));\n\
            \x20   } > FLASH\n\
            } INSERT AFTER .uninit;\n\
            \n\
            PROVIDE(start_to_end = __end_block_addr - __start_block_addr);\n\
            PROVIDE(end_to_start = __start_block_addr - __end_block_addr);\n";

    let mut f = File::create(out.join("memory.x")).unwrap();
    f.write_all(memory_x.as_bytes()).unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");
}
