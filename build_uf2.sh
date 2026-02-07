#!/bin/bash
# Build firmware and generate UF2 with correct RP2350 family ID
set -e

echo "Building firmware..."
cargo build --release

echo "Generating UF2..."
elf2uf2-rs target/thumbv8m.main-none-eabihf/release/uart_logger uart_logger.uf2

echo "Patching family ID to RP2350 (0xE48BFF59)..."
python3 -c "
import struct
data = bytearray(open('uart_logger.uf2','rb').read())
patched = 0
for i in range(len(data)//512):
    off = i*512
    if struct.unpack_from('<I',data,off)[0] == 0x0A324655:
        struct.pack_into('<I', data, off+28, 0xE48BFF59)
        patched += 1
open('uart_logger.uf2','wb').write(data)
print(f'Patched {patched} blocks')
"

echo ""
echo "Done! uart_logger.uf2 ready ($(wc -c < uart_logger.uf2) bytes)"
echo "Copy to RP2350 drive: cp uart_logger.uf2 /media/\$USER/RP2350/"
