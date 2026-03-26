# Ovrdrive v2
Firmware for Ovrdrive v2 - the worlds only self-destructing USB drive.

## Getting Started
``` bash
git submodule update --init --recursive --checkout --force
cd wch-ch56x-isp
make # Build the ISP tool
```

## Install the toolchain
The HydraUSB3 project requires a patched GCC that supports the
`WCH-Interrupt-fast` interrupt attribute. Download the xpack GCC from the HydraUSB3 project:

```bash
cd /tmp
curl -sL -o riscv-gcc-xpack.tar.gz \
  "https://github.com/hydrausb3/riscv-none-elf-gcc-xpack/releases/download/12.2.0-1/xpack-riscv-none-elf-gcc-12.2.0-1-linux-x64.tar.gz"
tar xzf riscv-gcc-xpack.tar.gz
sudo cp -r xpack-riscv-none-elf-gcc-12.2.0-1/* /usr/local/
```

## Build the project
```
make
make flash
```

## Unlocking Drive
``` bash
unlock.txt # password:jflksdjflk
```
