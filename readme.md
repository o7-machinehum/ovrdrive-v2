# Phantomdrive
![](img/Logo_white_big.png)

Phantomdrive is open source encrypted USB drive with a stealth mechanism to hide its second partition. To decrypt it you must create a file called unlock.txt containing your password in the format `password:addpasswordhere`, this is used to derive a AES-256 key. The drive automatically unmounts itself, remounts the remaining disk and encrypts and decrypts in place. It uses CH569W SoC, which has USB3, SDIO and an AES hardware block. It is programmable over USB using the `wch-ch56x-isp` library.

``` bash
|-- ee             # Hardware files
|-- Makefile
|-- readme.md      # This files
|-- test           # Test the cypto components
|-- ref            # Reference docs
|-- src            # Firmware
|-- tests          # Verification scripts
|-- wch-ch56x-bsp  # Board support package
`-- wch-ch56x-isp  # Programming software
```

# Getting Started
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

## Debugging
```
make UART=1 # Enable UART
make UART=1 DEBUG_USB=1 # Enable USB Debugging
```

## Build and Flashing the project
``` bash
make
# Remove flash drive
# While holding boot button, plug in
./wch-ch56x-isp/wch-ch56x-isp -d=off # This is needed just one to disable debug mode
make flash
```

## Unlocking Drive
``` bash
sudo echo "password:YourPasswordHere13245" > /mnt/unlock.txt
```

# Releasing the hardware
``` bash
./scripts/release_hardware.sh
```
