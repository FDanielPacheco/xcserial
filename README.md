# xcserial: Linux-based C serial port library.
`xcserial` is a lightweight C library providing convenient wrappers for low-level, non-canonical serial port control and I/O for Linux systems. 
It is designed for embedded systems interaction and offers fine-grained control over serial communication parameters such as baud rate, parity, stop bits, data bits, flow control and timeout.
The library can use be configured for minimum hardware resource usage while mantaining a sequential programming style.

## Installation

Platform available:
- aarch64-linux-gnu
- arm-linux-gnueabihf (devices like Raspberry Pi)
- x86_64-linux-gnu (mostly Intel and AMD)

Library installation:
```
# Download and extract pre-compiled library
wget https://github.com/FDanielPacheco/xcserial/releases/download/alpha/libxcserial-<platform>.zip
unzip libxcserial-<platform>.zip

# Install using the bash script
chmod u+x install.sh
./install.sh 
```

Build dynamic library (for all platforms):
```
git clone https://github.com/FDanielPacheco/xcserial.git
cd xcserial
make
```

Build and install dynamic or static library (for single platform):
```
make release TARGET_ARCH_LLC=<1> TARGET_ARCH_CC=<2> TYPE=<3> CF=-fPIC LF="-relocation-model=pic" LDF=""
```

`<1,2>`:
- arm, arm-linux-gnueabihf
- x86-64, x86_64-linux-gnu
- aarch64, aarch64-linux-gnu
 
`<3>`:
- Dynamic: so
- Static: a

## 
