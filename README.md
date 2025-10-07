# xcserial: Linux-based C serial port library.

`xcserial` xcserial is a lightweight, user-space C library that abstracts low-level serial port I/O control on Linux. It offers comprehensive configuration of baud rate, parity, flow control, and millisecond-level timeouts. The architecture supports two distinct operating modes:

- Synchronous: Utilizes the epoll system call for efficient, resource-sparing blocking I/O with precise response latency.
- Asynchronous: Implements a multithreaded, callback-driven mechanism ideal for event-based applications.
  
A core capability is device resilience, enabling automatic port reopening by matching specified udevadm device fields upon physical disconnection.
The library's stability and performance have been validated through its continuous deployment over one year within two scientific prototypes for autonomous underwater and mobile robotics.

---

## Installation

### Library Requirements

- C Standard Library (lc)
- POSIX Threads Library (lpthread)
- libudev (ludev)

In Debian-based Linux:
```bash
sudo apt install build-essential libudev-dev
```

### Prebuilt binaries

Available platforms:
- `x86_64-linux-gnu` (Intel/AMD)
- `arm-linux-gnueabihf` (e.g. Raspberry Pi 32-bit CPU)
- `aarch64-linux-gnu` (e.g. Raspberry Pi 64-bit CPU)

```bash
# Download and extract pre-compiled library
wget https://github.com/FDanielPacheco/xcserial/releases/download/alpha/libxcserial-<platform>.zip
unzip libxcserial-<platform>.zip

# Install using the provided script
chmod u+x install.sh
./install.sh 
```

### Build from source

Build Requirements:
- clang: Compiler frontend and Linker
- opt: Optimizer
- llc: Compiler backend
- llvm-ar: Build static library 

```bash
sudo apt install llvm clang
```

Clone the repository:
```bash
git clone https://github.com/FDanielPacheco/xcserial.git
cd xcserial
```

Build the dynamic library for the host platform:
```bash
make
```

Build the dynamic library for all platforms listed above:
```bash
make all
```

Build for a specific target and type independent on the host platform (require the dynamic/static libraries for that platform):
```bash
make release TARGET_ARCH_LLC=<arch> TARGET_ARCH_CC=<triplet> TYPE=<so|a> CF=-fPIC LF="-relocation-model=pic"
```

`<arch,triplet>` examples:
- `arm`, `arm-linux-gnueabihf`
- `x86-64`, `x86_64-linux-gnu`
- `aarch64`, `aarch64-linux-gnu`
 
`<TYPE>`: Dynamic: so | Static: a

---
## Usage Example 

### Synchronous (Blocking) Mode
```c
#include <xcserial.h>
#include <stdio.h>

int main( void ) {
  const char port[] = "/dev/ttyUSB0";
  serial_t serial;

  // Open port. Last three NULLs are for port configurations, udev params and async callbacks.
  if( -1 == serial_open( &serial, port, 0, NULL, NULL, NULL ) )
    return EXIT_FAILURE;

  // Set configuration
  serial_set_baudrate( B115200, &serial );
  serial_set_timeout( 10, &serial ); // 10ms read timeout

  // Print configuration to stdout
  char text[32]; 
  snprintf( text, sizeof(text), "[PID:%d]", getpid() );
  serial_print_config( 1, text, &serial );

  // Write formatted string
  int var1 = 123;
  float var2 = 45.67f;
  serial_writef( &serial, "Message:%d,%.2f\n", var1, var2 );

  // Read a chunk of bytes
  uint8_t buf[ BUFSIZ ];
  size_t len = serial_read( buf, sizeof(buf), 0, &serial ); 
  
  // Close the serial port
  serial_close( &serial );
  return 0;  
}
```

### Asynchronous (Callback) Mode
This mode runs the I/O loop in the background, executing callbacks when data is received or the device is disconnected.
```c
#include <xcserial.h>
#include <stdio.h>

uint8_t receive_buffer[ BUFSIZ ];
size_t buffer_length = 0; 

// 1. Data Reception Callback
void read_callback( const uint8_t * data, const size_t len ) {
  // Check available space and copy data
  if( BUFSIZ > buffer_length + len ){
    memcpy( &receive_buffer[buffer_length], data, len );
    buffer_length += len;
  } 
}

// 2. Disconnection Callback
int8_t disconnect_callback( void ) {
  // Logic to run when the device is physically disconnected
  printf("Device disconnected! Waiting for hotplug...\n");
  // ...
}

int main( void ) {
  // ... serial_open( ... )
  serial_t serial;
  // ... open and set params ...

  // Set the callback functions
  serial_async_set_callback( read_callback, disconnect_callback, &serial );

  // Infinite loop in the main thread (I/O runs in a separate thread)
  for( ; ; ){
    // Main thread is free to do CPU-intensive work or check flags
    if( buffer_length > 0 && !strcmp( (char*)receive_buffer, "PATTERN" ) ){
        // Process received data
    }
  }
}
```

### Hotplug Reopen Feature
Specify a list of udevadm fields to uniquely identify the device. If the port is lost, serial_reopen will monitor for a device matching those fields.
```c
int main( void ){
  serial_t serial;
  // ... open and set params ...

  // 1. Define the udevadm fields for tracking
  const char params[][NAME_MAX] = {
    "ID_MODEL",
    "ID_USB_VENDOR",
    "ID_SERIAL_SHORT"
  };
  serial_set_udev_param_list( params, 3, NAME_MAX, &serial );

  // ... main application logic ...

  // 2. Call this after a device loss is detected or in the disc_ck( ) callback
  uint16_t retry_attempts = 100; 
  if( -1 == serial_reopen( &serial, retry_attempts ) ){
    serial_close( &serial );
    return EXIT_FAILURE;
  }
  // ... continue program execution ...
}
```

## Documentation

API documentation generation (from xcserial directory):
```bash
make documentation
```
Manual pages: `man docs/man/man3/xcserial.c.3` or `man docs/man/man3/xcserial.h.3` \
HTML docs: `firefox docs/html/index.html`

Additional:
- [POSIX Serial Programming Guide](https://people.na.infn.it/~garufi/didattica/CorsoAcq/SerialProgrammingInPosixOSs.pdf)
- [ERRNO Table](https://man7.org/linux/man-pages/man3/errno.3.html)
- [Serial Lines Table](https://man7.org/linux/man-pages/man2/TIOCMSET.2const.html)

---
## Author

Fábio D. Pacheco \
Email: fabio.d.pacheco@inesctec.pt

## License

[LGPL-2.1 license](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html) 
