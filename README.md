# xcserial: Linux-based C serial port library.

`xcserial` is a lightweight C library providing convenient wrappers for low-level, non-canonical serial port control and I/O on Linux systems.  
It is designed for embedded and systems programming, offering fine-grained control over parameters such as baud rate, parity, stop bits, data bits, flow control, and timeouts.  

The library supports both:
- **Synchronous (blocking)** programming for simple, sequential use.
- **Asynchronous (callback-driven)** programming for event-based applications.

It can be configured to minimize hardware resource usage while keeping a clean and familiar programming style.

---

## Installation

### Prebuilt binaries

Available platforms:
- `aarch64-linux-gnu`
- `arm-linux-gnueabihf` (e.g. Raspberry Pi)
- `x86_64-linux-gnu` (Intel/AMD)

```bash
# Download and extract pre-compiled library
wget https://github.com/FDanielPacheco/xcserial/releases/download/alpha/libxcserial-<platform>.zip
unzip libxcserial-<platform>.zip

# Install using the provided script
chmod u+x install.sh
./install.sh 
```

### Build from source

Clone the repository:
```bash
git clone https://github.com/FDanielPacheco/xcserial.git
cd xcserial
```

Build the dynamic library for the host platform:
```
make
```

Build the dynamic library for all platforms available:
```
make all
```

Build for a specific target and type:
```
make release TARGET_ARCH_LLC=<arch> TARGET_ARCH_CC=<triplet> TYPE=<so|a> CF=-fPIC LF="-relocation-model=pic"
```

`<arch,triplet>` examples:
- `arm`, `arm-linux-gnueabihf`
- `x86-64`, `x86_64-linux-gnu`
- `aarch64`, `aarch64-linux-gnu`
 
`<TYPE>`: Dynamic: so | Static: a

## Usage Example 

Synchronous example
```
#include <xcserial.h>

int
main( void ){
  const char port[] = "/dev/ttyUSB0";
  const uint8_t readonly = 0; // read/write enable  
  serial_t serial;

  // Open serial port
  if( -1 == serial_open( &serial, port, readonly, NULL ) )
    exit( EXIT_FAILURE );

  // Change the baudrate, and add a timeout for read functions
  const baudrate_t baudrate = B115200; 
  serial_set_baudrate( baudrate, &serial );
  const timeout_ms = 10;
  serial_set_timeout( timeout_ms, &serial );

  // Print configuration
  const uint8_t output = 1; // print to the stdout
  char text[32]; snprintf( text, sizeof(text), "[%d]", getpid( ) );
  (void) serial_print_config( output, text, &serial );

  // Write formatted string
  size_t len = serial_writef( &serial, "Message:%d,%.2f\n", var1, var2 );
  if( !len ){
    serial_close( &serial );
    exit( EXIT_FAILURE );  
  }

  // Read a chunk of bytes
  uint8_t buf[ BUF_SIZE ];
  size_t offset = 0;  // offset in the buf array
  len = serial_read( buf, sizeof(buf), offset, &serial ); 
  
  // Close the serial port
  serial_close( &serial );
  return 0;   
}
```

Asynchronous example
```
#include <xcserial.h>

uint8_t buf[ BUF_SIZE ];
size_t buf_len = 0; 

void
read_ck( const uint8_t * data, const size_t len ){
  // Check if there is available space
  if( BUF_SIZE - buf_len > len ){
    memcpy( buf, data, len );
    buf_len += len;
  }  
}

int
main( void ){
  const char port[] = "/dev/ttyUSB0";
  const uint8_t readonly = 0; // read/write enable  
  serial_t serial;

  // Open serial port
  if( -1 == serial_open( &serial, port, readonly, NULL ) )
    exit( EXIT_FAILURE );

  // Change the baudrate, and add a callback for read functions
  const baudrate_t baudrate = B115200; 
  serial_set_baudrate( baudrate, &serial );
  serial_set_callback( read_ck, &serial );

  // Infinite loop
  for( ; ; ){
    if( !strcmp( buf, "Specific pattern" ) )
      // process data
    // doing something that requires CPU
  }
```

## Documentation

API documentation generation (from xcserial directory):
```
make documentation
```
Manual pages: `man docs/man/man3/xcserial.c.3` or `man docs/man/man3/xcserial.h.3` \
HTML docs: `firefox docs/html/index.html`

Additional:
- [POSIX Serial Programming Guide](https://people.na.infn.it/~garufi/didattica/CorsoAcq/SerialProgrammingInPosixOSs.pdf)
- [ERRNO Table](https://man7.org/linux/man-pages/man3/errno.3.html)
- [Serial Lines Table](https://man7.org/linux/man-pages/man2/TIOCMSET.2const.html)

## Author

Fábio D. Pacheco \
Email: fabio.d.pacheco@inesctec.pt

## License

[LGPL-2.1 license](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html) 
