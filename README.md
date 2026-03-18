# xcserial: High-Performance Linux Serial Port Library

`xcserial` is a lightweight, user-space C library designed for non-canonical serial port 
communication on Linux. It abstracts low-level `termios` and `ioctl` operations, providing
a robust interface for high-frequency data exchange and hardware-level device management. 

The library utilizes the Linux `epoll` subsystem to achieve efficient I/O multiplexing, 
ensuring minimal CPU utilization during blocking operations. A primary feature is its 
device resilience, which utilizes `udev` metadata to enable automatic port restoration 
following physical disconnection or hotplug events.

The architecture has been validated through extensive deployment in scientific robotics 
prototypes, specifically for autonomous underwater and mobile platforms.

---

## Installation

### System Requirements

The library depends on the following system components:
- Standard C Library (ISO C99/C11/C23)
- libudev (Hardware abstraction layer)

On Debian-based systems:
```bash
sudo apt install build-essential libudev-dev clang llvm
```

### Build from Source

The build system utilizes a Clang/LLVM toolchain for optimized machine code generation and cross-compilation support ([clang](https://clang.llvm.org/)).

1. Compilation for Host Architecture:
```bash
make
```

2. Cross-Compilation: 
Target specific architectures using the `ARCH` variable to invoke the corresponding LLVM triple.
```bash
make ARCH=arm      # ARM-hf  (e.g., Raspberry Pi 32-bit)
make ARCH=aarch64  # AArch64 (e.g., Raspberry Pi 5, Jetson)
```

3. Release Generation:
To generate a distribution package including headers, libraries, a `pkg-config` file, and an installation script:
```bash
make release ARCH=<target_arch>
```

---

## Usage Examples

The following examples progressively demonstrate the library's capabilities, 
from basic string I/O to advanced hardware tracking. In all examples, a timeout 
value of `0` directs the API to use the default internal timeout defined during configuration.

### 1. Simple String I/O
The most common use case: writing a formatted text command and reading a newline-terminated response.

```c
#include <xcserial.h>
#include <stdlib.h>
#include <string.h>

int main(void) {
        serial_t serial;
        int err = 0;

        err = serial_open(&serial, "/dev/ttyUSB0");
        if (0 > ) {
                strerror(-err);
                return EXIT_FAILURE;
        }

        // Configure the serial port with 115200 bps and 100 ms timeout for IO
        serial_set_baudrate(SERIAL_B115200, &serial);
        serial_set_timeout(100, &serial);

        // Write a formatted string
        serial_writef(&serial, 0, &err, "REQ_TEMP_ID:%d\n", 1);

        // Read until newline character
        char rx_buf[128];
        ssize_t bytes_read = serial_readl(rx_buf, sizeof(rx_buf), 0, &serial, &err);

        serial_close(&serial);
        return 0;
}
```

### 2. Raw Binary I/O
For fixed-size packets or device control protocols, standard read/write can be used.

```c
        // ... Initialization ...

        // Sending a 3-byte binary command
        uint8_t tx_buf[] = { 0x01, 0xFF, 0x0A };
        serial_write(tx_buf, sizeof(uint8_t), 3, &serial, 0, &err);

        // Reading a known 16-byte fixed payload
        uint8_t rx_buf[16];
        ssize_t bytes_read = serial_read(rx_buf, sizeof(uint8_t), 16, 0, &serial, &err);
```

### 3. Complex Delimiter Parsing
Some industrial protocols do not rely on a single newline, but rather a sequence of 
bytes to denote the end of a frame (EOF). `serial_read_delim()` handles multi-byte 
termination sequences. Additionally, instead of using the internal timeout defined with
`serial_set_timeout()` a custom timeout with 1 s is used.

```c
        // ... Initialization ...

        uint8_t rx_buf[1024];
        // Protocol frame ends with the specific sequence: 0xA1, 0xA2, 0xA3
        uint8_t delimiter[] = { 0xA1, 0xA2, 0xA3 };

        // Read data continuously until the 3-byte delimiter is matched, 
        // the buffer fills, or the timeout occurs.
        int timeout = 1000;
        ssize_t frame_len = serial_read_delim(
                rx_buf, sizeof(uint8_t), sizeof(rx_buf), 
                delimiter, 3, 
                timeout, 
                &serial, &err
        );
```

### 4. Hardware Resilience and Hotplugging
To ensure connectivity across device re-enumeration (e.g., if a USB cable is bumped 
and the node changes from `/dev/ttyUSB0` to `/dev/ttyUSB1`), `xcserial` can track 
devices via hardware-specific `udev` attributes.

```c
        // ... Initialization ...

        // Define hardware-specific fields to uniquely identify the device
        struct serial_udev_field fields[] = {
                {.label = "ID_VENDOR_ID"},
                {.label = "ID_MODEL_ID"},
                {.label = "ID_SERIAL_SHORT"}
        };
        struct serial_udev u_cfg = {
                .fields = fields,
                .size = 3
        };

        // Attach udev parameters to the serial handle
        serial_set_udev(&u_cfg, &serial);

        // Main application loop
        for ( ; ; ) {
                // Attempt a read operation...
                if (serial_read( ..., &err) < 0) {
                    // If the physical device is lost or disconnected:
                    if ( !serial_reopen(&serial) ) {
                        // The library successfully matched the udev parameters, 
                        // found the new sysfs path, and restored communication.
                        continue; 
                    }
                }
        }
```

### 5. Modem Line Control
The library provides direct low-level access to the physical modem control lines 
for hardware handshaking or custom signaling (e.g., RS-485 transceiver direction
control or forcing a hardware reset).

```c
        // ... Initialization ...

        // Set Data Terminal Ready (DTR) HIGH and Request To Send (RTS) LOW
        struct serial_line_state states[] = {
                {SERIAL_DTR, 1},
                {SERIAL_RTS, 0}
        };
        struct serial_lines table = { 
                .lines = states, 
                .size = 2 
        };

        // Apply states to the hardware pins
        serial_set_lines(&table, &serial);

        // Retrieve and log current line states
        const char *line_status = serial_get_lines(&serial);
```

### 6. External Integration
The library supports external integration of the serial object with an external 
`epoll` or any file descriptor dependent library, as demonstrated by the example below,
since the `serial_t` object has a file descriptor at `serial->fd`.

```c
int main(void) {
        serial_t ser;
        int epoll_fd, t_fd, sig_fd;
        struct epoll_event ev, events[MAX_EVENTS];
        sigset_t mask;

        // Initialize xcserial
        if (serial_open(&ser, "/dev/ttyACM0") < 0) 
                return EXIT_FAILURE;
        serial_set_baudrate(SERIAL_B115200, &ser);

        // Setup Signal Handling (SIGINT)
        sigemptyset(&mask);
        sigaddset(&mask, SIGINT);
        sigprocmask(SIG_BLOCK, &mask, NULL);
        sig_fd = signalfd(-1, &mask, 0);

        // Setup a Periodic Timer (1 second)
        t_fd = timerfd_create(CLOCK_MONOTONIC, 0);
        struct itimerspec period = { {1, 0}, {1, 0} };
        timerfd_settime(t_fd, 0, &period, NULL);

        // Create Global Epoll Instance
        epoll_fd = epoll_create1(0);

        // Register Serial RX
        ev.events = EPOLLIN; ev.data.fd = ser.fd;
        epoll_ctl(epoll_fd, EPOLL_CTL_ADD, ser.fd, &ev);

        // Register Timer
        ev.events = EPOLLIN; ev.data.fd = t_fd;
        epoll_ctl(epoll_fd, EPOLL_CTL_ADD, t_fd, &ev);

        // Register Signal
        ev.events = EPOLLIN; ev.data.fd = sig_fd;
        epoll_ctl(epoll_fd, EPOLL_CTL_ADD, sig_fd, &ev);

        printf("Event loop started. Press Ctrl+C to exit.\n");

        int run = 1;
        while (run) {
                int nfds = epoll_wait(epoll_fd, events, MAX_EVENTS, -1);

                for (int i = 0; i < nfds; i++) {
                    // HANDLE SERIAL DATA
                    if (events[i].data.fd == ser.fd) {
                        char buf[256];
                        int err = 0;
                        // Passing -1 ensures direct read() access without internal epoll blocking
                        ssize_t n = serial_read(buf, 1, sizeof(buf) - 1, -1, &ser, &err);
                        if (n > 0) {
                            buf[n] = '\0';
                            printf("[SERIAL] Received: %s", buf);
                        }
                    }
                    // HANDLE TIMER
                    else if (events[i].data.fd == t_fd) {
                        uint64_t miss;
                        read(t_fd, &miss, sizeof(miss));
                        printf("[TIMER] 1s Heartbeat - Sending Telemetry...\n");
                        serial_writef(&ser, -1, NULL, "HB_STAT:OK\n");
                    }
                    // HANDLE SIGNAL
                    else if (events[i].data.fd == sig_fd) {
                        printf("\n[SIGNAL] SIGINT received. Shutting down.\n");
                        run = 0;
                    }
                }
        }

        close(sig_fd);
        close(t_fd);
        close(epoll_fd);
        serial_close(&ser);
        return 0;
}
```

---

## Documentation

API documentation generation (from xcserial directory) via ([Doxygen](https://www.doxygen.nl/))

Generation:
```bash
make docs
```

Accessing MAN Pages:
```bash
man ./docs/man/man3/xcserial.h.3
```
Accessing Doxygen HTML interface:
```bash
<html-viewer> ./docs/html/index.html
```

Technical References:
- [Linux TIOCM Control Consts](https://man7.org/linux/man-pages/man2/TIOCMSET.2const.html)
- [POSIX Serial Programming](https://people.na.infn.it/~garufi/didattica/CorsoAcq/SerialProgrammingInPosixOSs.pdf)
- [Linux udev Documentation](https://www.kernel.org/pub/linux/utils/kernel/hotplug/udev/udev.html)
- [Linux epoll Documentation](https://www.man7.org/linux/man-pages/man7/epoll.7.html)
- [Linux errno Table](https://man7.org/linux/man-pages/man3/errno.3.html)

---

## Author

Fábio D. Pacheco
(pacheco.castro.fabio@gmail.com)

## License

`xcserial` is released under the [GNU Lesser General Public License v2.1](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html).