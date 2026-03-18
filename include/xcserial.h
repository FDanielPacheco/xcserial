/**
 * @file      xcserial.h
 * 
 * @version   0.4.0
 *
 * @date      16-03-2026
 *  
 * @author    Fábio D. Pacheco (pacheco.castro.fabio@gmail.com)
 *
 * @note
 *  xcserial - Serial Port Library for Linux
 *  Copyright (C) 2026 Fábio D. Pacheco 
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 *  USA
 *
 *
 *  Manuals: \n
 *    https://man7.org/linux/man-pages/man2/TIOCMSET.2const.html \n 
 *    https://people.na.infn.it/~garufi/didattica/CorsoAcq/SerialProgrammingInPosixOSs.pdf \n
 *    https://man7.org/linux/man-pages/man3/errno.3.html
 * 
 */

#ifndef XCSERIAL_H
#define XCSERIAL_H

#include <stdint.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#include "stblog.h"

#ifdef __cplusplus  
extern "C" {        
#endif

/** @brief Marker for invalid or uninitialized serial settings. */
#define SERIAL_INV 0
/** @brief Custom baudrate definition for 62.5k, common in specific industrial controllers. */
#define B62500 10014

/**
 * @enum serial_baudrate
 * @brief Standard and non-standard Linux baud rates.
 */
enum serial_baudrate {
        SERIAL_B50      = B50,     
        SERIAL_B75      = B75,     
        SERIAL_B110     = B110,    
        SERIAL_B134     = B134,    
        SERIAL_B150     = B150,    
        SERIAL_B200     = B200,    
        SERIAL_B300     = B300,    
        SERIAL_B600     = B600,    
        SERIAL_B1200    = B1200,   
        SERIAL_B1800    = B1800,   
        SERIAL_B2400    = B2400,   
        SERIAL_B4800    = B4800,   
        SERIAL_B9600    = B9600,   
        SERIAL_B19200   = B19200,  
        SERIAL_B38400   = B38400,  
        SERIAL_B57600   = B57600,  
        SERIAL_B62500   = B62500,  
        SERIAL_B115200  = B115200, 
        SERIAL_B230400  = B230400, 
        SERIAL_B460800  = B460800, 
        SERIAL_B500000  = B500000, 
        SERIAL_B576000  = B576000, 
        SERIAL_B921600  = B921600, 
        SERIAL_B1000000 = B1000000,
        SERIAL_B1152000 = B1152000,
        SERIAL_B1500000 = B1500000,
        SERIAL_B2000000 = B2000000,
};

/**
 * @enum serial_parity
 * @brief Parity bit configurations.
 */
enum serial_parity {
        SERIAL_PN = 1, //!< No parity
        SERIAL_PO,     //!< Odd parity
        SERIAL_PE,     //!< Even parity
};

/**
 * @enum serial_flowcontrol
 * @brief Flow control modes.
 */
enum serial_flowcontrol {
        SERIAL_FLC_N = 1, //!< No flow control
        SERIAL_FLC_S,     //!< Software flow control (XON/XOFF)
        SERIAL_FLC_H,     //!< Hardware flow control (RTS/CTS)
};

/**
 * @enum serial_databits
 * @brief Number of bits per character.
 */
enum serial_databits {
        SERIAL_DB8 = CS8, //!< 8 data bits
        SERIAL_DB7 = CS7, //!< 7 data bits
        SERIAL_DB6 = CS6, //!< 6 data bits
        SERIAL_DB5 = CS5, //!< 5 data bits
};

/**
 * @enum serial_stopbits
 * @brief Number of stop bits.
 */
enum serial_stopbits {
        SERIAL_SB1 = 1, //!< 1 stop bit
        SERIAL_SB2,     //!< 2 stop bits
};

/**
 * @enum serial_line
 * @brief Modem control line definitions using TIOCM constants.
 */
enum serial_line {
        SERIAL_DSR = TIOCM_LE,  //!< DSR - Data Set Ready 
        SERIAL_DTR = TIOCM_DTR, //!< DTR - Data Terminal Ready
        SERIAL_RTS = TIOCM_RTS, //!< RTS - Request To Send
        SERIAL_CTS = TIOCM_CTS, //!< CTS - Clear To Send
        SERIAL_DCD = TIOCM_CAR, //!< DCD - Data Carrier Detected
};

/**
 * @struct serial_line_state
 * @brief Represents the state (on/off) of a single modem line.
 */
struct serial_line_state {
        enum serial_line offs;  //!< The line to target
        uint8_t          state; //!< 1 for High/Set, 0 for Low/Clear
};

/**
 * @struct serial_lines
 * @brief Array wrapper for batch modem line operations.
 */
struct serial_lines {
        struct serial_line_state *lines; //!< Pointer to array of states
        size_t                    size;  //!< Number of lines in the array
};

/**
 * @struct serial_udev_field
 * @brief Key-value pair for hardware identification via udev.
 */
struct serial_udev_field {
        const char *label;      //!< Field label (e.g., "ID_VENDOR_ID")
        char        value[128]; //!< String value returned by udev
};

/**
 * @struct serial_udev
 * @brief Container for hardware metadata and persistent path tracking.
 */
struct serial_udev {
        struct serial_udev_field *fields;      //!< Custom fields to track
        size_t                    size;        //!< Count of custom fields
        char                      bus[128];    //!< Bus identifier
        char                      path[128];   //!< Physical device path
};

/**
 * @struct serial_config
 * @brief Full communication parameters for a serial port.
 */
struct serial_config {
        enum serial_baudrate     baudrate;     //!< Communication speed
        enum serial_flowcontrol  flowcontrol;  //!< Flow control mode
        enum serial_parity       parity;       //!< Parity check mode
        enum serial_databits     databits;     //!< Bits per frame
        enum serial_stopbits     stopbits;     //!< Stop bits
        int                      timeout_ms;   //!< Default read/write timeout
};

/**
 * @struct serial_epoll
 * @brief Epoll file descriptors.
 */
struct serial_epoll {
        int tx; 
        int rx;
};

/**
 * @struct serial_resv
 * @brief Reserved memory space for formated strings.
 */
struct serial_resv {
        char label[128]; 
        char value[2048];        
};

/**
 * @struct serial_t
 * @brief The main serial handle. 
 * @details This structure maintains the state of the connection, including
 * epoll file descriptors for asynchronous waiting and the logger.
 */
typedef struct {
        int                     fd;     //!< File descriptor of the opened port
        struct serial_config    cfg;    //!< Current port configuration
        struct serial_udev      udev;   //!< Hardware identification data
        struct serial_epoll     epoll;  //!< Managed epoll instances for I/O monitoring
        struct serial_resv      rsv;    //!< Internal scratch buffer for formatted output
        struct stblog_logger    logger; //!< Associated logging handle
} serial_t;

/**
 * @brief Opens a serial port.
 * @param serial Pointer to serial handle.
 * @param pathname System path to device (e.g., "/dev/ttyUSB0").
 * @return 0 on success, negative error code on failure.
 */
int serial_open(serial_t *serial, const char *pathname);

/**
 * @brief Closes the serial port and destroys internal epoll instances.
 * @param serial Pointer to serial handle.
 */
void serial_close(serial_t *serial);

/**
 * @brief Attempts to reopen the serial port using current handle metadata.
 * @param serial Pointer to serial handle.
 * @return 0 on success, negative error code on failure.
 */
int serial_reopen(serial_t *serial);

/**
 * @brief Updates the default timeout for the serial handle.
 * @param timeout_ms Timeout in milliseconds.
 * @param serial Pointer to serial handle.
 * @return 0 on success.
 */
int serial_set_timeout(const int timeout_ms, serial_t *serial);

/**
 * @brief Applies a full configuration to the hardware.
 * @param config Pointer to the configuration struct.
 * @param serial Pointer to serial handle.
 * @return 0 on success, -1 on termios failure.
 */
int serial_set_config(const struct serial_config *config, serial_t *serial);

/**
 * @brief Returns a human-readable string of the current configuration.
 * @param serial Pointer to serial handle.
 * @param err Pointer to integer for error code storage.
 * @return Pointer to internal string buffer.
 */
const char * serial_get_config(serial_t *serial, int *err);

/**
 * @brief Fills a config struct with standard defaults (9600 8N1).
 * @param config Pointer to config struct to fill.
 * @return 0 on success.
 */
int serial_default_config(struct serial_config *config);

/**
 * @brief Returns the current timeout as a string.
 * @param err Pointer to integer for error code storage.
 * @param serial Pointer to serial handle.
 * @return Formatted string of the timeout.
 */
const char * serial_get_timeout(int *err, serial_t *serial);

/**
 * @brief Standard read operation.
 * @param buf Destination buffer.
 * @param size Size of one element.
 * @param nmemb Number of elements to read.
 * @param timeout_ms Local timeout for this operation (-1 for non-blocking| 0 for internal timeout | 0< for specified timeout).
 * @param serial Pointer to serial handle.
 * @param err Pointer to integer for error code storage.
 * @return Number of bytes read, or negative on failure.
 */
ssize_t serial_read(void *buf, const size_t size, const ssize_t nmemb, const int timeout_ms, serial_t *serial, int *err);

/**
 * @brief Reads data until a specific delimiter sequence is found.
 * @param buf Destination buffer.
 * @param size Size of one element.
 * @param nmemb Max elements to read.
 * @param delim Delimiter sequence.
 * @param nmemb_delim Size of delimiter sequence.
 * @param timeout_ms Local timeout for this operation (-1 for non-blocking| 0 for internal timeout | 0< for specified timeout).
 * @param serial Pointer to serial handle.
 * @param err Pointer to integer for error code storage.
 * @return Bytes read including delimiter.
 */
ssize_t serial_read_delim(void *buf, const size_t size, const ssize_t nmemb, const void *delim, const ssize_t nmemb_delim, const int timeout_ms, serial_t *serial, int *err);

/**
 * @brief Line-based read (reads until newline).
 * @param buf Destination char buffer.
 * @param length Maximum length to read.
 * @param timeout_ms Local timeout for this operation (-1 for non-blocking| 0 for internal timeout | 0< for specified timeout).
 * @param serial Pointer to serial handle.
 * @param err Pointer to error code storage.
 * @return Number of characters read.
 */
ssize_t serial_readl(char *buf, const ssize_t length, const int timeout_ms, serial_t *serial, int *err);

/**
 * @brief Writes data to the serial port.
 * @param buf Source buffer.
 * @param size Size of one element.
 * @param nmemb Number of elements to write.
 * @param serial Pointer to serial handle.
 * @param timeout_ms Local timeout for this operation (-1 for non-blocking| 0 for internal timeout | 0< for specified timeout).
 * @param err Pointer to error code storage.
 * @return Number of bytes written.
 */
ssize_t serial_write(const void *buf, const size_t size, const ssize_t nmemb, serial_t *serial, const int timeout_ms, int *err);

/**
 * @brief Formatted write (printf-style) to the serial port.
 * @param serial Pointer to serial handle.
 * @param timeout_ms Local timeout for this operation (-1 for non-blocking| 0 for internal timeout | 0< for specified timeout).
 * @param err Pointer to error code storage.
 * @param fmt Format string.
 * @param ... Variadic arguments.
 * @return Number of bytes written.
 */
ssize_t serial_writef(serial_t *serial, const int timeout_ms, int *err, const char *fmt, ...);

/**
 * @brief Configures udev tracking parameters.
 * @param params Pointer to udev configuration.
 * @param serial Pointer to serial handle.
 * @return 0 on success.
 */
int serial_set_udev(struct serial_udev *params, serial_t *serial);

/**
 * @brief Retrieves hardware info string via udev.
 * @param serial Pointer to serial handle.
 * @param err Pointer to error code storage.
 * @return Formatted hardware info string.
 */
const char * serial_get_udev(int *err, serial_t *serial);

/** @brief Macro list for generating getter/setter pairs. */
#define SERIAL_LIST(f) \
        f(baudrate) \
        f(databits) \
        f(stopbits) \
        f(flowcontrol) \
        f(parity) 

/** @brief Helper macro to declare setters for individual config members. */
#define SERIAL_SET_DECL_X(type) \
        /** \
         * @brief Sets serial port paramters. \
         * @param val Value of type, refer to enum serial_##type. \
         * @param serial Pointer to serial handle. \
         * @return Formatted hardware info string. \
         */ \
        int serial_set_##type(const enum serial_##type val, serial_t *serial);

/** @brief Helper macro to declare getters for individual config members. */
#define SERIAL_GET_DECL_X(type) \
        /** \
         * @brief Retrieves serial port parameters. \
         * @param err Pointer to error code storage.
         * @param serial Pointer to serial handle. \
         * @return Formatted hardware info string. \
         */ \
        const char *serial_get_##type(int *err, serial_t *serial);

SERIAL_LIST(SERIAL_SET_DECL_X)
SERIAL_LIST(SERIAL_GET_DECL_X)

/**
 * @brief Retrieves the current status of all modem lines.
 * @param serial Pointer to serial handle.
 * @return String representation of line states.
 */
const char * serial_get_lines(serial_t *serial);

/**
 * @brief Sets multiple modem lines simultaneously.
 * @param table Pointer to line state table.
 * @param serial Pointer to serial handle.
 * @return 0 on success, -1 on ioctl failure.
 */
int serial_set_lines(const struct serial_lines *table, serial_t *serial);

/**
 * @brief Set single modem line.
 * @param entry The line state entry.
 * @param serial Pointer to serial handle.
 * @return 0 on success, -1 on ioctl failure.
 */
int serial_set_line(struct serial_line_state entry, serial_t *serial);


#ifdef __cplusplus  
}        
#endif

#endif /* XCSERIAL_H */
