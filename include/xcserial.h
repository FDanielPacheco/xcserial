/**
 * @file      xcserial.h
 * 
 * @version   0.4.0
 *
 * @date      16-03-2026
 *  
 * @author    Fábio D. Pacheco, 
 * @email     fabio.d.pacheco@inesctec.pt or pacheco.castro.fabio@gmail.com
 *
 *  @note
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

#include "log.h"

#ifdef __cplusplus  
extern "C" {        
#endif

#define SERIAL_INV 0
#define B62500 10014

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

enum serial_parity {
        SERIAL_PN = 1,
        SERIAL_PO,
        SERIAL_PE,
};

enum serial_flowcontrol {
        SERIAL_FLC_N = 1,
        SERIAL_FLC_S,
        SERIAL_FLC_H,
};

enum serial_databits {
        SERIAL_DB8 = CS8,
        SERIAL_DB7 = CS7, 
        SERIAL_DB6 = CS6,
        SERIAL_DB5 = CS5,
};

enum serial_stopbits {
        SERIAL_SB1 = 1,
        SERIAL_SB2,
};

enum serial_line {
        SERIAL_DSR = TIOCM_LE,  //!< DSR - Data Set Ready 
        SERIAL_DTR = TIOCM_DTR, //!< DTR - Data Terminal Ready
        SERIAL_RTS = TIOCM_RTS, //!< RTS - Request To Send
        SERIAL_CTS = TIOCM_CTS, //!< CTS - Clear To Send
        SERIAL_DCD = TIOCM_CAR, //!< DCD - Data Carrier Detected
};

struct serial_line_state {
        enum serial_line offs;
        uint8_t          state;
};

struct serial_lines {
        struct serial_line_state *lines;
        size_t                    size;
};

struct serial_udev_field {
        const char *label;
        char        value[128];
};

struct serial_udev {
        struct serial_udev_field *fields;
        size_t                    size;
        char                      bus[128];
        char                      path[128];
};

struct serial_config {
        enum serial_baudrate    baudrate;
        enum serial_flowcontrol flowcontrol;
        enum serial_parity      parity;
        enum serial_databits    databits;
        enum serial_stopbits    stopbits;
        int                     timeout_ms;
};

typedef struct {
        int                                      fd;
        struct serial_config                     cfg;
        struct serial_udev                       udev;
        struct { int tx, rx; }                   epoll;
        struct { char label[128], value[2048]; } rsv;
        struct log_logger                        logger;
} serial_t;

int
serial_open( 
        serial_t   *serial, 
        const char *pathname
);

void 
serial_close(
        serial_t   *serial
);

int 
serial_reopen(
        serial_t   *serial
);

int
serial_set_timeout(
        const int  timeout_ms,
        serial_t  *serial
);

int 
serial_set_config(
        const struct serial_config *config,        
        serial_t                   *serial
);

const char *
serial_get_config(
        serial_t *serial,
        int      *err 
);

int 
serial_default_config(
        struct serial_config *config 
);

int
serial_set_timeout(
        const int  timeout_ms,
        serial_t  *serial
);

const char * 
serial_get_timeout(
        int      *err,
        serial_t *serial
);

ssize_t
serial_read(
        void          *buf,
        const size_t   size, 
        const ssize_t  nmemb, 
        const int      timeout_ms,
        serial_t      *serial,
        int           *err
);

ssize_t
serial_read_delim(
        void          *buf,
        const size_t   size,
        const ssize_t  nmemb, 
        const void    *delim,
        const ssize_t  nmemb_delim, 
        const int      timeout_ms,
        serial_t      *serial,
        int           *err
);

ssize_t
serial_readl(
        char          *buf,
        const ssize_t  length, 
        const int      timeout_ms,
        serial_t      *serial,
        int           *err
);

ssize_t
serial_write(
        const void    *buf,
        const size_t   size, 
        const ssize_t  nmemb, 
        serial_t      *serial,
        const int      timeout_ms,
        int           *err
);

ssize_t 
serial_writef( 
        serial_t   *serial, 
        const int  timeout_ms,
        int        *err,
        const char *fmt, 
        ...
);

int
serial_set_udev(
        struct serial_udev *params,
        serial_t           *serial
);

const char *
serial_get_udev(
        serial_t *serial,
        int      *err
);

#define SERIAL_LIST(f) \
        f(baudrate) \
        f(databits) \
        f(stopbits) \
        f(flowcontrol) \
        f(parity) 

#define SERIAL_SET_DECL_X(type) \
        int serial_set_##type(const enum serial_##type val, serial_t *serial);
#define SERIAL_GET_DECL_X(type) \
        const char *serial_get_##type(int *err, serial_t *serial);

SERIAL_LIST(SERIAL_SET_DECL_X)
SERIAL_LIST(SERIAL_GET_DECL_X)

const char * 
serial_get_lines( 
        serial_t *serial 
);

int
serial_set_lines(
        const struct serial_lines *table,
        serial_t                  *serial 
);

#ifdef __cplusplus  
}        
#endif


#endif
