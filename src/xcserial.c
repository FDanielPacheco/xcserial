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

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <libudev.h>
#include <sys/epoll.h>
#include <time.h>

#include "xcserial.h"

#ifndef DISABLE_LOGGING
        #define LOG_SER(serial, err, prio, fmt, ...) \
                LOG_ERRNO(serial ? &(serial->logger) : 0, "serial", err, prio, fmt, ##__VA_ARGS__)
#else
        #define LOG_SER(serial, err, prio, fmt, ...) \
                ((void)0)
#endif

/** 
@brief Checks rather the serial port file descriptor and FILE pointer are healthy. 
 */
static int
_serial_nvalid( 
        const serial_t *serial,
        const uint8_t   quick
){
        if ( !serial ) {
                return -EINVAL;
        }
        if ( 0 > serial->fd ) {
                return -EBADF;
        }
        if (!quick) {
                if ( -1 == fcntl( serial->fd, F_GETFD ) ) {
                        return -EBADF;
                }   
        }
        return 0;
}

static int
_serial_epoll_setup(
        int       serial_fd,
        int      *epoll_fd,
        uint32_t  event
) {
        int err = 0;
        if (!epoll_fd) {
                err = -EINVAL;
                return err;
        }
        *epoll_fd = epoll_create1(0);
        if (-1 == *epoll_fd) {
                err = -errno;
                return err;
        }
        struct epoll_event epoll_ev;
        // EPOLLET To make edge trigger, meaning it will only trigger the epoll_wait once there is data arriving
        epoll_ev.events  = EPOLLHUP|event; 
        epoll_ev.data.fd = serial_fd;
        epoll_ctl(*epoll_fd, EPOLL_CTL_ADD, serial_fd, &epoll_ev);
        return err;
}

/** 
@brief Initialiaze and open the serial port. 
 */
int
serial_open( 
        serial_t   *serial, 
        const char *pathname
) {
        int err = 0;
        if ( !serial || !pathname || !strlen(pathname) ) {
                err = -EINVAL;
                goto cleanup;
        }

        if ( sizeof(serial->udev.path) < strlen(pathname) ) {
                err = -ENAMETOOLONG;
                LOG_SER(serial, err, LOG_ERROR, "serial_open");
                goto cleanup;
        }

        (void) memset(serial, 0, sizeof(serial_t));

        //                                           check later
        //                                               ↓    
        serial->fd = open(pathname, O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
        if ( 1 > serial->fd ) {
                err = -errno;
                LOG_SER(serial, err, LOG_ERROR, "serial_open");
                goto cleanup;
        }
        fcntl(serial->fd, F_SETFL, 0);

        if ( -1 == tcflush(serial->fd, TCIOFLUSH) ) {
                err = -errno;
                LOG_SER(serial, err, LOG_ERROR, "tcflush");
                goto cleanup;
        }
        
        (void) strncpy(serial->udev.path, pathname, sizeof(serial->udev.path));        
        struct serial_config _config;
        serial_default_config(&_config);
        err = serial_set_config(&_config, serial);
        if ( err ) {
                LOG_SER(serial, err, LOG_ERROR, "serial_open");
                goto cleanup;
        }
        
        err = _serial_epoll_setup(serial->fd, &serial->epoll.rx, EPOLLIN);
        if (err) {
                LOG_SER(serial, err, LOG_ERROR, "_serial_epoll_setup");
                goto cleanup;
        }
        err = _serial_epoll_setup(serial->fd, &serial->epoll.tx, EPOLLOUT);
        if (err) {
                LOG_SER(serial, err, LOG_ERROR, "_serial_epoll_setup");
                goto cleanup;
        }
cleanup:
        if (err) {
                serial_close(serial);
        }
        return err;
}

/** 
@brief Close the serial port. 
 */
void 
serial_close(
        serial_t *serial
) {
        if ( -1 != serial->fd ) {
                (void) tcdrain( serial->fd );
                (void) close(serial->fd);
                serial->fd = -1;
        }
        if ( -1 != serial->epoll.rx ) {
                (void) close(serial->epoll.rx);
                serial->epoll.rx = -1;
        }
        if ( -1 != serial->epoll.tx ) {
                (void) close(serial->epoll.tx);
                serial->epoll.tx = -1;
        }
}


static const char *
_serial_find_udev( 
        struct udev        *udev,
        struct serial_udev *id
){
        if ( !udev || !id ) {
                return NULL;
        }

        struct udev_enumerate *enumerate = udev_enumerate_new(udev);
        if ( !enumerate ) {
                return NULL;
        }

        udev_enumerate_add_match_subsystem(enumerate, "tty");
        for (size_t i = 0; i < id->size; ++i) {
                struct serial_udev_field *field = &(id->fields[i]);
                udev_enumerate_add_match_property(enumerate, field->label, field->value);
        }

        udev_enumerate_scan_devices(enumerate);
        struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate);
        struct udev_list_entry *entry;

        int8_t found = 0;
        udev_list_entry_foreach( entry, devices ){
                const char *syspath = udev_list_entry_get_name( entry );
                struct udev_device *dev = udev_device_new_from_syspath( udev, syspath );
                if ( !dev ) {
                        continue;
                }
                  
                const char *devnode = udev_device_get_devnode( dev );
                if ( devnode ){
                        strncpy(id->path, devnode, sizeof(id->path));
                        found = 1;
                        udev_device_unref(dev);
                        break; 
                }
                udev_device_unref(dev);
        }
        udev_enumerate_unref(enumerate);
        return found ? id->path : NULL;      
}


/** 
@brief Attempt to reopen the serial port. 
 */
int 
serial_reopen(
        serial_t   *serial
) {
        if ( !serial ){
                return -EINVAL;    
        }

        int err = 0;
        struct udev *udev = udev_new();  
        if ( !udev ){
                err = -errno;
                goto cleanup;
        }

        struct serial_udev id;
        memcpy(&id, &serial->udev, sizeof(struct serial_udev));

        serial_close(serial); 

        serial_t tmp;
        err = serial_open(&tmp, _serial_find_udev(udev, &id));
        if ( !err ){          
                serial_set_config(&serial->cfg, &tmp);
                serial_set_udev(&serial->udev, &tmp);
                memcpy(serial, &tmp, sizeof(serial_t));
                LOG_SER(serial, err, LOG_DEBUG, "success to reopen");
                return 0;
        }

        udev_unref( udev );
cleanup:
        LOG_SER(serial, err, LOG_ERROR, "serial_reopen");
        return -1;
}


static int 
_serial_set_baudrate(
        const int       baudrate, 
        struct termios *tty
) {
        if (!tty) {
                return -EINVAL;
        }
        int err = 0;
        err += cfsetispeed(tty, (speed_t) baudrate);
        err += cfsetospeed(tty, (speed_t) baudrate);
        if ( err ){
                return -errno;
        }
        return 0;
}

static int 
_serial_set_parity( 
        const int       parity, 
        struct termios *tty
){
        if (!tty) {
                return -EINVAL;
        }
        switch ( parity ){
        default:
                return -EINVAL;
        case SERIAL_PN:
                tty->c_cflag &= (tcflag_t) ~(PARENB);                                    // Disable parity (Clear bit)
                tty->c_iflag &= (tcflag_t) ~(INPCK);                                     // Disable parity checking
                break;
        case SERIAL_PO:
                tty->c_cflag |= (tcflag_t) (PARENB) | (PARODD);                          // Enable parity (Set bit) and Enable odd parity
                tty->c_iflag |= (tcflag_t) (INPCK);                                      // Enable parity checking
                break;
        case SERIAL_PE:
                tty->c_cflag |= (tcflag_t) (PARENB);                                     // Enable parity (Set bit)
                tty->c_cflag &= (tcflag_t) ~(PARODD);                                    // Enable even parity
                tty->c_iflag |= (tcflag_t) (INPCK);                                      // Enable parity checking
                break;
        }
        return 0;
}

static int 
_serial_set_stopbits( 
        const int       stopbits, 
        struct termios *tty
){
        if (!tty) {
                return -EINVAL;
        }
        if (SERIAL_SB1 == stopbits) {
                tty->c_cflag &= (tcflag_t) ~(CSTOPB);
        }
        else {
                tty->c_cflag |= (tcflag_t) (CSTOPB);
        }
        return 0;
}

static int 
_serial_set_databits( 
        const int       databits, 
        struct termios *tty
){
        if ( !tty ) {
                return -EINVAL;
        }
        tty->c_cflag &= (tcflag_t) ~CSIZE; 
        tty->c_cflag |= (tcflag_t) databits;
        return 0;
}

static int 
_serial_set_flowcontrol( 
        const int       flowcontrol, 
        struct termios *tty
){
        if ( !tty ) {
                return -EINVAL;
        }

        switch ( flowcontrol ){
        default:
                return -EINVAL;
        case SERIAL_FLC_N:
                tty->c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
                tty->c_cflag &= (tcflag_t) ~(CRTSCTS);
                tty->c_cc[VSTART] = 0;                                                   // Disable start character (XON) - disable software flow control
                tty->c_cc[VSTOP] = 0;                                                    // Disable stop character (XOFF) - disable software flow control
                break;
        case SERIAL_FLC_H:
                tty->c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
                tty->c_cflag |= (tcflag_t) (CRTSCTS);
                tty->c_cc[VSTART] = 0;                                                   // Disable start character (XON) - disable software flow control
                tty->c_cc[VSTOP] = 0;                                                    // Disable stop character (XOFF) - disable software flow control
                break;
        case SERIAL_FLC_S:
                tty->c_iflag |= (tcflag_t) (IXON | IXOFF | IXANY);
                tty->c_cflag &= (tcflag_t) ~(CRTSCTS);
                tty->c_cc[VSTART] = 1;                                                   // Enable start character (XON) - enable software flow control
                tty->c_cc[VSTOP] = 1;                                                    // Enable stop character (XOFF) - enable software flow control
                break;
        }
        return 0;
}

/** 
@brief Set timeout for any IO operation. 
 */
int
serial_set_timeout(
        const int  timeout_ms,
        serial_t  *serial
) {
        if (!serial) {
                int err = -EINVAL; 
                LOG_SER(serial, err, LOG_ERROR, "serial_set_timeout");
                return err;
        }
        serial->cfg.timeout_ms = timeout_ms;
        return 0;
}

/** 
@brief Attempt to submit a config to the serial port. 
 */
int 
serial_set_config(
        const struct serial_config *config,        
        serial_t                   *serial
) {
        int err = _serial_nvalid(serial, 0);
        if (err) {
                LOG_SER(serial, err, LOG_ERROR, "serial_set_config");
                return err;
        }
        if ( !config ) {
                err = -EINVAL;
                LOG_SER(serial, err, LOG_ERROR, "serial_set_config");
                return err;
        }
        
        struct termios tty;
        if ( tcgetattr( serial->fd, &tty ) ) {
                err = -errno;
                LOG_SER(serial, err, LOG_ERROR, "tcgetattr");
                goto cleanup;
        }

        cfmakeraw(&tty);
        memset(&tty.c_cc, 0, sizeof(cc_t)*NCCS);

        if ( (err = _serial_set_baudrate((int)config->baudrate, &tty)) ){
                LOG_SER(serial, err, LOG_ERROR, "_serial_set_baudrate");
                goto cleanup;
        }
        if ( (err = _serial_set_parity((int)config->parity, &tty)) ){
                LOG_SER(serial, err, LOG_ERROR, "_serial_set_parity");
                goto cleanup;
        }
        if ( (err = _serial_set_stopbits((int)config->stopbits, &tty)) ){
                LOG_SER(serial, err, LOG_ERROR, "_serial_set_stopbits");
                goto cleanup;
        }
        if ( (err = _serial_set_databits((int)config->databits, &tty)) ){
                LOG_SER(serial, err, LOG_ERROR, "_serial_set_databits");
                goto cleanup;
        }
        if ( (err = _serial_set_flowcontrol((int)config->flowcontrol, &tty)) ){
                LOG_SER(serial, err, LOG_ERROR, "_serial_set_flowcontrol");
                goto cleanup;
        }        
        if ( tcsetattr( serial->fd, TCSANOW, &tty ) ){
                LOG_SER(serial, -errno, LOG_ERROR, "tcsetattr");
                return -errno;
        }
        
        (void) serial_set_timeout(config->timeout_ms, serial);
        return 0;
cleanup:
        return err;
}

static int
__serial_get_baudrate( 
        struct termios *tty
) {
        return tty ? (int) cfgetospeed(tty) : SERIAL_INV;
}

static int
__serial_get_parity( 
        struct termios *tty
) {
        if ( !tty ) {
                return SERIAL_INV;
        }
        if ( !(tty->c_iflag & (tcflag_t) INPCK) ) {
                return SERIAL_PN;
        }
        if ( !(tty->c_cflag & (tcflag_t) PARODD) ) {
                return SERIAL_PE;
        }
        return SERIAL_PO;
}

static int
__serial_get_stopbits( 
        struct termios *tty
) {
        if ( !tty ) {
                return SERIAL_INV;
        }
        return !( tty->c_iflag & (tcflag_t) CSTOPB ) ? SERIAL_SB1 : SERIAL_SB2;
}

static int
__serial_get_databits( 
        struct termios *tty
) {
        return tty ? (int) (tty->c_cflag & (tcflag_t) CSIZE) : SERIAL_INV;
}

static int
__serial_get_flowcontrol( 
        struct termios *tty
){
        if ( !tty ) {
                return SERIAL_INV;
        }
        if( !(tty->c_cflag & (tcflag_t) CRTSCTS) ){
                if( !(tty->c_iflag & (tcflag_t) (IXON | IXOFF | IXANY)) ) {
                        return SERIAL_FLC_N;
                }
                else {
                        return SERIAL_FLC_S;
                }
        }
        return SERIAL_FLC_H;
}

struct lut_config_entry {
        const char *text;       
        const int  value;
};

struct lut_config {
        const char                    *label;
        const struct lut_config_entry *entry;
        const size_t                  size;
};

static const struct lut_config_entry lut_baudrate_entry[ ] = {
        {"50",      SERIAL_B50     },
        {"75",      SERIAL_B75     },
        {"110",     SERIAL_B110    },
        {"134",     SERIAL_B134    },
        {"150",     SERIAL_B150    },
        {"200",     SERIAL_B200    },
        {"300",     SERIAL_B300    },
        {"600",     SERIAL_B600    },
        {"1200",    SERIAL_B1200   },
        {"1800",    SERIAL_B1800   },
        {"2400",    SERIAL_B2400   },
        {"4800",    SERIAL_B4800   },
        {"9600",    SERIAL_B9600   },
        {"19200",   SERIAL_B19200  },
        {"38400",   SERIAL_B38400  },
        {"57600",   SERIAL_B57600  },
        {"62500",   SERIAL_B62500  },
        {"115200",  SERIAL_B115200 },
        {"230400",  SERIAL_B230400 },
        {"460800",  SERIAL_B460800 },
        {"500000",  SERIAL_B500000 },
        {"576000",  SERIAL_B576000 },
        {"921600",  SERIAL_B921600 },
        {"1000000", SERIAL_B1000000},
        {"1152000", SERIAL_B1152000},
        {"1500000", SERIAL_B1500000},
        {"2000000", SERIAL_B2000000},
        {"failed",  SERIAL_INV},
};
static const struct lut_config_entry lut_parity_entry[ ] = {
        {"none",    SERIAL_PN},
        {"odd" ,    SERIAL_PO},
        {"even",    SERIAL_PE},
        {"failed",  SERIAL_INV},
};
static const struct lut_config_entry lut_flowcontrol_entry[ ] = {
        {"none",    SERIAL_FLC_N},
        {"hardware",SERIAL_FLC_H},
        {"software",SERIAL_FLC_S},
        {"failed",  SERIAL_INV},
};
static const struct lut_config_entry lut_databits_entry[ ] = {
        {"5",       SERIAL_DB5},
        {"6",       SERIAL_DB6},
        {"7",       SERIAL_DB7},
        {"8",       SERIAL_DB8},
        {"failed",  SERIAL_INV},
};
static const struct lut_config_entry lut_stopbits_entry[ ] = {
        {"1",       SERIAL_SB1},
        {"2",       SERIAL_SB2},
        {"failed",  SERIAL_INV},
};
static const struct lut_config_entry lut_seriallines_entry[ ] = {
        {"dsr",     SERIAL_DSR},
        {"dtr",     SERIAL_DTR},
        {"rts",     SERIAL_RTS},
        {"cts",     SERIAL_CTS},
        {"dcd",     SERIAL_DCD},
        {"failed",  SERIAL_INV},
};

#define LUT_SIZE(s) (sizeof(s)/sizeof(s[0]))
static const struct lut_config lut_baudrate    = {.label="baudrate",    .entry=lut_baudrate_entry,    .size=LUT_SIZE(lut_baudrate_entry)    };
static const struct lut_config lut_parity      = {.label="parity",      .entry=lut_parity_entry,      .size=LUT_SIZE(lut_parity_entry)      };
static const struct lut_config lut_flowcontrol = {.label="flow control",.entry=lut_flowcontrol_entry, .size=LUT_SIZE(lut_flowcontrol_entry) };
static const struct lut_config lut_databits    = {.label="data bits",   .entry=lut_databits_entry,    .size=LUT_SIZE(lut_databits_entry)    };
static const struct lut_config lut_stopbits    = {.label="stop bits",   .entry=lut_stopbits_entry,    .size=LUT_SIZE(lut_stopbits_entry)    }; 
static const struct lut_config lut_seriallines = {.label="",            .entry=lut_seriallines_entry, .size=LUT_SIZE(lut_seriallines_entry) };

static int
_serial_get_str4opcode( 
        const int                 value, 
        const struct lut_config  *table,
        char                     *field_label,
        char                     *field_value
) {
        if ( !table  ) { 
                return -EINVAL; 
        }
        int found = 0;
        for ( size_t i = 0 ; i < table->size ; ++i ) {
                if ( value == table->entry[i].value ) {
                        if (field_label) {
                                strcpy(field_label, table->label);
                                found += 1;
                        }
                        if (field_value) {
                                strcpy(field_value, table->entry[i].text);
                                found += 1;
                        }
                }
        }
        return found > 1 ? 0 : -EINVAL;
}

static int
_serial_set_generic(
        int (*set)(int, struct termios *),
        const char *fname,
        int         val,
        int        *field,
        serial_t   *serial
) {
        int err = _serial_nvalid(serial, 1);
        if (err) {
                LOG_SER(serial, err, LOG_ERROR, fname);
                return err;
        }
        if ( !set || !fname || !field ) {
                err = -EINVAL;
                LOG_SER(serial, err, LOG_ERROR, fname);
                return err;
        }
        
        struct termios tty;
        if ( tcgetattr(serial->fd, &tty) ) {
                err = -errno;
                LOG_SER(serial, err, LOG_ERROR, "tcgetattr");
                return err;
        }
        
        if ( (err = set(val, &tty)) ) {
                LOG_SER(serial, err, LOG_ERROR, fname);
                return err;
        }
        if ( tcsetattr(serial->fd, TCSANOW, &tty) ) {
                err = -errno;
                LOG_SER(serial, err, LOG_ERROR, "tcsetattr");
                return err;
        }
        *field = val;
        return 0;
}

#define SERIAL_SET_IMPL_X(type) \
        int serial_set_##type(const enum serial_##type val, serial_t *serial) { \
                return _serial_set_generic( \
                        _serial_set_##type, \
                        "_serial_set_" #type, \
                        (int)val, \
                        (int*)&(serial->cfg.type),\
                        serial \
                ); \
        }

SERIAL_LIST(SERIAL_SET_IMPL_X)

static int
_serial_get_generic(
        int (*get)(struct termios *),
        const char              *fname,
        struct termios          *tty,
        const struct lut_config *lut,
        int                     *field,
        serial_t                *serial
) {
        int err = 0;
        if ( !get || !tty || !fname || !lut || !field ) {
                err = -EINVAL;
                LOG_SER(serial, err, LOG_ERROR, fname);
                return err;
        }
        err = _serial_nvalid(serial, 1);
        if (err) {
                LOG_SER(serial, err, LOG_ERROR, fname);
                return err;
        }
        int val = (int) get( tty );
        if (SERIAL_INV == val) {
                err = -EINVAL;
                LOG_SER(serial, err, LOG_ERROR, "tcgetattr");
                return err;
        }
        _serial_get_str4opcode(val, lut, serial->rsv.label, serial->rsv.value);
        *field = val;
        return 0;
}

#define _SERIAL_GET_IMPL_X(type) \
        static int _serial_get_##type(struct termios *tty, serial_t *serial) { \
                return _serial_get_generic( \
                        __serial_get_##type, \
                        "__serial_get_" #type, \
                        tty, \
                        &lut_##type, \
                        (int*)&(serial->cfg.type),\
                        serial \
                ); \
        }

SERIAL_LIST(_SERIAL_GET_IMPL_X)

static const char *
_serial_get_generic_str(
        int (*get)(struct termios *, serial_t *),
        const char *fname,
        int        *err, 
        serial_t   *serial
) {
        int err_ = 0, *_err = err ? err : &err_; 
        if ( !get || !fname ) {
                *_err = -EINVAL;
                LOG_SER(serial, *_err, LOG_ERROR, fname);
                return NULL;
        }

        *_err = _serial_nvalid(serial, 1);
        if (*_err) {
                LOG_SER(serial, *_err, LOG_ERROR, fname);
                return NULL;
        }
        struct termios tty;
        if ( tcgetattr(serial->fd, &tty) ) {
                *_err = -errno;
                LOG_SER(serial, *_err, LOG_ERROR, "tcgetattr");
                return NULL;
        }
        *_err = get(&tty, serial);
        if ( !(*_err) ) { \
                char buf[sizeof(serial->rsv.value)];
                snprintf(buf, sizeof(buf), "%s%s: %s\n", buf, serial->rsv.label, serial->rsv.value);
                strncpy(serial->rsv.value, buf,  sizeof(serial->rsv.value));
                return serial->rsv.value;
        }
        return NULL;
}

#define SERIAL_GET_IMPL_X( type ) \
        const char *serial_get_##type(int *err, serial_t *serial) { \
                return _serial_get_generic_str( \
                        _serial_get_##type, \
                        "_serial_get_" #type, \
                        err, \
                        serial \
                ); \
        }

SERIAL_LIST(SERIAL_GET_IMPL_X)

/** 
@brief Attempt to get the current config from the serial port. 
 */
const char *
serial_get_config(
        serial_t *serial,
        int      *err 
) {
        int err_ = 0, *_err = err ? err : &err_;

        *_err = _serial_nvalid(serial, 1);
        if (*_err) {
                LOG_SER(serial, *_err, LOG_ERROR, "tcgetattr");
                return NULL;
        }

        struct termios tty;
        if ( tcgetattr(serial->fd, &tty) ) {
                LOG_SER(serial, -errno, LOG_ERROR, "tcgetattr");
                return NULL;
        }

        char buf[ sizeof(serial->rsv.value) ];
        size_t size = sizeof(serial->rsv.value), len = 0;
        #define _SERIAL_GET_ALL(type) \
                *_err = _serial_get_##type(&tty, serial); \
                if (!err) {  \
                        len += (size_t) snprintf(&buf[len], size-len, "%s: %s\n", serial->rsv.label, serial->rsv.value); \
                }

        SERIAL_LIST(_SERIAL_GET_ALL)
        len += (size_t) snprintf(&buf[len], size-len, "%s\n", serial_get_timeout(_err, serial) );                

        (void) strncpy( serial->rsv.value, buf, sizeof(serial->rsv.value) );
        return serial->rsv.value;
}

/** 
@brief Set the serial port defaut configuration. 
 */
int 
serial_default_config(
        struct serial_config *config 
) {
        if ( !config ) {
                return -EINVAL;
        }
        *config = (struct serial_config) {
                .baudrate     = SERIAL_B9600,
                .flowcontrol  = SERIAL_FLC_N,
                .parity       = SERIAL_PN,
                .databits     = SERIAL_DB8,
                .stopbits     = SERIAL_SB1,
                .timeout_ms   = 10
        };
        return 0;
}

/** 
@brief Get the timeout value saved. 
 */
const char * 
serial_get_timeout(
        int      *err,
        serial_t *serial
) {
        int err_ = 0, *_err = err ? err : &err_; 

        *_err = _serial_nvalid(serial, 1);
        if (*_err) {
                LOG_SER(serial, *_err, LOG_ERROR, "serial_get_timeout");
                return NULL;
        }
        (void) snprintf( serial->rsv.value, sizeof(serial->rsv.value), "timeout: %d", serial->cfg.timeout_ms);
        return serial->rsv.value;
}

/** 
@brief Set the serial lines. 
 */
int
serial_set_lines(
        const struct serial_lines *table,
        serial_t                  *serial 
) {
        int err = 0;
        if ( (err =_serial_nvalid(serial, 1)) ) {
                goto cleanup;
        }
        if (!table) {
                err = -EINVAL;
                goto cleanup;
        }
        int status;
        if ( -1 == ioctl(serial->fd, TIOCMGET, &status) ) {
                err = -errno;
                goto cleanup;
        }
        for ( size_t i = 0; i < table->size; ++i ){
                struct serial_line_state *line = &(table->lines[i]);
                int offs = (int) line->offs;
                status = line->state ? status|offs : status&(~offs);
        }
        if ( -1 == ioctl(serial->fd, TIOCMSET, &status) ) {
                err = -errno;
                goto cleanup;
        }
        return 0;

cleanup:
        LOG_SER(serial, err, LOG_ERROR, "serial_set_lines");
        return err;
}

int 
serial_set_line(
        struct serial_line_state  entry, 
        serial_t                 *serial
) {
        struct serial_lines table = {
                .lines=&entry, 
                .size=1
        };
        return serial_set_lines(&table, serial);
}



/** 
@brief Get the serial lines. 
 */
const char * 
serial_get_lines( 
        serial_t *serial 
) {
        int err = 0;
        if ( (err =_serial_nvalid(serial, 1)) ) {
                goto cleanup;
        }
        int status;
        if ( -1 == ioctl(serial->fd, TIOCMGET, &status) ) {
                err = -errno;
                goto cleanup;
        }
        int opts[] = {SERIAL_DSR, SERIAL_DTR, SERIAL_RTS, SERIAL_CTS, SERIAL_DCD};
        int nelems = sizeof(opts)/sizeof(opts[0]);

        char buf[sizeof(serial->rsv.value)];
        for (int i = 0; i < nelems; ++i) {
                if (status & opts[i]) {
                        err =_serial_get_str4opcode( opts[i], &lut_seriallines, serial->rsv.label, serial->rsv.value );
                        if ( err ) {
                                goto cleanup;
                        }
                        snprintf( buf, sizeof(buf), "%s%s, ", buf, serial->rsv.value);
                }
        }
        strncpy( serial->rsv.value, buf, sizeof(serial->rsv.value) );
        return serial->rsv.value;
cleanup:
        LOG_SER(serial, err, LOG_ERROR, "serial_get_lines");
        return NULL;
}


static int
_serial_get_udev_values(
        struct udev *udev,
        serial_t    *serial
) {
        struct udev_device *dev = NULL;
        int err = 0;
        
        if (!serial || !udev) {
                err = -EINVAL;
                LOG_SER(serial, err, LOG_ERROR, "serial_set_udev");
                goto cleanup;
        }
        const char *basename = strrchr(serial->udev.path, '/');
        if ( !basename ) {
                err = -EINVAL;
                LOG_SER(serial, err, LOG_ERROR, "strrchr");
                goto cleanup;
        }
        basename++;   

        dev = udev_device_new_from_subsystem_sysname(udev, "tty", basename);
        if (!dev) {
                err = -errno;
                LOG_SER(serial, err, LOG_ERROR, "udev_device_new_from_subsystem_sysname");
                goto cleanup;
        }

        struct udev_device *parent = udev_device_get_parent(dev);
        if ( !parent ){
                err = -errno;
                goto cleanup;
        }

        const char *subsys = udev_device_get_subsystem(parent);
        if ( subsys ) {
                (void) strncpy( serial->udev.bus, subsys, sizeof(serial->udev.bus) );
                for (size_t i = 0; i < serial->udev.size; ++i ) {
                        struct serial_udev_field *field = &(serial->udev.fields[i]);
                        const char *value  = udev_device_get_property_value(dev, field->label);          
                        (void) snprintf(field->value, sizeof(field->value), "%s", value ? value : "");
                }
                err = 0;
        }

cleanup:
        if (dev) {
                udev_device_unref( dev );
        }
        return err;       
}

/** 
@brief Set the udev list with parameters to obtain for a given serial port. 
 */
int
serial_set_udev(
        struct serial_udev *params,
        serial_t           *serial
) {
        struct udev *udev = NULL;
        int err = 0;
        if ( (err =_serial_nvalid(serial, 1)) ) {
                LOG_SER(serial, err, LOG_ERROR, "_serial_nvalid");
                goto cleanup;
        }
        if ( !params ) {
                err = -EINVAL;
                LOG_SER(serial, err, LOG_ERROR, "serial_set_udev");
                goto cleanup;
        }
        strncpy( params->path, serial->udev.path, sizeof(serial->udev.path)); 
        serial->udev = *params;

        udev = udev_new( );
        if ( !udev ){
                err = -errno;
                LOG_SER(serial, err, LOG_ERROR, "udev_new");
                goto cleanup;
        }
        if ( (err = _serial_get_udev_values(udev, serial)) ){
                LOG_SER(serial, err, LOG_ERROR, "_serial_get_udev_values");
                goto cleanup;
        }
        err = 0;
cleanup:
        if (udev) {
                udev_unref( udev );
        }
        return err;       
}

/** 
@brief Get the values from the udev list previously set. 
 */
const char *
serial_get_udev(
        int      *err,
        serial_t *serial
) {
        int err_ = 0, *_err = err ? err : &err_;
        if ( (*_err =_serial_nvalid(serial, 1)) ) {
                goto cleanup;
        }
        struct serial_udev *table = &(serial->udev);
        if ( !table->size ) {
                *_err = -EINVAL;
                goto cleanup;
        }

        char *buf = serial->rsv.value;
        size_t size = sizeof(serial->rsv.value), len = 0; 
        for (size_t i = 0; i < table->size; ++i) {
                struct serial_udev_field *field = &(table->fields[i]);
                len += (size_t) snprintf( &buf[len], size-len, "%s: %s\n", field->label, field->value);
        }
        return serial->rsv.value;
cleanup:
        LOG_SER(serial, *_err, LOG_ERROR, "serial_get_udev");
        return NULL;       
}

static ssize_t
_serial_read(
        void          *buf,
        const size_t   size, 
        const ssize_t  nmemb, 
        serial_t      *serial,
        int           *err
) {
        int err_ = 0, *_err = err ? err : &err_;
        size_t length = (size_t) nmemb * size; 
        ssize_t wrote = read(serial->fd, buf, length);
        if (-1 == wrote) {
                *_err = -errno;
                return -1;
        }
        return wrote;
}

static int
_serial_epoll_loop(
        int       epoll_fd,
        const int timeout_ms
) {
        struct epoll_event event;
        int err = epoll_wait(epoll_fd, &event, 1, timeout_ms);
        if (-1 == err) {
                return -errno;
        }
        else {
                if (!err) {
                        return -ETIME;
                } 
                if (event.events & (EPOLLHUP | EPOLLERR)) {
                        return -ENODEV;
                } 
                if (event.events & (EPOLLIN | EPOLLOUT)) {
                        return 0;
                }
        }
        return -ETIME;
}

static int32_t
_serial_remaining_time(
        int64_t          total,
        struct timespec *start,
        struct timespec *now        
) {
        if (!start || !now) {
                return 0;
        }
        clock_gettime(CLOCK_MONOTONIC, now);
        int64_t total_ns = (int64_t)total * 1000000LL;
        int64_t start_ns = (int64_t)start->tv_sec  * 1000000000LL + start->tv_nsec;
        int64_t now_ns   = (int64_t)now->tv_sec    * 1000000000LL + now->tv_nsec;
        int64_t remaining_ns = total_ns - (now_ns - start_ns);
        return (int)((remaining_ns + 999999LL) / 1000000LL);
}

static ssize_t
_serial_read_epoll(
        void          *buf,
        const size_t   size, 
        const ssize_t  nmemb, 
        const int      timeout_ms,
        serial_t      *serial,
        int           *err
) {
        int err_ = 0, *_err = err ? err : &err_;

        int total_timeout = !timeout_ms ? serial->cfg.timeout_ms : timeout_ms;
        ssize_t totalbytes = nmemb * (ssize_t) size, writtenbytes = 0, readbytes;

        struct timespec start, now;
        clock_gettime(CLOCK_MONOTONIC, &start);

        uint8_t *u8_buf = (uint8_t *) buf;
        while (writtenbytes < totalbytes) {
                int remaining_ms = _serial_remaining_time(total_timeout, &start, &now);
                if (0 >= remaining_ms) {
                        *_err = -ETIME;
                        LOG_SER(serial, *_err, LOG_ERROR, "_serial_read");
                        return (writtenbytes > 0) ? writtenbytes : 0;    
                }

                readbytes = totalbytes - writtenbytes; 
                *_err = _serial_epoll_loop(serial->epoll.rx, remaining_ms);
                if (*_err) {
                        LOG_SER(serial, *_err, LOG_DEBUG, "_serial_epoll_loop");
                        return (writtenbytes > 0) && (-ETIME == *_err) ? writtenbytes : -1;    
                }
                ssize_t wrote = _serial_read( &u8_buf[writtenbytes], 1, readbytes, serial, _err);
                if ( -1 == wrote ) {
                        LOG_SER(serial, *_err, LOG_ERROR, "_serial_read");
                        return (writtenbytes > 0) && (-ETIME == *_err) ? writtenbytes : -1;
                }
                writtenbytes += wrote;
        }
        return writtenbytes;
}

/** 
@brief Read from the serial port. 
 */
ssize_t
serial_read(
        void          *buf,
        const size_t   size, 
        const ssize_t  nmemb, 
        const int      timeout_ms,
        serial_t      *serial,
        int           *err
) {
        int err_ = 0, *_err = err ? err : &err_;
        if ( (*_err = _serial_nvalid(serial, 1)) ) {
                LOG_SER(serial, *_err, LOG_ERROR, "serial_read");
                return -1;
        }
        if ( !nmemb || !buf ) {
                *_err = -EINVAL;
                LOG_SER(serial, *_err, LOG_ERROR, "serial_read");
                return -1;
        }
        ssize_t wrote = 0;
        if ( -1 == timeout_ms ) {
                wrote = _serial_read(buf, size, nmemb, serial, _err);
        }
        else {
                wrote = _serial_read_epoll(buf, size, nmemb, timeout_ms, serial, _err);
        }
        return wrote;               
}

/** 
@brief Read from the serial port until a certain group of characters are found. 
 */
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
) {
        int err_ = 0, *_err = err ? err : &err_;

        if ( (*_err = _serial_nvalid(serial, 1)) ) {
                LOG_SER(serial, *_err, LOG_ERROR, "serial_read_delim");
                return -1;
        }
        if (!nmemb || !buf || !nmemb_delim || !delim) {
                *_err = -EINVAL;
                LOG_SER(serial, *_err, LOG_ERROR, "serial_read_delim");
                return -1;
        }

        uint8_t *u8_buf = (uint8_t *)buf;
        ssize_t totalbytes = nmemb * (ssize_t) size, writtenbytes = 0;
        while (writtenbytes < totalbytes) {
                ssize_t wrote = serial_read(&u8_buf[writtenbytes], 1, nmemb_delim, timeout_ms, serial, _err);
                if (0 >= wrote) {
                        return (writtenbytes > 0) && (-ETIME == *_err) ? writtenbytes : -1;
                }
                ssize_t start_check = (writtenbytes < nmemb_delim) ? 0 : (writtenbytes - nmemb_delim + 1);
                writtenbytes += wrote;
                for (ssize_t idx = start_check; idx <= (writtenbytes - nmemb_delim); ++idx) {
                        if ( !memcmp(&u8_buf[idx], delim, (size_t) nmemb_delim) ) {
                                return writtenbytes; 
                        }
                }
        }
        return writtenbytes;
}

/** 
@brief Read until \n or until the timeout is trigger. 
 */
ssize_t
serial_readl(
        char          *buf,
        const ssize_t  length, 
        const int      timeout_ms,
        serial_t      *serial,
        int           *err
) {
        uint8_t delim = '\n'; 
        return serial_read_delim(buf, 1, length, &delim, 1, timeout_ms, serial, err);
}

static ssize_t 
_serial_write(
        const void    *buf,
        const size_t   size, 
        const ssize_t  nmemb, 
        serial_t      *serial,
        int           *err
) {
        int err_ = 0, *_err = err ? err : &err_;

        size_t length = (size_t) nmemb * size;
        ssize_t wrote = write(serial->fd, buf, length);
        if (-1 == wrote) {
                *_err = -errno;
                return -1;
        }
        return wrote;
}

static ssize_t
_serial_write_epoll(
        const void    *buf,
        const size_t   size, 
        const ssize_t  nmemb, 
        const int      timeout_ms,
        serial_t      *serial,
        int           *err
) {
        int err_ = 0, *_err = err ? err : &err_;

        int timeout_ms_val = !timeout_ms ? serial->cfg.timeout_ms : timeout_ms;
        *_err = _serial_epoll_loop(serial->epoll.tx, timeout_ms_val);
        if (*_err) {
                LOG_SER(serial, *_err, LOG_DEBUG, "_serial_epoll_loop");
                return -1;    
        }

        
        ssize_t wrote = _serial_write(buf, size, nmemb, serial, _err);
        if ( -1 == wrote ) {
                LOG_SER(serial, *_err, LOG_ERROR, "_serial_write");
                return -1;
        }
        return wrote;       
}


/** 
@brief Write a sequence of bytes. 
 */
ssize_t
serial_write(
        const void    *buf,
        const size_t   size, 
        const ssize_t  nmemb, 
        serial_t      *serial,
        const int      timeout_ms,
        int           *err
) {
        int err_ = 0, *_err = err ? err : &err_;
        
        if ( (*_err = _serial_nvalid(serial, 1)) ) {
                LOG_SER(serial, *_err, LOG_ERROR, "serial_write");
                return -1;
        }
        if ( !nmemb || !buf ) {
                *_err = -EINVAL;
                LOG_SER(serial, *_err, LOG_ERROR, "serial_write");
                return -1;
        }
        ssize_t wrote = 0;
        if ( -1 == timeout_ms ) {
                wrote = _serial_write(buf, size, nmemb, serial, _err);
        }
        else {
                wrote = _serial_write_epoll(buf, size, nmemb, timeout_ms, serial, _err);
        }
        return wrote;               
}


ssize_t 
serial_writef( 
        serial_t   *serial, 
        const int  timeout_ms,
        int        *err,
        const char *fmt, 
        ...
){
        int err_ = 0, *_err = err ? err : &err_;
        ssize_t ret = -1;
        
        if ( (*_err = _serial_nvalid(serial, 1)) ) {
                LOG_SER(serial, *_err, LOG_ERROR, "serial_writef");
                goto cleanup;
        }
        if ( !fmt ) {
                *_err = -EINVAL;
                LOG_SER(serial, *_err, LOG_ERROR, "serial_writef");
                goto cleanup;
        }

        va_list args;
        va_start(args, fmt);

        char buf[1024];
        int len = vsnprintf(buf, sizeof(buf), fmt, args);

        if ( 0 > len ) {
                *_err = -EINVAL;
                LOG_SER(serial, *_err, LOG_ERROR, "serial_writef");
                goto cleanup;
        }
        if ( (int) sizeof(buf) <= len ) {
                *_err = -EOVERFLOW;
                LOG_SER(serial, *_err, LOG_ERROR, "serial_writef");
                goto cleanup;
        }

        ret = serial_write(buf, 1, (ssize_t) len, serial, timeout_ms, _err);
cleanup:
        va_end(args);
        return ret;
}
