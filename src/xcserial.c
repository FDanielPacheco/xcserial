/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      xcserial.c
 * 
 * @version   1.0.1
 *
 * @date      18-09-2025
 *
 * @brief     Functions providing control, local settings, input, and output for serial ports on Linux, library developed for interacting with embedded systems in mind (non-cannonical).  
 *  
 * @author    Fábio D. Pacheco, 
 * @email     fabio.d.pacheco@inesctec.pt or pacheco.castro.fabio@gmail.com
 *
 * @copyright Copyright (c) [2025] [Fábio D. Pacheco]
 * 
 * @note      Manuals:
 *            https://man7.org/linux/man-pages/man2/TIOCMSET.2const.html \n 
 *            https://people.na.infn.it/~garufi/didattica/CorsoAcq/SerialProgrammingInPosixOSs.pdf \n
 *            https://man7.org/linux/man-pages/man3/errno.3.html
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Imported libraries
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#include <stdio.h>    
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>    
#include <string.h>  
#include <fcntl.h>
#include <unistd.h>
#include <errno.h> 
#include <termios.h>
#include <sys/ioctl.h>
#include <libudev.h>

#include <xcserial.h>

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Local Types
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

typedef struct{
        char       text[ NAME_MAX ];
        baudrate_t code;
} lut_t;

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Private enums
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

typedef enum{
        RX = 0, TX = 1,
} serial_direction_t;

typedef enum {
        FULL = 0, QUICK = 1, 
} serial_check_t;

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Local Function Prototype
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

//!< Udev API wrapper
int8_t clear_field_id( const char * field, const size_t length, serial_id_t * identificator );
int8_t add_field_id( const char * field, const size_t length, serial_id_t * identificator );
int8_t get_ids( const char * pathname, serial_id_t * identificator, struct udev * udev );
int8_t cmp_ids( serial_id_t * id1, serial_id_t * id2 );
int8_t cpy_fields_ids( serial_id_t * dst, const serial_id_t * src );
int8_t serial_get_udev_devs_list( char devs[ ][PATH_MAX], uint8_t size, size_t length, uint8_t * ndevs, struct udev * udev );
int8_t serial_find_udev( char * path, size_t psize, struct udev * udev, serial_id_t * id );

//!< Termios API wrapper
int8_t get_termios( int fd, struct termios * tty );
int8_t apply_termios( int fd, struct termios * tty );
int8_t _serial_set_baudrate( const baudrate_t baudrate, struct termios * tty );
int8_t _serial_set_parity( const parity_t parity, struct termios * tty );
int8_t _serial_set_stopbits( const stop_bits_t stop_bits, struct termios * tty );
int8_t _serial_set_databits( const data_bits_t data_bits, struct termios * tty );
int8_t _serial_set_flowcontrol( const flow_control_t flow_control, struct termios * tty );
int8_t _serial_set_rule( const uint8_t timeout, const uint8_t min, struct termios * tty );
baudrate_t _serial_get_baudrate( struct termios * tty );
parity_t _serial_get_parity( struct termios * tty );
stop_bits_t _serial_get_stopbits( struct termios * tty );
data_bits_t _serial_get_databits( struct termios * tty );
flow_control_t _serial_get_flowcontrol( struct termios * tty );

//!< Error handler
int8_t fs_error( serial_t * serial, size_t call_ret );
//!< Internal serial write wrapper
size_t _serial_write( serial_t * serial, const void * data, const size_t len );
size_t _serial_read( void * data, const size_t len, serial_t * serial );
size_t _serial_readline( char * data, const size_t len, serial_t * serial );

//!< Asyncronous method for unix, stdio mode
void * async_epoll_thread( void * arg );

//!< Look up tables helpers
const char *get_stringlut_from_code( void * code, const lut_t * table, const size_t nitems, const size_t size );
const char *get_baudrate_from_code( const baudrate_t code );
const char *get_flow_control_from_code( const flow_control_t code );
const char *get_parity_from_code( const parity_t code );
const char *get_data_bits_from_code( const data_bits_t code );
const char *get_stop_bits_from_code( const stop_bits_t code );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Waits for and returns the triggered serial port event.
 *
 * The `serial_manager_t` must be initialized with `serial_manage` prior to calling this function.
 *
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * @param[in] timeout Timeout in milliseconds (-1 for indefinite blocking).
 * @param[in] side If 0 the input trigger will listened, otherwise the output trigger.
 *
 * @return On success, 0 is returned if data is available for reading (EPOLLIN). 1 is returned if the serial port is ready for writing (EPOLLOUT).
 *         On error, the function returns -1 and sets `errno` to indicate the error, this can represent timeout.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_event_wait( serial_t * serial, const int timeout, const serial_direction_t side );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief  Connects the serial port with epoll for event-driven operation. \n
 * Instead of continuous polling, epoll allows the application to sleep until a serial port event occurs, drastically reducing CPU saturation during idle periods.
 *  
 * @param[out] serial The serial port structure (`serial_t`) to be filled.
 * 
 * @return Upon success, the serial port is attached to epoll for event-driven operation, and the 0 is returned. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 *
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_event_enable( serial_t * serial );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Checks if a serial port is open and valid.
 *
 * This function checks if the file descriptor associated with the given serial port structure is valid and open.
 *
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon validating the serial port, it returns 1. \n 
 *         Otherwise, 0 is returned and `errno` is set to indicate a possible error.
 * 
 *  - `EINVAL`: Invalid argument
 *
 * @note This function only checks the validity of the file descriptor.  It does not guarantee that the serial port is actually ready for communication.  
 *       There might be other issues (e.g., hardware problems) that prevent successful communication even if this function returns `1`.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
uint8_t serial_valid( const serial_t * serial, const serial_check_t check );

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Local Macros
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#define error_print( txt, ... ) fprintf( stderr, "Error: " txt ", at line %d in file %s\nErrno: %d, %s\n", ##__VA_ARGS__, __LINE__, __FILE__, errno, strerror(errno) )

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Lookup tables
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

static const 
lut_t lut_baudrate[ ] = {
        {"0",       B0      },
        {"50",      B50     },
        {"75",      B75     },
        {"110",     B110    },
        {"134",     B134    },
        {"150",     B150    },
        {"200",     B200    },
        {"300",     B300    },
        {"600",     B600    },
        {"1200",    B1200   },
        {"1800",    B1800   },
        {"2400",    B2400   },
        {"4800",    B4800   },
        {"9600",    B9600   },
        {"19200",   B19200  },
        {"38400",   B38400  },
        {"57600",   B57600  },
        {"62500",   B62500  },
        {"115200",  B115200 },
        {"230400",  B230400 },
        {"460800",  B460800 },
        {"500000",  B500000 },
        {"576000",  B576000 },
        {"921600",  B921600 },
        {"1000000", B1000000},
        {"1152000", B1152000},
        {"1500000", B1500000},
        {"2000000", B2000000},
        {"failed",  _BINV},
};

static const 
lut_t lut_parity[ ] = {
        {"none", BPARITY_NONE},
        {"odd" , BPARITY_ODD },
        {"even", BPARITY_EVEN},
        {"failed", _BPARITY_INV},
};

static const
lut_t lut_flow_control[ ] = {
        {"none"    , FLOWCONTROL_NONE    },
        {"hardware", FLOWCONTROL_HARDWARE},
        {"software", FLOWCONTROL_SOFTWARE},
        {"failed",   _FLOWCONTROL_INV},
};

static const
lut_t lut_data_bits[ ] = {
        {"5", DATA_BITS_5},
        {"6", DATA_BITS_6},
        {"7", DATA_BITS_7},
        {"8", DATA_BITS_8},
        {"failed", _DATA_BITS_INV},
};

static const
lut_t lut_stop_bits[ ] = {
        {"1", STOP_BITS_1},
        {"2", STOP_BITS_2},
        {"failed", _STOP_BITS_INV},
};

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Function Description
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_open( 
        serial_t * serial, 
        const char * pathname, 
        serial_open_opts_t * opts
){ 
        if ( !serial ){
                errno = EINVAL;
                error_print( "serial struct is `NULL`" );
                return -1;
        }

        if ( !pathname ){
                errno = EINVAL;
                error_print( "pathname is `NULL`" );
                return -1;
        }

        if ( strlen(pathname) >= PATH_MAX ){
                errno = ENAMETOOLONG;
                error_print( "pathname is greater than %d", PATH_MAX );
                return -1;
        }

        uint8_t            readonly = !opts ? 0            : opts->readonly; 
        serial_config_t  * config   = !opts ? NULL         : opts->config; 
        serial_id_t      * id       = !opts ? NULL         : opts->id; 
        serial_async_t   * async    = !opts ? NULL         : opts->async;
        serial_iomode_t    iomode   = !opts ? SERIAL_STDIO : opts->iomode;

        memset( serial, 0, sizeof(serial_t) );

        if ( NULL != config ) {
                readonly = config->readonly;
        }

        if ( readonly ) {
                serial->fd = open( pathname, O_RDONLY | O_NOCTTY );
        }
        else {
                serial->fd = open( pathname, O_RDWR | O_NOCTTY );
        }

        if ( -1 == serial->fd ){
                error_print( "open(%s)", pathname );
                return -1;
        }

        if ( readonly ) {
                serial->fp = fdopen( serial->fd, "r" );
        }
        else {
                serial->fp = fdopen( serial->fd, "r+" );
        }

        if ( !serial->fp ){
                error_print( "fdopen" );
                if (-1 != serial->fd) {
                        close( serial->fd );
                }
                return -1;
        } 

        setbuf(serial->fp, NULL);

        if ( !strncpy( serial->pathname, pathname, sizeof(serial->pathname) - 1 ) ) {
                return -1;
        }

        serial->config.readonly = (readonly != 0);

        if ( !config ){
                serial_config_t config_default;
                if ( -1 == serial_default_config( &config_default ) ){
                        error_print( "serial_default_config" );
                        serial_close( serial );
                        return -1;
                }

                if ( -1 == serial_set_config( &config_default, serial ) ){
                        error_print( "serial_set_config" );
                        serial_close( serial );
                        return -1;
                }
        }
        else {
                if ( -1 == serial_set_config( config, serial ) ){
                        error_print( "serial_set_config" );
                        serial_close( serial );
                        return -1;
                }
        }

        if ( -1 == serial_event_enable( serial ) ){
                error_print( "config_update" );
                serial_close( serial );
                return -1;
        }

        if ( NULL != id ){
                struct udev * udev = udev_new( );
                if ( !udev ){
                        error_print( "udev_new" );
                        serial_close( serial );
                        return -1;
                }

                if ( -1 == get_ids( serial->pathname, id, udev ) ){
                        error_print( "get_ids" );
                        serial_close( serial );
                        return -1;
                }

                if ( -1 == serial_set_udev_id( id, serial ) ){
                        error_print( "serial_set_udev_id" );
                        serial_close( serial );
                        return -1;
                }

                udev_unref( udev );
        }

        if ( NULL != async ) {
                memcpy( &(serial->async), async, sizeof(serial_async_t) );
        }

        serial->async.close = 1;
        serial->iomode = iomode;
        return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_reopen( 
        serial_t * serial, 
        uint16_t iterations
){
        if ( !serial ){
                errno = EINVAL;
                return -1;    
        }

        struct udev * udev = udev_new();  
        if ( !udev ){
                error_print( "udev_new" );
                return -1;
        }

        serial_id_t id;
        memset( &id, 0, sizeof(serial_id_t) );
        cpy_fields_ids( &id, &serial->id );

        serial_close( serial );

        char pathname[PATH_MAX];
        int8_t found = -1;

        for ( uint16_t j = 0 ; j < iterations ; ++j ){
                if ( !(found = serial_find_udev( pathname, sizeof(pathname), udev, &id )) ) {
                        break;
                }
                sleep(2);
        }

        udev_unref( udev );

        if ( -1 == found ) {
                return -1;
        }

        serial_t tmp;
        serial_open_opts_t opts = {
                .readonly = serial->config.readonly,
                .config   = &(serial->config),
                .id       = &(serial->id),
                .async    = &(serial->async),
                .iomode   = serial->iomode,
        };

        if ( -1 == serial_open( &tmp, pathname, &opts ) ){          
                if ( EBUSY == errno ){
                        error_print("serial_open\n");
                        return -1;
                }
        }
        else {
                if ( !serial_valid( &tmp, FULL ) || (-1 == serial_set_databits( serial->config.data_bits, &tmp )) ){
                        serial_close( &tmp );
                        error_print("serial_open\n");
                        return -1;
                }          

                memcpy( serial, &tmp, sizeof(serial_t) );
                return 0;
        }

        return -1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_find_udev( 
        char        * path,
        size_t        psize,
        struct udev * udev,
        serial_id_t * id
){
        if ( !id || !udev || !path ) {
                return -1;
        }

        struct udev_enumerate * enumerate = udev_enumerate_new(udev);
        if ( !enumerate ) {
                return -1;
        }

        udev_enumerate_add_match_subsystem(enumerate, "tty");
        for ( int i = 0 ; i < id->ndev ; ++i ) {
                udev_enumerate_add_match_property(enumerate, id->dev[i].field, id->dev[i].value);
        }

        udev_enumerate_scan_devices(enumerate);
        struct udev_list_entry * devices = udev_enumerate_get_list_entry(enumerate);
        struct udev_list_entry * entry;

        int8_t found = -1;

        udev_list_entry_foreach( entry, devices ){
                const char * syspath = udev_list_entry_get_name( entry );
                struct udev_device * dev = udev_device_new_from_syspath( udev, syspath );
                if ( !dev ) {
                        continue;
                }
                  
                const char * devnode = udev_device_get_devnode( dev );
                if ( devnode ){
                        strncpy( path, devnode, psize);
                        found = 0;
                        udev_device_unref(dev);
                        break; 
                }
                udev_device_unref(dev);
        }

        udev_enumerate_unref(enumerate);
        return found;      
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_udev_id( 
        const serial_id_t * id, 
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) || !id ){
                errno = EINVAL;
                return -1;
        }
        memcpy( &(serial->id), id, sizeof(serial_id_t) );
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_get_udev_devs_list( 
        char devs[ ][PATH_MAX], 
        uint8_t size, 
        size_t length, 
        uint8_t * ndevs,
        struct udev * udev
){
        if ( !devs ){
                errno = EINVAL;
                return -1;
        }

        struct udev_enumerate * enumerate = udev_enumerate_new( udev );
        if ( !enumerate ){
                error_print( "udev_enumerate_new" );
                return -1;
        }

        if ( 0 > udev_enumerate_add_match_subsystem( enumerate, "tty" ) ){
                error_print( "udev_enumerate_add_match_subsystem" );
                udev_enumerate_unref( enumerate );
                return -1;
        }

        if ( 0 > udev_enumerate_scan_devices( enumerate ) ){
                error_print( "udev_enumerate_scan_devices" );
                udev_enumerate_unref( enumerate );
                return -1;
        }

        struct udev_list_entry * devices = udev_enumerate_get_list_entry( enumerate );
        if ( !devices ){
                error_print( "udev_enumerate_get_list_entry" );
                udev_enumerate_unref( enumerate );
                return -1;
        }

        *ndevs = 0;
        struct udev_list_entry * entry;

        udev_list_entry_foreach( entry, devices ){
                const char * pathname = udev_list_entry_get_name( entry );
                if ( !pathname ) {
                        continue;
                }

                struct udev_device * dev = udev_device_new_from_syspath( udev, pathname );
                if ( !dev ){
                        error_print( "udev_device_new_from_syspath" );
                        udev_enumerate_unref( enumerate );
                        return -1;
                }

                const char * devnode = udev_device_get_devnode( dev );
                if ( devnode ){
                        if ( size <= *ndevs ){
                                errno = ENOBUFS;
                                break;
                        }
                        else if ( !strncpy( devs[ (*ndevs)++ ], devnode, length ) ) {
                                break;
                        }
                }
                udev_device_unref( dev );
        }
        udev_enumerate_unref( enumerate );
        return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_udev_param_list( 
        const char params[][NAME_MAX], 
        uint8_t n, 
        size_t length, 
        serial_t * serial
){
        if( !serial || !params ){
                errno = EINVAL;
                return -1;
        } 

        serial->id.ndev = 0;

        for ( uint8_t i = 0 ; i < n ; ++i ){
                const char * param = params[i];
                if ( !param ) {
                        continue;
                }
                
                if ( -1 == add_field_id( param, length, &serial->id ) ) {
                        break;
                }
        }

        struct udev * udev = udev_new( );
        if ( !udev ){
                error_print("udev_new");
                return -1;
        }

        if ( -1 == get_ids( serial->pathname, &serial->id, udev ) ){
                error_print("get_ids");
                return -1;
        }

        udev_unref( udev );
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
clear_field_id( 
        const char * field, 
        const size_t length, 
        serial_id_t * identificator
){
        if ( !identificator || !field ){
                errno = EINVAL;
                return -1;
        }

        uint8_t ndev = identificator->ndev;

        for ( uint8_t i = 0 ; i < ndev ; ++i ){
                if ( !strncmp( field, identificator->dev[ i ].field, length ) ){
                        for ( uint8_t j = i ; j < ndev - 1 ; ++j ){
                                strcpy( identificator->dev[ i ].field, identificator->dev[ i + 1 ].field );
                                strcpy( identificator->dev[ i ].value, identificator->dev[ i + 1 ].value );
                        }
                }
        }
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
add_field_id( 
        const char * field, 
        const size_t length, 
        serial_id_t * identificator
){

        if ( !identificator || !field ){
                errno = EINVAL;
                return -1;
        }

        if ( !strncpy( identificator->dev[ identificator->ndev ].field, field, length ) ) {
                return -1;
        }

        identificator->ndev++;
        return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
get_ids( 
        const char * pathname, 
        serial_id_t * identificator,
        struct udev * udev
){
        if ( !pathname || !identificator ){
                errno = EINVAL;
                return -1;
        }

        const char * basename = strrchr( pathname, '/');
        if ( !basename ) {
                return -1;
        }

        basename++;   

        struct udev_device * dev = udev_device_new_from_subsystem_sysname( udev, "tty", basename );
        if ( !dev ) {
                return -1;
        }

        struct udev_device * parent = udev_device_get_parent( dev );
        if ( !parent ){
                udev_device_unref( dev );
                return -1;
        }

        const char * subsys = udev_device_get_subsystem( parent );
        if ( subsys ){
                if ( NULL != strncpy( identificator->bus, subsys, NAME_MAX ) ) {
                        for ( uint8_t i = 0 ; i < identificator->ndev ; ++i ){
                                serial_udev_paramater_t * parameter = &(identificator->dev[i]);

                                const char * value = udev_device_get_property_value( dev, parameter->field );          
                                const char * svalue = value ? value : " ";
                                if ( !strncpy( parameter->value, svalue, NAME_MAX ) ) {
                                        break;
                                }
                        }
                }
        }

        udev_device_unref( dev );
        return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
cmp_ids( 
        serial_id_t * id1, 
        serial_id_t * id2
){
        if ( !id1 || !id2 ) {
                errno = EINVAL;
                return -1;
        }

        uint8_t n = id1->ndev;
        if ( id1->ndev > id2->ndev ) {
                n = id2->ndev;
        }

        if ( !n ) {
                errno = EINVAL;
                return -1;
        }

        for ( uint8_t i = 0 ; i < n ; ++i ) {
                if ( 0 != strcmp( id1->dev[i].value, id2->dev[i].value ) ) {
                        return 0;
                }
        }

        return 1;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
cpy_fields_ids( 
        serial_id_t * dst, 
        const serial_id_t * src
){
        if ( !src || !dst ){
                errno = EINVAL;
                return -1;
        }

        for ( uint8_t i = 0 ; i < src->ndev ; ++i ){
                if ( !strncpy( dst->dev[ i + dst->ndev ].field, src->dev[i].field, NAME_MAX ) ) {
                        return -1;
                }
                if ( !strncpy( dst->dev[ i + dst->ndev ].value, src->dev[i].value, NAME_MAX ) ) {
                        return -1;
                }
        }

        dst->ndev += src->ndev;
        return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_udev_param_field( 
        const char * field, 
        size_t length, 
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        if ( !field || !length ){
                errno = EINVAL;
                return -1;
        }

        char params[ 1 ][ NAME_MAX ];
        if ( !strncpy( params[0], field, NAME_MAX - 1 ) ){
                error_print( "strncpy" );
                return -1;
        }

        if ( -1 == serial_set_udev_param_list( params, 1, NAME_MAX, serial ) ){
                error_print( "serial_set_udev_param_list" );
                return -1;
        }
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_udev_param_value( 
        const char   *field, 
        size_t       length, 
        serial_t     *serial
){
        if ( !serial_valid( serial, QUICK ) || !field ){
                errno = EINVAL;
                return NULL;
        }

        size_t _length = NAME_MAX;
        if ( length > NAME_MAX ) {
                _length = length; 
        }

        for ( uint8_t i = 0 ; i < serial->id.ndev ; ++i ) {
                if ( !strncmp( field, serial->id.dev[i].field, _length ) ) {
                        return serial->id.dev[i].value;
                }
        }
        return NULL;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_event_enable( 
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        for ( uint8_t i = 0 ; i < 2 ; ++i ) {
                serial->event[i].fd = epoll_create1( 0 );
                if ( -1 == serial->event[i].fd ) {
                        error_print( "epoll_create1" );
                        return -1;
                } 

                struct epoll_event ev;
                if ( RX == i ) {
                        ev.events = EPOLLIN;
                }
                else {
                        ev.events = EPOLLOUT;
                } 

                ev.events |= EPOLLET;
                ev.data.fd = serial->fd;      

                if ( -1 == epoll_ctl( serial->event[i].fd, EPOLL_CTL_ADD, serial->fd, &ev ) ) {
                        error_print( "epoll_ctl" );
                        return -1;
                }
        }
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_close( 
        serial_t * serial
){
        if ( !serial_valid( serial, FULL ) ) {
                return -1;
        }

        if ( EOF == fclose( serial->fp ) ){
                error_print( "fclose" );
                return -1;
        }

        serial->fp = NULL;

        // if( !serial->async.close ){
        //   serial->async.close = 1;
        //   if( 0 != pthread_join( serial->async.thread, NULL) ){
        //     error_print( "pthread_join" );
        //     return -1;
        //   }
        // }

        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_default_config( 
        serial_config_t * config
){
        if ( !config ){
                errno = EINVAL;
                error_print( "config null" );
                return -1;
        }
        
        config->baudrate         = B9600;
        config->data_bits        = DATA_BITS_8;
        config->flow_control     = FLOWCONTROL_NONE;
        config->parity           = BPARITY_NONE;
        config->stop_bits        = STOP_BITS_1;
        config->timeout_ds       = 0;
        config->event_timeout_ms = 100;
        config->min_bytes        = 0;
        config->readonly         = 0;

        return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
_serial_set_baudrate( 
        const baudrate_t baudrate, 
        struct termios * tty
){
        if ( !tty ) {
                return -1;
        }

        int result = cfsetispeed( tty, baudrate );
        if ( 0 != result ){
                error_print( "cfsetispeed" );
                return -1;
        }

        result = cfsetospeed( tty, baudrate );
        if ( 0 != result ){
                error_print( "cfsetospeed" );
                return -1;
        }
        return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_baudrate( 
        const baudrate_t baudrate, 
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        struct termios tty;
        if ( !get_termios( serial->fd, &tty ) ) {
                return -1;
        }

        int8_t ret = _serial_set_baudrate( baudrate, &tty );
        if ( -1 == ret ) {
                return -1;
        }

        if ( !apply_termios( serial->fd, &tty ) ) {
                return -1;
        }

        serial->config.baudrate = baudrate;
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
_serial_set_parity( 
        const parity_t parity, 
        struct termios * tty
){
        if ( !tty ) {
                return -1;
        }

        switch ( parity ){
        default:
                errno = EINVAL;
                return -1;

        case BPARITY_NONE:
                tty->c_cflag &= (tcflag_t) ~(PARENB);                                    // Disable parity (Clear bit)
                tty->c_iflag &= (tcflag_t) ~(INPCK);                                     // Disable parity checking
                break;

        case BPARITY_ODD:
                tty->c_cflag |= (tcflag_t) (PARENB) | (PARODD);                          // Enable parity (Set bit) and Enable odd parity
                tty->c_iflag |= (tcflag_t) (INPCK);                                      // Enable parity checking
                break;

        case BPARITY_EVEN:
                tty->c_cflag |= (tcflag_t) (PARENB);                                     // Enable parity (Set bit)
                tty->c_cflag &= (tcflag_t) ~(PARODD);                                    // Enable even parity
                tty->c_iflag |= (tcflag_t) (INPCK);                                      // Enable parity checking
                break;
        }

        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_parity( 
        const parity_t parity, 
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        struct termios tty;
        if ( !get_termios( serial->fd, &tty ) ) {
                return -1;
        }

        int8_t ret = _serial_set_parity( parity, &tty );
        if ( -1 == ret ) {
                return -1;
        }

        if ( !apply_termios( serial->fd, &tty ) ) {
                return -1;
        }

        serial->config.parity = parity;
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
_serial_set_stopbits( 
        const stop_bits_t stop_bits, 
        struct termios * tty
){
        if ( !tty ) {
                return -1;
        }

        switch ( stop_bits ){
        default:
                errno = EINVAL;
                return -1;

        case STOP_BITS_1:
                tty->c_cflag &= (tcflag_t) ~(CSTOPB);                                    // Set 1 stop bit 
                break;

        case STOP_BITS_2:      
                tty->c_cflag |= (tcflag_t) (CSTOPB);                                     // Set 2 stop bits
                break;
        }

        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_stopbits( 
        const stop_bits_t stop_bits, 
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        struct termios tty;
        if ( !get_termios( serial->fd, &tty ) ) {
                return -1;
        }

        int8_t ret = _serial_set_stopbits( stop_bits, &tty );
        if ( -1 == ret ) {
                return -1;
        }

        if ( !apply_termios( serial->fd, &tty ) ) {
                return -1;
        }

        serial->config.stop_bits = stop_bits;
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
_serial_set_databits( 
        const data_bits_t data_bits, 
        struct termios * tty
){
        if ( !tty ) {
                return -1;
        }

        tty->c_cflag &= (tcflag_t) ~CSIZE;                                          // Clear all the size bits, then use one of the statements below
        tty->c_cflag |= (tcflag_t) data_bits;                                       // Bits per word
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_databits( 
        const data_bits_t data_bits, 
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        struct termios tty;
        if ( !get_termios( serial->fd, &tty ) ) {
                return -1;
        }

        int8_t ret = _serial_set_databits( data_bits, &tty );
        if ( -1 == ret ) {
                return -1;
        }

        if ( !apply_termios( serial->fd, &tty ) ) {
                return -1;
        }

        serial->config.data_bits = data_bits;
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
_serial_set_flowcontrol( 
        const flow_control_t flow_control, 
        struct termios * tty
){
        if ( !tty ) {
                return -1;
        }

        switch ( flow_control ){
        default:
                errno = EINVAL;
                return -1;

        case FLOWCONTROL_NONE:
                tty->c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
                tty->c_cflag &= (tcflag_t) ~(CRTSCTS);
                tty->c_cc[VSTART] = 0;                                                   // Disable start character (XON) - disable software flow control
                tty->c_cc[VSTOP] = 0;                                                    // Disable stop character (XOFF) - disable software flow control
                break;

        case FLOWCONTROL_HARDWARE:
                tty->c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
                tty->c_cflag |= (tcflag_t) (CRTSCTS);
                tty->c_cc[VSTART] = 0;                                                   // Disable start character (XON) - disable software flow control
                tty->c_cc[VSTOP] = 0;                                                    // Disable stop character (XOFF) - disable software flow control
                break;

        case FLOWCONTROL_SOFTWARE:
                tty->c_iflag |= (tcflag_t) (IXON | IXOFF | IXANY);
                tty->c_cflag &= (tcflag_t) ~(CRTSCTS);
                tty->c_cc[VSTART] = 1;                                                   // Enable start character (XON) - enable software flow control
                tty->c_cc[VSTOP] = 1;                                                    // Enable stop character (XOFF) - enable software flow control
                break;
        }

        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_flowcontrol( 
        const flow_control_t flow_control, 
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        struct termios tty;
        if ( !get_termios( serial->fd, &tty ) ) {
                return -1;
        }

        int8_t ret = _serial_set_flowcontrol( flow_control, &tty );
        if ( -1 == ret ) {
                return -1;
        }

        if ( !apply_termios( serial->fd, &tty ) ) {
                return -1;
        }

        serial->config.flow_control = flow_control;
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
_serial_set_rule( 
        const uint8_t timeout, 
        const uint8_t min, 
        struct termios * tty
){
        if ( !tty ) {
                return -1;
        }

        cfmakeraw( tty );
        memset( &tty->c_cc, 0, sizeof( cc_t ) * NCCS );

        tty->c_cc[VEOF]     = 4;                                                     // Set EOF character to EOT (Ctrl+D, ASCII 4) - or 0 if not used
        tty->c_cc[VTIME]    = timeout;                                               // Set timeout for read() in tenths of a second
        tty->c_cc[VMIN]     = min;                                                   // Set minimum number of bytes for read() to return
        tty->c_cc[VINTR]    = 0;                                                     // Disable interrupt character (Ctrl+C)
        tty->c_cc[VQUIT]    = 0;                                                     // Disable quit character (Ctrl+\)
        tty->c_cc[VSUSP]    = 0;                                                     // Disable suspend character (Ctrl+Z)
        // tty->c_cc[VERASE]   = 0;                                                  // Disable erase character (backspace) - not relevant in raw mode
        // tty->c_cc[VKILL]    = 0;                                                  // Disable kill character (Ctrl+U) - not relevant in raw mode
        // tty->c_cc[VSWTC]    = 0;                                                  // Disable switch character - not usually needed
        // tty->c_cc[VEOL]     = 0;                                                  // Disable end-of-line character - not relevant in raw mode
        // tty->c_cc[VREPRINT] = 0;                                                  // Disable reprint character - not relevant in raw mode
        // tty->c_cc[VDISCARD] = 0;                                                  // Disable discard character - not relevant in raw mode
        // tty->c_cc[VWERASE]  = 0;                                                  // Disable word erase character - not relevant in raw mode
        // tty->c_cc[VLNEXT]   = 0;                                                  // Disable literal next character - not relevant in raw mode
        // tty->c_cc[VEOL2]    = 0;                                                  // Disable alternate end-of-line character - not relevant in raw mode

        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_rule( 
        const uint8_t timeout, 
        const uint8_t min, 
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        struct termios tty;
        if ( !get_termios( serial->fd, &tty ) ) {
                return -1;
        }

        int8_t ret = _serial_set_rule( timeout, min, &tty );
        if ( -1 == ret ) {
                return -1;
        }

        if ( !apply_termios( serial->fd, &tty ) ) {
                return -1;
        }

        serial->config.timeout_ds = timeout;
        serial->config.min_bytes = min;
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_timeout( 
  const int timeout, 
  serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }
        serial->config.event_timeout_ms = timeout;
        return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_config( 
        const serial_config_t * config, 
        serial_t * serial
){
        if ( !config ) {
                errno = EINVAL;
                error_print( "config null" );
                return -1;
        }

        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        struct termios tty;
        if ( !get_termios( serial->fd, &tty ) ) {
                error_print( "get_termios" );
                return -1;
        }

        if ( -1 == _serial_set_baudrate( config->baudrate, &tty ) ){
                error_print( "_serial_set_baudrate" );
                return -1;
        }

        if ( -1 == _serial_set_parity( config->parity, &tty ) ){
                error_print( "_serial_set_parity" );
                return -1;
        }

        if ( -1 == _serial_set_stopbits( config->stop_bits, &tty ) ){
                error_print( "_serial_set_stopbits" );
                return -1;
        }

        if ( -1 == _serial_set_databits( config->data_bits, &tty ) ){
                error_print( "_serial_set_databits" );
                return -1;
        }

        if ( -1 == _serial_set_flowcontrol( config->flow_control, &tty ) ){
                error_print( "_serial_set_flowcontrol" );
                return -1;
        }

        if ( -1 == _serial_set_rule( config->timeout_ds, config->min_bytes, &tty ) ){
                error_print( "_serial_set_rule" );
                return -1;
        }

        if ( -1 == serial_set_timeout( config->event_timeout_ms, serial ) ){
                error_print( "serial_set_timeout" );
                return -1;
        }

        if ( !apply_termios( serial->fd, &tty ) ){
                error_print( "apply_termios" );
                return -1;
        }

        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
_serial_read( 
        void * data, 
        const size_t len, 
        serial_t * serial
){
        size_t size = 0;
        switch ( serial->iomode ){
        case SERIAL_STDIO: 
                size = fread( data, 1, len, serial->fp );
                break;

        case SERIAL_POSIX:
                size = (size_t) read( serial->fd, data, len);
                break;

        case SERIAL_URING: 
                break;
        }
        return size;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
_serial_readline( 
        char * data, 
        const size_t len, 
        serial_t * serial
){
        size_t idx = 0;
        switch ( serial->iomode ){
        case SERIAL_STDIO: 
                if ( !fgets( data, (int) len, serial->fp ) ){
                        fs_error( serial, 0 ); 
                        return 0;
                }
                idx = strlen( data );
                break;

        case SERIAL_POSIX:
                for( idx = 0 ; idx < len - 1 ; ++idx ){
                        int err = (int) read( serial->fd, &data[idx], 1 );
                        if( 1 != err )
                                fs_error( serial, (size_t) err ); 

                        if( '\n' == data[idx] ){
                                data[idx + 1] = '\0';          
                                break;
                        }
                }
                break;

        case SERIAL_URING: 
                break;
        }

        return idx;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_readline( 
        char * buf, 
        const size_t size, 
        const size_t offset, 
        serial_t * serial
){  
        if ( !buf ){
                errno = EINVAL;
                error_print( "buf null" );
                return 0;
        }

        if ( !serial_valid( serial, QUICK ) ) {
                return 0;
        }

        if ( size <= offset ){
                errno = ENOMEM;
                error_print( "serial_readline overflow" );
                return 0;
        }

        if ( 0 != serial->config.event_timeout_ms ){
                int8_t ev = serial_event_wait( serial, serial->config.event_timeout_ms, RX );
                if( 1 > ev ){
                        if( (EPOLLHUP == errno) || (EBADF == errno) ) {
                                errno = ENODEV;
                        }
                        errno = ETIME;
                }
        }

        clearerr( serial->fp );
        errno = 0;

        size_t len = _serial_readline( buf + offset, size - offset, serial );
        fs_error( serial, len );
        if ( (ENODEV == errno) || !serial_get_databits( NULL, serial ) ) {
                return 0;
        }

        return strlen( buf + offset );  
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_read( 
        char * buf, 
        const size_t size, 
        const size_t offset, 
        const size_t length, 
        serial_t * serial
){
        if ( !buf ){
                errno = EINVAL;
                error_print( "buf null" );
                return 0;
        }

        if ( !length ){
                errno = EINVAL;
                error_print( "length is 0" );
                return 0;
        }

        if ( !serial_valid( serial, QUICK ) ) {
                return 0;
        }

        if ( size < (offset + length) ){
                errno = ENOMEM;
                error_print( "serial_read overflow" );
                return 0;
        }

        clearerr( serial->fp );

        size_t total = 0;
        while ( total < length ) {
                        size_t to_read = length;

                        if ( 0 != serial->config.event_timeout_ms ) {
                                int8_t ev = serial_event_wait( serial, serial->config.event_timeout_ms, RX );
                                if ( 1 > ev ){
                                        if ( (EPOLLHUP == errno) || (EBADF == errno) ) {
                                                errno = ENODEV;
                                        }
                                        errno = ETIME;
                                        break;      
                                }

                                size_t arrived = serial_available( serial );
                                if ( !arrived && (EIO == errno) ){
                                        errno = ENODEV;
                                        return 0;
                                }

                        if ( !total ){
                                if ( arrived > length ) {
                                        to_read = length;
                                }
                                else {
                                        to_read = arrived;
                                }
                        }
                        else {
                                if ( arrived + total > length ) {
                                        to_read = length - total;
                                }
                                else {
                                        to_read = arrived;
                                }
                        }
                }

                clearerr( serial->fp );
                errno = 0;

                size_t received = _serial_read( buf + offset + total, to_read, serial );
                total += received;

                fs_error( serial, received );
                if ( (ENODEV == errno) || !serial_get_databits( NULL, serial ) ){
                        total = 0;
                        break;
                }
                if ( ETIME == errno )
                        break;
        }

        return total;  
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
_serial_write( 
        serial_t * serial, 
        const void * data, 
        const size_t len
){
        if ( 0 != serial->config.event_timeout_ms ) {
                int8_t ev = serial_event_wait( serial, serial->config.event_timeout_ms, TX );  
                if ( 1 > ev ) {
                        if( (EPOLLHUP == errno) || (EBADF == errno) ) {
                                errno = ENODEV;
                        }
                        errno = ETIME;
                }
        }

        size_t size = 0;
        switch ( serial->iomode ){
        case SERIAL_STDIO: 
                size = fwrite( data, 1, len, serial->fp );
                break;

        case SERIAL_POSIX: 
                size = (size_t) write( serial->fd, (uint8_t *) data, len );
                break;

        case SERIAL_URING: 
                break;
        }

        return size;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_write( 
        serial_t * serial, 
        const uint8_t * data, 
        const size_t len
){
        if ( !data ) {
                errno = EINVAL;
                error_print( "data null" );
                return 0;
        }

        if ( !serial_valid( serial, QUICK ) ) {
                return 0;
        }

        size_t size = _serial_write( serial, data, len );

        if ( size < len ) {
                return (size_t) fs_error( serial, size );
        }

        // if( -1 == serial_flush( serial ) )
        //   return (size_t) fs_error( serial, 0 );

        return size;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_writef( 
        serial_t * serial, 
        const char * format, 
        ...
){
        if ( !format ) {
                errno = EINVAL;
                error_print( "format null" );
                return 0;
        }

        if ( !serial_valid( serial, QUICK ) ) {
                return 0;
        }

        va_list args;
        va_start( args, format );

        char buf[ PIPE_BUF ];
        int len = vsnprintf( buf, sizeof(buf), format, args );

        if ( 0 > len ) {
                error_print( "vsnprintf" );
                va_end( args );
                return 0;
        }

        if ( (int) sizeof( buf ) <= len ) {
                error_print( "write overflow" );
                va_end( args );
                return 0;
        }

        size_t size = _serial_write( serial, buf, (size_t) len );

        va_end( args );
        if ( size < (size_t) len ) {
                return (size_t) fs_error( serial, size );
        }

        // if( -1 == serial_flush( serial ) )
        //   return (size_t) fs_error( serial, size );

        return size;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
uint8_t 
serial_valid( 
        const serial_t * serial,
        const serial_check_t check 
){
        if ( !serial ) {
                errno = EINVAL;
                error_print( "serial null" );
                return 0;
        }

        if ( 0 > serial->fd ) {
                errno = EBADF;
                return 0;
        }

        if ( (FULL == check) && -1 == fcntl( serial->fd, F_GETFD ) ) {
                errno = EBADF;
                return 0;
        }   

        return 1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_drain( 
        const serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        if ( -1 == tcdrain( serial->fd ) ) {
                error_print( "tcdrain" );
                return -1;
        }

        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_flush( 
        const serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        int8_t err = 0;
        switch ( serial->iomode ){
        case SERIAL_STDIO:
                err = 0 != fflush( serial->fp ) ? -1 : 0; 
                break;

        case SERIAL_POSIX: 
                break;

        case SERIAL_URING: 
                break;
        }

        if ( -1 == err ){
                error_print( "fflush" );
                return -1;
        }  

        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_available( 
        const serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return 0;
        }

        int32_t len;
        if ( -1 == ioctl( serial->fd, FIONREAD, &len ) ) {
                error_print( "available ioctl" );
                return 0;
        }  

        if ( 0 > len ){
                error_print( "available ioctl" );
                return 0;
        }

        return (size_t) len;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_event_wait( 
        serial_t * serial, 
        const int timeout,
        const serial_direction_t side
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        struct epoll_event ev;
        int event = -1;

        for ( ; ; ) {
                if ( RX == side ) {
                        event = epoll_wait( serial->event[RX].fd, &ev, 1, timeout );
                }
                else {
                        event = epoll_wait( serial->event[TX].fd, &ev, 1, timeout );
                }
                if ( -1 == event ){
                        perror("epoll_wait");
                        return -1;
                }
                // Timeout
                if ( !event ) {
                        return 0;
                }
                if ( RX == side ) {
                        if( EPOLLIN & ev.events ) {
                                return EPOLLIN;
                        }
                }
                if ( TX == side ) {
                        if ( EPOLLOUT & ev.events ) {
                                return EPOLLOUT;
                        }
                }
                if( EPOLLHUP & ev.events ) {
                        return EPOLLHUP;
                }
                if( EPOLLERR & ev.events ) {
                        return EPOLLERR;
                }
        }  

        return -1;  
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_line_state( 
        const serial_lines_t line, 
        uint8_t state, 
        const serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        int status;
        if ( -1 == ioctl( serial->fd, TIOCMGET, &status ) ) {
                error_print( "serial_set_line_state ioctl" );
                return -1;
        }

        if ( state ) {
                status |= (int) line;
        }
        else {
                status &= (int) ~line;
        } 

        if ( -1 == ioctl( serial->fd, TIOCMSET, &status ) ) {
                error_print( "serial_set_line_state ioctl" );
                return -1;
        }
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_get_line_state( 
        const serial_lines_t line, 
        uint8_t * state, 
        const serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        int status;
        if ( -1 == ioctl( serial->fd, TIOCMGET, &status ) ){
                error_print( "serial_get_line_state ioctl" );
                return -1;
        }

        *state = ( (uint8_t) status & line) != 0;
        return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_get_config( 
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        struct termios tty;
        if ( !get_termios( serial->fd, &tty ) ) {
                return -1;
        }

        (void) serial_get_baudrate( &tty, serial );
        (void) serial_get_parity( &tty, serial );
        (void) serial_get_databits( &tty, serial );
        (void) serial_get_stopbits( &tty, serial );
        (void) serial_get_flowcontrol( &tty, serial );
        (void) serial_get_rule( &tty, serial ); 
        (void) serial_get_event_timeout( serial );

        return 0;
}          

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_print_config( 
        uint8_t out, 
        const char * initial, 
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return NULL;
        }

        struct termios tty;
        if ( !get_termios( serial->fd, &tty ) ) {
                return NULL;
        }

        memset( serial->resv1, 0, sizeof(serial->resv1) );
        int len = snprintf( 
                serial->resv1, sizeof(serial->resv1), 
                "%sSerial Port %s Configuration\n"
                "%sBaud Rate: %s [bps]\n"
                "%sParity: %s\n"
                "%sData Bits: %s [b]\n"
                "%sStop Bits: %s [b]\n"
                "%sFlow Control: %s\n"
                "%sTimeout, Minimum Number Bytes: %s [ds, B]\n"
                "%sEvent timeout: %s [ms]\n"
                "%sBus type: %s\n",
                initial, serial->pathname, 
                initial, serial_get_baudrate( &tty, serial ),
                initial, serial_get_parity( &tty, serial ),
                initial, serial_get_databits( &tty, serial ), 
                initial, serial_get_stopbits( &tty, serial ),
                initial, serial_get_flowcontrol( &tty, serial ),
                initial, serial_get_rule( &tty, serial ),
                initial, serial_get_event_timeout( serial ),
                initial, serial->id.bus
        );

        for( uint8_t i = 0 ; i < serial->id.ndev ; ++i ) {
                len += snprintf( serial->resv1 + len, sizeof(serial->resv1) - (size_t) len, "%s%s: %s\n", initial, serial->id.dev[i].field, serial->id.dev[i].value );
        }

        if( out ) {
                printf( "%s\n", serial->resv1 );
        }

        return serial->resv1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char *
get_stringlut_from_code( 
  void * code, 
  const lut_t * table, 
  const size_t nitems, 
  const size_t size
){
        if( !code || !table ) {
                return NULL;
        }

        for ( size_t i = 0 ; i < nitems ; ++i ) {
                if ( !memcmp( code, &( table[ i ].code ), size ) ) {
                        return table[ i ].text;
                }
        }

        return NULL;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char *
get_baudrate_from_code( 
        const baudrate_t code
){
        return get_stringlut_from_code( 
                (void *) &code, 
                lut_baudrate,
                sizeof( lut_baudrate ) / sizeof( lut_baudrate[0] ),
                sizeof( baudrate_t )
        );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char *
get_parity_from_code( 
  const parity_t code
){
        return get_stringlut_from_code( 
                (void *) &code, 
                lut_parity,
                sizeof( lut_parity ) / sizeof( lut_parity[0] ),
                sizeof( parity_t )
        );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char *
get_data_bits_from_code( 
  const data_bits_t code
){
        return get_stringlut_from_code( 
                (void *) &code, 
                lut_data_bits,
                sizeof( lut_data_bits ) / sizeof( lut_data_bits[0] ),
                sizeof( data_bits_t )
        );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char *
get_flow_control_from_code( 
  const flow_control_t code
){
        return get_stringlut_from_code( 
                (void *) &code, 
                lut_flow_control,
                sizeof( lut_flow_control ) / sizeof( lut_flow_control[0] ),
                sizeof( flow_control_t )
        );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char *
get_stop_bits_from_code( 
        const stop_bits_t code
){
        return get_stringlut_from_code( 
                (void *) &code, 
                lut_stop_bits,
                sizeof( lut_stop_bits ) / sizeof( lut_stop_bits[0] ),
                sizeof( stop_bits_t )
        );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
baudrate_t 
_serial_get_baudrate( 
        struct termios * tty
){
        if ( !tty ) {
               return _BINV;
        }
        return cfgetospeed( tty );
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_baudrate( 
        struct termios * tty,
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return NULL;
        }

        if ( !tty ){   
                struct termios ltty;
                if ( !get_termios( serial->fd, &ltty ) ) {
                        return NULL;
                }
                serial->config.baudrate = _serial_get_baudrate( &ltty );
        }
        else {
                serial->config.baudrate = _serial_get_baudrate( tty );
        }

        return get_baudrate_from_code( serial->config.baudrate );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
parity_t 
_serial_get_parity( 
        struct termios * tty 
){
        if ( !tty ) {
                return _BPARITY_INV;
        }
        if ( !( tty->c_iflag & (tcflag_t) INPCK ) ) {
                return BPARITY_NONE;
        }
        if ( !( tty->c_cflag & (tcflag_t) PARODD ) ) {
                return BPARITY_EVEN;
        }
        return BPARITY_ODD;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_parity( 
        struct termios * tty,
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return NULL;
        }
 
        if ( !tty ) {
                struct termios ltty;
                if ( !get_termios( serial->fd, &ltty ) )
                return NULL;
                serial->config.parity = _serial_get_parity( &ltty );
        }
        else {
                serial->config.parity = _serial_get_parity( tty );
        }
        return get_parity_from_code( serial->config.parity );  
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
stop_bits_t 
_serial_get_stopbits( 
        struct termios * tty
){
        if ( !tty ) {
                return _STOP_BITS_INV;
        }
        if ( !( tty->c_iflag & (tcflag_t) CSTOPB ) ) {
                return STOP_BITS_1;
        }
        return STOP_BITS_2;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_stopbits( 
        struct termios * tty,
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return NULL;
        }
 
        if ( !tty ){
                struct termios ltty;
                if ( !get_termios( serial->fd, &ltty ) ) {
                        return NULL;
                }
                serial->config.stop_bits = _serial_get_stopbits( &ltty ); 
        }
        else {
                serial->config.stop_bits = _serial_get_stopbits( tty ); 
        }
        return get_stop_bits_from_code( serial->config.stop_bits );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
data_bits_t 
_serial_get_databits( 
        struct termios * tty
){
        if ( !tty ) {
                return _DATA_BITS_INV;
        }
        return tty->c_cflag & (tcflag_t) CSIZE;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_databits( 
        struct termios * tty,
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return NULL;
        }

        if ( !tty ){
                struct termios ltty;
                if ( !get_termios( serial->fd, &ltty ) ) {
                        return NULL;
                }
                serial->config.data_bits = _serial_get_databits( &ltty );
        }
        else {
                serial->config.data_bits = _serial_get_databits( tty );
        }
        return get_data_bits_from_code( serial->config.data_bits );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
flow_control_t 
_serial_get_flowcontrol( 
        struct termios * tty
){
        if ( !tty ) {
                return _FLOWCONTROL_INV;
        }

        if( !(tty->c_cflag & (tcflag_t) CRTSCTS) ){
                if( !(tty->c_iflag & (tcflag_t) (IXON | IXOFF | IXANY) ) ) {
                        return FLOWCONTROL_NONE;
                }
                else {
                        return FLOWCONTROL_SOFTWARE;
                }
        }
        return FLOWCONTROL_HARDWARE;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_flowcontrol( 
        struct termios * tty,
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return NULL;
        }

        if ( !tty ) {
                struct termios ltty;
                if ( !get_termios( serial->fd, &ltty ) ) {
                        return NULL;
                }
                serial->config.flow_control = _serial_get_flowcontrol( &ltty );
        }
        else {
                serial->config.flow_control = _serial_get_flowcontrol( tty );
        }
        return get_flow_control_from_code( serial->config.flow_control );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_rule( 
        struct termios * tty,
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return NULL;
        }

        if ( !tty ){
                struct termios ltty;
                if ( !get_termios( serial->fd, &ltty ) ) {
                        return NULL;
                }
                serial->config.timeout_ds = ltty.c_cc[VTIME];
                serial->config.min_bytes = ltty.c_cc[VMIN];
        }
        else {
                serial->config.timeout_ds = tty->c_cc[VTIME];
                serial->config.min_bytes = tty->c_cc[VMIN];
        }
        memset( serial->resv2, 0, sizeof(serial->resv2) );
        snprintf( serial->resv2, sizeof(serial->resv2), "%hhd, %hhd", serial->config.timeout_ds, serial->config.min_bytes );
        return serial->resv2;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_event_timeout( 
        serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) ) {
                return NULL;
        }
        memset( serial->resv2, 0, sizeof(serial->resv2) );
        snprintf( serial->resv2, sizeof(serial->resv2), "%d", serial->config.event_timeout_ms );
        return serial->resv2;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
get_termios( 
        int fd, 
        struct termios * tty
){
        if ( !tty ) {
                errno = EINVAL;
                error_print( "tty null" );
                return 0;
        }

        int result = tcgetattr( fd, tty );
        if ( 0 != result ) {
                error_print( "tcgetattr" );
                errno = ENODEV;
                return 0;
        }
        return 1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
apply_termios( 
        int fd, 
        struct termios * tty
){
        if ( !tty ) {
                errno = EINVAL;
                error_print( "tty null" );
                return 0;
        }
        int result = tcsetattr( fd, TCSANOW, tty );
        if ( 0 != result ) {
                error_print( "tcsetattr" );
                return 0;
        }
        return 1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
fs_error( 
        serial_t * serial,
        size_t call_ret
){
        switch ( serial->iomode ){
        case SERIAL_STDIO: 
                if ( feof( serial->fp ) ){
                        clearerr( serial->fp );       
                        if( !serial_valid( serial, FULL ) )
                          errno = ENODEV;
                        else
                          errno = ETIME;
                }
                else if ( ferror( serial->fp ) ){
                        error_print( "ferror" );
                        errno = ENODEV;
                }
                break;
            
        case SERIAL_POSIX: 
                if ( !call_ret ){
                        if ( !serial_valid( serial, FULL ) ) {
                                errno = ENODEV;
                        }
                        else {
                                errno = ETIME;
                        }
                } 

        case SERIAL_URING: 
                break;
        }

        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_async_set_callback( 
        serial_read_callback_t handler_read, 
        serial_disconnect_callback_t handler_disconnect, 
        serial_t * serial
){
        serial->async.rcb = handler_read;
        serial->async.dcb = handler_disconnect;
        serial->async.close = 0;

        // pthread_attr_t * attr = NULL;
        // if( 0 != pthread_create( &(serial->async.thread), attr, async_epoll_thread, serial ) )
        //   return -1;

        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void * 
async_epoll_thread( 
        void * arg
){
        serial_t * serial = (serial_t *) arg;

        if ( !serial_valid( serial, QUICK ) ) {
                return NULL;
        }

        uint8_t buf[ BUFSIZ ];

        for ( ; ; ){
                if ( serial->async.close ) {
                        break;
                }

                size_t len = serial_read( (char *) buf, sizeof(buf), 0, sizeof(buf)-1, serial );    

                if( !len ){
                        if( (errno == ENODEV) || (errno == EIO) ){        
                                serial->async.close = 1;
                                if( -1 == serial->async.dcb( ) ) {
                                        return NULL;
                                }
                                serial->async.close = 0;
                        }
                        continue;
                }

                serial->async.rcb( buf, len );
        }    

        return NULL;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_poll( 
        serial_t * serial 
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        uint8_t buf[ BUFSIZ ];
        size_t len = serial_read( (char *) buf, sizeof(buf), 0, sizeof(buf)-1, serial );    

        if ( !len && ((errno == ENODEV) || (errno == EIO)) ) {
                serial->async.close = 1;
                if ( -1 == serial->async.dcb( ) ) {
                        return -1;
                }
                serial->async.close = 0;
        }

        serial->async.rcb( buf, len );
        return 0;  
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_iomode( 
        const serial_iomode_t iomode, 
        serial_t * serial 
){
        if ( !serial_valid( serial, QUICK ) ) {
                return -1;
        }

        serial->iomode = iomode;    
        return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char *
serial_get_iomode(
        serial_iomode_t * iomode, 
        const serial_t * serial
){
        if ( !serial_valid( serial, QUICK ) || !iomode ) {
                return NULL;
        }

        *iomode = serial->iomode;  
        switch ( *iomode ){
        case SERIAL_STDIO: 
                return "libstdio";
        case SERIAL_POSIX: 
                return "libunist";
        case SERIAL_URING: 
                return "liburing";
        }  
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * End of file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
