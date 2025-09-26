 /***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      xcserial.h
 * 
 * @version   1.0.0
 *
 * @date      18-09-2025
 *
 * @brief     Prototype of functions providing control, local settings, input, and output for serial ports on Linux, library developed for interacting with embedded systems in mind (non-cannonical).  
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
 * Definition file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#ifndef XCSERIAL_H
#define XCSERIAL_H

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Imported libraries
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#include <stdio.h>    
#include <stdint.h>
#include <stdarg.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <linux/limits.h>
#include <pthread.h>

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * External C++ extern macro
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#ifdef __cplusplus  
extern "C" {        
#endif

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Definitions
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Enums
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

//!< The options available for the flow control on the serial communication.
typedef enum {
  FLOWCONTROL_NONE,                                                            //!< No flow control is used.
  FLOWCONTROL_HARDWARE,                                                        //!< Hardware flow control is used.
  FLOWCONTROL_SOFTWARE,                                                        //!< Software flow control is used.
} flow_control_t;

//!< The options available for the parity mode on the serial communication.
typedef enum {
  BPARITY_NONE,                                                                 //!< No parity bit is used.
  BPARITY_ODD,                                                                  //!< Parity bit used and it checks for odd.
  BPARITY_EVEN,                                                                 //!< Parity bit used and it checks for even.
} parity_t;

//!< The options available for the number of data bits used on the serial communication.
typedef enum {
  DATA_BITS_5 = CS5,                                                           //!< 5 Data bits that compose the content of a serial frame
  DATA_BITS_6 = CS6,                                                           //!< 6 Data bits that compose the content of a serial frame
  DATA_BITS_7 = CS7,                                                           //!< 7 Data bits that compose the content of a serial frame
  DATA_BITS_8 = CS8,                                                           //!< 8 Data bits that compose the content of a serial frame
} data_bits_t;

//!< The options available for the number of stop bits used  on the serial communication.
typedef enum {
  STOP_BITS_1,                                                                 //!< 1 Stop bit after each N data bits
  STOP_BITS_2,                                                                 //!< 2 Stop bits after each N data bits
} stop_bits_t;

//!< The options available for the real/virtual lines present on the serial communication.
typedef enum {
  SERIAL_DSR = TIOCM_LE,                                                       //!< DSR - Data Set Ready 
  SERIAL_DTR = TIOCM_DTR,                                                      //!< DTR - Data Terminal Ready
  SERIAL_RTS = TIOCM_RTS,                                                      //!< RTS - Request To Send
  SERIAL_CTS = TIOCM_CTS,                                                      //!< CTS - Clear To Send
  SERIAL_DCD = TIOCM_CAR,                                                      //!< DCD - Data Carrier Detected
} serial_lines_t;

// Not present in POSIX
#define B62500 10014

typedef speed_t baudrate_t;                                                    //!< Type used by the termios API.

typedef void   (*serial_read_callback_t)( uint8_t *, size_t );
typedef int8_t (*serial_disconnect_callback_t)( void );

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Structs
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

//!< The serial port configuration data structure. Can be used to configure the serial port running configuration. 
typedef struct {
  baudrate_t      baudrate;                                                   //!< The baud rate of the communication in bits per second, example B9600.
  flow_control_t  flow_control;                                               //!< The hardware flow control, example FLOWCONTROL_HARDWARE.
  parity_t        parity;                                                     //!< The detection of error parity, example PARITY_ODD.
  data_bits_t     data_bits;                                                  //!< The number of bits per serial word, example DATA_BITS_8.
  stop_bits_t     stop_bits;                                                  //!< The number of stop bits per serial word, example STOP_BITS_1.
  uint8_t         timeout_ds;                                                 //!< The time any read function will wait in deciseconds for the information to arrive, example 200.
  int             event_timeout_ms;                                           //!< The timeout for waiting for a event
  uint8_t         min_bytes;                                                  //!< The minimum number of bytes to necessary receive before returning the read function.
  uint8_t         readonly:1;                                                 //!< If the file pointer will be given in read only mode, example false = r+.
} serial_config_t;

typedef struct{
  char field[NAME_MAX];
  char value[NAME_MAX];
} serial_udev_paramater_t;

//!< The serial identification used if the device is disconnected
typedef struct{
  char bus[NAME_MAX];
  serial_udev_paramater_t dev[16];
  uint8_t ndev;
} serial_id_t;

//!< The async manager of the serial port, used for asnyc reads, hot-plug disconnect and other features
typedef struct{
  serial_read_callback_t        rcb;                                           //!< The user callback function defined for async functionallity
  serial_disconnect_callback_t  dcb;                                           //!< The user callback function when the serial device is disconnected
  pthread_t                     thread;                                        //!< The thread responsible for the callback when used 
  uint8_t                       close;
} serial_async_t;

//!< The event manager of the serial port, used to improve the use of the serial port in terms of CPU and flow control.
typedef struct{
  int                     fd;                                                  //!< The file descriptor for the events generated by the kernel
  struct epoll_event      ev;                                                  //!< The type of event that happen
} serial_event_t;

//!< The serial port data structure. It defines the layer of abstraction between this library and the operations done at a lower level. 
typedef struct{
  char             pathname[PATH_MAX];                                         //!< The path to the serial port, example "/dev/ttyUSB0"
  int               fd;                                                        //!< The file descriptor for the serial port opened.
  FILE             *fp;                                                        //!< The file pointer for the serial port opened.
  serial_config_t  config;                                                     //!< The serial port configuration
  serial_event_t   event;                                                      //!< The serial port event configuration, used to minimize the syncronous operations and additionally enable async operation
  serial_async_t   async;
  serial_id_t      id;
} serial_t;

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Prototypes
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Initializes the serial port interface.
 *  
 * @param[out] serial The serial port structure (`serial_t`) to be filled.
 * @param[in]  pathname The absolute path for the device file, example `/dev/ttyUSB0`
 * @param[in]  readonly The permission of the file descriptor associated with the serial port, 0 is read/write and 1 is read only. 
 * @param[in]  config The serial port configuration data structure (`serial_config_t`), this configuration must have been updated with the user's desired information, can be nullable if the user desire the default configuration present in `serial_default_config`. 
 * @param[in]  id Indication of the udevam parameters that identify this serial port, used for reconnection upon disconnection.
 * 
 * @return Upon success, performing the process of opening the serial port, it returns 0 and the serial struct filled. \n
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENAMETOOLONG`: Pathname to long 
 * 
 * @b Example \n
 *  Opening the serial port with 115200 bps and 1 second until timeout, other configurations are the default: \n
 *  Method-1:
 * @code{.c}
 *    serial_config_t cfg;
 *    serial_default_config( &cfg );
 *    cfg.baudrate = B115200;
 *    cfg.timeout = 10;
 *    serial_t serial;    
 *    if( -1 == serial_open( &serial, "/dev/ttyUSB0", 0, &cfg, NULL ) )
 *      // handle error
 *    // ready to use ...
 * @endcode
 *   Method-2:
 * @code{.c}
 *    serial_t serial;    
 *    if( -1 == serial_open( &serial, "/dev/ttyUSB0", 0, NULL, NULL ) )
 *      // handle error
 *    serial_set_baudrate( B115200, &serial );
 *    serial_set_rule( 10, 0, &serial );
 *    // ready to use ...
 * @endcode
 *   
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_open( serial_t * serial, const char * pathname, uint8_t readonly, const serial_config_t * config, serial_id_t * id, serial_async_t * async );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Attempts to reopen a previously opened serial port. \n
 * This function can only be used after `serial_open()` has successfully initialized the serial port. \n
 * It repeatedly searches for the device and tries to reopen it using the current configuration stored in the `serial_t` structure. \n
 * 
 * The reconnect process: \n
 *  - Checks if the device is available. \n
 *  - If available, reopens it with the existing configuration. \n
 *  - If not, waits for a short period before trying again, increasing the sleep time with each iteration. \n
 *  - Continues until the specified number of iterations is reached. \n
 * 
 * If `serial_set_udev_param()` was previously used with a list of udev parameters, this function will use those parameters to identify the device. \n
 * Otherwise, it will attempt to reconnect based on the last known pathname. Note that relying solely on the pathname is more error-prone due to potential device reassignment. \n
 *  
 * @param[out] serial The serial port structure (`serial_t`) to be filled after reconnecting.
 * @param[in] iterations How many times it will try to reconnect to the serial port before giving up.
 * 
 * @return Upon success, performing the process of opening the serial port, it returns 0. \n
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor \n
 *  - `ENODEV`: No device was found \n
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_reopen( serial_t * serial, uint16_t iterations );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief  Connects the serial port with epoll for event-driven operation. \n
 * Instead of continuous polling, epoll allows the application to sleep until a serial port event occurs, drastically reducing CPU saturation during idle periods.
 *  
 * @param[out] serial The serial port structure (`serial_t`) to be filled.
 * 
 * @return Upon success, the serial port is attached to epoll for event-driven operation, and the 0 is returned. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor \n
 *  - ...
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_event_enable( serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Closes the connection with the serial port interface.
 *  
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * 
 * @return Upon success, closing the serial port, it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_close( serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Flushes the information on buffer from the file pointer associated with the serial port passed as argument.
 * 
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * 
 * @return Upon success, flushing serial port, it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor
 *  
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_flush( const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Waits for all pending output information to be transmitted on the serial port.
 *
 * This is typically used to ensure that all data has been sent before closing the port or performing other operations.
 *
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, draining the serial port, it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor
 * 
 * @note While draining the program will be halted.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_drain( const serial_t * serial );


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
uint8_t serial_valid( const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Sets to the default values the configuration structure passed as argument.
 *  
 * @param[out]  config The serial port configuration data structure (`serial_config_t`) to be filled. 
 * 
 * @return Upon success, setting the default values to the passed structure, it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 *
 *  - `EINVAL`: Invalid argument
 * 
 * @note The default values are:
 *  - Baud Rate [bps]  : 9600  \n 
 *  - Data Bits [b]    : 8     \n 
 *  - Stop Bits [b]    : 1     \n 
 *  - Parity           : None  \n 
 *  - Flow Control     : None  \n 
 *  - Read Only        : False \n 
 *  - Timeout [s]      : 10    \n 
 *  - Nº Min Bytes [B] : 0  
 *   
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_default_config( serial_config_t * config );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the serial port configuration based on the configuration structure passed as argument.
 *  
 * @param[in] config The serial port configuration data structure (`serial_config_t`), this configuration must have been updated with the user's desired information. 
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * 
 * @return Upon success, the serial port current configuration is updated, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_set_config( const serial_config_t * config, serial_t * serial );           


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Gets the current serial port configuration and fills the configuration structure inside the serial object.
 *
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * 
 * @return Upon success, the serial port current configuration is obtained, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_get_config( serial_t * serial );           


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial port configuration, optionally printing it to standard output.
 *
 * @param[in] out If non-zero, the output string will be printed to standard output
 * @param[in] initial Text that will come before the parameters, like indicating the PID.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return A pointer to the string with the information. \n
 *         Otherwise NULL is returned and errno is set.
 *
 * - `EINVAL`: Invalid argument \n
 * - `ENOMEM`: Memory allocation failed \n
 * - `EBADF`: Bad file descriptor
 *
 * @note The user must free the returned string.
 *  
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * serial_print_config( uint8_t out, const char * initial, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the serial port baud rate configuration.
 * 
 * @param[in] baudrate The value of the desired baud rate (`baudrate_t`), example B9600.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, the serial port current configuration is updated, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *  - `EBADF`: Bad file descriptor
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_set_baudrate( const baudrate_t baudrate, serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the serial port parity option configuration.
 * 
 * @param[in] parity The value of the parity mode desired (`parity_t`), example PARITY_NONE.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, the serial port current configuration is updated, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *  - `EBADF`: Bad file descriptor
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_set_parity( const parity_t parity, serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the serial port number of stop bits.
 * 
 * @param[in] stop_bits The number of the stop bits desired (`stop_bits_t`), example STOP_BITS_1.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, the serial port current configuration is updated, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *  - `EBADF`: Bad file descriptor
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_set_stopbits( const stop_bits_t stop_bits, serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the serial port number of data bits.
 * 
 * @param[in] data_bits The number of the data bits desired (`data_bits_t`), example DATA_BITS_8.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, the serial port current configuration is updated, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *  - `EBADF`: Bad file descriptor
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_set_databits( const data_bits_t data_bits, serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the serial port flow control option configuration.
 * 
 * @param[in] flow_control The value of the flow control mode desired (`flow_control_t`), example FLOWCONTROL_SOFTWARE.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, the serial port current configuration is updated, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *  - `EBADF`: Bad file descriptor
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_set_flowcontrol( const flow_control_t flow_control, serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the serial port behavior in read operations.
 *  
 * @param[in] timeout The time any read function will wait in deciseconds for the information to arrive.
 * @param[in] min The minimum number of bytes to necessary receive before returning the read function.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, the serial port current configuration is updated, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *  - `EBADF`: Bad file descriptor
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_set_rule( const uint8_t timeout, const uint8_t min, serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the serial port timeout in read operations.
 *  
 * @param[in] timeout The time the API's read functions will wait in milliseconds for the information to arrive.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, the serial port current configuration is updated, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *  - `EBADF`: Bad file descriptor
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_set_timeout( const int timeout, serial_t * serial );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial port baud rate and fill a `baudrate_t` variable passed as argument.
 *        If the serial port is with problems, the past configuration will be returned.
 * 
 * @param[out] baudrate A pointer to a variable `baudrate_t` to be filled with the current serial port baud rate, can be nullable.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, it returns a pointer to the string with the information. \n 
 *         Otherwise NULL is returned and errno is set. 
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * serial_get_baudrate( baudrate_t * baudrate, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial port parity mode and fill a `parity_t` variable passed as argument.
 *        If the serial port is with problems, the past configuration will be returned.
 * 
 * @param[out] parity A pointer to a variable `parity_t` to be filled with the current serial port parity mode, can be nullable.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, it returns a pointer to the string with the information. \n 
 *         Otherwise NULL is returned and errno is set. 
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *
 * @note The user must free the returned string.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * serial_get_parity( parity_t * parity, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial flow control mode and fill a `flow_control_t` variable passed as argument.
 *        If the serial port is with problems, the past configuration will be returned.
 * 
 * @param[out] flow_control A pointer to a variable `flow_control_t` to be filled with the current serial port flow control option, can be nullable.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return A pointer to the string with the information. \n 
 *         Otherwise NULL is returned and errno is set. 
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *
 * @note The user must free the returned string.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * serial_get_flowcontrol( flow_control_t * flow_control, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial port timeout in deciseconds and minimum number of bytes and fills the timeout and minimum number of bytes variables passed as argument.
 *        If the serial port is with problems, the past configuration will be returned.
 *
 * @param[out] timeout A pointer to a variable `uint8_t` to be filled with the current serial port timeout value in deciseconds, before any serial port associated read function returns, can be nullable.
 * @param[out] min A pointer to a variable `uint8_t` to be filled with the current serial port minimum number of bytes before any serial port associated read function returns, can be nullable.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return A pointer to the string with the information. \n 
 *         Otherwise NULL is returned and errno is set. 
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *
 * @note The user must free the returned string.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * serial_get_rule( uint8_t * timeout, uint8_t * min, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial port timeout in deciseconds and minimum number of bytes and fills the timeout and minimum number of bytes variables passed as argument.
 *        If the serial port is with problems, the past configuration will be returned.
 *
 * @param[out] timeout A pointer to a variable `int` to be filled with the current serial port event timeout value in milliseconds, can be nullable.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return A pointer to the string with the information. \n 
 *         Otherwise NULL is returned and errno is set. 
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *
 * @note The user must free the returned string.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * serial_get_event_timeout( int * timeout, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial port number of stop bits and fill a `stop_bits_t` variable passed as argument.
 *        If the serial port is with problems, the past configuration will be returned.
 * 
 * @param[out] stop_bits A pointer to a variable `stop_bits_t` to be filled with the current serial port number of stop bits, can be nullable.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return A pointer to the string with the information. \n 
 *         Otherwise NULL is returned and errno is set. 
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * serial_get_stopbits( stop_bits_t * stop_bits, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial port number of data bits and fill a `data_bits_t` variable passed as argument.
 *        If the serial port is with problems, the past configuration will be returned.
 * 
 * @param[out] data_bits A pointer to a variable `data_bits_t` to be filled with the current serial port number of data bits, can be nullable.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return A pointer to the string with the information. \n 
 *         Otherwise NULL is returned and errno is set. 
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * serial_get_databits( data_bits_t * data_bits, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Reads a line of data from the serial port input buffer (file stream) into a buffer passed as argument.
 *
 * This function reads data from the file stream associated with the provided `serial` structure,
 * storing it in the buffer pointed to by `buf`, starting at the specified `offset`. Reading
 * stops when a newline character ('\n') is encountered, the buffer is full (up to `size` - 1
 * characters from the start of the buffer), or an error occurs.
 *
 * @param[out] buf The buffer to store the received data. Must be large enough to hold the expected data, taking into account the `offset`.
 * @param[in] size The total size of the buffer `buf`. This is used to prevent buffer overflows.
 * @param[in] offset The offset within the buffer `buf` where the received data should be stored. This allows for appending to existing content in the buffer.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, the function returns the number of bytes actually read. \n
 *         On error, the function returns 0 and sets `errno` to indicate the error or timeout was reached.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `ETIME`: Timeout was reached \n
 *  - `EBADF`: Bad file descriptor
 * 
 * @note Possible outcomes depending on the rule set on `serial_set_rule` or configuration passed `serial_configuration_t`.
 * 
 * min > 0 and timeout > 0: \n
 *  - The function will wait until either min bytes are received or timeout deciseconds pass since the first byte. \n
 *  - Returns when min bytes arrive, or when the timeout expires, returning any bytes received.
 * 
 * min > 0 and timeout = 0: \n
 *  -  The read will wait until min bytes are received. \n
 *  -  No timeout occurs, so the read may block indefinitely.
 * 
 * min = 0 and timeout > 0: \n
 *  - The read returns immediately if data is available. \n
 *  - If no data, it waits for a single byte or until timeout expires.
 * 
 * min = 0 and timeout = 0: \n
 *  - The read returns immediately with any available bytes. \n
 *  - Never blocks.
 *
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t serial_readLine( char * buf, const size_t size, const size_t offset, serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Reads a specified number of bytes from the serial port input buffer (file stream) into a buffer passed as argument.
 *
 * This function reads up to `length` bytes from the file stream associated with the provided
 * `serial` structure and stores them in the buffer pointed to by `buf`, starting at the
 * specified `offset`. The function may return fewer bytes than requested if an end-of-file
 * (EOF) is encountered or a timeout occurs.
 *
 * @param[out] buf The buffer to store the received data. Must be large enough to hold the expected data, taking into account the `offset`.
 * @param[in] size The total size of the buffer `buf`. This is used to prevent buffer overflows.
 * @param[in] offset The offset within the buffer `buf` where the received data should be stored. This allows for appending to existing content in the buffer.
 * @param[in] length The maximum number of bytes to read from the serial port.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, the function returns the number of bytes actually read. \n
 *         On error, the function returns 0 and sets `errno` to indicate the error or timeout was reached.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `ETIME`: Timeout was reached \n
 *  - `EBADF`: Bad file descriptor
 * 
 * @note Possible outcomes depending on the rule set on `serial_set_rule` or configuration passed `serial_configuration_t`.
 * 
 * min > 0 and timeout > 0: \n
 *  - The function will wait until either min bytes are received or timeout deciseconds pass since the first byte. \n
 *  - Returns when min bytes arrive, or when the timeout expires, returning any bytes received.
 * 
 * min > 0 and timeout = 0: \n
 *  -  The read will wait until min bytes are received. \n
 *  -  No timeout occurs, so the read may block indefinitely.
 * 
 * min = 0 and timeout > 0: \n
 *  - The read returns immediately if data is available. \n
 *  - If no data, it waits for a single byte or until timeout expires.
 * 
 * min = 0 and timeout = 0: \n
 *  - The read returns immediately with any available bytes. \n
 *  - Never blocks.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t serial_read( char * buf, const size_t size, const size_t offset, const size_t length, serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Formats a string and sends it to the serial port output buffer (file stream).
 *
 * This function formats a string using `vsnprintf()` and then writes the formatted string to the serial port associated with the provided `serial` structure using `fwrite()`.
 *
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * @param[in] format The format string, as used in `printf()`.
 * @param[in] ... Variable arguments, as used in `printf()`.
 *
 * @return Upon success, the function returns the number of bytes wrote to the input buffer. \n
 *         On error, the function returns 0 and sets `errno` to indicate the error.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor
 *
 * @note The formatted string is written to a fixed-size buffer before being sent to the serial port.
 *       If the formatted string is too long for the buffer, the function will return 0.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t serial_writef( serial_t * serial, const char * format, ... );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Sends a sequence of bytes to the serial port output buffer (file stream).
 *
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * @param[in] data The sequence of bytes to send via the serial port.
 * @param[in] len The number of bytes on data.
 * 
 * @return Upon success, the function returns the number of bytes wrote to the input buffer. \n
 *         On error, the function returns 0 and sets `errno` to indicate the error.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor
 *
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t serial_write( serial_t * serial, const uint8_t * data, const size_t len );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns the number of bytes available in the serial port input buffer (file stream).
 *
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return On success, the function returns the number of bytes on the serial port buffer.
 *         On error, the function returns 0 and sets `errno` to indicate the error.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t serial_available( const serial_t * serial );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Waits for and returns the triggered serial port event.
 *
 * The `serial_manager_t` must be initialized with `serial_manage` prior to calling this function.
 *
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * @param[in] timeout Timeout in milliseconds (-1 for indefinite blocking).
 *
 * @return On success, 0 is returned if data is available for reading (EPOLLIN). 1 is returned if the serial port is ready for writing (EPOLLOUT).
 *         On error, the function returns -1 and sets `errno` to indicate the error, this can represent timeout.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor \n
 *  - ...
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_event_wait( serial_t * serial, const int timeout );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Sets the state (asserted or deasserted) of a serial port control line.
 *
 * This function sets the specified control line (DTR, RTS, CTS, etc.) of the serial port to the desired state.
 *
 * @param[in] line The control line to set, the options can be: `SERIAL_DSR`, `SERIAL_DTR`, `SERIAL_RTS`, `SERIAL_CTS` and `SERIAL_DCD`.
 * @param[in] state `1` to assert the line, `0` to deassert it.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, the serial port will have the line selected asserted, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_set_line_state( const serial_lines_t line, uint8_t state, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Gets the state (asserted or deasserted) of a serial port control line.
 *
 * This function reads the modem status bits of the serial port and determines whether the specified control line (DTR, RTS, CTS, etc.) is currently asserted or deasserted.
 *
 * @param[in] line The control line to set, the options can be: `SERIAL_DSR`, `SERIAL_DTR`, `SERIAL_RTS`, `SERIAL_CTS` and `SERIAL_DCD`.
 * @param[out] state `1` to assert the line, `0` to deassert it.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, the serial port will have the line selected asserted, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_get_line_state( const serial_lines_t line, uint8_t *state, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Enables async mode and assigns a function for the receiver callback.
 *
 * The callback READ returns void, and receives two parameters:
 * - uint8_t * data: data read from serial_read
 * - size_t length: data length received 
 * 
 * The callback DISCONNECT returns void and receives no parameters:
 *
 * @param[in] handler_read Function called as a callback when data is received.
 * @param[in] handler_disconnect Function called as a callback when the device connected gets disconected.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * 
 * @return Upon success, the serial port will have activate the async thread, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_async_set_callback( serial_read_callback_t handler_read, serial_disconnect_callback_t handler_disconnect, serial_t * serial );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Gets the list of devices available on the system.
 *
 * @param[out] devs Upon success, `devs` will be filled with `ndevs` tty available ports.
 * @param[in] size The total size of the list `devs`.
 * @param[in] length The length of each string inside `devs` array.
 * @param[out] ndevs Upon success, it will indicate the number of items in `devs`. 
 * 
 * @return Upon success, will fill `devs` and `ndevs`, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_get_udev_devs_list( char devs[ ][PATH_MAX], uint8_t size, size_t length, uint8_t * ndevs );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Sets a list of udevam field to associate with serial port opened.
 *
 * @param[in] params A list of parameters to look for, check udevadm info /dev/tty____.
 * @param[in] nparams The number of the parameter to fill, number of items in param.
 * @param[in] length The length of each string inside `params` array.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * 
 * @return Upon success, the serial port will have set the `params` as a serial port identifiers, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_set_udev_param_list( const char params[][NAME_MAX], uint8_t nparams, size_t length, serial_t * serial );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Sets the an udevam field to associate with serial port opened.
 *
 * @param[in] field A parameter to look for, check udevadm info /dev/tty____.
 * @param[in] length The length of `field`.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * 
 * @return Upon success, the serial port will have set the `field` as a serial port identifiers, and it returns 0. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_set_udev_param_field( const char * field, size_t length, serial_t * serial );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Gets the udevam value associated with the field specified on the serial port used.
 *
 * @param[in] field A parameter to look for, check udevadm info /dev/tty____.
 * @param[in] length The length of `field`.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * 
 * @return Upon success, the serial port will return the string with the value associated with field indicated. \n 
 *         Otherwise, NULL is returned and `errno` is set to indicate the error.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * serial_get_udev_param_value( const char * field, size_t length, serial_t * serial );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Copy the identifier structu `id` to the serial port structure `serial`.
 *
 * @param[in] id The identifier struct to copy to the serial port.
 * @param[out] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * 
 * @return Upon success, the serial port will get updated with the `id` argument, and 0 will be returned. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_set_udev_id( const serial_id_t * id, serial_t * serial );

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * External C++ extern macro
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#ifdef __cplusplus  
}
#endif

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Definition file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#endif

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * End of file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
