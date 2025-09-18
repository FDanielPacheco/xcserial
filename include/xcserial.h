 /***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      xcserial.h
 * 
 * @version   4.0
 *
 * @date      12-02-2025
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

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * External C++ extern macro
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#ifdef __cplusplus  
extern "C" {        
#endif

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Definitions
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#define BFS_GET 16                                                             //!< The max dimension of the strings used on the get functions of this library. 

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Enums
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

//!< The options available for the flow control on the serial communication.
typedef enum {
  FLOWCONTROL_NONE,                                                            //!< No flow control is used.
  FLOWCONTROL_HARDWARE,                                                        //!< Hardware flow control is used.
  FLOWCONTROL_SOFTWARE,                                                        //!< Software flow control is used.
} flowControl_t;

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
} dataBits_t;

//!< The options available for the number of stop bits used  on the serial communication.
typedef enum {
  STOP_BITS_1,                                                                 //!< 1 Stop bit after each N data bits
  STOP_BITS_2,                                                                 //!< 2 Stop bits after each N data bits
} stopBits_t;

//!< The options available for the real/virtual lines present on the serial communication.
typedef enum {
  SERIAL_DSR = TIOCM_LE,                                                       //!< DSR - Data Set Ready 
  SERIAL_DTR = TIOCM_DTR,                                                      //!< DTR - Data Terminal Ready
  SERIAL_RTS = TIOCM_RTS,                                                      //!< RTS - Request To Send
  SERIAL_CTS = TIOCM_CTS,                                                      //!< CTS - Clear To Send
  SERIAL_DCD = TIOCM_CAR,                                                      //!< DCD - Data Carrier Detected
} serialLines_t;

// Not present in POSIX
#define B62500 10014

typedef speed_t baudRate_t;                                                    //!< Type used by the termios API.

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Structs
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

//!< The serial port configuration data structure. Can be used to configure the serial port running configuration. 
typedef struct {
  baudRate_t      baudrate;                                                    //!< The baud rate of the communication in bits per second, example B9600.
  flowControl_t   flow;                                                        //!< The hardware flow control, example FLOWCONTROL_HARDWARE.
  parity_t        parity;                                                      //!< The detection of error parity, example PARITY_ODD.
  dataBits_t      dataBits;                                                    //!< The number of bits per serial word, example DATA_BITS_8.
  stopBits_t      stopBits;                                                    //!< The number of stop bits per serial word, example STOP_BITS_1.
  uint8_t         timeout;                                                     //!< The time any read function will wait in deciseconds for the information to arrive, example 200.
  uint8_t         minBytes;                                                    //!< The minimum number of bytes to necessary receive before returning the read function.
  uint8_t         readonly:1;                                                  //!< If the file pointer will be given in read only mode, example false = r+.
} serial_config_t;

//!< The serial port data structure. It defines the layer of abstraction between this library and the operations done at a lower level. 
typedef struct{
  char             pathname[PATH_MAX];                                         //!< The path to the serial port, example "/dev/ttyUSB0"
  int               fd;                                                        //!< The file descriptor for the serial port opened.
  FILE             *fp;                                                        //!< The file pointer for the serial port opened.
  serial_config_t  config;                                                     //!< The serial port configuration
} serial_t;

//!< The event manager of the serial port, used to improve the use of the serial port in terms of CPU and flow control.
typedef struct{
  serial_t           sr;                                                       //!< The serial port structure
  int                fd;                                                       //!< The file descriptor for the events generated by the Kernel
  struct epoll_event ev;                                                       //!< The type of event that happen
} serial_manager_t;

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
 *    if( -1 == serial_open( &serial, "/dev/ttyUSB0", 0, &cfg ) )
 *      // handle error
 *    // ready to use ...
 * @endcode
 *   Method-2:
 * @code{.c}
 *    serial_t serial;    
 *    if( -1 == serial_open( &serial, "/dev/ttyUSB0", 0, NULL ) )
 *      // handle error
 *    serial_set_baudrate( B115200, &serial );
 *    serial_set_rule( 10, 0, &serial );
 *    // ready to use ...
 * @endcode
 *   
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_open( serial_t * serial, const char * pathname, uint8_t readonly, const serial_config_t * config );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief  Connects the serial port with epoll for event-driven operation. \n
 * Instead of continuous polling, epoll allows the application to sleep until a serial port event occurs, drastically reducing CPU saturation during idle periods.
 *  
 * @param[out] manager A pointer to the serial port manager structure (`serial_manager_t`) that will be populated by this function.
 * @param[in] events A bitmask of `EPOLL_EVENTS` specifying the events to monitor. Use `EPOLLIN` to trigger on incoming data and `EPOLLOUT` to trigger when the port is ready for writing.
 * 
 * @return Upon success, the serial port is attached to epoll for event-driven operation, and the 0 is returned. \n 
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 *
 *  - `EINVAL`: Invalid argument \n
 *  - `EBADF`: Bad file descriptor \n
 *  - ...
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t serial_manage( serial_manager_t * manager, enum EPOLL_EVENTS events );


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
 * It can be used to clear the input buffer (discarding unread data) `TCIFLUSH`, the output buffer (discarding
 * pending data) `TCOFLUSH`, or both `TCIOFLUSH`.
 * 
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 * @param[in] option Selection to which buffer should be discarded, options can be: `TCIFLUSH`, `TCOFLUSH` or `TCIOFLUSH`.
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
 * @param[in] out If non-zero, the output string will be printed to standard output.
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
char * serial_print_config( uint8_t out, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the serial port baud rate configuration.
 * 
 * @param[in] baudrate The value of the desired baud rate (`baudRate_t`), example B9600.
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
int8_t serial_set_baudrate( const baudRate_t baudrate, const serial_t * serial );


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
int8_t serial_set_parity( const parity_t parity, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the serial port number of stop bits.
 * 
 * @param[in] stopBits The number of the stop bits desired (`stopBits_t`), example STOP_BITS_1.
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
int8_t serial_set_stopbits( const stopBits_t stopBits, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the serial port number of data bits.
 * 
 * @param[in] dataBits The number of the data bits desired (`dataBits_t`), example DATA_BITS_8.
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
int8_t serial_set_databits( const dataBits_t dataBits, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the serial port flow control option configuration.
 * 
 * @param[in] flowControl The value of the flow control mode desired (`flowControl_t`), example FLOWCONTROL_SOFTWARE.
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
int8_t serial_set_flowcontrol( const flowControl_t flowControl, const serial_t * serial );


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
int8_t serial_set_rule( const uint8_t timeout, const uint8_t min, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial port baud rate and fill a `baudRate_t` variable passed as argument.
 * 
 * @param[out] baudrate A pointer to a variable `baudRate_t` to be filled with the current serial port baud rate, can be nullable.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, it returns a pointer to the string with the information. \n 
 *         Otherwise NULL is returned and errno is set. 
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *  - `EBADF`: Bad file descriptor
 *
 * @note The user must free the returned string.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
char * serial_get_baudrate( baudRate_t * baudrate, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial port parity mode and fill a `parity_t` variable passed as argument.
 * 
 * @param[out] parity A pointer to a variable `parity_t` to be filled with the current serial port parity mode, can be nullable.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return Upon success, it returns a pointer to the string with the information. \n 
 *         Otherwise NULL is returned and errno is set. 
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *  - `EBADF`: Bad file descriptor
 *
 * @note The user must free the returned string.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
char * serial_get_parity( parity_t * parity, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial flow control mode and fill a `flowControl_t` variable passed as argument.
 * 
 * @param[out] flowControl A pointer to a variable `flowControl_t` to be filled with the current serial port flow control option, can be nullable.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return A pointer to the string with the information. \n 
 *         Otherwise NULL is returned and errno is set. 
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *  - `EBADF`: Bad file descriptor
 *
 * @note The user must free the returned string.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
char * serial_get_flowcontrol( flowControl_t * flowControl, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial port timeout in deciseconds and minimum number of bytes and fills the timeout and minimum number of bytes variables passed as argument.
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
 *  - `EBADF`: Bad file descriptor
 *
 * @note The user must free the returned string.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
char * serial_get_rule( uint8_t * timeout, uint8_t * min, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial port number of stop bits and fill a `stopBits_t` variable passed as argument.
 * 
 * @param[out] stopBits A pointer to a variable `stopBits_t` to be filled with the current serial port number of stop bits, can be nullable.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return A pointer to the string with the information. \n 
 *         Otherwise NULL is returned and errno is set. 
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *  - `EBADF`: Bad file descriptor
 *
 * @note The user must free the returned string.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
char * serial_get_stopbits( stopBits_t * stopBits, const serial_t * serial );


/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Returns a string with the current serial port number of data bits and fill a `dataBits_t` variable passed as argument.
 * 
 * @param[out] dataBits A pointer to a variable `dataBits_t` to be filled with the current serial port number of data bits, can be nullable.
 * @param[in] serial The serial port structure (`serial_t`) associated with the serial port itself.
 *
 * @return A pointer to the string with the information. \n 
 *         Otherwise NULL is returned and errno is set. 
 * 
 *  - `EINVAL`: Invalid argument \n
 *  - `ENOMEM`: Memory allocation failed \n
 *  - `EBADF`: Bad file descriptor
 *
 * @note The user must free the returned string.
 * 
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
char * serial_get_databits( dataBits_t * dataBits, const serial_t * serial );


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
size_t serial_readLine( char * buf, const size_t size, const size_t offset, const serial_t * serial );


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
size_t serial_read( char * buf, const size_t size, const size_t offset, const size_t length, const serial_t * serial );


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
size_t serial_writef( const serial_t * serial, const char * format, ... );

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
size_t serial_write( const serial_t * serial, const uint8_t * data, const size_t len );

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
 * @param[in] manager The initialized serial port manager structure (`serial_manager_t`).
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
int8_t serial_wait( serial_manager_t * manager, const int timeout );

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
int8_t serial_set_line_state( const serialLines_t line, uint8_t state, const serial_t * serial );


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
int8_t serial_get_line_state( const serialLines_t line, uint8_t *state, const serial_t * serial );

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
