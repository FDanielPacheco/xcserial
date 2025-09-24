/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      xcserial.c
 * 
 * @version   1.0.0
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
 * Local Function Prototype
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

int8_t clear_field_id( const char * field, const size_t length, serial_id_t * identificator );
int8_t add_field_id( const char * field, const size_t length, serial_id_t * identificator );
int8_t get_ids( const char * pathname, serial_id_t * identificator );
int8_t cmp_ids( serial_id_t * id1, serial_id_t * id2 );
int8_t cpy_fields_ids( serial_id_t * dst, const serial_id_t * src );

int8_t get_termios( int fd, struct termios * tty );
int8_t apply_termios( int fd, struct termios * tty );

int8_t fs_error( serial_t * serial );

void * async_epoll_thread( void * arg );

const char * get_stringlut_from_code( void * code, const lut_t * table, const size_t nitems, const size_t size );
const char * get_baudrate_from_code( const baudrate_t code );
const char * get_flow_control_from_code( const flow_control_t code );
const char * get_parity_from_code( const parity_t code );
const char * get_data_bits_from_code( const data_bits_t code );
const char * get_stop_bits_from_code( const stop_bits_t code );

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Local Macros
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#define error_print( txt, ... ) fprintf( stderr, "Error: " txt ",at line %d in file %s\n Errno: %d, %s\n", ##__VA_ARGS__, __LINE__, __FILE__, errno, strerror(errno) )

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
};

static const 
lut_t lut_parity[ ] = {
  {"none", BPARITY_NONE},
  {"odd" , BPARITY_ODD },
  {"even", BPARITY_EVEN},
};

static const
lut_t lut_flow_control[ ] = {
  {"none"    , FLOWCONTROL_NONE    },
  {"hardware", FLOWCONTROL_HARDWARE},
  {"software", FLOWCONTROL_SOFTWARE},
};

static const
lut_t lut_data_bits[ ] = {
  {"5", DATA_BITS_5},
  {"6", DATA_BITS_6},
  {"7", DATA_BITS_7},
  {"8", DATA_BITS_8},
};

static const
lut_t lut_stop_bits[ ] = {
  {"1", STOP_BITS_1},
  {"2", STOP_BITS_2},
};

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Function Description
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_open( serial_t * serial, const char * pathname, uint8_t readonly, const serial_config_t * config, serial_id_t * id, serial_async_t * async ){
  if( !serial ){
    errno = EINVAL;
    error_print( "serial struct is `NULL`" );
    return -1;
  }

  if( !pathname ){
    errno = EINVAL;
    error_print( "pathname is `NULL`" );
    return -1;
  }

  if( strlen(pathname) >= PATH_MAX ){
    errno = ENAMETOOLONG;
    error_print( "pathname is greater than %d", PATH_MAX );
    return -1;
  }
  
  memset( serial, 0, sizeof(serial_t) );

  if( NULL != config )
    readonly = config->readonly;

  if( readonly )
    serial->fd = open( pathname, O_RDONLY | O_NOCTTY );
  else 
    serial->fd = open( pathname, O_RDWR | O_NOCTTY );

  if( -1 == serial->fd ){
    error_print( "open(%s)", pathname );
    return -1;
  }

  if( readonly )
    serial->fp = fdopen( serial->fd, "r" );
  else   
    serial->fp = fdopen( serial->fd, "r+" );

  if( !serial->fp ){
    error_print( "fdopen" );
    if( -1 == close( serial->fd ) )
      error_print( "close" );
    return -1;
  } 

  if( !strncpy( serial->pathname, pathname, sizeof(serial->pathname) - 1 ) )
    return -1;

  serial->config.readonly = (readonly != 0);

  if( !config ){
    serial_config_t config_default;
    if( -1 == serial_default_config( &config_default ) ){
      error_print( "serial_default_config" );
      serial_close( serial );
      return -1;
    }

    if( -1 == serial_set_config( &config_default, serial ) ){
      error_print( "serial_set_config" );
      serial_close( serial );
      return -1;
    }
  }
  else{
    if( -1 == serial_set_config( config, serial ) ){
      error_print( "serial_set_config" );
      serial_close( serial );
      return -1;
    }
  }

  if( -1 == serial_event_enable( serial ) ){
    error_print( "config_update" );
    serial_close( serial );
    return -1;
  }

  if( NULL != id ){
    if( -1 == get_ids( serial->pathname, id ) ){
      error_print( "get_ids" );
      serial_close( serial );
      return -1;
    }
    
    if( -1 == serial_set_udev_id( id, serial ) ){
      error_print( "serial_set_udev_id" );
      serial_close( serial );
      return -1;
    }
    
  }

  if( NULL != async )
    memcpy( &(serial->async), async, sizeof(serial_async_t) );
  serial->async.close = 1;

  return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_reopen( serial_t * serial, uint16_t iterations ){
  if( !serial ){
    errno = EINVAL;
    return -1;    
  }

  uint8_t nmax_devs = 255;
  uint8_t n_list_devs;
  char list_devs[ nmax_devs ][PATH_MAX];

  uint32_t delay = 1;
  uint32_t factor = 2;

  serial_close( serial );

  for( uint16_t j = 0 ; j < iterations ; ++j ){
    if( -1 == serial_get_udev_devs_list( list_devs, nmax_devs, PATH_MAX, &n_list_devs ) )
      return EXIT_FAILURE;

    for( uint8_t i = 0 ; i < n_list_devs ; ++i ){
      serial_id_t id;
      memset( &id, 0, sizeof(serial_id_t) );

      const char * pathname = list_devs[ i ];
      
      if( -1 == cpy_fields_ids( &id, &(serial->id) ) )
        continue;

      if( -1 == get_ids( pathname, &id ) )
        continue;

      size_t len = 0;
      char output[ PATH_MAX ];
      for( uint8_t k = 0 ; k < id.ndev ; ++k )
        len += (size_t) snprintf( output + len, sizeof(output) - (size_t) len, "%s: %s\n", id.dev[k].field, id.dev[k].value );

      if( 0 < cmp_ids( &id, &(serial->id) ) ){
        serial_t tmp;

        if( -1 == serial_open( &tmp, pathname, serial->config.readonly, &(serial->config), &(serial->id), &(serial->async) ) ){          
          if( EBUSY == errno ){
            error_print("serial_open\n");
            return -1;
          }
        }
        else{
          if( !serial_valid( &tmp ) || (-1 == serial_set_databits( serial->config.data_bits, &tmp )) ){
            error_print("serial_open\n");
            return -1;
          }          

          memcpy( serial, &tmp, sizeof(serial_t) );
          return 0;
        }
      }
    }
    
    printf("Next iteration in %d s ...\n", delay);
    sleep( delay );
    if( delay <= 30 )
      delay *= factor;
  }

  return -1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_udev_id( const serial_id_t * id, serial_t * serial ){
  if( !serial_valid( serial ) || !id ){
    errno = EINVAL;
    return -1;
  }

  memcpy( &(serial->id), id, sizeof(serial_id_t) );
  return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_get_udev_devs_list( char devs[ ][PATH_MAX], uint8_t size, size_t length, uint8_t * ndevs ){
  if( !devs ){
    errno = EINVAL;
    return -1;
  }

  struct udev * udev = udev_new();  
  if( !udev ){
    error_print( "udev_new" );
    return -1;
  }

  struct udev_enumerate * enumerate = udev_enumerate_new( udev );
  if( !enumerate ){
    error_print( "udev_enumerate_new" );
    udev_unref( udev );
    return -1;
  }

  if( 0 > udev_enumerate_add_match_subsystem( enumerate, "tty" ) ){
    error_print( "udev_enumerate_add_match_subsystem" );
    udev_enumerate_unref( enumerate );
    udev_unref( udev );
    return -1;
  }

  if( 0 > udev_enumerate_scan_devices( enumerate ) ){
    error_print( "udev_enumerate_scan_devices" );
    udev_enumerate_unref( enumerate );
    udev_unref( udev );
    return -1;
  }

  struct udev_list_entry * devices = udev_enumerate_get_list_entry( enumerate );
  if( !devices ){
    error_print( "udev_enumerate_get_list_entry" );
    udev_enumerate_unref( enumerate );
    udev_unref( udev );
    return -1;
  }

  *ndevs = 0;
  struct udev_list_entry * entry;

  udev_list_entry_foreach( entry, devices ){
    const char * pathname = udev_list_entry_get_name( entry );
    if( !pathname )
      continue;
    
    struct udev_device * dev = udev_device_new_from_syspath( udev, pathname );
    if( !dev ){
      error_print( "udev_device_new_from_syspath" );
      udev_enumerate_unref( enumerate );
      udev_unref( udev );
      return -1;
    }

    const char * devnode = udev_device_get_devnode( dev );
    if( devnode ){
      if( size <= *ndevs ){
        errno = ENOBUFS;
        break;
      }
      else
        if( !strncpy( devs[ (*ndevs)++ ], devnode, length ) )
          break;
    }
    
    udev_device_unref( dev );
  }

  udev_enumerate_unref( enumerate );
  udev_unref( udev );
  return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_udev_param_list( const char params[][NAME_MAX], uint8_t n, size_t length, serial_t * serial ){
  if( !serial || !params ){
    errno = EINVAL;
    return -1;
  } 
  
  serial->id.ndev = 0;

  for( uint8_t i = 0 ; i < n ; ++i ){
    const char * param = params[i];
    if( !param )
      continue;

    if( -1 == add_field_id( param, length, &(serial->id) ) )
      break;
  }

  if( -1 == get_ids( serial->pathname, &(serial->id) ) ){
    error_print( "get_ids" );
    return -1;
  }  

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
clear_field_id( const char * field, const size_t length, serial_id_t * identificator ){
  if( !identificator || !field ){
    errno = EINVAL;
    return -1;
  }

  uint8_t ndev = identificator->ndev;

  for( uint8_t i = 0 ; i < ndev ; ++i ){
    if( !strncmp( field, identificator->dev[ i ].field, length ) )
      for( uint8_t j = i ; j < ndev - 1 ; ++j ){
        strcpy( identificator->dev[ i ].field, identificator->dev[ i + 1 ].field );
        strcpy( identificator->dev[ i ].value, identificator->dev[ i + 1 ].value );
      }
  }

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
add_field_id( const char * field, const size_t length, serial_id_t * identificator ){
  if( !identificator || !field ){
    errno = EINVAL;
    return -1;
  }
  
  if( !strncpy( identificator->dev[ identificator->ndev ].field, field, length ) )
    return -1;

  identificator->ndev++;
  return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
get_ids( const char * pathname, serial_id_t * identificator ){
  if( !pathname || !identificator ){
    errno = EINVAL;
    return -1;
  }

  struct udev * udev = udev_new( );
  if( !udev )
    return -1;
  
  const char * basename = strrchr( pathname, '/');
  if( !basename ){
    udev_unref( udev );
    return -1;
  }
  basename++;   

  struct udev_device * dev = udev_device_new_from_subsystem_sysname( udev, "tty", basename );
  if( !dev ){
    udev_unref( udev );
    return -1;
  }

  struct udev_device * parent = udev_device_get_parent( dev );
  if( !parent ){
    udev_unref( udev );
    udev_device_unref( dev );
    return -1;
  }

  const char * subsys = udev_device_get_subsystem( parent );
  if( subsys ){
    if( NULL != strncpy( identificator->bus, subsys, NAME_MAX ) )
      for( uint8_t i = 0 ; i < identificator->ndev ; ++i ){
        serial_udev_paramater_t * parameter = &(identificator->dev[i]);
        const char * value = !udev_device_get_property_value( dev, parameter->field ) ? " " : udev_device_get_property_value( dev, parameter->field );          
        if( !strncpy( parameter->value, value, NAME_MAX ) )
          break;
      }
  }

  udev_unref( udev );
  udev_device_unref( dev );
  return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
cmp_ids( serial_id_t * id1, serial_id_t * id2 ){
  if( !id1 || !id2 ){
    errno = EINVAL;
    return -1;
  }

  uint8_t n = id1->ndev;
  if( id1->ndev > id2->ndev )
    n = id2->ndev;

  if( !n ){
    errno = EINVAL;
    return -1;
  }

  for( uint8_t i = 0 ; i < n ; ++i )
    if( 0 != strcmp( id1->dev[i].value, id2->dev[i].value ) )
      return 0;

  return 1;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
cpy_fields_ids( serial_id_t * dst, const serial_id_t * src ){
  if( !src || !dst ){
    errno = EINVAL;
    return -1;
  }
  
  for( uint8_t i = 0 ; i < src->ndev ; ++i )
    if( !strncpy( dst->dev[ i + dst->ndev ].field, src->dev[i].field, NAME_MAX ) )
      return -1;
  
  dst->ndev += src->ndev;

  return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_udev_param_field( const char * field, size_t length, serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;

  if( !field || !length ){
    errno = EINVAL;
    return -1;
  }

  char params[ 1 ][ NAME_MAX ];
  if( !strncpy( params[0], field, NAME_MAX - 1 ) ){
    error_print( "strncpy" );
    return -1;
  }

  if( -1 == serial_set_udev_param_list( params, 1, NAME_MAX, serial ) ){
    error_print( "serial_set_udev_param_list" );
    return -1;
  }
  
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_udev_param_value( const char * field, size_t length, serial_t * serial ){
  if( !serial_valid( serial ) || !field ){
    errno = EINVAL;
    return NULL;
  }

  serial_id_t id;
  
  if( -1 == add_field_id( field, length, &id ) ){
    error_print( "add_field_id" );
    return NULL;
  }

  if( -1 == get_ids( serial->pathname, &id ) ){
    error_print( "get_ids" );
    return NULL;
  }
  
  return strdup( id.dev[0].value );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_event_enable( serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;
    
  serial->event.fd = epoll_create1( 0 );
  if( -1 == serial->event.fd ){
    error_print( "epoll_create1" );
    return -1;
  } 

  struct epoll_event ev;
  ev.events = EPOLLIN;
  ev.data.fd = serial->fd;      

  if( -1 == epoll_ctl( serial->event.fd, EPOLL_CTL_ADD, serial->fd, &ev ) ){
    error_print( "epoll_ctl" );
    return -1;
  }

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_close( serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;

  if( EOF == fclose( serial->fp ) ){
    error_print( "fclose" );
    return -1;
  }

  if( !serial->async.close ){
    serial->async.close = 1;
    if( 0 != pthread_join( serial->async.thread, NULL) ){
      error_print( "pthread_join" );
      return -1;
    }
  }

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_default_config( serial_config_t * config ){
  if( !config ){
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
serial_set_baudrate( const baudrate_t baudrate, serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;

  struct termios tty;
  if( !get_termios( serial->fd, &tty ) )
    return -1;

  int result = cfsetispeed( &tty, baudrate );
  if( 0 != result ){
    error_print( "cfsetispeed" );
    return -1;
  }

  result = cfsetospeed( &tty, baudrate );
  if( 0 != result ){
    error_print( "cfsetospeed" );
    return -1;
  }
  
  if( !apply_termios( serial->fd, &tty ) )
    return -1;

  serial->config.baudrate = baudrate;
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_parity( const parity_t parity, serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;

  struct termios tty;
  if( !get_termios( serial->fd, &tty ) )
    return -1;
  
  switch( parity ){
    default:
      errno = EINVAL;
      return -1;
      
    case BPARITY_NONE:
      tty.c_cflag &= (tcflag_t) ~(PARENB);                                    // Disable parity (Clear bit)
      tty.c_iflag &= (tcflag_t) ~(INPCK);                                     // Disable parity checking
      break;
    
    case BPARITY_ODD:
      tty.c_cflag |= (tcflag_t) (PARENB) | (PARODD);                          // Enable parity (Set bit) and Enable odd parity
      tty.c_iflag |= (tcflag_t) (INPCK);                                      // Enable parity checking
      break;

    case BPARITY_EVEN:
      tty.c_cflag |= (tcflag_t) (PARENB);                                     // Enable parity (Set bit)
      tty.c_cflag &= (tcflag_t) ~(PARODD);                                    // Enable even parity
      tty.c_iflag |= (tcflag_t) (INPCK);                                      // Enable parity checking
      break;
  }

  if( !apply_termios( serial->fd, &tty ) )
    return -1;

  serial->config.parity = parity;
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_stopbits( const stop_bits_t stop_bits, serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;

  struct termios tty;
  if( !get_termios( serial->fd, &tty ) )
    return -1;

  switch( stop_bits ){
    default:
      errno = EINVAL;
      return -1;

    case STOP_BITS_1:
      tty.c_cflag &= (tcflag_t) ~(CSTOPB);                                    // Set 1 stop bit 
      break;

    case STOP_BITS_2:      
      tty.c_cflag |= (tcflag_t) (CSTOPB);                                     // Set 2 stop bits
      break;
  }

  if( !apply_termios( serial->fd, &tty ) )
    return -1;

  serial->config.stop_bits = stop_bits;
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_databits( const data_bits_t data_bits, serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;

  struct termios tty;
  if( !get_termios( serial->fd, &tty ) )
    return -1;

  tty.c_cflag &= (tcflag_t) ~CSIZE;                                          // Clear all the size bits, then use one of the statements below
  tty.c_cflag |= (tcflag_t) data_bits;                                       // Bits per word

  if( !apply_termios( serial->fd, &tty ) )
    return -1;

  serial->config.data_bits = data_bits;
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_flowcontrol( const flow_control_t flow_control, serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;

  struct termios tty;
  if( !get_termios( serial->fd, &tty ) )
    return -1;

  switch( flow_control ){
    default:
      errno = EINVAL;
      return -1;

    case FLOWCONTROL_NONE:
      tty.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
      tty.c_cflag &= (tcflag_t) ~(CRTSCTS);
      tty.c_cc[VSTART] = 0;                                                   // Disable start character (XON) - disable software flow control
      tty.c_cc[VSTOP] = 0;                                                    // Disable stop character (XOFF) - disable software flow control
      break;

    case FLOWCONTROL_HARDWARE:
      tty.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
      tty.c_cflag |= (tcflag_t) (CRTSCTS);
      tty.c_cc[VSTART] = 0;                                                   // Disable start character (XON) - disable software flow control
      tty.c_cc[VSTOP] = 0;                                                    // Disable stop character (XOFF) - disable software flow control
      break;
    
    case FLOWCONTROL_SOFTWARE:
      tty.c_iflag |= (tcflag_t) (IXON | IXOFF | IXANY);
      tty.c_cflag &= (tcflag_t) ~(CRTSCTS);
      tty.c_cc[VSTART] = 1;                                                   // Enable start character (XON) - enable software flow control
      tty.c_cc[VSTOP] = 1;                                                    // Enable stop character (XOFF) - enable software flow control
      break;
  }

  if( !apply_termios( serial->fd, &tty ) )
    return -1;

  serial->config.flow_control = flow_control;
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_rule( const uint8_t timeout, const uint8_t min, serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;

  struct termios tty;
  if( !get_termios( serial->fd, &tty ) )
    return -1;

  tty.c_lflag &= (tcflag_t) ~(ICANON);                                        // Disable canonical mode
  tty.c_lflag &= (tcflag_t) ~(ECHO);                                          // Disable echo
  tty.c_lflag &= (tcflag_t) ~(ECHOE);                                         // Disable erasure
  tty.c_lflag &= (tcflag_t) ~(ECHONL);                                        // Disable new-line echo
  tty.c_lflag &= (tcflag_t) ~(ISIG);                                          // Disable interpretation of INTR, QUIT and SUSP
  tty.c_oflag &= (tcflag_t) ~(OPOST);                                         // Set to raw output
  tty.c_oflag &= (tcflag_t) ~(ONLCR);                                         // Disable the conversion of new line to CR/LF
  tty.c_iflag &= (tcflag_t) ~(IGNBRK);                                        // Disable ignore break condition
  tty.c_iflag &= (tcflag_t) ~(BRKINT);                                        // Disable send a SIGINT when a break condition is detected
  tty.c_iflag &= (tcflag_t) ~(INLCR);                                         // Disable map NL to CR
  tty.c_iflag &= (tcflag_t) ~(IGNCR);                                         // Disable ignore CR
  tty.c_iflag &= (tcflag_t) ~(ICRNL);                                         // Disable map CR to NL
  tty.c_cc[VEOF]     = 4;                                                     // Set EOF character to EOT (Ctrl+D, ASCII 4) - or 0 if not used
  tty.c_cc[VTIME]    = timeout;                                               // Set timeout for read() in tenths of a second
  tty.c_cc[VMIN]     = min;                                                   // Set minimum number of bytes for read() to return
  tty.c_cc[VINTR]    = 0;                                                     // Disable interrupt character (Ctrl+C)
  tty.c_cc[VQUIT]    = 0;                                                     // Disable quit character (Ctrl+\)
  tty.c_cc[VERASE]   = 0;                                                     // Disable erase character (backspace) - not relevant in raw mode
  tty.c_cc[VKILL]    = 0;                                                     // Disable kill character (Ctrl+U) - not relevant in raw mode
  tty.c_cc[VSWTC]    = 0;                                                     // Disable switch character - not usually needed
  tty.c_cc[VSUSP]    = 0;                                                     // Disable suspend character (Ctrl+Z)
  tty.c_cc[VEOL]     = 0;                                                     // Disable end-of-line character - not relevant in raw mode
  tty.c_cc[VREPRINT] = 0;                                                     // Disable reprint character - not relevant in raw mode
  tty.c_cc[VDISCARD] = 0;                                                     // Disable discard character - not relevant in raw mode
  tty.c_cc[VWERASE]  = 0;                                                     // Disable word erase character - not relevant in raw mode
  tty.c_cc[VLNEXT]   = 0;                                                     // Disable literal next character - not relevant in raw mode
  tty.c_cc[VEOL2]    = 0;                                                     // Disable alternate end-of-line character - not relevant in raw mode

  if( !apply_termios( serial->fd, &tty ) )
    return -1;

  serial->config.timeout_ds = timeout;
  serial->config.min_bytes = min;
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_timeout( const int timeout, serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;
  
  serial->config.event_timeout_ms = timeout;
  return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_config( const serial_config_t * config, serial_t * serial ){
  if( !config ){
    errno = EINVAL;
    error_print( "config null" );
    return -1;
  }

  if( !serial_valid( serial ) )
    return -1;

  if( -1 == serial_set_baudrate( config->baudrate, serial ) ){
    error_print( "serial_set_baudrate" );
    return -1;
  }

  if( -1 == serial_set_parity( config->parity, serial ) ){
    error_print( "serial_set_parity" );
    return -1;
  }

  if( -1 == serial_set_stopbits( config->stop_bits, serial ) ){
    error_print( "serial_set_stopbits" );
    return -1;
  }

  if( -1 == serial_set_databits( config->data_bits, serial ) ){
    error_print( "serial_set_databits" );
    return -1;
  }
      
  if( -1 == serial_set_flowcontrol( config->flow_control, serial ) ){
    error_print( "serial_set_flowcontrol" );
    return -1;
  }
  
  if( -1 == serial_set_rule( config->timeout_ds, config->min_bytes, serial ) ){
    error_print( "serial_set_rule" );
    return -1;
  }

  if( -1 == serial_set_timeout( config->event_timeout_ms, serial ) ){
    error_print( "serial_set_timeout" );
    return -1;
  }

  serial_get_config( serial );
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_readLine( char * buf, const size_t size, const size_t offset, serial_t * serial ){  
  if( !buf ){
    errno = EINVAL;
    error_print( "buf null" );
    return 0;
  }

  if( !serial_valid( serial ) )
    return 0;

  if( size <= offset ){
    errno = ENOMEM;
    error_print( "serial_readLine overflow" );
    return 0;
  }

  if( 0 != serial->config.event_timeout_ms ){
    int8_t ev = serial_event_wait( serial, serial->config.event_timeout_ms );
    if( 1 > ev ){
      if( (EPOLLHUP == errno) || (EBADF == errno) )
        errno = ENODEV;
      errno = ETIME;
    }
  }
  
  clearerr( serial->fp );
  errno = 0;

  fgets( buf + offset, (int) (size - offset), serial->fp );

  fs_error( serial );
  if( (ENODEV == errno) || !serial_get_databits( NULL, serial ) ){
    errno = ENODEV;
    return 0;
  }

  return strlen( buf + offset );  
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_read( char * buf, const size_t size, const size_t offset, const size_t length, serial_t * serial ){
  if( !buf ){
    errno = EINVAL;
    error_print( "buf null" );
    return 0;
  }

  if( !length ){
    errno = EINVAL;
    error_print( "length is 0" );
    return 0;
  }

  if( !serial_valid( serial ) )
    return 0;
  
  if( size < (offset + length) ){
    errno = ENOMEM;
    error_print( "serial_read overflow" );
    return 0;
  }

  clearerr( serial->fp );
  
  size_t total = 0;
  while( total < length ){
    size_t to_read = length;
    
    if( 0 != serial->config.event_timeout_ms ){
      int8_t ev = serial_event_wait( serial, serial->config.event_timeout_ms );
      if( 1 > ev ){
        if( (EPOLLHUP == errno) || (EBADF == errno) )
          errno = ENODEV;
        errno = ETIME;
        break;      
      }
      
      size_t arrived = serial_available( serial );
      if( !arrived && (EIO == errno) ){
        errno = ENODEV;
        return 0;
      }
    
      if( !total ){
        if( arrived > length )
          to_read = length;
        else
          to_read = arrived;
      }
      else{
        if( arrived + total > length )
          to_read = length - total;
        else 
          to_read = arrived;
      }
    }

    clearerr( serial->fp );
    errno = 0;

    size_t received = fread( buf + offset + total, 1, to_read, serial->fp );
    
    total += received;

    fs_error( serial );
    if( (ENODEV == errno) || !serial_get_databits( NULL, serial ) ){
      total = 0;
      break;
    }
    if( ETIME == errno )
      break;
  }

  return total;  
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_writef( serial_t * serial, const char * format, ... ){
  if( !format ) {
    errno = EINVAL;
    error_print( "format null" );
    return 0;
  }

  if( !serial_valid( serial ) )
    return 0;

  va_list args;
  va_start( args, format );

  char buf[ PIPE_BUF ];
  int len = vsnprintf( buf, sizeof(buf), format, args );

  if( 0 > len ) {
    error_print( "vsnprintf" );
    va_end( args );
    return 0;
  }

  if( (int) sizeof( buf ) <= len ) {
    error_print( "write overflow" );
    va_end( args );
    return 0;
  }

  size_t size = fwrite( buf, 1, (size_t) len, serial->fp );

  va_end( args );
  if( size < (size_t) len )
    return (size_t) fs_error( serial );

  if( -1 == serial_flush( serial ) )
    return (size_t) fs_error( serial );

  return size;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_write( serial_t * serial, const uint8_t * data, const size_t len ){
  if( !data ) {
    errno = EINVAL;
    error_print( "data null" );
    return 0;
  }

  if( !serial_valid( serial ) )
    return 0;

  size_t size = fwrite( data, 1, len, serial->fp );

  if( size < len )
    return (size_t) fs_error( serial );

  if( -1 == serial_flush( serial ) )
    return (size_t) fs_error( serial );
    
  return size;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
uint8_t 
serial_valid( const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    error_print( "serial null" );
    return 0;
  }

  if( 0 > serial->fd ){
    errno = EBADF;
    return 0;
  }

  if( -1 == fcntl( serial->fd, F_GETFD ) ){
    errno = EBADF;
    return 0;
  }   
  
  return 1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_drain( const serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;
  
  if( -1 == tcdrain( serial->fd ) ){
    error_print( "tcdrain" );
    return -1;
  }

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_flush( const serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;
  
  if( 0 != fflush( serial->fp ) ){
    error_print( "fflush" );
    return -1;
  }

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_available( const serial_t * serial ){
  if( !serial_valid( serial ) )
    return 0;
  
  int32_t len;
  if( -1 == ioctl( serial->fd, FIONREAD, &len ) ) {
    error_print( "available ioctl" );
    return 0;
  }  

  if( 0 > len ){
    error_print( "available ioctl" );
    return 0;
  }

  return (size_t) len;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_event_wait( serial_t * serial, const int timeout ){
  if( !serial_valid( serial ) )
    return -1;

  struct epoll_event ev; 
  int event = epoll_wait( serial->event.fd, &ev, 1, timeout );
  if( -1 == event ){
    perror("epoll_wait");
    return -1;
  }

  // Timeout
  if( !event )
    return 0;

  if( EPOLLIN & ev.events )
    return EPOLLIN;
  
  if( EPOLLHUP & ev.events )
    return EPOLLHUP;

  if( EPOLLERR & ev.events )
    return EPOLLERR;
    
  return -1;  
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_line_state( const serial_lines_t line, uint8_t state, const serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;

  int status;
  if( -1 == ioctl( serial->fd, TIOCMGET, &status ) ){
    error_print( "serial_set_line_state ioctl" );
    return -1;
  }
  
  if( state ) 
    status |= (int) line;
  else 
    status &= (int) ~line;

  if( -1 == ioctl( serial->fd, TIOCMSET, &status ) ){
    error_print( "serial_set_line_state ioctl" );
    return -1;
  }

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_get_line_state( const serial_lines_t line, uint8_t *state , const serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;

  int status;
  if( -1 == ioctl( serial->fd, TIOCMGET, &status ) ){
    error_print( "serial_get_line_state ioctl" );
    return -1;
  }
  
  *state = ( (uint8_t) status & line) != 0;
  return 0;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_get_config( serial_t * serial ){
  if( !serial_valid( serial ) )
    return -1;
  
  (void) serial_get_baudrate( &(serial->config.baudrate), serial );
  (void) serial_get_parity( &(serial->config.parity), serial );
  (void) serial_get_databits( &(serial->config.data_bits), serial );
  (void) serial_get_stopbits( &(serial->config.stop_bits), serial );
  (void) serial_get_flowcontrol( &(serial->config.flow_control), serial );
  (void) serial_get_rule( &(serial->config.timeout_ds), &serial->config.min_bytes, serial ); 
  (void) serial_get_event_timeout( &(serial->config.event_timeout_ms), serial );
  return 0;
}          

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_print_config( uint8_t out, const char * initial, const serial_t * serial ){
  if( !serial_valid( serial ) )
    return NULL;
  
  const size_t size = PATH_MAX + 1;
  char * output = (char *) malloc( size );
  if( !output ){
    errno = ENOMEM;
    error_print( "malloc" );
    return NULL;
  }
  
  int len = snprintf( output, size, 
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
                    initial, serial_get_baudrate(NULL, serial),
                    initial, serial_get_parity(NULL, serial),
                    initial, serial_get_databits(NULL, serial), 
                    initial, serial_get_stopbits(NULL, serial),
                    initial, serial_get_flowcontrol(NULL, serial),
                    initial, serial_get_rule(NULL, NULL, serial),
                    initial, serial_get_event_timeout(NULL, serial),
                    initial, serial->id.bus
  );

  for( uint8_t i = 0 ; i < serial->id.ndev ; ++i )
    len += snprintf( output + len, size - (size_t) len, "%s%s: %s\n", initial, serial->id.dev[i].field, serial->id.dev[i].value );

  char * r = (char *) realloc( output, (size_t) len + 1 );
  if( !r ){
    free( output );
    error_print( "realloc" );
    return NULL;
  }

  if( out ) 
    printf( "%s\n", r );

  return r;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char *
get_stringlut_from_code( void * code, const lut_t * table, const size_t nitems, const size_t size ){
  if( !code || !table )
    return NULL;

  for( size_t i = 0 ; i < nitems ; ++i )
    if( !memcmp( code, &( table[ i ].code ), size ) )
      return table[ i ].text;

  return NULL;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char *
get_baudrate_from_code( const baudrate_t code ){
  return get_stringlut_from_code( 
    (void *) &code, 
    lut_baudrate,
    sizeof( lut_baudrate ) / sizeof( lut_baudrate[0] ),
    sizeof( baudrate_t )
  );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char *
get_parity_from_code( const parity_t code ){
  return get_stringlut_from_code( 
    (void *) &code, 
    lut_parity,
    sizeof( lut_parity ) / sizeof( lut_parity[0] ),
    sizeof( parity_t )
  );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char *
get_data_bits_from_code( const data_bits_t code ){
  return get_stringlut_from_code( 
    (void *) &code, 
    lut_data_bits,
    sizeof( lut_data_bits ) / sizeof( lut_data_bits[0] ),
    sizeof( data_bits_t )
  );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char *
get_flow_control_from_code( const flow_control_t code ){
  return get_stringlut_from_code( 
    (void *) &code, 
    lut_flow_control,
    sizeof( lut_flow_control ) / sizeof( lut_flow_control[0] ),
    sizeof( flow_control_t )
  );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char *
get_stop_bits_from_code( const stop_bits_t code ){
  return get_stringlut_from_code( 
    (void *) &code, 
    lut_stop_bits,
    sizeof( lut_stop_bits ) / sizeof( lut_stop_bits[0] ),
    sizeof( stop_bits_t )
  );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_baudrate( baudrate_t * baudrate, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    return NULL;
  }

  struct termios tty;
  if( !get_termios( serial->fd, &tty ) ){
    if( NULL != baudrate )
      *baudrate = serial->config.baudrate;      
    return get_baudrate_from_code( serial->config.baudrate );
  }
  
  const baudrate_t br = cfgetospeed( &tty );
  if( NULL != baudrate )
    *baudrate = br;      

  return get_baudrate_from_code( br );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_parity( parity_t * parity, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    return NULL;
  }

  struct termios tty;
  if( !get_termios( serial->fd, &tty ) ){
    if( NULL != parity )
      *parity = serial->config.parity;      
    return get_parity_from_code( serial->config.parity );
  }
  
  if( !( tty.c_iflag & (tcflag_t) INPCK ) ){
    if( NULL != parity )
      *parity = BPARITY_NONE;
    return get_parity_from_code( BPARITY_NONE );
  } 
  
  if( !( tty.c_cflag & (tcflag_t) PARODD ) ){
    if( NULL != parity )
      *parity = BPARITY_EVEN;
    return get_parity_from_code( BPARITY_EVEN );
  } 

  if( NULL != parity )
    *parity = BPARITY_ODD;    

  return get_parity_from_code( BPARITY_ODD );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_stopbits( stop_bits_t * stop_bits, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    return NULL;
  }

  struct termios tty;
  if( !get_termios( serial->fd, &tty ) ){
    if( NULL != stop_bits )
      *stop_bits = serial->config.stop_bits;      
    return get_stop_bits_from_code( serial->config.stop_bits );
  }

  if( !( tty.c_iflag & (tcflag_t) CSTOPB ) ){
    if( NULL != stop_bits )
      *stop_bits = STOP_BITS_1;
    return get_stop_bits_from_code( STOP_BITS_1 );
  }

  if( NULL != stop_bits )
    *stop_bits = STOP_BITS_2;
  return get_stop_bits_from_code( STOP_BITS_1 );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_databits( data_bits_t * data_bits, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    return NULL;
  }

  struct termios tty;
  if( !get_termios( serial->fd, &tty ) ){
    if( NULL != data_bits )
      *data_bits = serial->config.data_bits;     
    return get_data_bits_from_code( serial->config.data_bits );
  }

  tcflag_t _data_bits = tty.c_cflag & (tcflag_t) CSIZE;
  if( NULL != data_bits ) 
    *data_bits = _data_bits;
  return get_data_bits_from_code( _data_bits );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_flowcontrol( flow_control_t * flow_control, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    return NULL;
  }

  struct termios tty;
  if( !get_termios( serial->fd, &tty ) ){
    if( NULL != flow_control )
      *flow_control = serial->config.flow_control;     
    return get_flow_control_from_code( serial->config.flow_control );
  }

  if( !(tty.c_cflag & (tcflag_t) CRTSCTS) ){
    if( !(tty.c_iflag & (tcflag_t) (IXON | IXOFF | IXANY) ) ){
      if( NULL != flow_control )
        *flow_control = FLOWCONTROL_NONE;
      return get_flow_control_from_code( FLOWCONTROL_NONE );
    }

    if( NULL != flow_control )
      *flow_control = FLOWCONTROL_SOFTWARE;
    return get_flow_control_from_code( FLOWCONTROL_SOFTWARE );
  }

  if( NULL != flow_control )
    *flow_control = FLOWCONTROL_HARDWARE;
  return get_flow_control_from_code( FLOWCONTROL_HARDWARE );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_rule( uint8_t * timeout, uint8_t * min, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    return NULL;
  }

  const size_t txt_len = 16;
  char * txr_repre = ( char * ) malloc( txt_len );
  if( !txr_repre ){
    errno = ENOMEM;
    error_print( "malloc" );
    return NULL;
  } 

  struct termios tty;
  if( !get_termios( serial->fd, &tty ) ){
    if( NULL != timeout )
      *timeout = serial->config.timeout_ds;
    if( NULL != min )
      *min = serial->config.min_bytes;

    snprintf( txr_repre, txt_len, "%hhd, %hhd", serial->config.timeout_ds, serial->config.min_bytes );
    return txr_repre;
  }
  
  if( NULL != timeout )
    *timeout = tty.c_cc[VTIME];
  if( NULL != min )
    *min = tty.c_cc[VMIN];

  snprintf( txr_repre, txt_len, "%hhd, %hhd", serial->config.timeout_ds, serial->config.min_bytes );
  return txr_repre;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
const char * 
serial_get_event_timeout( int * timeout, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    return NULL;
  }

  const size_t txt_len = 16;
  char * txr_repre = ( char * ) malloc( txt_len );
  if( !txr_repre ){
    errno = ENOMEM;
    error_print( "malloc" );
    return NULL;
  } 

  if( NULL != timeout )
    *timeout = serial->config.event_timeout_ms;

  snprintf( txr_repre, txt_len, "%d", serial->config.event_timeout_ms );
  return txr_repre;
}


/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
get_termios( int fd, struct termios * tty ){
  if( !tty ){
    errno = EINVAL;
    error_print( "tty null" );
    return 0;
  }

  int result = tcgetattr( fd, tty );
  if( 0 != result ){
    error_print( "tcgetattr" );
    return 0;
  }
  return 1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
apply_termios( int fd, struct termios * tty ){
  if( !tty ){
    errno = EINVAL;
    error_print( "tty null" );
    return 0;
  }

  int result = tcsetattr( fd, TCSANOW, tty );
  if( 0 != result ){
    error_print( "tcsetattr" );
    return 0;
  }
  return 1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
fs_error( serial_t * serial ){
  FILE * fp = serial->fp;
  if( feof( fp ) ){
    clearerr( fp );
    errno = ETIME;
    
    if( !serial_valid( serial ) )
      errno = ENODEV;
  }
  else {
    if( ferror( fp ) ){
      error_print( "ferror" );
      errno = ENODEV;
    }
  }

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_async_set_callback( serial_read_callback_t handler_read, serial_disconnect_callback_t handler_disconnect, serial_t * serial ){
  serial->async.rcb = handler_read;
  serial->async.dcb = handler_disconnect;
  serial->async.close = 0;

  pthread_attr_t * attr = NULL;
  if( !pthread_create( &(serial->async.thread), attr, async_epoll_thread, serial ) )
    return -1;
  
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void * 
async_epoll_thread( void * arg ){
  serial_t * serial = (serial_t *) arg;
  
  if( !serial_valid( serial ) )
    return NULL;

  uint8_t buf[ BUFSIZ ];

  for( ; ; ){
    if( serial->async.close )
      break;

    size_t len = serial_read( (char *) buf, sizeof(buf), 0, sizeof(buf)-1, serial );    
    if( 0 < len )
      serial->async.rcb( buf, len );

    if( !len ){
      switch( errno ){
        case EIO:
        case ENODEV:
        case EBADF:
        case EPOLLHUP:
        case EINVAL:
          serial->async.dcb( );
        
        default:
          break;
      }
    }
  }    

  return NULL;
}


/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * End of file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
