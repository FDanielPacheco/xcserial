#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "xcserial.h"

#define N_PARAMS  3

int
test( 
  serial_t * serial, 
  int _timeout_ms, 
  uint8_t _timeout_ds, 
  const char * send, 
  const char * received, 
  int id, 
  int op,
  int wait
){
  printf("\n------- [TEST %d] --------\n", id );

  serial_set_timeout( _timeout_ms, serial );
  serial_set_rule( _timeout_ds, 0, serial );

  const uint8_t output = 1;
  if( !serial_print_config( output, "", serial ) ){
    fprintf( stderr, "Error: %d, %s\n", errno, strerror(errno) );
    return 0;
  }

  if( wait ){
    printf("Press Enter to continue:\n");
    getchar( );
  }

  size_t nread = 0, nwrite;
  uint8_t buf[ BUFSIZ ];

  while( !nread ){

    nwrite = 0;

    while( !nwrite ){
      nwrite = serial_writef( serial, "%s", send );
    
      if( !nwrite ){

        if( (errno == ENODEV) || (errno == EIO) ){
          printf("Device got disconnected (%d)%s...\n", serial->fd, serial->pathname );
          if( -1 == serial_reopen( serial, 1000 ) ){
            serial_close( serial );
            return 0;
          }
          else{
            nread = 0;

            serial_set_line_state( SERIAL_DTR, 0, serial );
            usleep( 1e3 );
            serial_set_line_state( SERIAL_DTR, 1, serial );
            usleep( 2e6 );

            if( !serial_print_config( output, "", serial ) ){
              fprintf( stderr, "Error: %d, %s\n", errno, strerror(errno) );
              return 0;
            }

            printf("Device got reconnected (%d)%s...\n", serial->fd, serial->pathname );
          }
        }
        else{
          serial_close( serial );
          return 0;
        }
      }
      
    }

    printf("Wrote: %ld [B]\n", nwrite );

    size_t offset = 0;

    if( op )
      nread = serial_read( (char *) buf, sizeof(buf), offset, sizeof(buf)-1, serial ); 
    else
      nread = serial_readline( (char *) buf, sizeof(buf), 0, serial );
    
    if( !nread ){
      if( (errno == ENODEV) || (errno == EIO) ){        
        printf("Device got disconnected (%d)%s...\n", serial->fd, serial->pathname );
                
        if( -1 == serial_reopen( serial, 1000 ) ){
          serial_close( serial );
          return 0;
        }
        else{
          nread = 0;

          serial_set_line_state( SERIAL_DTR, 0, serial );
          usleep( 1e3 );
          serial_set_line_state( SERIAL_DTR, 1, serial );
          usleep( 2e6 );

          if( !serial_print_config( output, "", serial ) ){
            fprintf( stderr, "Error: %d, %s\n", errno, strerror(errno) );
            return 0;
          }

          printf("Device got reconnected (%d)%s...\n", serial->fd, serial->pathname );
        }
      }
      else{
        if( ETIME == errno ){
          serial_set_line_state( SERIAL_DTR, 0, serial );
          usleep( 1e3 );
          serial_set_line_state( SERIAL_DTR, 1, serial );
          usleep( 2e6 );
          
          return -1;
        }
          
        printf("Failed with errno = %d\n", errno );
        serial_close( serial );
        return 0;
      }
    }

  }

  printf("Read: %ld [B]\nMessage: %s\n", nread, (char *) buf );    
  if( !strcmp( (char *) buf, received ) )
    return 1;

  return 0;
}

int 
main( void ){
  const char  *    pathname = "/dev/ttyACM0";
  const uint8_t    readonly = 0;
  const baudrate_t baudrate = B19200; 
  serial_t serial; 

  const char params[ N_PARAMS ][NAME_MAX] = {
    "ID_MODEL",
    "ID_USB_VENDOR",
    "ID_SERIAL_SHORT"
  };

  if( -1 == serial_open( &serial, pathname, readonly, NULL, NULL, NULL ) )
    return EXIT_FAILURE;

  if( -1 == serial_set_udev_param_list( params, N_PARAMS, NAME_MAX, &serial ) )
    return EXIT_FAILURE;

  serial_set_baudrate( baudrate, &serial );

  serial_set_line_state( SERIAL_DTR, 0, &serial );
  usleep( 1e3 );
  serial_set_line_state( SERIAL_DTR, 1, &serial );
  usleep( 2e6 );
   
  const char field[ ] = "ID_MODEL_FROM_DATABASE";
  printf("%s: %s\n", field, serial_get_udev_param_value( field, strlen(field), &serial ) );

  typedef struct {
    int          timeout_ms;
    uint8_t      timeout_ds;
    const char * send;
    const char * received;
    int          op;   
  } test_params_t;

  test_params_t parameters[ ] = {
    {100 , 0 , "UTEST:WRITE\n"   , "UTEST:OK:WRITE"     , 1},
    {0   , 10, "UTEST:WRITE\n"   , "UTEST:OK:WRITE"     , 1},
    {3000, 0 , "UTEST:WRITE\n"   , "UTEST:OK:WRITE"     , 1},
    {0   , 3 , "UTEST:WRITE_LF\n", "UTEST:OK:WRITE_LF\n", 0},
    {10  , 0 , "UTEST:WRITE_LF\n", "UTEST:OK:WRITE_LF\n", 0},
    {0   , 30, "UTEST:WRITE_LF\n", "UTEST:OK:WRITE_LF\n", 0},
  };
  int len_parameters = sizeof( parameters )/sizeof( parameters[0] );

  serial_iomode_t iomodes[2] = {SERIAL_STDIO, SERIAL_POSIX}; 

  for( int i = 0, j = 0 ; ; ++j ){
    i = !i ? 1 : 0;
    serial_set_iomode( iomodes[i], &serial );  

    int err = test( 
      &serial, 
      parameters[0].timeout_ms, parameters[0].timeout_ds, 
      parameters[0].send, parameters[0].received, j, parameters[0].op,
      0 
    );

    char response[NAME_MAX];
    switch( err ){
      case 0:  strcpy( response, "Failed" ); break;
      case -1: strcpy( response, "Timeout" ); break;
      default: strcpy( response, "Sucess" ); break;
    }

    printf(
      "Result %d: %s\n", 
      i, response 
    );
    printf("------------------------\n" );

    if( !strcmp( response, "Failed") ){
      printf( "Failed at %d...\n", j ); 
      exit( EXIT_FAILURE );
    }
    usleep( 10 );
  }
  
  for( int j = 0 ; j < 2 ; ++j ){
    serial_set_iomode( iomodes[j], &serial );  
    for( int i = 0 ; i < len_parameters ; ++i ){
      printf(
        "Result %d: %s\n", 
        i, 
        test( 
          &serial, 
          parameters[i].timeout_ms, parameters[i].timeout_ds, 
          parameters[i].send, parameters[i].received, i, parameters[i].op,
          1 
        ) ? "Passed" : "Failed"
      );
      printf("\n------------------------\n" );
    }
  }

  serial_close( &serial );
  return 0;   
}
