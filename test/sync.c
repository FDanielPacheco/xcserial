#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "xcserial.h"

#define N_PARAMS  3

int
test( serial_t * serial, int _timeout_ms, uint8_t _timeout_ds, const char * send, const char * received, int id, int op ){
  printf("\n------- [TEST %d] --------\n", id );

  serial_set_timeout( _timeout_ms, serial );
  serial_set_rule( _timeout_ds, 0, serial );

  const uint8_t output = 1;
  if( !serial_print_config( output, "", serial ) ){
    fprintf( stderr, "Error: %d, %s\n", errno, strerror(errno) );
    return 0;
  }

  printf("Press Enter to continue:\n");
  getchar( );

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
      nread = serial_readLine( (char *) buf, sizeof(buf), 0, serial );
    
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

  int test_id = 1;
  printf("Result %d: %s\n", test_id, test( &serial, 100, 0, "UTEST:WRITE\n", "UTEST:OK:WRITE", test_id, 1 ) ? "Passed" : "Failed" );
  printf("\n------------------------\n" );

  test_id = 2;
  printf("Result %d: %s\n", test_id, test( &serial, 0, 10, "UTEST:WRITE\n", "UTEST:OK:WRITE", test_id, 1 ) ? "Passed" : "Failed" );
  printf("\n------------------------\n" );

  test_id = 3;
  printf("Result %d: %s\n", test_id, test( &serial, 3000, 0, "UTEST:WRITE\n", "UTEST:OK:WRITE", test_id, 1 ) ? "Passed" : "Failed" );
  printf("\n------------------------\n" );

  test_id = 4;
  printf("Result %d: %s\n", test_id, test( &serial, 0, 3, "UTEST:WRITE_LF\n", "UTEST:OK:WRITE_LF\n", test_id, 0 ) ? "Passed" : "Failed" );
  printf("\n------------------------\n" );

  test_id = 5;
  printf("Result %d: %s\n", test_id, test( &serial, 10, 0, "UTEST:WRITE_LF\n", "UTEST:OK:WRITE_LF\n", test_id, 0 ) ? "Passed" : "Failed" );
  printf("\n------------------------\n" );

  test_id = 6;
  printf("Result %d: %s\n", test_id, test( &serial, 0, 30, "UTEST:WRITE_LF\n", "UTEST:OK:WRITE_LF\n", test_id, 0 ) ? "Passed" : "Failed" );
  printf("\n------------------------\n" );
  
  serial_close( &serial );
  return 0;   
}
