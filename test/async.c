#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "xcserial.h"

#define N_PARAMS  3

serial_t serial;
char buf[BUFSIZ];
size_t buflen = 0;
uint8_t flexit = 0;
uint8_t flfail = 0;
uint8_t first = 1;

void 
handler_read( uint8_t * data, size_t len ){
  if( buflen + len < BUFSIZ ){
    memcpy( &buf[buflen], data, len );
    buflen += len;
  }
} 

int8_t 
handler_disconnect( void ){
  flfail = 1;
  int8_t r = serial_reopen( &serial, 1000 );
  if( -1 == r )
    flexit = 1;
  else{
    serial_set_line_state( SERIAL_DTR, 0, &serial );
    usleep( 1e3 );
    serial_set_line_state( SERIAL_DTR, 1, &serial );
    usleep( 2e6 );
    
    flfail = 0;
    first = 1;
  }
  return r;
} 


int 
main( void ){
  const char  *    pathname = "/dev/ttyACM0";
  const uint8_t    readonly = 0;
  const baudrate_t baudrate = B19200; 

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
     
  serial_set_timeout( 100, &serial );
  if( -1 == serial_async_set_callback( handler_read, handler_disconnect, &serial ) ){
    serial_close( &serial );
    return EXIT_FAILURE;
  }

  serial_set_line_state( SERIAL_DTR, 0, &serial );
  usleep( 1e3 );
  serial_set_line_state( SERIAL_DTR, 1, &serial );
  usleep( 2e6 );

  const char field[ ] = "ID_MODEL_FROM_DATABASE";
  printf("%s: %s\n", field, serial_get_udev_param_value( field, strlen(field), &serial ) );

  for( int i = 0, j = 0 ; ; ){
    usleep( 1000 );
    
    while( 0 != flfail ){
      if( flexit ){
        serial_close( &serial );
        return EXIT_FAILURE;
      }
      usleep( 1000 );
    }

    if( first ){
      printf("Message sent with %ld bytes...\n", serial_writef( &serial, "UTEST:WRITE_LF\n" ) );
      first = 0;
    }

    if( !strncmp( "UTEST:OK:WRITE_LF\n", buf, buflen ) && (0 != buflen) ){
      printf("\n------- [TEST %d] --------\n", j );
      printf("Message (%zu):%s\n", buflen, buf );
      printf("Status: Sucess\n");
      memset( buf, 0, sizeof(buf) );
      buflen = 0;

      if( !i ){
        serial_set_iomode( SERIAL_STDIO, &serial );  
        printf("Test with: SERIAL_STDIO\n");
        i--;
      }
      else{
        serial_set_iomode( SERIAL_POSIX, &serial );  
        printf("Test with: SERIAL_POSIX\n");
        i++;
      }

      j++;
      printf("Writting other message...\n");
      printf("Message sent with %ld bytes...\n", serial_writef( &serial, "UTEST:WRITE_LF\n" ) );
        // printf("Failed to send the message...\n");

      printf("------------------------\n" );
    }
  }

}
