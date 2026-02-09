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

void
reset_ucontroller(
  void
) {
  serial_set_line_state( SERIAL_DTR, 0, &serial );
  usleep( 1e3 );
  serial_set_line_state( SERIAL_DTR, 1, &serial );
  usleep( 2e6 );
}

void 
handler_read( uint8_t * data, size_t len ){
  if( buflen + len < BUFSIZ ){
    memcpy( &buf[buflen], data, len );
    buflen += len;
  }
} 

int8_t 
handler_disconnect( void ){
  printf("Device got disconnected (%d)%s...\n", serial.fd, serial.pathname );
  flfail = 1;
  int8_t r = serial_reopen( &serial, 1000 );
  if( -1 == r )
    flexit = 1;
  else{
    reset_ucontroller( );    
    flfail = 0;
  }
  return r;
} 


int 
main( void ){
  const char  *    pathname = "/dev/ttyACM0";
  const baudrate_t baudrate = B19200; 

  const char params[ N_PARAMS ][NAME_MAX] = {
    "ID_MODEL",
    "ID_USB_VENDOR",
    "ID_SERIAL_SHORT"
  };

  if( -1 == serial_open( &serial, pathname, NULL ) )
    return EXIT_FAILURE;

  if( -1 == serial_set_udev_param_list( params, N_PARAMS, NAME_MAX, &serial ) )
    return EXIT_FAILURE;

  serial_set_baudrate( baudrate, &serial );

  if( -1 == serial_async_set_callback( handler_read, handler_disconnect, &serial ) ){
    serial_close( &serial );
    return EXIT_FAILURE;
  }

  reset_ucontroller( );    

  for( int j  = 0 ; j < 500 ; ++j ){
    usleep( 5e5 );
    
    size_t len = serial_writef( &serial, "UTEST:WRITE_LF\n" );
    if( !len && ((errno == ENODEV) || (errno == EIO))  )
      printf("Failed to write...\n");
    else{
      printf("Writting other message...\n");
      printf("Message sent with %ld bytes...\n", len );
    }

    if( !strncmp( "UTEST:OK:WRITE_LF\n", buf, 18 ) && (0 != buflen) ){
      printf("\n------- [TEST %d] --------\n", j );
      printf("Message (%zu):%s\n", buflen, buf );
      printf("Status: Sucess\n");

      memmove( buf, buf, buflen );
      memset( buf+buflen-18, 0, sizeof(buf) );
      buflen -= 18;

      printf("------------------------\n" );
    }    

    serial_poll( &serial );
  }

  exit( 0 );
}
