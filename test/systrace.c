#include <xcserial.h>

int
main( void ){
  const char port[] = "/dev/ttyACM0";
  serial_t serial;

  // Open port. Last three NULLs are for port configurations, udev params and async callbacks.
  if( -1 == serial_open( &serial, port, NULL ) )
    return -1;

  // Set configuration
  serial_set_baudrate( B19200, &serial );
  serial_set_timeout( 10, &serial ); // 10ms read timeout

  // Write formatted string
  serial_writef( &serial, "UTEST:WRITE\n" );

  // Read a chunk of bytes
  // uint8_t buf[ BUFSIZ ];
  // size_t len = serial_read( buf, sizeof(buf), 0, &serial ); 
  
  // Close the serial port
  serial_close( &serial );
  return 0;  
}
