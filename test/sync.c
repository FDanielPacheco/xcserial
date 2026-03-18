#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "xcserial.h"
#include "stblog.h"

#define N_PARAMS 3

enum{ FAILED, TIMEOUT, SUCCESS };

int reset_ucontroller(serial_t *serial);

int handle_disconnect(serial_t *serial) {
        LOG_ERRNO(0, "test-sync", -ENODEV, LOG_INFO, "" );
        for ( int i=0; i<100; ++i ) {
                if ( !serial_reopen(serial) ) {
                      return SUCCESS;    
                } 
                usleep(1000000);
        }
        reset_ucontroller(serial);
        return SUCCESS;
}

int reset_ucontroller(serial_t *serial) {
        int err = 0;
        
        while( 0 != (err = serial_set_line((struct serial_line_state){SERIAL_DTR, 0}, serial)) ) {
                LOG_ERRNO(0, "test-sync", err, LOG_INFO, "" );
                if (!handle_disconnect(serial)) {
                        return FAILED;
                }
        }
        usleep(10000);

        while( 0 != (err = serial_set_line((struct serial_line_state){SERIAL_DTR, 1}, serial)) ) {
                LOG_ERRNO(0, "test-sync", err, LOG_INFO, "" );
                if (!handle_disconnect(serial)) {
                        return FAILED;
                }
        }
        usleep(2000000);
        return SUCCESS;
}

int test(serial_t *serial, int _timeout_ms, const char *send, const char *received, int id, int op, int wait) {
        printf("\n------- [TEST %d] --------\n", id);

        if (wait) {
                printf("Press Enter to continue:\n");
                getchar();
        }

        serial_set_timeout(_timeout_ms, serial);

        int err = 0;
        const char *cfg = serial_get_config(serial, &err);
        if (!cfg) {
                LOG_ERRNO(0, "test-sync", err, LOG_ERROR, "failed" );
                if (!handle_disconnect(serial)) {
                        return FAILED;
                }
        }

        ssize_t nread = 0, nwrite;
        uint8_t buf[BUFSIZ];

        nwrite = serial_writef(serial, 0, &err, "%s", send);
        if (nwrite <= 0) {
                if (err == -ENODEV) {
                        if (!handle_disconnect(serial)) {
                                return FAILED;
                        }
                } 
        }
        LOG_ERRNO(0, "test-sync", err, LOG_DEBUG, "wrote: %ld [B]", nwrite );

        if (op) {
                nread = serial_read(buf, 1, sizeof(buf) - 1, 0, serial, &err);
        }
        else {
                nread = serial_readl((char *)buf, sizeof(buf), 0, serial, &err);
        }
        if (nread <= 0) {
                if (err == -ENODEV) {
                        if (!handle_disconnect(serial)) {
                                return FAILED;
                        }
                } 
                else if (err == -ETIME) {
                        if (FAILED == reset_ucontroller(serial) ) {
                                return FAILED;
                        }
                } 
        }

        buf[nread] = '\0';
        LOG_ERRNO(0, "test-sync", 0, LOG_DEBUG, "Read: %ld [B]\nMessage: %s\n", nread, (char *)buf );
        if (!strcmp((char *)buf, received)) {
                return SUCCESS;
        }
        
        return FAILED;
}

int 
main(void) {
        const char *pathname = "/dev/ttyACM0";
        serial_t serial;

        struct serial_udev_field fields[] = {
                {.label = "ID_MODEL"},
                {.label = "ID_USB_VENDOR"},
                {.label = "ID_SERIAL_SHORT"}
        };
        struct serial_udev udev_cfg = {
                .fields = fields,
                .size = N_PARAMS
        };

        if (serial_open(&serial, pathname) < 0) {
                goto cleanup;
        }
        serial_set_baudrate(SERIAL_B19200, &serial);
        serial_set_udev(&udev_cfg, &serial);
        reset_ucontroller(&serial);
        
        int err_code = 0;
        printf("--Hardware Info--\n%s", serial_get_udev(&err_code, &serial));

        struct test_params {
                int         timeout_ms;
                const char *send;
                const char *expect;
                int         op;
        };

        struct test_params parameters[] = {
                {100,  "UTEST:WRITE\n",    "UTEST:OK:WRITE",    1},
                {1000, "UTEST:WRITE\n",    "UTEST:OK:WRITE",    1},
                {3000, "UTEST:WRITE\n",    "UTEST:OK:WRITE",    1},
                {300,  "UTEST:WRITE_LF\n", "UTEST:OK:WRITE_LF\n", 0},
                {15,   "UTEST:WRITE_LF\n", "UTEST:OK:WRITE_LF\n", 0},
                {3000, "UTEST:WRITE_LF\n", "UTEST:OK:WRITE_LF\n", 0},
        };

        const int ntests = 100;
        for (int i = 0; i < ntests; ++i) {
                int res = test(
                        &serial, 
                        parameters[0].timeout_ms, 
                        parameters[0].send, 
                        parameters[0].expect, 
                        i, 
                        parameters[0].op, 
                        0
                );

                const char *status;
                if (res == SUCCESS)      { status = "Success"; } 
                else if (res == TIMEOUT) { status = "Timeout"; }
                else                     { status = "Failed"; }
                printf("Result iteration %d: %s\n------------------------\n", i, status);
                usleep(10);
        }

        const int len_parameters = sizeof(parameters)/sizeof(parameters[0]);
        for (int i = 0; i < len_parameters; ++i) {
                int res = test(
                        &serial, 
                        parameters[i].timeout_ms, 
                        parameters[i].send, 
                        parameters[i].expect, 
                        i, 
                        parameters[i].op, 
                        1
                );
                const char *status;
                if (res == SUCCESS)      { status = "Success"; } 
                else if (res == TIMEOUT) { status = "Timeout"; }
                else                     { status = "Failed"; }
                printf("Result iteration %d: %s\n------------------------\n", i, status);
        }

cleanup:
        serial_close(&serial);
        return 0;
}
