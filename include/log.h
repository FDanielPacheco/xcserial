/**
 * @file      log.h
 * 
 * @version   0.1.0
 *
 * @date      16-03-2026
 *  
 * @author    Fábio D. Pacheco, 
 * @email     fabio.d.pacheco@inesctec.pt or pacheco.castro.fabio@gmail.com
 *
 *  @note
 *  log - simple logging system 
 *  Copyright (C) 2026 Fábio D. Pacheco 
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 *  USA
 * 
 */

#ifndef LOG_H
#define LOG_H

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

enum log_level {
        LOG_DEBUG = 0,
        LOG_INFO,
        LOG_WARN,
        LOG_ERROR,
        LOG_FATAL
};

struct log_report {
        const char     *label;     
        const char     *filename;
        int             line;
        int             err_code;
        enum log_level  prio;    
};

struct log_logger {
        void (*handler)(struct log_report, const char *);
        int enable;
};

static inline void
log_default(
        struct log_report  report,
        const char        *msg
);

static inline void 
log_emit(
        struct log_logger *lg,
        struct log_report  report,
        const char        *fmt,
        ...
);

static inline void
log_default(
        struct log_report  report,
        const char        *msg
){
        const char *log_level[] = {"DEBUG", "INFO ", "WARN ", "ERROR", "FATAL"};
        const char *log_color[] = {
            "\x1b[90m", /* DEBUG  - gray */
            "\x1b[32m", /* INFO   - green */
            "\x1b[33m", /* WARN   - yellow */
            "\x1b[31m", /* ERROR  - red */
            "\x1b[35;1m"/* FATAL  - bold magenta */
        };
        const char *reset_color = "\x1b[0m";

        if ( report.prio < 0 || report.prio > LOG_FATAL ) {
                report.prio = LOG_DEBUG;
        }
        char *errmsg = !report.err_code ? "" : strerror(-report.err_code);
        fprintf(
                stderr,
                "%s[%s][%s] (%s:%d) %s:%s %s\n",
                log_color[report.prio],
                report.label,
                log_level[report.prio],
                report.filename,
                report.line,
                errmsg,
                msg,
                reset_color
        );
}

/** 
@brief Call either external logging system or internal logging 
 */
static inline void
log_emit(
        struct log_logger *lg,
        struct log_report  report,
        const char        *fmt,
        ...
){
        char buf[256];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);

        if (lg && lg->enable) {
                lg->handler(report, buf);
        }
        else {
                log_default(report, buf);
        }
}

#ifndef DISABLE_LOGGING
        #define LOG_ERRNO(logger, label, err, prio, fmt, ...)                      \
                log_emit(                                                          \
                        (logger),                                                  \
                        (struct log_report){label, __FILE__, __LINE__, err, prio}, \
                        fmt, ##__VA_ARGS__                                         \
                )
#else
        #define LOG_ERRNO(logger, label, err, prio, fmt, ...) \
                ((void)0)
#endif

#endif
