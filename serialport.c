/*
 * This file is part of the libserialport project.
 *
 * Copyright (C) 2010-2012 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2010-2015 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2013-2015 Martin Ling <martin-libserialport@earth.li>
 * Copyright (C) 2013 Matthias Heidbrink <m-sigrok@heidbrink.biz>
 * Copyright (C) 2014 Aurelien Jacobs <aurel@gnuage.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "libserialport_internal.h"
#include <termios.h>
#include <bits/fcntl.h>
#include <fcntl.h>
#include <malloc.h>
#include <string.h>

static const struct std_baudrate std_baudrates[] = {
        BAUD(50), BAUD(75), BAUD(110), BAUD(134), BAUD(150), BAUD(200),
        BAUD(300), BAUD(600), BAUD(1200), BAUD(1800), BAUD(2400), BAUD(4800),
        BAUD(9600), BAUD(19200), BAUD(38400), BAUD(57600), BAUD(115200),
        BAUD(230400),
};

#define NUM_STD_BAUDRATES ARRAY_SIZE(std_baudrates)

void (*sp_debug_handler)(const char *format, ...) = sp_default_debug_handler;

SP_API char *sp_get_port_name(const struct sp_port *port)
{
    TRACE("%p", port);

    if (!port)
        return NULL;

    RETURN_STRING(port->name);
}

SP_API char *sp_get_port_description(const struct sp_port *port)
{
    TRACE("%p", port);

    if (!port || !port->description)
        return NULL;

    RETURN_STRING(port->description);
}

SP_API enum sp_transport sp_get_port_transport(const struct sp_port *port)
{
    TRACE("%p", port);

    RETURN_INT(port ? port->transport : SP_TRANSPORT_NATIVE);
}

SP_API enum sp_return sp_get_port_usb_bus_address(const struct sp_port *port,
                                                  int *usb_bus,int *usb_address)
{
    TRACE("%p", port);

    if (!port)
        RETURN_ERROR(SP_ERR_ARG, "Null port");
    if (port->transport != SP_TRANSPORT_USB)
        RETURN_ERROR(SP_ERR_ARG, "Port does not use USB transport");
    if (port->usb_bus < 0 || port->usb_address < 0)
        RETURN_ERROR(SP_ERR_SUPP, "Bus and address values are not available");

    if (usb_bus)
        *usb_bus = port->usb_bus;
    if (usb_address)
        *usb_address = port->usb_address;

    RETURN_OK();
}

SP_API enum sp_return sp_get_port_usb_vid_pid(const struct sp_port *port,
                                              int *usb_vid, int *usb_pid)
{
    TRACE("%p", port);

    if (!port)
        RETURN_ERROR(SP_ERR_ARG, "Null port");
    if (port->transport != SP_TRANSPORT_USB)
        RETURN_ERROR(SP_ERR_ARG, "Port does not use USB transport");
    if (port->usb_vid < 0 || port->usb_pid < 0)
        RETURN_ERROR(SP_ERR_SUPP, "VID:PID values are not available");

    if (usb_vid)
        *usb_vid = port->usb_vid;
    if (usb_pid)
        *usb_pid = port->usb_pid;

    RETURN_OK();
}

SP_API char *sp_get_port_usb_manufacturer(const struct sp_port *port)
{
    TRACE("%p", port);

    if (!port || port->transport != SP_TRANSPORT_USB || !port->usb_manufacturer)
        return NULL;

    RETURN_STRING(port->usb_manufacturer);
}

SP_API char *sp_get_port_usb_product(const struct sp_port *port)
{
    TRACE("%p", port);

    if (!port || port->transport != SP_TRANSPORT_USB || !port->usb_product)
        return NULL;

    RETURN_STRING(port->usb_product);
}

SP_API char *sp_get_port_usb_serial(const struct sp_port *port)
{
    TRACE("%p", port);

    if (!port || port->transport != SP_TRANSPORT_USB || !port->usb_serial)
        return NULL;

    RETURN_STRING(port->usb_serial);
}

SP_API char *sp_get_port_bluetooth_address(const struct sp_port *port)
{
    TRACE("%p", port);

    if (!port || port->transport != SP_TRANSPORT_BLUETOOTH
        || !port->bluetooth_address)
        return NULL;

    RETURN_STRING(port->bluetooth_address);
}

SP_API enum sp_return sp_get_port_handle(const struct sp_port *port,
                                         void *result_ptr)
{
    TRACE("%p, %p", port, result_ptr);

    if (!port)
        RETURN_ERROR(SP_ERR_ARG, "Null port");
    if (!result_ptr)
        RETURN_ERROR(SP_ERR_ARG, "Null result pointer");

#ifdef _WIN32
    HANDLE *handle_ptr = result_ptr;
	*handle_ptr = port->hdl;
#else
    int *fd_ptr = result_ptr;
    *fd_ptr = port->fd;
#endif

    RETURN_OK();
}

SP_API enum sp_return sp_copy_port(const struct sp_port *port,
                                   struct sp_port **copy_ptr)
{
    TRACE("%p, %p", port, copy_ptr);

    if (!copy_ptr)
        RETURN_ERROR(SP_ERR_ARG, "Null result pointer");

    *copy_ptr = NULL;

    if (!port)
        RETURN_ERROR(SP_ERR_ARG, "Null port");

    if (!port->name)
        RETURN_ERROR(SP_ERR_ARG, "Null port name");

    DEBUG("Copying port structure");

    RETURN_INT(sp_get_port_by_name(port->name, copy_ptr));
}

SP_API void sp_free_port(struct sp_port *port)
{
    TRACE("%p", port);

    if (!port) {
        DEBUG("Null port");
        RETURN();
    }

    DEBUG("Freeing port structure");

    if (port->name)
        free(port->name);
    if (port->description)
        free(port->description);
    if (port->usb_manufacturer)
        free(port->usb_manufacturer);
    if (port->usb_product)
        free(port->usb_product);
    if (port->usb_serial)
        free(port->usb_serial);
    if (port->bluetooth_address)
        free(port->bluetooth_address);

    free(port);

    RETURN();
}

SP_PRIV struct sp_port **list_append(struct sp_port **list,
                                     const char *portname)
{
    RETURN_ERROR(NULL, "list_append not implemented");
}

SP_API enum sp_return sp_list_ports(struct sp_port ***list_ptr)
{
    TRACE("%p", list_ptr);

    if (!list_ptr)
        RETURN_ERROR(SP_ERR_ARG, "Null result pointer");

    *list_ptr = NULL;

    RETURN_ERROR(SP_ERR_SUPP, "sp_list_ports not implemented");
}

SP_API void sp_free_port_list(struct sp_port **list)
{
    unsigned int i;

    TRACE("%p", list);

    if (!list) {
        DEBUG("Null list");
        RETURN();
    }

    DEBUG("Freeing port list");

    for (i = 0; list[i]; i++)
        sp_free_port(list[i]);
    free(list);

    RETURN();
}

#define CHECK_PORT() do { \
	if (!port) \
		RETURN_ERROR(SP_ERR_ARG, "Null port"); \
	if (!port->name) \
		RETURN_ERROR(SP_ERR_ARG, "Null port name"); \
} while (0)
#ifdef _WIN32
#define CHECK_PORT_HANDLE() do { \
	if (port->hdl == INVALID_HANDLE_VALUE) \
		RETURN_ERROR(SP_ERR_ARG, "Port not open"); \
} while (0)
#else
#define CHECK_PORT_HANDLE() do { \
	if (port->fd < 0) \
		RETURN_ERROR(SP_ERR_ARG, "Port not open"); \
} while (0)
#endif
#define CHECK_OPEN_PORT() do { \
	CHECK_PORT(); \
	CHECK_PORT_HANDLE(); \
} while (0)

#ifdef WIN32
/** To be called after port receive buffer is emptied. */
static enum sp_return restart_wait(struct sp_port *port)
{
	DWORD wait_result;

	if (port->wait_running) {
		/* Check status of running wait operation. */
		if (GetOverlappedResult(port->hdl, &port->wait_ovl,
				&wait_result, FALSE)) {
			DEBUG("Previous wait completed");
			port->wait_running = FALSE;
		} else if (GetLastError() == ERROR_IO_INCOMPLETE) {
			DEBUG("Previous wait still running");
			RETURN_OK();
		} else {
			RETURN_FAIL("GetOverlappedResult() failed");
		}
	}

	if (!port->wait_running) {
		/* Start new wait operation. */
		if (WaitCommEvent(port->hdl, &port->events,
				&port->wait_ovl)) {
			DEBUG("New wait returned, events already pending");
		} else if (GetLastError() == ERROR_IO_PENDING) {
			DEBUG("New wait running in background");
			port->wait_running = TRUE;
		} else {
			RETURN_FAIL("WaitCommEvent() failed");
		}
	}

	RETURN_OK();
}
#endif

SP_API enum sp_return sp_open(struct sp_port *port, enum sp_mode flags)
{
    // TODO: Implement sp_open
    RETURN_ERROR(SP_ERR_SUPP, "sp_open not implemented");
}

SP_API enum sp_return sp_close(struct sp_port *port)
{
    TRACE("%p", port);

    CHECK_OPEN_PORT();

    DEBUG_FMT("Closing port %s", port->name);

    // TODO: Implement sp_close
    RETURN_ERROR(SP_ERR_SUPP, "sp_close not implemented");
}

SP_API enum sp_return sp_flush(struct sp_port *port, enum sp_buffer buffers)
{
    TRACE("%p, 0x%x", port, buffers);

    CHECK_OPEN_PORT();

    DEBUG_FMT("Flushing port %s", port->name);

    // TODO: Implement sp_flush
}

SP_API enum sp_return sp_drain(struct sp_port *port)
{
    TRACE("%p", port);

    CHECK_OPEN_PORT();

    DEBUG_FMT("Draining port %s", port->name);

    // TODO: Implement sp_drain
    RETURN_ERROR(SP_ERR_SUPP, "sp_drain not implemented");
}

SP_API enum sp_return sp_blocking_write(struct sp_port *port, const void *buf,
                                        size_t count, unsigned int timeout_ms)
{
    TRACE("%p, %p, %d, %d", port, buf, count, timeout_ms);

    CHECK_OPEN_PORT();

    if (!buf)
        RETURN_ERROR(SP_ERR_ARG, "Null buffer");

    if (timeout_ms)
        DEBUG_FMT("Writing %d bytes to port %s, timeout %d ms",
                  count, port->name, timeout_ms);
    else
        DEBUG_FMT("Writing %d bytes to port %s, no timeout",
                  count, port->name);

    if (count == 0)
        RETURN_INT(0);

    RETURN_ERROR(SP_ERR_SUPP, "sp_blocking_write not implemented");
}

SP_API enum sp_return sp_nonblocking_write(struct sp_port *port,
                                           const void *buf, size_t count)
{
    TRACE("%p, %p, %d", port, buf, count);

    CHECK_OPEN_PORT();

    if (!buf)
        RETURN_ERROR(SP_ERR_ARG, "Null buffer");

    DEBUG_FMT("Writing up to %d bytes to port %s", count, port->name);

    // TODO: Implement nonblocking write
    RETURN_ERROR(SP_ERR_SUPP, "sp_nonblocking_write not implemented");
}

SP_API enum sp_return sp_blocking_read(struct sp_port *port, void *buf,
                                       size_t count, unsigned int timeout_ms)
{
    TRACE("%p, %p, %d, %d", port, buf, count, timeout_ms);

    CHECK_OPEN_PORT();

    if (!buf)
        RETURN_ERROR(SP_ERR_ARG, "Null buffer");

    if (timeout_ms)
        DEBUG_FMT("Reading %d bytes from port %s, timeout %d ms",
                  count, port->name, timeout_ms);
    else
        DEBUG_FMT("Reading %d bytes from port %s, no timeout",
                  count, port->name);

    if (count == 0)
        RETURN_INT(0);

    RETURN_ERROR(SP_ERR_SUPP, "sp_blocking_read not implemented");
}

SP_API enum sp_return sp_blocking_read_next(struct sp_port *port, void *buf,
                                            size_t count, unsigned int timeout_ms)
{
    TRACE("%p, %p, %d, %d", port, buf, count, timeout_ms);

    CHECK_OPEN_PORT();

    if (!buf)
        RETURN_ERROR(SP_ERR_ARG, "Null buffer");

    if (count == 0)
        RETURN_ERROR(SP_ERR_ARG, "Zero count");

    if (timeout_ms)
        DEBUG_FMT("Reading next max %d bytes from port %s, timeout %d ms",
                  count, port->name, timeout_ms);
    else
        DEBUG_FMT("Reading next max %d bytes from port %s, no timeout",
                  count, port->name);


    RETURN_ERROR(SP_ERR_SUPP, "Not implemented");
}

SP_API enum sp_return sp_nonblocking_read(struct sp_port *port, void *buf,
                                          size_t count)
{
    TRACE("%p, %p, %d", port, buf, count);

    CHECK_OPEN_PORT();

    if (!buf)
        RETURN_ERROR(SP_ERR_ARG, "Null buffer");

    DEBUG_FMT("Reading up to %d bytes from port %s", count, port->name);

    RETURN_ERROR(SP_ERR_SUPP, "sp_nonblocking_read not implemented");
}

SP_API enum sp_return sp_input_waiting(struct sp_port *port)
{
    TRACE("%p", port);

    CHECK_OPEN_PORT();

    DEBUG_FMT("Checking input bytes waiting on port %s", port->name);

    RETURN_ERROR(SP_ERR_SUPP, "sp_input_waiting not implemented");
}

SP_API enum sp_return sp_output_waiting(struct sp_port *port)
{
    TRACE("%p", port);

    CHECK_OPEN_PORT();

    DEBUG_FMT("Checking output bytes waiting on port %s", port->name);

    RETURN_ERROR(SP_ERR_SUPP, "sp_output_waiting not implemented");
}

SP_API enum sp_return sp_new_event_set(struct sp_event_set **result_ptr)
{
    struct sp_event_set *result;

    TRACE("%p", result_ptr);

    if (!result_ptr)
        RETURN_ERROR(SP_ERR_ARG, "Null result");

    *result_ptr = NULL;

    if (!(result = malloc(sizeof(struct sp_event_set))))
        RETURN_ERROR(SP_ERR_MEM, "sp_event_set malloc() failed");

    memset(result, 0, sizeof(struct sp_event_set));

    *result_ptr = result;

    RETURN_OK();
}

static enum sp_return add_handle(struct sp_event_set *event_set,
                                 event_handle handle, enum sp_event mask)
{
    void *new_handles;
    enum sp_event *new_masks;

    TRACE("%p, %d, %d", event_set, handle, mask);

    if (!(new_handles = realloc(event_set->handles,
                                sizeof(event_handle) * (event_set->count + 1))))
        RETURN_ERROR(SP_ERR_MEM, "Handle array realloc() failed");

    event_set->handles = new_handles;

    if (!(new_masks = realloc(event_set->masks,
                              sizeof(enum sp_event) * (event_set->count + 1))))
        RETURN_ERROR(SP_ERR_MEM, "Mask array realloc() failed");

    event_set->masks = new_masks;

    ((event_handle *) event_set->handles)[event_set->count] = handle;
    event_set->masks[event_set->count] = mask;

    event_set->count++;

    RETURN_OK();
}

SP_API enum sp_return sp_add_port_events(struct sp_event_set *event_set,
                                         const struct sp_port *port, enum sp_event mask)
{
    TRACE("%p, %p, %d", event_set, port, mask);

    if (!event_set)
        RETURN_ERROR(SP_ERR_ARG, "Null event set");

    if (!port)
        RETURN_ERROR(SP_ERR_ARG, "Null port");

    if (mask > (SP_EVENT_RX_READY | SP_EVENT_TX_READY | SP_EVENT_ERROR))
        RETURN_ERROR(SP_ERR_ARG, "Invalid event mask");

    if (!mask)
        RETURN_OK();

#ifdef _WIN32
    enum sp_event handle_mask;
	if ((handle_mask = mask & SP_EVENT_TX_READY))
		TRY(add_handle(event_set, port->write_ovl.hEvent, handle_mask));
	if ((handle_mask = mask & (SP_EVENT_RX_READY | SP_EVENT_ERROR)))
		TRY(add_handle(event_set, port->wait_ovl.hEvent, handle_mask));
#else
    TRY(add_handle(event_set, port->fd, mask));
#endif

    RETURN_OK();
}

SP_API void sp_free_event_set(struct sp_event_set *event_set)
{
    TRACE("%p", event_set);

    if (!event_set) {
        DEBUG("Null event set");
        RETURN();
    }

    DEBUG("Freeing event set");

    if (event_set->handles)
        free(event_set->handles);
    if (event_set->masks)
        free(event_set->masks);

    free(event_set);

    RETURN();
}

SP_API enum sp_return sp_wait(struct sp_event_set *event_set,
                              unsigned int timeout_ms)
{
    TRACE("%p, %d", event_set, timeout_ms);

    if (!event_set)
        RETURN_ERROR(SP_ERR_ARG, "Null event set");

    // function is not implemented
    RETURN_ERROR(SP_ERR_FAIL, "SP_WAIT is not implemented");
}



static enum sp_return get_config(struct sp_port *port, struct port_data *data,
                                 struct sp_port_config *config)
{
    unsigned int i;

    TRACE("%p, %p, %p", port, data, config);

    DEBUG_FMT("Getting configuration for port %s", port->name);


    // TODO: Implement get_config
    RETURN_ERROR(SP_ERR_SUPP, "get_config not implemented");
}

static enum sp_return set_config(struct sp_port *port, struct port_data *data,
                                 const struct sp_port_config *config)
{
    // TODO: Implement set_config
    RETURN_ERROR(SP_ERR_SUPP, "set_config not implemented");
}

SP_API enum sp_return sp_new_config(struct sp_port_config **config_ptr)
{
    struct sp_port_config *config;

    TRACE("%p", config_ptr);

    if (!config_ptr)
        RETURN_ERROR(SP_ERR_ARG, "Null result pointer");

    *config_ptr = NULL;

    if (!(config = malloc(sizeof(struct sp_port_config))))
        RETURN_ERROR(SP_ERR_MEM, "Config malloc failed");

    config->baudrate = -1;
    config->bits = -1;
    config->parity = -1;
    config->stopbits = -1;
    config->rts = -1;
    config->cts = -1;
    config->dtr = -1;
    config->dsr = -1;

    *config_ptr = config;

    RETURN_OK();
}

SP_API void sp_free_config(struct sp_port_config *config)
{
    TRACE("%p", config);

    if (!config)
        DEBUG("Null config");
    else
        free(config);

    RETURN();
}

SP_API enum sp_return sp_get_config(struct sp_port *port,
                                    struct sp_port_config *config)
{
    //struct port_data data;

    TRACE("%p, %p", port, config);

    CHECK_OPEN_PORT();

    // TODO: Implement sp_get_config
    RETURN_ERROR(SP_ERR_SUPP, "sp_get_config not implemented");
}

SP_API enum sp_return sp_set_config(struct sp_port *port,
                                    const struct sp_port_config *config)
{
    RETURN_ERROR(SP_ERR_SUPP, "sp_set_config not implemented");
}

SP_API enum sp_return sp_set_config_flowcontrol(struct sp_port_config *config,
                                                enum sp_flowcontrol flowcontrol)
{
    if (!config)
        RETURN_ERROR(SP_ERR_ARG, "Null configuration");

    if (flowcontrol > SP_FLOWCONTROL_DTRDSR)
        RETURN_ERROR(SP_ERR_ARG, "Invalid flow control setting");

    if (flowcontrol == SP_FLOWCONTROL_XONXOFF)
        config->xon_xoff = SP_XONXOFF_INOUT;
    else
        config->xon_xoff = SP_XONXOFF_DISABLED;

    if (flowcontrol == SP_FLOWCONTROL_RTSCTS) {
        config->rts = SP_RTS_FLOW_CONTROL;
        config->cts = SP_CTS_FLOW_CONTROL;
    } else {
        if (config->rts == SP_RTS_FLOW_CONTROL)
            config->rts = SP_RTS_ON;
        config->cts = SP_CTS_IGNORE;
    }

    if (flowcontrol == SP_FLOWCONTROL_DTRDSR) {
        config->dtr = SP_DTR_FLOW_CONTROL;
        config->dsr = SP_DSR_FLOW_CONTROL;
    } else {
        if (config->dtr == SP_DTR_FLOW_CONTROL)
            config->dtr = SP_DTR_ON;
        config->dsr = SP_DSR_IGNORE;
    }

    RETURN_OK();
}

SP_API enum sp_return sp_set_flowcontrol(struct sp_port *port,
                                         enum sp_flowcontrol flowcontrol)
{
    RETURN_ERROR(SP_ERR_SUPP, "sp_set_flowcontrol not implemented");
}

SP_API enum sp_return sp_get_signals(struct sp_port *port,
                                     enum sp_signal *signals)
{
    RETURN_ERROR(SP_ERR_SUPP, "sp_get_signals not implemented");
}

SP_API enum sp_return sp_start_break(struct sp_port *port)
{
    TRACE("%p", port);

    // throw an error that this function is not supported
    RETURN_ERROR(SP_ERR_SUPP, "Start break not supported");
}

SP_API enum sp_return sp_end_break(struct sp_port *port)
{
    TRACE("%p", port);

    CHECK_OPEN_PORT();

    RETURN_ERROR(SP_ERR_SUPP, "End break not supported");
}

SP_API int sp_last_error_code(void)
{
    // this function is not supported
    return SP_ERR_SUPP;
}

SP_API char *sp_last_error_message(void)
{
    TRACE_VOID();

    // this function is not supported
    return NULL;
}

SP_API void sp_free_error_message(char *message)
{
    TRACE("%s", message);

    // this function is not supported
}

SP_API void sp_set_debug_handler(void (*handler)(const char *format, ...))
{
    TRACE("%p", handler);

    // this function is not supported
}

SP_API void sp_default_debug_handler(const char *format, ...)
{
    // this function is not supported
}

SP_API int sp_get_major_package_version(void)
{
    return SP_PACKAGE_VERSION_MAJOR;
}

SP_API int sp_get_minor_package_version(void)
{
    return SP_PACKAGE_VERSION_MINOR;
}

SP_API int sp_get_micro_package_version(void)
{
    return SP_PACKAGE_VERSION_MICRO;
}

SP_API const char *sp_get_package_version_string(void)
{
    return SP_PACKAGE_VERSION_STRING;
}

SP_API int sp_get_current_lib_version(void)
{
    return SP_LIB_VERSION_CURRENT;
}

SP_API int sp_get_revision_lib_version(void)
{
    return SP_LIB_VERSION_REVISION;
}

SP_API int sp_get_age_lib_version(void)
{
    return SP_LIB_VERSION_AGE;
}

SP_API const char *sp_get_lib_version_string(void)
{
    return SP_LIB_VERSION_STRING;
}

/** @} */
