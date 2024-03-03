/*
 * This file is part of the libserialport project.
 *
 * Copyright (C) 2019 Martin Ling <martin-libserialport@earth.li>
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

SP_PRIV void time_get(struct time *time)
{

}

SP_PRIV void time_set_ms(struct time *time, unsigned int ms)
{
	time->tv.tv_sec = ms / 1000;
	time->tv.tv_usec = (ms % 1000) * 1000;
}

SP_PRIV void time_add(const struct time *a,
		const struct time *b, struct time *result)
{

}

SP_PRIV void time_sub(const struct time *a,
		const struct time *b, struct time *result)
{

}

SP_PRIV bool time_greater(const struct time *a, const struct time *b)
{

}

SP_PRIV void time_as_timeval(const struct time *time, struct timeval *tv)
{
	*tv = time->tv;
}

SP_PRIV unsigned int time_as_ms(const struct time *time)
{
	return time->tv.tv_sec * 1000 + time->tv.tv_usec / 1000;
}

SP_PRIV void timeout_start(struct timeout *timeout, unsigned int timeout_ms)
{
	timeout->ms = timeout_ms;

	/* Get time at start of operation. */
	time_get(&timeout->start);
	/* Define duration of timeout. */
	time_set_ms(&timeout->delta, timeout_ms);
	/* Calculate time at which we should give up. */
	time_add(&timeout->start, &timeout->delta, &timeout->end);
	/* Disable limit unless timeout_limit() called. */
	timeout->limit_ms = 0;
	/* First blocking call has not yet been made. */
	timeout->calls_started = false;
}

SP_PRIV void timeout_limit(struct timeout *timeout, unsigned int limit_ms)
{
	timeout->limit_ms = limit_ms;
	timeout->overflow = (timeout->ms > timeout->limit_ms);
	time_set_ms(&timeout->delta_max, timeout->limit_ms);
}

SP_PRIV bool timeout_check(struct timeout *timeout)
{
	if (!timeout->calls_started)
		return false;

	if (timeout->ms == 0)
		return false;

	time_get(&timeout->now);
	time_sub(&timeout->end, &timeout->now, &timeout->delta);
	if (timeout->limit_ms)
		if ((timeout->overflow = time_greater(&timeout->delta, &timeout->delta_max)))
			timeout->delta = timeout->delta_max;

	return time_greater(&timeout->now, &timeout->end);
}

SP_PRIV void timeout_update(struct timeout *timeout)
{
	timeout->calls_started = true;
}

#ifndef _WIN32
SP_PRIV struct timeval *timeout_timeval(struct timeout *timeout)
{
	if (timeout->ms == 0)
		return NULL;

	time_as_timeval(&timeout->delta, &timeout->delta_tv);

	return &timeout->delta_tv;
}
#endif

SP_PRIV unsigned int timeout_remaining_ms(struct timeout *timeout)
{
	if (timeout->limit_ms && timeout->overflow)
		return timeout->limit_ms;
	else
		return time_as_ms(&timeout->delta);
}
