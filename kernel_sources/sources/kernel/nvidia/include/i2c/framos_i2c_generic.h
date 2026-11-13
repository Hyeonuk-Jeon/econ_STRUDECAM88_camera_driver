/*
 * framos_i2c_generic.h - i2c generic driver
 *
 * Copyright (c) 2022. FRAMOS.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __framos_i2c_get_client__
#define __framos_i2c_get_client__

struct i2c_client *get_broadcast_client(struct i2c_client *image_sensor);

#endif /* __framos_i2c_get_client__ */
