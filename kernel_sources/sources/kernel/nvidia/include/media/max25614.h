/*
 * Copyright (c) 2018, NVIDIA Corporation.  All rights reserved.
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

/**
 * @file
 * <b>MAX25614 API: For Analog Devices MAX25614 LED flash</b>
 *
 * @b Description: Defines elements used to set up and use a
 *  Analog Devices MAX25614 LED flash.
 */

#ifndef __MAX25614_H__
#define __MAX25614_H__

/**
 * \defgroup max25614 MAX25614 LED flash driver
 *
 * Controls the MAX25614 LED flash module.
 *
 * @{
 */

int max25614_write_reg(struct device *dev, u8 addr, u8 val);

int max25614_read_reg(struct device *dev, u8 addr, u8 *val);

/** @} */

#endif  /* __MAX25614_H__ */
