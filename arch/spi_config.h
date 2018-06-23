
/*
 * Copyright (c) 2006-2008 by Roland Riegel <feedback@roland-riegel.de>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef SPI_CONFIG_H
#define SPI_CONFIG_H

/**
 * \addtogroup arch
 *
 * @{
 */
/**
 * \addtogroup arch_spi
 *
 * @{
 */
/**
 * \file
 * SPI support configuration (license: GPLv2 or LGPLv2.1)
 */

#define spi_config_pin_mosi() DDRB |= (1 << DDB2)
#define spi_config_pin_sck() DDRB |= (1 << DDB1)
#define spi_config_pin_miso() DDRB &= ~(1 << DDB3)

/**
 * @}
 * @}
 */

#endif

