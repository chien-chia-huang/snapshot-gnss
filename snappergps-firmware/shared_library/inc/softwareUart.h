/****************************************************************************
 * softwareUart.h
 * SnapperGPS
 * July 2026
 *****************************************************************************/

#ifndef __SOFTWAREUART_H
#define __SOFTWAREUART_H

#include <stdint.h>

void SoftwareUart_init(void);

void SoftwareUart_enable(void);

void SoftwareUart_disable(void);

void SoftwareUart_transmit(const uint8_t *data, uint32_t length);

#endif /* __SOFTWAREUART_H */
