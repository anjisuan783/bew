/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/


#ifndef COMMON_CRC32_H_
#define COMMON_CRC32_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t crc32(uint32_t crc, const void *buf, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_CRC32_H_ */
