//
//  VoodooI2CSynapticsConstants.h
//  VoodooI2CSynaptics
//
//  Created by Alexandre on 25/12/2017.
//  Copyright © 2017 Alexandre Daoud. All rights reserved.
//

#ifndef VoodooI2CSynapticsConstants_h
#define VoodooI2CSynapticsConstants_h

#define BITS_PER_LONG 64
#define GENMASK(h, l) \
(((~0UL) << (l)) & (~0UL >> (BITS_PER_LONG - 1 - (h))))
#define BIT(nr)                 (1UL << (nr))
#define BIT_MASK(nr)            (1UL << ((nr) % BITS_PER_LONG))
#define BIT_WORD(nr)            ((nr) / BITS_PER_LONG)
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

#define EIO              5      /* I/O error */
#define ENOMEM          12      /* Out of memory */

// Error codes
#define EPERM           1
#define ENOENT          2
#define ESRCH           3
#define EINTR           4
#define EIO             5
#define ENXIO           6
#define E2BIG           7
#define ENOEXEC         8
#define EBADF           9
#define ECHILD          10
#define EAGAIN          11
#define ENOMEM          12
#define EACCES          13
#define EFAULT          14
#define EBUSY           16
#define EEXIST          17
#define EXDEV           18
#define ENODEV          19
#define ENOTDIR         20
#define EISDIR          21
#define ENFILE          23
#define EMFILE          24
#define ENOTTY          25
#define EFBIG           27
#define ENOSPC          28
#define ESPIPE          29
#define EROFS           30
#define EMLINK          31
#define EPIPE           32
#define EDOM            33
#define EDEADLK         36
#define ENAMETOOLONG    38
#define ENOLCK          39
#define ENOSYS          40
#define ENOTEMPTY       41

#define __le16 UInt16
#define __le32 UInt32

#define RMI_MOUSE_REPORT_ID		0x01 /* Mouse emulation Report */
#define RMI_WRITE_REPORT_ID		0x09 /* Output Report */
#define RMI_READ_ADDR_REPORT_ID		0x0a /* Output Report */
#define RMI_READ_DATA_REPORT_ID		0x0b /* Input Report */
#define RMI_ATTN_REPORT_ID		0x0c /* Input Report */
#define RMI_SET_RMI_MODE_REPORT_ID	0x0f /* Feature Report */

/* flags */
#define RMI_READ_REQUEST_PENDING	0
#define RMI_READ_DATA_PENDING		1
#define RMI_STARTED			2

#define RMI_SLEEP_NORMAL		0x0
#define RMI_SLEEP_DEEP_SLEEP		0x1

/* device flags */
#define RMI_DEVICE			BIT(0)
#define RMI_DEVICE_HAS_PHYS_BUTTONS	BIT(1)

/*
 * retrieve the ctrl registers
 * the ctrl register has a size of 20 but a fw bug split it into 16 + 4,
 * and there is no way to know if the first 20 bytes are here or not.
 * We use only the first 12 bytes, so get only them.
 */
#define RMI_F11_CTRL_REG_COUNT		12


#define RMI_PAGE(addr) (((addr) >> 8) & 0xff)

#define RMI4_MAX_PAGE 0xff
#define RMI4_PAGE_SIZE 0x0100

#define PDT_START_SCAN_LOCATION 0x00e9
#define PDT_END_SCAN_LOCATION	0x0005
#define RMI4_END_OF_PDT(id) ((id) == 0x00 || (id) == 0xff)

#define RMI_DEVICE_F01_BASIC_QUERY_LEN	11

#endif /* VoodooI2CSynapticsConstants_h */
