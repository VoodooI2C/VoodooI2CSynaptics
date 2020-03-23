//
//  VoodooI2CSynapticsDevice.hpp
//  VoodooI2CSynaptics
//
//  Created by Alexandre on 25/12/2017.
//  Based on code written by CoolStar and Kishor Prins
//  Copyright © 2017 Alexandre Daoud. All rights reserved.
//

#ifndef VoodooI2CSynapticsDevice_hpp
#define VoodooI2CSynapticsDevice_hpp

#include <IOKit/IOLib.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOService.h>

#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/acpi/IOACPIPlatformDevice.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOCommandGate.h>
#include <IOKit/IOTimerEventSource.h>

#include "../../../VoodooI2C/VoodooI2C/VoodooI2CDevice/VoodooI2CDeviceNub.hpp"
#include "../../../Multitouch Support/VoodooI2CMultitouchInterface.hpp"
#include "../../../Dependencies/helpers.hpp"

#include "VoodooI2CSynapticsConstants.h"

enum rmi_mode_type {
    RMI_MODE_OFF = 0,
    RMI_MODE_ATTN_REPORTS = 1,
    RMI_MODE_NO_PACKED_ATTN_REPORTS = 2,
};

struct rmi_function {
    unsigned page;            /* page of the function */
    uint16_t query_base_addr;        /* base address for queries */
    uint16_t command_base_addr;        /* base address for commands */
    uint16_t control_base_addr;        /* base address for controls */
    uint16_t data_base_addr;        /* base address for datas */
    unsigned int interrupt_base;    /* cross-function interrupt number
                                     * (uniq in the device)*/
    unsigned int interrupt_count;    /* number of interrupts */
    unsigned int report_size;    /* size of a report */
    unsigned long irq_mask;        /* mask of the interrupts
                                    * (to be applied against ATTN IRQ) */
};


struct __attribute__((__packed__)) pdt_entry {
    uint8_t query_base_addr : 8;
    uint8_t command_base_addr : 8;
    uint8_t control_base_addr : 8;
    uint8_t data_base_addr : 8;
    uint8_t interrupt_source_count : 3;
    uint8_t bits3and4 : 2;
    uint8_t function_version : 2;
    uint8_t bit7 : 1;
    uint8_t function_number : 8;
};

class VoodooI2CSynapticsDevice : public IOService
{
    typedef IOService super;
    OSDeclareDefaultStructors(VoodooI2CSynapticsDevice);
    
private:
    IOACPIPlatformDevice* acpi_device;
    VoodooI2CDeviceNub* api;
    IOCommandGate* command_gate;
    UInt16 hid_descriptor_register;
    IOInterruptEventSource* interrupt_source;
    
    OSArray* transducers;
    
    uint16_t max_x;
    uint16_t max_y;
    
    int page;
    
    unsigned long flags;
    
    struct rmi_function f01;
    struct rmi_function f11;
    struct rmi_function f12;
    struct rmi_function f30;
    
    unsigned int max_fingers;
    unsigned int x_size_mm;
    unsigned int y_size_mm;
    bool read_f11_ctrl_regs;
    uint8_t f11_ctrl_regs[RMI_F11_CTRL_REG_COUNT];
    
    unsigned int gpio_led_count;
    unsigned int button_count;
    unsigned long button_mask;
    unsigned long button_state_mask;
    
    unsigned long device_flags;
    unsigned long firmware_id;
    
    uint8_t f01_ctrl0;
    uint8_t interrupt_enable_mask;
    bool restore_interrupt_mask;
    
    VoodooI2CMultitouchInterface* mt_interface;
    
protected:
    bool awake;
    const char* name;
    IOWorkLoop* work_loop;
    bool reading;
    
public:
    void stop(IOService* device) override;
    
    bool start(IOService* api);
    
    bool init(OSDictionary* properties);
    
    VoodooI2CSynapticsDevice* probe(IOService* provider, SInt32* score);
    
    void interruptOccured(OSObject* owner, IOInterruptEventSource* src, int intCount);
    
    void get_input();
    
    IOReturn setPowerState(unsigned long powerState, IOService *whatDevice) override;
    
    int rmi_read_block(uint16_t addr, uint8_t *buf, const int len);
    int rmi_write_report(uint8_t *report, size_t report_size);
    int rmi_read(uint16_t addr, uint8_t *buf);
    int rmi_write_block(uint16_t addr, uint8_t *buf, const int len);
    int rmi_write(uint16_t addr, uint8_t *buf);
    void rmi_register_function(struct pdt_entry *pdt_entry, int page, unsigned interrupt_count);
    
    int rmi_scan_pdt();
    int rmi_populate_f01();
    int rmi_populate_f11();
    int rmi_populate_f12();
    int rmi_populate_f30();
    int rmi_populate();
    
    int rmi_set_mode(uint8_t mode);
    
    int rmi_set_page(uint8_t _page);
    
    void rmi_f11_process_touch(OSArray* transducers, int transducer_id, AbsoluteTime timestamp, uint8_t finger_state, uint8_t *touch_data);
    int rmi_f11_input(OSArray* transducers, AbsoluteTime timestamp, uint8_t *rmiInput);
    int rmi_f30_input(OSArray* transducers, AbsoluteTime timestamp, uint8_t irq, uint8_t *rmiInput, int size);
    void TrackpadRawInput(uint8_t report[40]);
    
    bool publish_multitouch_interface();
    void unpublish_multitouch_interface();
    
    void releaseResources();
};


#endif /* VoodooI2CSynapticsDevice_hpp */
