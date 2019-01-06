//
//  VoodooI2CSynapticsDevice.cpp
//  VoodooI2CSynaptics
//
//  Created by Alexandre on 25/12/2017.
//  Based on code written by CoolStar and Kishor Prins
//  Copyright © 2017 Alexandre Daoud. All rights reserved.
//

#include "VoodooI2CSynapticsDevice.hpp"

#define super IOService
OSDefineMetaClassAndStructors(VoodooI2CSynapticsDevice, IOService);

void VoodooI2CSynapticsDevice::rmi_f11_process_touch(OSArray* transducers, int transducer_id, AbsoluteTime timestamp, uint8_t finger_state, uint8_t *touch_data)
{
    int x, y, wx, wy;
    int wide, major, minor;
    int z;
    
    VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer, transducers->getObject(transducer_id));
    if(!transducer) {
        IOLog("%s::%s::Failed to cast transducer f11 for id=%d\n", getName(), name, transducer_id);
        return;
    }
    
    x = (touch_data[0] << 4) | (touch_data[2] & 0x0F);
    y = (touch_data[1] << 4) | (touch_data[2] >> 4);
    wx = touch_data[3] & 0x0F;
    wy = touch_data[3] >> 4;
    wide = (wx > wy);
    major = max(wx, wy);
    minor = min(wx, wy);
    z = touch_data[4];
    
    y = max_y - y;
    
    x *= mt_interface->logical_max_x;
    x /= mt_interface->physical_max_x;
    
    y *= mt_interface->logical_max_y;
    y /= mt_interface->physical_max_y;
    
    transducer->id = transducer_id;
    transducer->secondary_id = transducer_id;
    transducer->coordinates.x.update(x, timestamp);
    transducer->coordinates.y.update(y, timestamp);
    transducer->coordinates.z.update(z, timestamp);
    //transducer->tip_pressure.update(z, timestamp);
    transducer->is_valid = finger_state == 0x01;
    transducer->tip_switch.update(finger_state == 0x01, timestamp);
}

int VoodooI2CSynapticsDevice::rmi_f11_input(OSArray* transducers, AbsoluteTime timestamp, uint8_t *rmiInput) {
    //begin rmi parse
    int offset;
    int i;
    
    offset = (max_fingers >> 2) + 1;
    for (i = 0; i < max_fingers; i++) {
        int fs_byte_position = i >> 2;
        int fs_bit_position = (i & 0x3) << 1;
        int finger_state = (rmiInput[fs_byte_position] >> fs_bit_position) &
        0x03;
        int position = offset + 5 * i;
        rmi_f11_process_touch(transducers, i, timestamp, finger_state, &rmiInput[position]);
    }
    return f11.report_size;
}

int VoodooI2CSynapticsDevice::rmi_f30_input(OSArray* transducers, AbsoluteTime timestamp, uint8_t irq, uint8_t *rmiInput, int size)
{
    int i;
    bool value;
    
    if (!(irq & f30.irq_mask))
        return 0;
    
    if (size < (int)f30.report_size) {
        IOLog("%s::%s::Click Button pressed, but the click data is missing\n", getName(), name);
        return 0;
    }
    
    for (i = 0; i < gpio_led_count; i++) {
        if (button_mask & BIT(i)) {
            value = (rmiInput[i / 8] >> (i & 0x07)) & BIT(0);
            value = (button_state_mask & BIT(i)) ? !value : value;
            
            for(int i = 0; i < max_fingers; i++) {
                VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer, transducers->getObject(i));
                if(!transducer) {
                    IOLog("%s::%s::Failed to cast transducer f30 for id=%d\n", getName(), name, i);
                    return 0;
                }
                
                transducer->physical_button.update(value, timestamp);
            }
            
            break;
        }
    }
    return f30.report_size;
}

void VoodooI2CSynapticsDevice::TrackpadRawInput(uint8_t report[40]){
    if (report[0] != RMI_ATTN_REPORT_ID)
        return;
    
    int index = 2;
    
    int reportSize = 40;
    
    AbsoluteTime timestamp;
    
    clock_get_uptime(&timestamp);
    
    if (f11.interrupt_base < f30.interrupt_base) {
        index += rmi_f11_input(transducers, timestamp, &report[index]);
        index += rmi_f30_input(transducers, timestamp, report[1], &report[index], reportSize - index);
    }
    else {
        index += rmi_f30_input(transducers, timestamp, report[1], &report[index], reportSize - index);
        index += rmi_f11_input(transducers, timestamp, &report[index]);
    }
    
    if (mt_interface) {
        VoodooI2CMultitouchEvent event;
        event.transducers = transducers;
        event.contact_count = transducers->getCount();
        
        mt_interface->handleInterruptReport(event, timestamp);
    }
}

VoodooI2CSynapticsDevice* VoodooI2CSynapticsDevice::probe(IOService* provider, SInt32* score) {
    if (!super::probe(provider, score))
        return NULL;
    
    name = getMatchedName(provider);
    
    acpi_device = OSDynamicCast(IOACPIPlatformDevice, provider->getProperty("acpi-device"));
    
    if (!acpi_device) {
        IOLog("%s::%s Could not get ACPI device\n", getName(), name);
        return NULL;
    }
    
    api = OSDynamicCast(VoodooI2CDeviceNub, provider);
    
    if (!api) {
        IOLog("%s::%s Could not get VoodooI2C API access\n", getName(), name);
        return NULL;
    }
    
    if (rmi_populate()) {
        IOLog("%s::%s Unable to probe Synaptics RMI functions\n", getName(), name);
        return NULL;
    }
    
    return this;
}

void VoodooI2CSynapticsDevice::releaseResources() {
    unpublish_multitouch_interface();
    
    if (interrupt_source) {
        interrupt_source->disable();
        work_loop->removeEventSource(interrupt_source);
        interrupt_source->release();
        interrupt_source = NULL;
    }
    
    if (work_loop) {
        work_loop->release();
        work_loop = NULL;
    }
    
    if (acpi_device) {
        acpi_device->release();
        acpi_device = NULL;
    }
    
    if (api) {
        if (api->isOpen(this))
            api->close(this);
        api->release();
        api = NULL;
    }
    
    if (transducers) {
        for (int i = 0; i < transducers->getCount(); i++) {
            OSObject* object = transducers->getObject(i);
            if (object) {
                object->release();
            }
        }
        OSSafeReleaseNULL(transducers);
    }
}

bool VoodooI2CSynapticsDevice::start(IOService* api) {
    if (!super::start(api))
        return false;
    
    reading = true;
    
    work_loop = getWorkLoop();
    
    if (!work_loop) {
        IOLog("%s::%s Could not get work loop\n", getName(), name);
        goto exit;
    }
    
    work_loop->retain();
    
    acpi_device->retain();
    api->retain();
    
    if (!api->open(this)) {
        IOLog("%s::%s Could not open API\n", getName(), name);
        goto exit;
    }
    
    interrupt_source = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventAction, this, &VoodooI2CSynapticsDevice::interruptOccured), api, 0);
    if (!interrupt_source) {
        IOLog("%s::%s Could not get interrupt event source\n", getName(), name);
        goto exit;
    }
    
    work_loop->addEventSource(interrupt_source);
    interrupt_source->enable();
    
    PMinit();
    api->joinPMtree(this);
    registerPowerDriver(this, VoodooI2CIOPMPowerStates, kVoodooI2CIOPMNumberPowerStates);
    
    setProperty("VoodooI2CServices Supported", OSBoolean::withBoolean(true));
    
    publish_multitouch_interface();
    
    reading = false;
    
    
    return true;
exit:
    releaseResources();
    return false;
}

void VoodooI2CSynapticsDevice::stop(IOService* provider) {
    releaseResources();
    PMstop();
    super::stop(provider);
}

bool VoodooI2CSynapticsDevice::init(OSDictionary* properties) {
    transducers = NULL;
    if (!super::init(properties))
        return false;
    
    
    awake = true;
    
    return true;
}

int VoodooI2CSynapticsDevice::rmi_read_block(uint16_t addr, uint8_t *buf, const int len) {
    int ret = 0;
    IOReturn ret2;
    
    if (RMI_PAGE(addr) != page) {
        ret = rmi_set_page(RMI_PAGE(addr));
        if (ret < 0)
            goto exit;
    }
    
    uint8_t writeReport[21];
    for (int i = 0; i < 21; i++) {
        writeReport[i] = 0;
    }
    writeReport[0] = RMI_READ_ADDR_REPORT_ID;
    writeReport[1] = 0;
    writeReport[2] = addr & 0xFF;
    writeReport[3] = (addr >> 8) & 0xFF;
    writeReport[4] = len & 0xFF;
    writeReport[5] = (len >> 8) & 0xFF;
    if (rmi_write_report(writeReport, sizeof(writeReport)) < 0)
        return -1;
    
    uint8_t i2cInput[42];
    ret2 = api->readI2C(i2cInput, sizeof(i2cInput));
    
    if (ret2 != kIOReturnSuccess)
        return -1;
    
    // IOLog("RMI Read Commence\n");
    uint8_t rmiInput[40];
    for (int i = 0; i < 40; i++) {
        // IOLog("0x%x ", i2cInput[i]);
        rmiInput[i] = i2cInput[i + 2];
    }
    // IOLog("\n");
    // IOLog("RMI Read End\n");
    if (rmiInput[0] == RMI_READ_DATA_REPORT_ID) {
        for (int i = 0; i < len; i++) {
            buf[i] = rmiInput[i + 2];
        }
    }
exit:
    return ret;
}

int VoodooI2CSynapticsDevice::rmi_write_report(uint8_t *report, size_t report_size){
    uint8_t command[25];
    command[0] = 0x25;
    command[1] = 0x00;
    command[2] = 0x17;
    command[3] = 0x00;
    for (int i = 0; i < report_size; i++) {
        command[i + 4] = report[i];
    }
    IOReturn ret = api->writeI2C(command, sizeof(command));
    
    if (ret != kIOReturnSuccess)
        return -1;
    else
        return 0;
}

int VoodooI2CSynapticsDevice::rmi_read(uint16_t addr, uint8_t *buf){
    IOReturn ret = rmi_read_block(addr, buf, 1);
    
    if (ret != kIOReturnSuccess)
        return -1;
    else
        return 0;
}

int VoodooI2CSynapticsDevice::rmi_write_block(uint16_t addr, uint8_t *buf, const int len)
{
    int ret;
    
    uint8_t writeReport[21];
    for (int i = 0; i < 21; i++) {
        writeReport[i] = 0;
    }
    
    if (RMI_PAGE(addr) != page) {
        ret = rmi_set_page(RMI_PAGE(addr));
        if (ret < 0)
            goto exit;
    }
    
    writeReport[0] = RMI_WRITE_REPORT_ID;
    writeReport[1] = len;
    writeReport[2] = addr & 0xFF;
    writeReport[3] = (addr >> 8) & 0xFF;
    for (int i = 0; i < len; i++) {
        writeReport[i + 4] = buf[i];
    }
    
    ret = rmi_write_report(writeReport, sizeof(writeReport));
    if (ret < 0) {
        IOLog("%s::%s::failed to write request output report (%d)\n", getName(), name, ret);
        goto exit;
    }
    ret = 0;
    
exit:
    return ret;
}

int VoodooI2CSynapticsDevice::rmi_set_page(uint8_t _page)
{
    uint8_t writeReport[21];
    int retval;
    
    writeReport[0] = RMI_WRITE_REPORT_ID;
    writeReport[1] = 1;
    writeReport[2] = 0xFF;
    writeReport[4] = _page;
    
    retval = rmi_write_report(writeReport,
                              sizeof(writeReport));
    
    page = _page;
    return retval;
}

int VoodooI2CSynapticsDevice::rmi_write(uint16_t addr, uint8_t *buf)
{
    return rmi_write_block(addr, buf, 1);
}

static unsigned long rmi_gen_mask(unsigned irq_base, unsigned irq_count)
{
    return GENMASK(irq_count + irq_base - 1, irq_base);
}

void VoodooI2CSynapticsDevice::rmi_register_function(struct pdt_entry *pdt_entry, int _page, unsigned interrupt_count)
{
    struct rmi_function *f = NULL;
    uint16_t page_base = page << 8;
    
    // IOLog("got RMI function: 0x%x\n", pdt_entry->function_number);
    
    switch (pdt_entry->function_number) {
        case 0x01:
            f = &f01;
            break;
        case 0x11:
            f = &f11;
            break;
        case 0x12:
            f = &f12;
            break;
        case 0x30:
            f = &f30;
            break;
    }
    
    if (f) {
        f->page = _page;
        f->query_base_addr = page_base | pdt_entry->query_base_addr;
        f->command_base_addr = page_base | pdt_entry->command_base_addr;
        f->control_base_addr = page_base | pdt_entry->control_base_addr;
        f->data_base_addr = page_base | pdt_entry->data_base_addr;
        f->interrupt_base = interrupt_count;
        f->interrupt_count = pdt_entry->interrupt_source_count;
        f->irq_mask = rmi_gen_mask(f->interrupt_base,
                                   f->interrupt_count);
        interrupt_enable_mask |= f->irq_mask;
    }
}

int VoodooI2CSynapticsDevice::rmi_scan_pdt()
{
    struct pdt_entry entry;
    int _page;
    bool page_has_function;
    int i;
    int retval;
    int interrupt = 0;
    uint16_t page_start, pdt_start, pdt_end;
    
    IOLog("%s::%s::Scanning PDT...\n", getName(), name);
    
    for (_page = 0; (_page <= RMI4_MAX_PAGE); _page++) {
        page_start = RMI4_PAGE_SIZE * _page;
        pdt_start = page_start + PDT_START_SCAN_LOCATION;
        pdt_end = page_start + PDT_END_SCAN_LOCATION;
        
        page_has_function = false;
        for (i = pdt_start; i >= pdt_end; i -= sizeof(entry)) {
            retval = rmi_read_block(i, (uint8_t *)&entry, sizeof(entry));
            
            if (retval) {
                IOLog("%s::%s::Read of PDT entry at %#06x failed.\n", getName(), name, i);
                goto error_exit;
            }
            
            if (RMI4_END_OF_PDT(entry.function_number))
                break;
            
            page_has_function = true;
            
            rmi_register_function(&entry, _page, interrupt);
            interrupt += entry.interrupt_source_count;
        }
        
        if (!page_has_function)
            break;
    }
    
    IOLog("%s::%s::Done with PDT scan.\n", getName(), name);
    retval = 0;
    
error_exit:
    return retval;
}

int VoodooI2CSynapticsDevice::rmi_populate_f01()
{
    uint8_t basic_queries[RMI_DEVICE_F01_BASIC_QUERY_LEN];
    uint8_t info[3];
    int ret;
    bool has_query42;
    bool has_lts;
    bool has_sensor_id;
    bool has_ds4_queries = false;
    bool has_build_id_query = false;
    bool has_package_id_query = false;
    uint16_t query_offset = f01.query_base_addr;
    uint16_t prod_info_addr;
    uint8_t ds4_query_len;
    
    if (!f01.query_base_addr) {
        IOLog("%s::%s::Could not find F01\n", getName(), name);
        return -1;
    }
    
    ret = rmi_read_block(query_offset, basic_queries,
                         RMI_DEVICE_F01_BASIC_QUERY_LEN);
    if (ret) {
        IOLog("%s::%s::Can not read basic queries from Function 0x1.\n", getName(), name);
        return ret;
    }
    
    has_lts = !!(basic_queries[0] & BIT(2));
    has_sensor_id = !!(basic_queries[1] & BIT(3));
    has_query42 = !!(basic_queries[1] & BIT(7));
    
    query_offset += 11;
    prod_info_addr = query_offset + 6;
    query_offset += 10;
    
    if (has_lts)
        query_offset += 20;
    
    if (has_sensor_id)
        query_offset++;
    
    if (has_query42) {
        ret = rmi_read(query_offset, info);
        if (ret) {
            IOLog("%s::%s::Can not read query42.\n", getName(), name);
            return ret;
        }
        has_ds4_queries = !!(info[0] & BIT(0));
        query_offset++;
    }
    
    if (has_ds4_queries) {
        ret = rmi_read(query_offset, &ds4_query_len);
        if (ret) {
            IOLog("%s::%s::Can not read DS4 Query length.\n", getName(), name);
            return ret;
        }
        query_offset++;
        
        if (ds4_query_len > 0) {
            ret = rmi_read(query_offset, info);
            if (ret) {
                IOLog("%s::%s::Can not read DS4 query.\n", getName(), name);
                return ret;
            }
            
            has_package_id_query = !!(info[0] & BIT(0));
            has_build_id_query = !!(info[0] & BIT(1));
        }
    }
    
    if (has_package_id_query)
        prod_info_addr++;
    
    if (has_build_id_query) {
        ret = rmi_read_block(prod_info_addr, info, 3);
        if (ret) {
            IOLog("%s::%s::Can not read product info.\n", getName(), name);
            return ret;
        }
        
        firmware_id = info[1] << 8 | info[0];
        firmware_id += info[2] * 65536;
    }
    
    ret = rmi_read_block(f01.control_base_addr, info,
                         2);
    
    if (ret) {
        IOLog("%s::%s::can not read f01 ctrl registers\n", getName(), name);
        return ret;
    }
    
    f01_ctrl0 = info[0];
    
    if (!info[1]) {
        /*
         * Do to a firmware bug in some touchpads the F01 interrupt
         * enable control register will be cleared on reset.
         * This will stop the touchpad from reporting data, so
         * if F01 CTRL1 is 0 then we need to explicitly enable
         * interrupts for the functions we want data for.
         */
        restore_interrupt_mask = true;
        
        ret = rmi_write(f01.control_base_addr + 1,
                        &interrupt_enable_mask);
        if (ret) {
            IOLog("%s::%s::can not write to control reg 1: %d.\n", getName(), name, ret);
            return ret;
        }
        IOLog("%s::%s::Firmware bug fix needed!!! :/\n", getName(), name);
    }
    
    return 0;
}

int VoodooI2CSynapticsDevice::rmi_populate_f11()
{
    uint8_t buf[20];
    int ret;
    bool has_query9;
    bool has_query10 = false;
    bool has_query11;
    bool has_query12;
    bool has_query27;
    bool has_query28;
    bool has_query36 = false;
    bool has_physical_props;
    bool has_gestures;
    bool has_rel;
    bool has_data40 = false;
    bool has_dribble = false;
    bool has_palm_detect = false;
    unsigned x_size, y_size;
    uint16_t query_offset;
    
    if (!f11.query_base_addr) {
        IOLog("%s::%s::No F11 2D sensor found\n", getName(), name);
        return 0;
    }
    
    /* query 0 contains some useful information */
    ret = rmi_read(f11.query_base_addr, buf);
    if (ret) {
        IOLog("%s::%s::can not get query 0: %d.\n", getName(), name, ret);
        return ret;
    }
    has_query9 = !!(buf[0] & BIT(3));
    has_query11 = !!(buf[0] & BIT(4));
    has_query12 = !!(buf[0] & BIT(5));
    has_query27 = !!(buf[0] & BIT(6));
    has_query28 = !!(buf[0] & BIT(7));
    
    /* query 1 to get the max number of fingers */
    ret = rmi_read(f11.query_base_addr + 1, buf);
    if (ret) {
        IOLog("%s::%s::can not get NumberOfFingers: %d.\n", getName(), name, ret);
        return ret;
    }
    max_fingers = (buf[0] & 0x07) + 1;
    setProperty("Max Fingers", OSNumber::withNumber(max_fingers, sizeof(max_fingers) * 8));
    if (max_fingers > 5)
        max_fingers = 10;
    
    transducers = OSArray::withCapacity(max_fingers);
    if (!transducers) {
        return false;
    }
    DigitiserTransducerType type = kDigitiserTransducerFinger;
    for (int i = 0; i < max_fingers; i++) {
        VoodooI2CDigitiserTransducer* transducer = VoodooI2CDigitiserTransducer::transducer(type, NULL);
        transducers->setObject(transducer);
    }
    
    f11.report_size = max_fingers * 5 +
    DIV_ROUND_UP(max_fingers, 4);
    
    if (!(buf[0] & BIT(4))) {
        IOLog("%s::%s::No absolute events, giving up.\n", getName(), name);
        return -ENODEV;
    }
    
    has_rel = !!(buf[0] & BIT(3));
    has_gestures = !!(buf[0] & BIT(5));
    
    ret = rmi_read(f11.query_base_addr + 5, buf);
    if (ret) {
        IOLog("%s::%s::can not get absolute data sources: %d.\n", getName(), name, ret);
        return ret;
    }
    
    has_dribble = !!(buf[0] & BIT(4));
    
    /*
     * At least 4 queries are guaranteed to be present in F11
     * +1 for query 5 which is present since absolute events are
     * reported and +1 for query 12.
     */
    query_offset = 6;
    
    if (has_rel)
        ++query_offset; /* query 6 is present */
    
    if (has_gestures) {
        /* query 8 to find out if query 10 exists */
        ret = rmi_read(f11.query_base_addr + query_offset + 1, buf);
        if (ret) {
            IOLog("%s::%s::can not read gesture information: %d.\n", getName(), name,
                  ret);
            return ret;
        }
        has_palm_detect = !!(buf[0] & BIT(0));
        has_query10 = !!(buf[0] & BIT(2));
        
        query_offset += 2; /* query 7 and 8 are present */
    }
    
    if (has_query9)
        ++query_offset;
    
    if (has_query10)
        ++query_offset;
    
    if (has_query11)
        ++query_offset;
    
    /* query 12 to know if the physical properties are reported */
    if (has_query12) {
        ret = rmi_read(f11.query_base_addr
                       + query_offset, buf);
        if (ret) {
            IOLog("%s::%s::can not get query 12: %d.\n", getName(), name, ret);
            return ret;
        }
        has_physical_props = !!(buf[0] & BIT(5));
        
        if (has_physical_props) {
            query_offset += 1;
            ret = rmi_read_block(f11.query_base_addr
                                 + query_offset, buf, 4);
            if (ret) {
                IOLog("%s::%s::can not read query 15-18: %d.\n", getName(), name,
                      ret);
                return ret;
            }
            
            x_size = buf[0] | (buf[1] << 8);
            y_size = buf[2] | (buf[3] << 8);
            
            x_size_mm = x_size / 10;
            y_size_mm = y_size / 10;
            
            setProperty("X per mm", OSNumber::withNumber(x_size_mm, sizeof(x_size_mm) * 8));
            setProperty("Y per mm", OSNumber::withNumber(y_size_mm, sizeof(y_size_mm) * 8));
            
            IOLog("%s::%s::size in mm: %d x %d\n", getName(), name,
                  x_size_mm, y_size_mm);
            
            /*
             * query 15 - 18 contain the size of the sensor
             * and query 19 - 26 contain bezel dimensions
             */
            query_offset += 12;
        }
    }
    
    if (has_query27)
        ++query_offset;
    
    if (has_query28) {
        ret = rmi_read(f11.query_base_addr
                       + query_offset, buf);
        if (ret) {
            IOLog("%s::%s::can not get query 28: %d.\n", getName(), name, ret);
            return ret;
        }
        
        has_query36 = !!(buf[0] & BIT(6));
    }
    
    if (has_query36) {
        query_offset += 2;
        ret = rmi_read(f11.query_base_addr
                       + query_offset, buf);
        if (ret) {
            IOLog("%s::%s::can not get query 36: %d.\n", getName(), name, ret);
            return ret;
        }
        
        has_data40 = !!(buf[0] & BIT(5));
    }
    
    
    if (has_data40)
        f11.report_size += max_fingers * 2;
    
    ret = rmi_read_block(f11.control_base_addr,
                         f11_ctrl_regs, RMI_F11_CTRL_REG_COUNT);
    if (ret) {
        IOLog("%s::%s::can not read ctrl block of size 11: %d.\n", getName(), name, ret);
        return ret;
    }
    
    /* data->f11_ctrl_regs now contains valid register data */
    read_f11_ctrl_regs = true;
    
    max_x = f11_ctrl_regs[6] | (f11_ctrl_regs[7] << 8);
    max_y = f11_ctrl_regs[8] | (f11_ctrl_regs[9] << 8);
    
    setProperty("Max X", OSNumber::withNumber(max_x, 32));
    setProperty("Max Y", OSNumber::withNumber(max_y, 32));
    
    IOLog("%s::%s::Trackpad Resolution: %d x %d\n", getName(), name, max_x, max_y);
    
    if (has_dribble) {
        f11_ctrl_regs[0] = f11_ctrl_regs[0] & ~BIT(6);
        ret = rmi_write(f11.control_base_addr,
                        f11_ctrl_regs);
        if (ret) {
            IOLog("%s::%s::can not write to control reg 0: %d.\n", getName(), name,
                  ret);
            return ret;
        }
    }
    
    if (has_palm_detect) {
        f11_ctrl_regs[11] = f11_ctrl_regs[11] & ~BIT(0);
        ret = rmi_write(f11.control_base_addr + 11,
                        &f11_ctrl_regs[11]);
        if (ret) {
            IOLog("%s::%s::can not write to control reg 11: %d.\n", getName(), name,
                  ret);
            return ret;
        }
    }
    
    return 0;
}

int VoodooI2CSynapticsDevice::rmi_populate_f12() {
    if (!f12.query_base_addr) {
        return 0;
    } else {
        IOLog("%s::%s Found F12 but this protocol is not yet implemented\n", getName(), name);
        return -1;
    }
}

int VoodooI2CSynapticsDevice::rmi_populate_f30()
{
    uint8_t buf[20];
    int ret;
    bool has_gpio, has_led;
    unsigned bytes_per_ctrl;
    uint8_t ctrl2_addr;
    int ctrl2_3_length;
    int i;
    
    /* function F30 is for physical buttons */
    if (!f30.query_base_addr) {
        IOLog("%s::%s::No GPIO/LEDs found, giving up.\n", getName(), name);
        return -ENODEV;
    }
    
    ret = rmi_read_block(f30.query_base_addr, buf, 2);
    if (ret) {
        IOLog("%s::%s::can not get F30 query registers: %d.\n", getName(), name, ret);
        return ret;
    }
    
    has_gpio = !!(buf[0] & BIT(3));
    has_led = !!(buf[0] & BIT(2));
    gpio_led_count = buf[1] & 0x1f;
    
    /* retrieve ctrl 2 & 3 registers */
    bytes_per_ctrl = (gpio_led_count + 7) / 8;
    /* Ctrl0 is present only if both has_gpio and has_led are set*/
    ctrl2_addr = (has_gpio && has_led) ? bytes_per_ctrl : 0;
    /* Ctrl1 is always be present */
    ctrl2_addr += bytes_per_ctrl;
    ctrl2_3_length = 2 * bytes_per_ctrl;
    
    f30.report_size = bytes_per_ctrl;
    
    ret = rmi_read_block(f30.control_base_addr + ctrl2_addr,
                         buf, ctrl2_3_length);
    if (ret) {
        IOLog("%s::%s::can not read ctrl 2&3 block of size %d: %d.\n", getName(), name,
              ctrl2_3_length, ret);
        return ret;
    }
    
    button_count = 0;
    button_mask = 0;
    button_state_mask = 0;
    
    for (i = 0; i < gpio_led_count; i++) {
        int byte_position = i >> 3;
        int bit_position = i & 0x07;
        uint8_t dir_byte = buf[byte_position];
        uint8_t data_byte = buf[byte_position + bytes_per_ctrl];
        bool dir = (dir_byte >> bit_position) & BIT(0);
        bool dat = (data_byte >> bit_position) & BIT(0);
        
        if (dir == 0) {
            /* input mode */
            if (dat) {
                /* actual buttons have pull up resistor */
                button_count++;
                button_mask += BIT(i);
                button_state_mask += BIT(i);
            }
        }
        
    }
    
    return 0;
}

int VoodooI2CSynapticsDevice::rmi_set_mode(uint8_t mode) {
    uint8_t command[] = { 0x22, 0x00, 0x3f, 0x03, 0x0f, 0x23, 0x00, 0x04, 0x00, RMI_SET_RMI_MODE_REPORT_ID, mode }; //magic bytes from Linux
    IOReturn ret = api->writeI2C(command, sizeof(command));
    if (ret != kIOReturnSuccess)
        return -1;
    else
        return 0;
}

int VoodooI2CSynapticsDevice::rmi_populate() {
    int ret;
    
    ret = rmi_set_mode(RMI_MODE_ATTN_REPORTS);
    if (ret) {
        IOLog("%s::%s::PDT set mode failed with code %d\n", getName(), name, ret);
        return ret;
    }
    
    ret = rmi_scan_pdt();
    if (ret) {
        IOLog("%s::%s::PDT scan failed with code %d\n", getName(), name, ret);
        return ret;
    }
    
    ret = rmi_populate_f01();
    if (ret) {
        IOLog("%s::%s::Error while initializing F01 (%d)\n", getName(), name, ret);
        return ret;
    }
    
    ret = rmi_populate_f11();
    if (ret) {
        IOLog("%s::%s::Error while initializing F11 (%d)\n", getName(), name, ret);
        return ret;
    }
    
    ret = rmi_populate_f12();
    if (ret) {
        IOLog("%s::%s::Possible error while initialising F12 (%d)\n", getName(), name, ret);
        return ret;
    }
    
    if (!f11.query_base_addr && !f12.query_base_addr) {
        IOLog("%s::%s::Could not find either F11 or F12, not able to drive this device\n", getName(), name);
        return -1;
    }
    
    ret = rmi_populate_f30();
    if (ret) {
        IOLog("%s::%s::Error while initializing F30 (%d)\n", getName(), name, ret);
        return ret;
    }
    
    return 0;
}

IOReturn VoodooI2CSynapticsDevice::setPowerState(unsigned long powerState, IOService *whatDevice){
    if (powerState == 0){
        //Going to sleep
        
        awake = false;
        
        IOLog("%s::Going to Sleep!\n", getName());
    } else {
        if (!awake){
            rmi_populate();
            
            awake = true;
            IOLog("%s::Woke up from Sleep!\n", getName());
        } else {
            IOLog("%s::Trackpad already awake! Not reinitializing.\n", getName());
        }
    }
    return kIOPMAckImplied;
}


void VoodooI2CSynapticsDevice::interruptOccured(OSObject* owner, IOInterruptEventSource* src, int intCount){
    if (reading || !awake)
        return;
    
    thread_t new_thread;
    kern_return_t ret = kernel_thread_start(OSMemberFunctionCast(thread_continue_t, this, &VoodooI2CSynapticsDevice::get_input), this, &new_thread);
    if (ret != KERN_SUCCESS) {
        reading = false;
        IOLog("%s Thread error while attemping to get input report\n", getName());
    } else {
        thread_deallocate(new_thread);
    }
}

void VoodooI2CSynapticsDevice::get_input() {
    reading = true;
    
    uint8_t reg = 0;
    api->writeI2C(&reg, 1);
    
    uint8_t i2cInput[42];
    api->readI2C(i2cInput, sizeof(i2cInput));
    
    uint8_t rmiInput[40];
    for (int i = 0; i < 40; i++) {
        rmiInput[i] = i2cInput[i + 2];
    }
    
    if (rmiInput[0] == 0x00){
        goto exit;
    }
    
    if (rmiInput[0] != RMI_ATTN_REPORT_ID) {
        goto exit;
    }
    
    TrackpadRawInput(rmiInput);
exit:
    reading = false;
}

bool VoodooI2CSynapticsDevice::publish_multitouch_interface() {
    mt_interface = new VoodooI2CMultitouchInterface();
    if (!mt_interface) {
        IOLog("%s::%s No memory to allocate VoodooI2CMultitouchInterface instance\n", getName(), name);
        goto multitouch_exit;
    }
    if (!mt_interface->init(NULL)) {
        IOLog("%s::%s Failed to init multitouch interface\n", getName(), name);
        goto multitouch_exit;
    }
    if (!mt_interface->attach(this)) {
        IOLog("%s::%s Failed to attach multitouch interface\n", getName(), name);
        goto multitouch_exit;
    }
    if (!mt_interface->start(this)) {
        IOLog("%s::%s Failed to start multitouch interface\n", getName(), name);
        goto multitouch_exit;
    }
    // Assume we are a touchpad
    mt_interface->setProperty(kIOHIDDisplayIntegratedKey, true);
    // 0x04f3 is Elan's Vendor Id
    mt_interface->setProperty(kIOHIDVendorIDKey, 0x04f3, 32);
    // mt_interface->setProperty(kIOHIDProductIDKey, 0x0, 32);
    mt_interface->setProperty("Firmware ID", firmware_id, 32);
    
    mt_interface->logical_max_x = x_size_mm * 10;
    mt_interface->logical_max_y = y_size_mm * 10;
    
    mt_interface->physical_max_x = max_x;
    mt_interface->physical_max_y = max_y;
    
    mt_interface->registerService();
    return true;
multitouch_exit:
    unpublish_multitouch_interface();
    return false;
}

void VoodooI2CSynapticsDevice::unpublish_multitouch_interface() {
    if (mt_interface) {
        mt_interface->stop(this);
        mt_interface->release();
        mt_interface = NULL;
    }
}
