/**
 * @file ee24lc64.cpp
 * @brief Implementation of the Ee24lc64 class for accessing an I2C EEPROM.
 *
 * This file provides the implementation for reading and writing data to an Ee24lc64 I2C EEPROM for Raspberry PI Pico.
 */

#include "ee24lc64.hpp"
#include "i2c_scanner.hpp"

#include <cstring>
#include <iostream>

/**
 * @brief Constructor for the Ee24lc64 class.
 *
 * @param i2c Pointer to the I2C instance.
 * @param addr I2C address of the Ee24lc64 device.
 */
Ee24lc64::Ee24lc64(i2c_inst_t *i2c, const uint8_t &addr)
    : PromBase(i2c, addr), pages_(256), bpp_(32)
{
}

/**
 * @brief Destructor for the Ee24lc64 class.
 *
 * @throw Nothing.
 */
Ee24lc64::~Ee24lc64() throw() {}

/**
 * @brief Write data to the Ee24lc64 device.
 *
 * @param addr Start address for the data to be written.
 * @param bytes Pointer to the data to be written.
 * @param len Number of bytes to write.
 * @return The number of bytes written, or an error code.
 */
int Ee24lc64::write(uint16_t addr, const void *bytes, size_t len) const
{
    // std::cout << __FILE__ << " " << __FUNCTION__ << " addr:" << (int)addr_ << " v:" << i2c_device_present(i2c_, addr_) << std::endl;
    int ret(0);
    int idx(0);

    // Handle the case when the starting address is not aligned to the page size
    if (addr % bpp_)
    {
        size_t l(bpp_ - (addr % bpp_));
        if (l < len)
        {
            ret = do_write(addr, bytes, l);
            addr += l;
            idx += l;
            len -= l;
            if (ret < 0)
                len = 0;
        }
    }

    // Write data in chunks of bpp_ bytes
    while (len > 0)
    {
        size_t l = std::min(len, (size_t)bpp_);
        ret = do_write(addr, (uint8_t *)bytes + idx, l);
        addr += l;
        idx += l;
        len -= l;
        if (ret < 0)
            len = 0;
    }

    if (ret < 0)
        return ret;

    return idx;
}

/**
 * @brief Read data from the Ee24lc64 device.
 *
 * @param addr Start address of the data to be read.
 * @param bytes Pointer to the buffer to store the read data.
 * @param len Number of bytes to read.
 * @return The number of bytes read, or an error code.
 */
int Ee24lc64::read(const uint16_t &addr, const void *bytes, size_t len) const
{
    // std::cout << __FILE__ << " " << __FUNCTION__ << " addr:" << (int)addr_ << " v:" << i2c_device_present(i2c_, addr_) << std::endl;
    // Set the address for the read operation
    int ret = do_write(addr, 0, 0);
    if (ret < 0)
        return ret;

    // Read data from the device with a timeout
    ret = i2c_read_timeout_us(i2c_, addr_, (uint8_t *)bytes, len, false, 300 * (len + 1));
    while (ret == PICO_ERROR_GENERIC)
        ret = i2c_read_timeout_us(i2c_, addr_, (uint8_t *)bytes, len, false, 300 * (len + 1));

    return ret;
}

/**
 * @brief Internal function to perform write operations on the Ee24lc64 device.
 *
 * @param addr Start address for the data to be written.
 * @param bytes Pointer to the data to be written.
 * @param len Number of bytes to write.
 * @return The result of the write operation, or an error code.
 */
inline int Ee24lc64::do_write(const uint16_t &addr, const void *bytes, const size_t &len) const
{
    uint8_t buf[bpp_ + 2];
    buf[1] = addr & 0x00FF;
    buf[0] = (addr >> 8) & 0x00FF;

    // Copy the data to the buffer
    size_t l = std::min(len, (size_t)bpp_);
    std::memcpy(buf + 2, bytes, l);

    // Perform the write operation with a timeout
    int ret(i2c_write_timeout_us(i2c_, addr_, buf, l + 2, false, 5000));
    while (ret == PICO_ERROR_GENERIC)
        ret = i2c_write_timeout_us(i2c_, addr_, buf, l + 2, false, 5000);

    return ret;
}

/**
 * @brief Get the size of the Ee24lc64 device in bytes.
 *
 * @return Size of the device in bytes.
 */
size_t Ee24lc64::size() const // bytes
{
    return bpp_ * pages_;
}
