/**
 * @file ee24lc64.hpp
 * @brief This file contains the declaration of the Ee24lc64 class.
 */

#ifndef EE24LC64_HPP
#define EE24LC64_HPP

#include "prom_base.hpp"

/**
 * @brief The Ee24lc64 class provides an interface to the 24LC64 EEPROM chip.
 *
 * * This file provides the implementation for reading and writing data to an Ee24lc64 I2C EEPROM for Raspberry PI Pico.
 */
class Ee24lc64 : public PromBase
{
public:
    /**
     * @brief Construct a new Ee24lc64 object.
     *
     * @param i2c The I2C instance to use.
     * @param addr The I2C address of the chip.
     */
    Ee24lc64(i2c_inst_t *i2c, const uint8_t &addr = 0x50);

    /**
     * @brief Destroy the Ee24lc64 object
     *
     * @throw no exception
     */
    virtual ~Ee24lc64() throw();

    /**
     * @brief Write data to the chip.
     *
     * @param addr The address to write to.
     * @param bytes The data to write.
     * @param len The length of the data to write.
     * @return int The number of bytes written, or a negative error code.
     */
    int write(uint16_t addr, const void *bytes, size_t len) const;

    /**
     * @brief Read data from the chip.
     *
     * @param addr The address to read from.
     * @param bytes The buffer to read into.
     * @param len The length of the data to read.
     * @return int The number of bytes read, or a negative error code.
     */
    int read(const uint16_t &addr, const void *bytes, size_t len) const;

    /**
     * @brief Get the size of the EEPROM in bytes
     *
     * @return size_t Size of the EEPROM in bytes
     */
    size_t size() const; // bytes

private:
    /**
     * @brief Write data to the chip.
     *
     * @param addr The address to write to.
     * @param bytes The data to write.
     * @param len The length of the data to write.
     * @return int The number of bytes written, or a negative error code.
     */
    inline int do_write(const uint16_t &addr, const void *bytes, const size_t &len) const;

    const uint32_t pages_; /**< The number of pages in the chip. */
    const uint32_t bpp_;   /**< The number of bytes per page. */

    // Disallow copy and assignment
    Ee24lc64(const Ee24lc64 &);
    Ee24lc64 &operator=(const Ee24lc64 &);
};

#endif // EE24LC64_HPP