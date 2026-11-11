/**
 * @file i2cdev.h
 * @defgroup i2cdev i2cdev
 * @{
 *
 * ESP-IDF I2C master functions for communication with I2C slave
 *
 * Thread safety is per device descriptor: every access to a device must be
 * performed under I2C_DEV_TAKE_MUTEX()/I2C_DEV_GIVE_MUTEX(). The bus itself is
 * serialised by the ESP-IDF I2C driver. With CONFIG_I2CDEV_NOLOCK enabled no
 * per-device mutex exists and serialisation becomes the caller's responsibility.
 *
 * This header is served by two implementations, selected by the build system:
 * i2cdev.c (new i2c_master driver, ESP-IDF v5.3 and later, unless legacy is
 * requested explicitly) and i2cdev_legacy.c (legacy driver/i2c.h, used on
 * older ESP-IDF or with CONFIG_I2CDEV_USE_LEGACY_DRIVER=y).
 *
 * Copyright (C) 2018 Ruslan V. Uss <unclerus@gmail.com>
 * Updated 2025 by quinkq to use newer ESP-IDF I2C master driver API
 *
 * MIT Licensed as described in the file LICENSE
 *
 * ============================================================================
 * OPTIONAL I2C PULLUP AUTO-CONFIGURATION
 * ============================================================================
 *
 * This library can optionally enable internal I2C pullups when no explicit
 * pullup configuration is provided. Feature is DISABLED by default for
 * backward compatibility (CONFIG_I2CDEV_AUTO_ENABLE_PULLUPS=n).
 *
 * Optional auto-pullup (CONFIG_I2CDEV_AUTO_ENABLE_PULLUPS=y):
 * - If both pullup flags are false (not set/default state), automatically enables internal pullups
 * - Implemented by the i2c_master based driver only (i2cdev.c), on any target
 * - Legacy driver always uses explicit configuration
 *
 *
 * Example - Enable internal pullups:
 * i2c_dev_t sensor = {
 *     .port = I2C_NUM_0,
 *     .addr = 0x48,
 *     .cfg = {
 *         .sda_io_num = GPIO_NUM_21,
 *         .scl_io_num = GPIO_NUM_22,
 *         .sda_pullup_en = true,  // Enable internal pullups
 *         .scl_pullup_en = true,  // Enable internal pullups
 *         .master.clk_speed = 400000
 *     }
 * };
 *
 * ============================================================================
 */

#ifndef __I2CDEV_H__
#define __I2CDEV_H__

#include <esp_err.h>
#include <esp_idf_lib_helpers.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_idf_version.h>

#include <driver/gpio.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
#include <driver/i2c_master.h>
#else
#include <driver/i2c.h>
#endif

// Define missing types for older ESP-IDF versions
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
typedef enum
{
    I2C_ADDR_BIT_LEN_7 = 0, /*!< I2C 7bit address for slave mode */
    I2C_ADDR_BIT_LEN_10,    /*!< I2C 10bit address for slave mode */
} i2c_addr_bit_len_t;
#endif

// Clock stretching timeout for the legacy driver only: i2cdev_legacy.c uses it
// when dev->timeout_ticks is 0. The i2c_master based driver has no such knob.
#if HELPER_TARGET_IS_ESP8266
#define I2CDEV_MAX_STRETCH_TIME 0xffffffff
#else
#include <soc/i2c_reg.h> // For I2C_TIME_OUT_VALUE_V, etc.
#if defined(I2C_TIME_OUT_VALUE_V)
#define I2CDEV_MAX_STRETCH_TIME I2C_TIME_OUT_VALUE_V
#elif defined(I2C_TIME_OUT_REG_V)
#define I2CDEV_MAX_STRETCH_TIME I2C_TIME_OUT_REG_V
#else
#define I2CDEV_MAX_STRETCH_TIME 0x00ffffff
#endif
#endif /* HELPER_TARGET_IS_ESP8266 */

#ifndef CONFIG_I2CDEV_TIMEOUT
#define CONFIG_I2CDEV_TIMEOUT 1000 // Default 1 second timeout
#endif

#ifndef CONFIG_I2CDEV_NOLOCK
#define CONFIG_I2CDEV_NOLOCK 0 // Enable locking by default
#endif

#ifndef CONFIG_I2CDEV_MAX_DEVICES_PER_PORT
#define CONFIG_I2CDEV_MAX_DEVICES_PER_PORT 8 // Legacy driver only: devices per I2C port
#endif

#ifndef CONFIG_I2CDEV_DEFAULT_SDA_PIN
#define CONFIG_I2CDEV_DEFAULT_SDA_PIN 21 // Default SDA pin
#endif

#ifndef CONFIG_I2CDEV_DEFAULT_SCL_PIN
#define CONFIG_I2CDEV_DEFAULT_SCL_PIN 22 // Default SCL pin
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief I2C transaction type for legacy probe
 */
typedef enum
{
    I2C_DEV_WRITE = 0, /**< Write operation for probe */
    I2C_DEV_READ       /**< Read operation for probe */
} i2c_dev_type_t;

/**
 * I2C device descriptor
 *
 * This structure supports both legacy ESP-IDF I2C driver and modern i2c_master driver.
 *
 * @note INITIALIZATION CHECKLIST - fill the descriptor fields before the
 *       first I/O operation on it, not before i2c_dev_create_mutex().
 *
 * REQUIRED (set by user):
 * - dev->port: port number, 0 .. I2C_NUM_MAX-1
 * - dev->addr: device I2C address (for example 0x48)
 * - dev->cfg.sda_io_num: SDA pin, -1 means "use the Kconfig default"
 * - dev->cfg.scl_io_num: SCL pin, -1 means "use the Kconfig default"
 *
 * OPTIONAL (set by user if needed):
 * - dev->addr_bit_len: address format, defaults to 7-bit
 * - dev->cfg.master.clk_speed: clock speed in Hz, 0 means 400000 with a warning
 * - dev->cfg.sda_pullup_en: enable internal SDA pullup
 * - dev->cfg.scl_pullup_en: enable internal SCL pullup
 * - dev->timeout_ticks: clock stretching timeout, legacy driver only
 *
 * AUTO-POPULATED (library fills these):
 * - dev->dev_handle: device handle, created lazily, modern driver only
 * - dev->sda_pin: actual SDA pin used by the bus
 * - dev->scl_pin: actual SCL pin used by the bus
 *
 * @note dev->mutex is NOT populated automatically: it is created by
 *       i2c_dev_create_mutex() and does not exist at all with
 *       CONFIG_I2CDEV_NOLOCK enabled.
 *
 * @note dev->cfg.clk_flags exists only to keep the i2c_config_t compatible
 *       layout; both implementations ignore it.
 *
 * @note One port is bound to one pin pair: the first operation fixes SDA/SCL
 *       for the port, and a device that asks for other pins on the same port
 *       fails with ESP_ERR_INVALID_STATE.
 *
 * @note 10-bit addressing requires SOC_I2C_SUPPORT_10BIT_ADDR; elsewhere
 *       addr_bit_len is forced to 7-bit (the driver logs a warning).
 *
 * @note LIFETIME: the library keeps a pointer to the descriptor once the
 *       device has been added to the bus. The descriptor must stay valid
 *       until i2c_dev_delete_mutex() was called for it (or i2cdev_done()
 *       released the port): do not copy, move or free it while registered,
 *       i2cdev_done() walks the registered descriptors.
 *
 * @note BACKWARD COMPATIBILITY DESIGN:
 * The custom 'cfg' structure mimics ESP-IDF's deprecated i2c_config_t layout
 * to maintain zero-change compatibility with existing device drivers.
 * ESP-IDF ≥5.2 deprecated i2c_config_t and split it into separate bus/device
 * configs, but this library preserves the familiar field paths like:
 * dev->cfg.sda_io_num, dev->cfg.scl_io_num, dev->cfg.master.clk_speed
 */
typedef struct
{
    // ═══ Core Device Identity (REQUIRED) ═══
    i2c_port_t port;                 //!< I2C port number: 0 .. I2C_NUM_MAX-1
    uint16_t addr;                   //!< Device I2C address (e.g., 0x48 for 7-bit)
    i2c_addr_bit_len_t addr_bit_len; //!< Address format: I2C_ADDR_BIT_LEN_7 (default) or I2C_ADDR_BIT_LEN_10 where SOC_I2C_SUPPORT_10BIT_ADDR

    // ═══ Library Internal State ═══
    SemaphoreHandle_t mutex; //!< Device mutex - Created by i2c_dev_create_mutex(), unused with CONFIG_I2CDEV_NOLOCK
    void *dev_handle;        //!< Device handle - Modern driver only, created lazily (when actual I2C operation is performed)
    int sda_pin;             //!< Actual SDA pin used - Populated after port setup
    int scl_pin;             //!< Actual SCL pin used - Populated after port setup

    // ═══ Legacy Driver Compatibility ═══
    uint32_t timeout_ticks; //!< Clock stretching timeout - Legacy driver only

    // ═══ User Configuration ═══
    // Configuration structure with i2c_config_t compatible field layout.
    struct
    {
        gpio_num_t sda_io_num; //!< Desired SDA pin (-1 = use Kconfig default)
        gpio_num_t scl_io_num; //!< Desired SCL pin (-1 = use Kconfig default)
        uint8_t sda_pullup_en; //!< Enable internal SDA pullup (optional)
        uint8_t scl_pullup_en; //!< Enable internal SCL pullup (optional)
        uint32_t clk_flags;    //!< Reserved for i2c_config_t layout compatibility: ignored by both drivers
        struct
        {
            uint32_t clk_speed; //!< Clock speed in Hz, 0 = 400000 with a warning in the log
        } master;               //!< Master-specific config (mimics old i2c_config_t.master)
    } cfg;                      //!< Configuration set by device drivers (i2c_config_t compatible layout)
} i2c_dev_t;

/**
 * @brief Initialize I2C subsystem (port mutexes and internal states)
 *
 * @note This should be called once at the beginning of your application
 * before any I2C devices are initialized.
 *
 * @note Calling it more than once is allowed and costs nothing beyond creating
 * the port mutexes that are missing, so it is safe to call again after
 * i2cdev_done(). A port whose bus is still installed (because its cleanup
 * failed) keeps that bus instead of being reset.
 *
 * @return `ESP_OK` on success, `ESP_ERR_NO_MEM` if a port mutex could not be
 *         created
 */
esp_err_t i2cdev_init(void);

/**
 * @brief Release I2C subsystem (deletes all devices, buses, and mutexes)
 *
 * @note Call this when no more I2C operations will be performed
 * to clean up resources.
 *
 * @note Cleanup is best effort and may be partial. If a bus or one of its
 *       devices cannot be released (for example, the bus was handed out via
 *       i2cdev_get_shared_handle() and is still used by another component), the
 *       port state is kept intact and its mutex is not deleted, so the call can
 *       be repeated once the obstacle is gone. Ports whose mutex is held by
 *       another task are skipped for the same reason.
 *
 * @return ESP_OK if every port was released; on a partial failure the first
 *         error is returned (ESP_ERR_TIMEOUT if a port mutex could not be
 *         taken, otherwise the error reported by the I2C driver). A non-ESP_OK
 *         result means the subsystem was not fully released and i2cdev_done()
 *         may be called again.
 */
esp_err_t i2cdev_done(void);

/**
 * @brief Create mutex for device descriptor
 *
 * @note The descriptor fields (port, address, pins, speed) are read by the data
 *       transfer functions, so they must be filled before the first I/O
 *       operation on the device, not before this call.
 *
 * @note This call only creates the per-device mutex and normalises
 *       dev->addr_bit_len: it does not touch the bus. The port mutexes are
 *       created by i2cdev_init(), the bus is created lazily by the first
 *       operation on the port (i2c_dev_check_present() and i2c_dev_probe()
 *       create the bus as well), and the device handle is added to the bus on
 *       the first real transfer. Probe calls do not attach the device.
 *
 * @param dev Pointer to device descriptor
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if @p dev is NULL,
 *         `ESP_ERR_NO_MEM` if the device mutex could not be created. With
 *         @c CONFIG_I2CDEV_NOLOCK no mutex is created, so this call cannot fail
 *         with `ESP_ERR_NO_MEM` in that configuration.
 */
esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev);

/**
 * @brief Delete mutex for device descriptor and perform device cleanup
 *
 * @note This function performs cleanup tasks including removing the device from the
 *       I2C bus, deregistering it, and deleting its mutex.
 *
 * @note The bus itself is NOT deleted here: it lives from i2cdev_init() (or the
 *       first I2C operation) until i2cdev_done(). Removing the last tracked device
 *       therefore leaves the shared bus available for any other consumer of
 *       i2cdev_get_shared_handle(). A device handle whose removal failed is
 *       preserved instead of being dropped: the failure is reported and the
 *       operation can be retried.
 *
 * @note With @c CONFIG_I2CDEV_NOLOCK enabled no per-device mutex exists, but the
 *       cleanup described above is still performed in that configuration.
 *
 * @note If the descriptor's port is out of range, or i2cdev_init() was never
 *       called, nothing port-scoped can be cleaned up: the call logs a warning,
 *       deletes only the per-device mutex and returns `ESP_OK`.
 *
 * @param dev Pointer to device descriptor
 * @return `ESP_OK` on success or when the port was never initialised (see the
 *         note above), `ESP_ERR_INVALID_ARG` if @p dev is NULL,
 *         `ESP_ERR_TIMEOUT` if the port mutex could not be taken, otherwise the
 *         error reported by the I2C driver while removing the device
 */
esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev);

/**
 * @brief Take device mutex
 *
 * @note With @c CONFIG_I2CDEV_NOLOCK enabled this is a no-op that returns
 *       `ESP_OK`.
 *
 * @param dev Pointer to device descriptor
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if @p dev is NULL,
 *         `ESP_ERR_INVALID_STATE` if the device mutex was not created,
 *         `ESP_ERR_TIMEOUT` if the mutex was not released within
 *         @c CONFIG_I2CDEV_TIMEOUT milliseconds
 */
esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev);

/**
 * @brief Give device mutex
 *
 * @note With @c CONFIG_I2CDEV_NOLOCK enabled this is a no-op that returns
 *       `ESP_OK`.
 *
 * @param dev Pointer to device descriptor
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if @p dev is NULL,
 *         `ESP_ERR_INVALID_STATE` if the device mutex was not created,
 *         `ESP_FAIL` if the mutex was not taken before
 */
esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev);

/**
 * @brief Check the availability of a device on the I2C bus
 *
 * Probes the bus with a bare START/STOP pair and reports whether the device
 * ACKs. No device handle is added to the bus and the per-device mutex is not
 * used, so the device list stays untouched. Two side effects remain: the port
 * is set up if this is the first operation on it (the bus is created and the
 * internal pullups are configured from the descriptor), and the probe always
 * runs at 100 kHz, independent of dev->cfg.master.clk_speed. dev->addr_bit_len
 * is ignored: the address is sent as a 7-bit address.
 *
 * @param dev Pointer to the device descriptor. Pins and address must be configured.
 * @return `ESP_OK` if the device ACKs (is present), an error code otherwise.
 *         `ESP_ERR_INVALID_ARG` if @p dev is NULL or its port is out of range,
 *         `ESP_ERR_INVALID_STATE` if the port was never initialized (call
 *         i2cdev_init() first), `ESP_ERR_TIMEOUT` if the port mutex could not be
 *         taken. If the bus answered but the device did not ACK, the error comes
 *         from the I2C driver (typically `ESP_ERR_NOT_FOUND` or `ESP_ERR_TIMEOUT`).
 */
esp_err_t i2c_dev_check_present(const i2c_dev_t *dev);

/**
 * @brief Check the availability of a device on the I2C bus
 *
 * Legacy-compatible entry point. The legacy implementation issues an operation
 * of @p operation_type; the i2c_master based implementation ignores that
 * argument and forwards the call to i2c_dev_check_present().
 *
 * @param dev Device descriptor.
 * @param operation_type Operation type (I2C_DEV_WRITE or I2C_DEV_READ), ignored
 *        by the i2c_master based implementation.
 * @return Same codes as i2c_dev_check_present(): `ESP_OK` if the device ACKs,
 *         `ESP_ERR_INVALID_ARG` for a NULL descriptor or a port out of range,
 *         `ESP_ERR_INVALID_STATE` if the port was never initialised,
 *         `ESP_ERR_TIMEOUT` if the port mutex could not be taken, otherwise the
 *         error reported by the I2C driver (`ESP_ERR_NOT_FOUND` when the device
 *         does not ACK).
 */
esp_err_t i2c_dev_probe(const i2c_dev_t *dev, i2c_dev_type_t operation_type);

/**
 * @brief Read from device
 *
 * @param dev Pointer to device descriptor
 * @param[in] out_data Data to write before reading, or NULL for a plain read
 * @param out_size Size of data to write before reading
 * @param[out] in_data Buffer to store data read
 * @param in_size Number of bytes to read
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if @p dev or @p in_data is
 *         NULL, if @p in_size is 0 or if the descriptor's port is out of range,
 *         `ESP_ERR_INVALID_STATE` if i2cdev_init() was not called,
 *         `ESP_ERR_TIMEOUT` if the port mutex could not be taken, otherwise the
 *         error reported by the I2C driver
 *
 * @p out_data and @p out_size count only together: if either of them is empty,
 * a plain read is performed and nothing is written first. Reading a register
 * therefore needs both; i2c_dev_read_reg() is the shorthand for it.
 *
 * @note The call is retried up to four times (I2C_MAX_RETRIES + 1) with 20, 40
 *       and 80 ms of backoff, and the device is set up again before every
 *       attempt. The retry loop also covers errors that cannot go away, such as
 *       `ESP_ERR_INVALID_ARG`. In the worst case the call blocks for about four
 *       times @c CONFIG_I2CDEV_TIMEOUT plus the backoff delays.
 */
esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size);

/**
 * @brief Write to device
 *
 * @param dev Pointer to device descriptor
 * @param[in] out_reg Register address to write to, or NULL to write data as is
 * @param out_reg_size Size of register address
 * @param[in] out_data Data to write, or NULL to write only the register
 * @param out_size Size of data to write
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if @p dev is NULL, if both
 *         pairs are empty or if the descriptor's port is out of range,
 *         `ESP_ERR_INVALID_STATE` if i2cdev_init() was not called,
 *         `ESP_ERR_TIMEOUT` if the port mutex could not be taken, otherwise the
 *         error reported by the I2C driver
 *
 * @p out_reg and @p out_reg_size count only together, the same for @p out_data
 * and @p out_size: an empty half of a pair is ignored.
 *
 * @note The retry policy is the same as in i2c_dev_read().
 */
esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size);

/**
 * @brief Read from device register (8-bit register address)
 *
 * @param dev Pointer to device descriptor
 * @param reg Command to write before reading
 * @param[out] data Buffer to store data
 * @param size Number of bytes to read
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if @p dev or @p data is
 *         NULL, if @p size is 0 or if the descriptor's port is out of range,
 *         `ESP_ERR_INVALID_STATE` if i2cdev_init() was not called,
 *         `ESP_ERR_TIMEOUT` if the port mutex could not be taken, otherwise the
 *         error reported by the I2C driver
 *
 * @note The register address is a single byte, sent as a write before the
 *       repeated START of the read. The retry policy is the same as in
 *       i2c_dev_read().
 */
esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *data, size_t size);

/**
 * @brief Write to device register (8-bit register address)
 *
 * @param dev Pointer to device descriptor
 * @param reg Command to write before writing data
 * @param data Buffer with data to write
 * @param size Number of bytes to write
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if @p dev or @p data is
 *         NULL, if @p size is 0 or if the descriptor's port is out of range,
 *         `ESP_ERR_INVALID_STATE` if i2cdev_init() was not called,
 *         `ESP_ERR_TIMEOUT` if the port mutex could not be taken, otherwise the
 *         error reported by the I2C driver
 *
 * @note The register address is a single byte prepended to @p data. To write
 *       only the register byte use i2c_dev_write(dev, &reg, 1, NULL, 0). The
 *       retry policy is the same as in i2c_dev_read().
 */
esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *data, size_t size);

/**
 * @brief Get shared I2C bus handle for external ESP-IDF components
 *
 * This function allows external ESP-IDF components (like esp_lcd) to access
 * the i2cdev-managed I2C bus handle for shared bus operations. The bus lock of
 * the ESP-IDF driver serialises transactions, so i2cdev-based sensors and
 * native ESP-IDF components can share one bus. Only the bus is covered by that:
 * devices added by an external component are unknown to i2cdev, so removing
 * them stays the caller's job (see the cleanup step in the example).
 *
 * @note USAGE PATTERN:
 * 1. Initialize i2cdev subsystem with i2cdev_init()
 * 2. Create at least one i2c_dev_t device on the desired port and perform at least
 *    one I2C operation on it (the bus is created lazily, on the first operation)
 * 3. Call this function to retrieve the bus handle
 * 4. Pass the handle to ESP-IDF components (e.g., esp_lcd_new_panel_io_i2c)
 *
 * @note USAGE EXAMPLE:
 * // 1. Initialize i2cdev
 * i2cdev_init();
 *
 * // 2. Describe a device on the port; the fields are read by the first operation
 * i2c_dev_t sensor = {
 *     .port = I2C_NUM_0,
 *     .addr = 0x29,  // TSL2591 light sensor
 *     .cfg = {
 *         .sda_io_num = GPIO_NUM_21,
 *         .scl_io_num = GPIO_NUM_22,
 *         .master.clk_speed = 400000
 *     }
 * };
 * i2c_dev_create_mutex(&sensor);
 * // The bus does not exist until the first operation on the port: this call
 * // creates it. Its result is not checked on purpose - any transfer that sets
 * // the port up does the job, even one that fails to find the device.
 * (void)i2c_dev_check_present(&sensor);
 *
 * // 3. Get the shared bus handle
 * i2c_master_bus_handle_t bus_handle;
 * ESP_ERROR_CHECK(i2cdev_get_shared_handle(I2C_NUM_0, (void **)&bus_handle));
 *
 * // 4. Use it with esp_lcd or other ESP-IDF components
 * esp_lcd_panel_io_handle_t io_handle;
 * esp_lcd_panel_io_i2c_config_t io_config = {
 *     .dev_addr = 0x3C,  // SSD1306 display
 *     .control_phase_bytes = 1,
 *     .dc_bit_offset = 6,
 *     .lcd_cmd_bits = 8,
 *     .lcd_param_bits = 8,
 * };
 * ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(bus_handle, &io_config, &io_handle));
 *
 * // 5. Cleanup, in this order
 * esp_lcd_panel_io_del(io_handle);          // external device leaves the bus
 * i2c_dev_delete_mutex(&sensor);            // per-device mutex and device handle
 * i2cdev_done();                            // now the bus can be deleted
 *
 * // Both the sensor and display use the same I2C bus. Transactions are
 * // serialised by the ESP-IDF bus lock; per-device serialisation stays the
 * // caller's job: an i2c_dev_t handle must be used under
 * // I2C_DEV_TAKE_MUTEX()/I2C_DEV_GIVE_MUTEX(), and the external component
 * // serialises its own device handle. Delete the external device before
 * // i2cdev_done(): i2cdev does not track it and cannot remove it from the bus.
 *
 * @param port I2C port number (e.g., I2C_NUM_0)
 * @param[out] bus_handle Pointer to store the bus handle
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if port is invalid or bus_handle is NULL
 * @return ESP_ERR_INVALID_STATE if i2cdev_init() was not called or bus for this port is not initialized yet
 * @return ESP_ERR_TIMEOUT if port mutex could not be acquired
 *
 * @warning The returned handle is managed by i2cdev. Do NOT call i2c_del_master_bus()
 *          on it directly: the bus lives until i2cdev_done(), which releases it.
 */
esp_err_t i2cdev_get_shared_handle(i2c_port_t port, void **bus_handle);

/**
 * @brief Take device mutex with error checking
 *
 * @note With @c CONFIG_I2CDEV_NOLOCK enabled this is a no-op that returns
 *       @c ESP_OK; only @p dev is validated.
 */
#define I2C_DEV_TAKE_MUTEX(dev)                                                                                                                                                                        \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        esp_err_t __ = i2c_dev_take_mutex(dev);                                                                                                                                                        \
        if (__ != ESP_OK)                                                                                                                                                                              \
            return __;                                                                                                                                                                                 \
    }                                                                                                                                                                                                  \
    while (0)

/**
 * @brief Give device mutex with error checking
 *
 * @note With @c CONFIG_I2CDEV_NOLOCK enabled this is a no-op that returns
 *       @c ESP_OK; only @p dev is validated.
 */
#define I2C_DEV_GIVE_MUTEX(dev)                                                                                                                                                                        \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        esp_err_t __ = i2c_dev_give_mutex(dev);                                                                                                                                                        \
        if (__ != ESP_OK)                                                                                                                                                                              \
            return __;                                                                                                                                                                                 \
    }                                                                                                                                                                                                  \
    while (0)

/**
 * @brief Execute operation, assuming the device mutex is held. Gives the mutex
 *        ONLY on error.
 *
 * On success the mutex stays taken, so the caller must release it with
 * I2C_DEV_GIVE_MUTEX(). With @c CONFIG_I2CDEV_NOLOCK that release is a no-op.
 */
#define I2C_DEV_CHECK(dev, X)                                                                                                                                                                          \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        esp_err_t ___ = X; /* Execute operation */                                                                                                                                                     \
        if (___ != ESP_OK)                                                                                                                                                                             \
        {                                                                                                                                                                                              \
            /* Give mutex ONLY if error occurred */                                                                                                                                                    \
            i2c_dev_give_mutex(dev);                                                                                                                                                                   \
            return ___;                                                                                                                                                                                \
        }                                                                                                                                                                                              \
    }                                                                                                                                                                                                  \
    while (0)

/**
 * @brief Execute operation, assuming the device mutex is held. Gives the mutex
 *        ONLY on error and logs the failure.
 *
 * Like I2C_DEV_CHECK(), the mutex stays taken on success, so the caller must
 * call I2C_DEV_GIVE_MUTEX(). The failure is logged through @c TAG, which has to
 * be defined in the scope where the macro is used.
 */
#define I2C_DEV_CHECK_LOGE(dev, X, msg, ...)                                                                                                                                                           \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        esp_err_t ___ = X; /* Execute operation */                                                                                                                                                     \
        if (___ != ESP_OK)                                                                                                                                                                             \
        {                                                                                                                                                                                              \
            /* Give mutex ONLY if error occurred */                                                                                                                                                    \
            i2c_dev_give_mutex(dev);                                                                                                                                                                   \
            ESP_LOGE(TAG, msg, ##__VA_ARGS__);                                                                                                                                                         \
            return ___;                                                                                                                                                                                \
        }                                                                                                                                                                                              \
    }                                                                                                                                                                                                  \
    while (0)

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __I2CDEV_H__ */
