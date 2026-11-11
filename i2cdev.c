/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file i2cdev.c
 *
 * ESP-IDF I2C master thread-safe functions for communication with I2C slave
 *
 * Copyright (C) 2018 Ruslan V. Uss <unclerus@gmail.com>
 * Updated 2025 by quinkq to use newer ESP-IDF I2C master driver API
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "i2cdev.h"
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "i2cdev";

// Fallback definition for platforms without 10-bit address support
#ifndef I2C_ADDR_BIT_LEN_10
#define I2C_ADDR_BIT_LEN_10 1
#endif

#define I2C_DEFAULT_FREQ_HZ         400000
#define I2C_MAX_RETRIES             3
#define I2C_RETRY_BASE_DELAY_MS     20
#define I2CDEV_MAX_STACK_ALLOC_SIZE 32 // Stack allocation threshold to avoid heap fragmentation for small buffers

// Node of the per-port list of devices whose handle must be removed on cleanup.
// The list has no fixed size: any number of devices can share a port.
typedef struct i2c_dev_node
{
    struct i2c_dev_node *next;
    i2c_dev_t *dev;
} i2c_dev_node_t;

typedef struct
{
    SemaphoreHandle_t lock;             // Mutex for exclusive access to this port's state
    i2c_master_bus_handle_t bus_handle; // Handle to the initialized I2C master bus
    bool installed;                     // Flag indicating if the bus for this port has been installed
    int sda_pin_current;                // Actual SDA pin the bus was initialized with
    int scl_pin_current;                // Actual SCL pin the bus was initialized with
    i2c_dev_node_t *devices;            // Devices added to this bus, newest first
} i2c_port_state_t;

static i2c_port_state_t i2c_ports[I2C_NUM_MAX] = { 0 };

// Helper to register a device
static esp_err_t register_device(i2c_dev_t *dev)
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;
    int port = dev->port;
    if (port < 0 || port >= I2C_NUM_MAX)
        return ESP_ERR_INVALID_ARG;

    // Note: Port mutex should be held by caller
    i2c_dev_node_t *node = calloc(1, sizeof(i2c_dev_node_t));
    if (!node)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not allocate device tracking node", dev->addr, port);
        return ESP_ERR_NO_MEM;
    }

    node->dev = dev;
    node->next = i2c_ports[port].devices;
    i2c_ports[port].devices = node;
    ESP_LOGV(TAG, "[0x%02x at %d] Registered device for cleanup", dev->addr, port);
    return ESP_OK;
}

// Helper to drop a device from the per-port list. The node is allocated by
// register_device(), so it must be freed here.
static void deregister_device(i2c_dev_t *dev)
{
    i2c_dev_node_t **link = &i2c_ports[dev->port].devices;
    while (*link)
    {
        if ((*link)->dev == dev)
        {
            i2c_dev_node_t *node = *link;
            *link = node->next;
            free(node);
            ESP_LOGV(TAG, "[0x%02x at %d] Deregistered device", dev->addr, dev->port);
            return;
        }
        link = &(*link)->next;
    }
}

// Helper to release the whole per-port device list
static void free_device_list(i2c_port_state_t *port_state)
{
    i2c_dev_node_t *node = port_state->devices;
    while (node)
    {
        i2c_dev_node_t *next = node->next;
        free(node);
        node = next;
    }
    port_state->devices = NULL;
}

esp_err_t i2cdev_init(void)
{
    ESP_LOGV(TAG, "Initializing I2C subsystem...");

    // There is no separate "initialized" flag: the state of the subsystem is
    // derived from i2c_ports[]. A port that owns a mutex is usable, so calling
    // i2cdev_init() again only creates the missing mutexes. This makes
    // init -> done -> init work, and a partially failed init can be retried.
    for (int i = 0; i < I2C_NUM_MAX; i++)
    {
        if (!i2c_ports[i].lock)
        {
            i2c_ports[i].lock = xSemaphoreCreateMutex();
            if (!i2c_ports[i].lock)
            {
                ESP_LOGE(TAG, "Could not create port mutex %d", i);
                return ESP_ERR_NO_MEM;
            }
            ESP_LOGV(TAG, "Created port mutex %d", i);
        }

        // Reset a port only while no bus is installed. A bus handle that is
        // still alive (i2cdev_done() failed to delete it) must be kept,
        // otherwise the platform I2C port stays occupied while i2cdev loses
        // its only reference to it.
        if (!i2c_ports[i].installed)
        {
            i2c_ports[i].bus_handle = NULL;
            i2c_ports[i].sda_pin_current = -1;
            i2c_ports[i].scl_pin_current = -1;
            free_device_list(&i2c_ports[i]);
        }
    }

    ESP_LOGV(TAG, "I2C subsystem initialized.");
    return ESP_OK;
}

esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev)
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;

#if !CONFIG_I2CDEV_NOLOCK
    ESP_LOGV(TAG, "[0x%02x at %d] Creating device mutex...", dev->addr, dev->port);
    if (dev->mutex)
    {
        ESP_LOGW(TAG, "[0x%02x at %d] device mutex already exists (Handle: %p)", dev->addr, dev->port, dev->mutex);
        return ESP_OK; // Already created
    }

    dev->mutex = xSemaphoreCreateMutex();
    if (!dev->mutex)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not create device mutex", dev->addr, dev->port);
        return ESP_ERR_NO_MEM; // Use ESP_ERR_NO_MEM for memory allocation failures
    }

    ESP_LOGV(TAG, "[0x%02x at %d] Device mutex created (Handle: %p)", dev->addr, dev->port, dev->mutex);
#else
    ESP_LOGV(TAG, "[0x%02x at %d] Mutex creation skipped (CONFIG_I2CDEV_NOLOCK=1)", dev->addr, dev->port);
#endif

    // Address bit length is not a locking concern: normalise it in both modes.
    // The device itself is registered on the bus (and therefore tracked for
    // cleanup) in i2c_setup_device(), where the handle is actually created.
    if (dev->addr_bit_len != I2C_ADDR_BIT_LEN_7
#if SOC_I2C_SUPPORT_10BIT_ADDR
            && dev->addr_bit_len != I2C_ADDR_BIT_LEN_10
#endif
       )
    {
        ESP_LOGV(TAG, "[0x%02x at %d] Setting default 7-bit address format", dev->addr, dev->port);
        dev->addr_bit_len = I2C_ADDR_BIT_LEN_7;
    }

    return ESP_OK;
}

// Drops the per-device mutex. Bus-level cleanup does not depend on it, and
// with CONFIG_I2CDEV_NOLOCK=1 no per-device mutex is ever created.
static void delete_device_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
    if (dev->mutex)
    {
        vSemaphoreDelete(dev->mutex);
        dev->mutex = NULL;
    }
#else
    (void)dev;
#endif
}

esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev)
{
    esp_err_t result = ESP_OK;

    if (!dev)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] Deleting device mutex and cleaning up resources", dev->addr, dev->port);

    // The bus lifecycle (device handle, the per-port device list and the port's
    // master bus) is handled in both locking modes: disabling device mutexes
    // must not disable cleanup.
    if (dev->port < 0 || dev->port >= I2C_NUM_MAX || !i2c_ports[dev->port].lock)
    {
        if (dev->port < 0 || dev->port >= I2C_NUM_MAX)
        {
            ESP_LOGE(TAG, "[0x%02x at %d] Invalid port %d, skipping port-scoped cleanup", dev->addr, dev->port, dev->port);
        }
        else
        {
            ESP_LOGW(TAG, "[0x%02x at %d] I2C subsystem not initialized, skipping port-scoped cleanup", dev->addr, dev->port);
        }
        delete_device_mutex(dev);
        return ESP_OK;
    }

    i2c_port_state_t *port_state = &i2c_ports[dev->port];

    // Take the port mutex once and run the whole cleanup under it.
    if (xSemaphoreTake(port_state->lock, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)) != pdTRUE)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not take port mutex for cleanup", dev->addr, dev->port);
        return ESP_ERR_TIMEOUT;
    }

    // Capture whether this device was actually added to the bus before
    // we clear the field.
    bool was_on_bus = dev->dev_handle;

    // 1) Remove the device from the bus, if it was added.
    // The handle is released only when the driver reports success: on failure
    // the device stays attached to the bus, and keeping the pointer allows a
    // repeated call to retry the removal instead of silently losing the handle.
    if (was_on_bus)
    {
        ESP_LOGV(TAG, "[0x%02x at %d] Removing device handle %p from bus", dev->addr, dev->port, dev->dev_handle);
        esp_err_t rm_res = i2c_master_bus_rm_device((i2c_master_dev_handle_t)dev->dev_handle);
        if (rm_res != ESP_OK)
        {
            ESP_LOGW(TAG, "[0x%02x at %d] Failed to remove device handle: %s", dev->addr, dev->port, esp_err_to_name(rm_res));
            result = rm_res;
        }
        else
        {
            dev->dev_handle = NULL;
        }
    }

    // 2) Deregister the device from the per-port list under the same port mutex
    deregister_device(dev);

    // The bus itself is not deleted here: it lives from i2cdev_init() (first
    // operation) until i2cdev_done(). Removing the last known device must not
    // tear the bus down, because other consumers may hold the shared bus
    // handle (i2cdev_get_shared_handle()) or keep devices that this library
    // does not track.

    xSemaphoreGive(port_state->lock);

    // 4) Finally, drop the per-device mutex
    delete_device_mutex(dev);

    return result;
}

esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev)
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;

#if !CONFIG_I2CDEV_NOLOCK
    ESP_LOGV(TAG, "[0x%02x at %d] Attempting to take device mutex (Handle: %p)...", dev->addr, dev->port, dev->mutex);
    if (!dev->mutex)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Attempt to take NULL device mutex!", dev->addr, dev->port);
        return ESP_ERR_INVALID_STATE; // Mutex doesn't exist
    }

    TickType_t timeout_ticks = pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT);
    ESP_LOGV(TAG, "[0x%02x at %d] Taking device mutex with timeout %d ms (%lu ticks)", dev->addr, dev->port, CONFIG_I2CDEV_TIMEOUT, (unsigned long)timeout_ticks);
    if (!xSemaphoreTake(dev->mutex, timeout_ticks))
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not take device mutex (Timeout after %d ms)", dev->addr, dev->port, CONFIG_I2CDEV_TIMEOUT);
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGV(TAG, "[0x%02x at %d] Device mutex taken successfully.", dev->addr, dev->port);
#else
    ESP_LOGV(TAG, "[0x%02x at %d] Mutex take skipped (CONFIG_I2CDEV_NOLOCK=1)", dev->addr, dev->port);
#endif
    return ESP_OK;
}

esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev)
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;

#if !CONFIG_I2CDEV_NOLOCK
    ESP_LOGV(TAG, "[0x%02x at %d] Giving device mutex (Handle: %p)...", dev->addr, dev->port, dev->mutex);
    if (!dev->mutex)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Attempt to give NULL device mutex!", dev->addr, dev->port);
        return ESP_ERR_INVALID_STATE;
    }

    if (!xSemaphoreGive(dev->mutex))
    {
        // This case should ideally not happen if the mutex was taken correctly
        ESP_LOGE(TAG, "[0x%02x at %d] Could not give device mutex (Was it taken?) (Handle: %p)", dev->addr, dev->port, dev->mutex);
        return ESP_FAIL;
    }
    ESP_LOGV(TAG, "[0x%02x at %d] Device mutex given successfully.", dev->addr, dev->port);
#else
    ESP_LOGV(TAG, "[0x%02x at %d] Mutex give skipped (CONFIG_I2CDEV_NOLOCK=1)", dev->addr, dev->port);
#endif
    return ESP_OK;
}

// i2c_setup_port: Initializes the I2C master bus for a given port if not already done.
// It uses pin configurations from dev->cfg.sda_io_num and dev->cfg.scl_io_num.
// The pins for a port are fixed after the first device initializes it.
static esp_err_t i2c_setup_port(i2c_dev_t *dev) // dev is non-const to update dev->sda_pin, dev->scl_pin
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;
    if (dev->port < 0 || dev->port >= I2C_NUM_MAX)
    {
        ESP_LOGE(TAG, "Invalid I2C port number: %d", dev->port);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t res = ESP_OK;
    i2c_port_state_t *port_state = &i2c_ports[dev->port];

    // Check if i2cdev subsystem is initialized
    if (!port_state->lock)
    {
        ESP_LOGE(TAG, "[Port %d] I2C subsystem not initialized - call i2cdev_init() first", dev->port);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGV(TAG, "[Port %d] Setup request for device 0x%02x", dev->port, dev->addr);
    if (xSemaphoreTake(port_state->lock, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)) != pdTRUE)
    {
        ESP_LOGE(TAG, "[Port %d] Could not take port mutex for setup", dev->port);
        return ESP_ERR_TIMEOUT;
    }

    if (!port_state->installed)
    {
        // Pin Selection Logic: Use device-specified pins, fallback to Kconfig defaults if -1
        gpio_num_t sda_pin = (dev->cfg.sda_io_num == (gpio_num_t) -1) ? (gpio_num_t)CONFIG_I2CDEV_DEFAULT_SDA_PIN : dev->cfg.sda_io_num;
        gpio_num_t scl_pin = (dev->cfg.scl_io_num == (gpio_num_t) -1) ? (gpio_num_t)CONFIG_I2CDEV_DEFAULT_SCL_PIN : dev->cfg.scl_io_num;

        // Validate pins (basic check, gpio_is_valid_gpio could be used for more robust check)
        if (sda_pin < 0 || scl_pin < 0)
        {
            ESP_LOGE(TAG, "[Port %d] Invalid SCL/SDA pins: SDA=%d, SCL=%d. Check driver or Kconfig defaults.", dev->port, sda_pin, scl_pin);
            xSemaphoreGive(port_state->lock);
            return ESP_ERR_INVALID_ARG;
        }

        // SDA and SCL must be different pins. Zero-initialized i2c_dev_t leaves
        // both at GPIO0, which would configure one pin as SDA and SCL at once.
        if (sda_pin == scl_pin)
        {
            ESP_LOGE(TAG, "[Port %d] SDA and SCL are the same pin (GPIO%d). Set dev->cfg.sda_io_num/dev->cfg.scl_io_num or Kconfig defaults.", dev->port, (int)sda_pin);
            xSemaphoreGive(port_state->lock);
            return ESP_ERR_INVALID_ARG;
        }

        /*
         * OPTIONAL I2C PULLUP AUTO-CONFIGURATION
         *
         * By default: Uses whatever sda_pullup_en/scl_pullup_en you set (usually false)
         *
         * When CONFIG_I2CDEV_AUTO_ENABLE_PULLUPS=y: If both pullup flags are false,
         * automatically change them to true to enable internal pullups (~45kΩ).
         *
         * Manual pullup configuration:
         * - Set sda_pullup_en=true, scl_pullup_en=true for internal pullups
         * - Set sda_pullup_en=false, scl_pullup_en=false for external pullups
         *
         * The IDF bus config has a single `enable_internal_pullup` flag for both
         * lines, so a mixed request (only SDA or only SCL) is applied per line with
         * gpio_set_pull_mode() right after the bus is created.
         */

        // Read user's pullup configuration (default false if not set)
        bool sda_pullup = dev->cfg.sda_pullup_en;
        bool scl_pullup = dev->cfg.scl_pullup_en;

#if CONFIG_I2CDEV_AUTO_ENABLE_PULLUPS
        // CONFIG_I2CDEV_AUTO_ENABLE_PULLUPS=y: If user didn't configure pullups, enable them automatically
        if (!sda_pullup && !scl_pullup)
        {
            sda_pullup = true;
            scl_pullup = true;
            ESP_LOGI(TAG, "[Port %d] Auto-enabling internal pullups (CONFIG_I2CDEV_AUTO_ENABLE_PULLUPS=y)", dev->port);
        }
#endif

        ESP_LOGI(TAG,
                 "[Port %d] First initialization. Configuring bus with SDA=%d, SCL=%d (Pullups "
                 "SCL:%d SDA:%d)",
                 dev->port, sda_pin, scl_pin, scl_pullup, sda_pullup);

        i2c_master_bus_config_t bus_config =
        {
            .i2c_port = dev->port,
            .sda_io_num = sda_pin,
            .scl_io_num = scl_pin,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = (sda_pullup && scl_pullup),
            // Bus speed is not set here. It's per-device or a global target for the bus can be set
            // if desired, but i2c_master supports per-device speeds.
        };
        // LP_I2C requires LP_I2C_SCLK_DEFAULT; HP I2C uses the standard default.
        // clk_source and lp_source_clk are union members in i2c_master_bus_config_t,
        // but LP_I2C_SCLK_DEFAULT is not a valid i2c_clock_source_t value, so
        // i2c_new_master_bus returns ESP_ERR_NOT_SUPPORTED if we pass I2C_CLK_SRC_DEFAULT
        // for an LP port.
#if SOC_LP_I2C_SUPPORTED
        if (dev->port == (i2c_port_t)LP_I2C_NUM_0)
            bus_config.clk_source = LP_I2C_SCLK_DEFAULT;
        else
#endif
            bus_config.clk_source = I2C_CLK_SRC_DEFAULT;

        res = i2c_new_master_bus(&bus_config, &port_state->bus_handle);
        if (res == ESP_OK)
        {
            // The IDF bus config enables internal pullups on both lines at once,
            // so a mixed request (only SDA or only SCL) is applied per line here.
            if (sda_pullup != scl_pullup)
            {
                gpio_num_t pullup_pin = sda_pullup ? sda_pin : scl_pin;
                esp_err_t pull_res = gpio_set_pull_mode(pullup_pin, GPIO_PULLUP_ONLY);
                if (pull_res == ESP_OK)
                    ESP_LOGI(TAG, "[Port %d] Internal pullup enabled on GPIO%d only (SDA:%d, SCL:%d)", dev->port, (int)pullup_pin, sda_pullup, scl_pullup);
                else
                    ESP_LOGW(TAG, "[Port %d] Could not enable internal pullup on GPIO%d: %s", dev->port, (int)pullup_pin, esp_err_to_name(pull_res));
            }

            port_state->installed = true;
            port_state->sda_pin_current = sda_pin;
            port_state->scl_pin_current = scl_pin;
            dev->sda_pin = sda_pin; // Update dev struct with actual pins used
            dev->scl_pin = scl_pin;
            ESP_LOGI(TAG, "[Port %d] Successfully installed I2C master bus (Handle: %p).", dev->port, port_state->bus_handle);
        }
        else
        {
            ESP_LOGE(TAG, "[Port %d] Failed to create master bus: %d (%s)", dev->port, res, esp_err_to_name(res));
            port_state->installed = false;
            port_state->bus_handle = NULL;
            port_state->sda_pin_current = -1;
            port_state->scl_pin_current = -1;
        }
    }
    else
    {
        ESP_LOGV(TAG, "[Port %d] Port already installed (SDA=%d, SCL=%d, Handle: %p).", dev->port, port_state->sda_pin_current, port_state->scl_pin_current, port_state->bus_handle);
        // Pin Consistency Check: For subsequent devices, ensure pins match already-configured bus
        gpio_num_t sda_desired = (dev->cfg.sda_io_num == (gpio_num_t) -1) ? (gpio_num_t)port_state->sda_pin_current : dev->cfg.sda_io_num;
        gpio_num_t scl_desired = (dev->cfg.scl_io_num == (gpio_num_t) -1) ? (gpio_num_t)port_state->scl_pin_current : dev->cfg.scl_io_num;

        if (sda_desired != port_state->sda_pin_current || scl_desired != port_state->scl_pin_current)
        {
            ESP_LOGE(TAG,
                     "[Port %d] Pin mismatch for device 0x%02x! Bus on SDA=%d,SCL=%d. Device wants "
                     "SDA=%d,SCL=%d",
                     dev->port, dev->addr, port_state->sda_pin_current, port_state->scl_pin_current, sda_desired, scl_desired);
            res = ESP_ERR_INVALID_STATE; // Cannot change pins for an installed bus
        }
        else
        {
            dev->sda_pin = port_state->sda_pin_current; // Update dev struct with actual pins used
            dev->scl_pin = port_state->scl_pin_current;
        }
        // The bus outlives individual devices: it is deleted in i2cdev_done()
    }

    xSemaphoreGive(port_state->lock);
    ESP_LOGV(TAG, "[Port %d] Port setup finished with res %d.", dev->port, res);
    return res;
}

// i2c_setup_device: Ensures port is set up and adds the device to the bus if not already added.
// It also registers the device in the port's device list for cleanup purposes.
static esp_err_t i2c_setup_device(i2c_dev_t *dev) // dev is non-const - modifies dev->dev_handle, dev->addr_bit_len
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] Setting up device context...", dev->addr, dev->port);

    esp_err_t res = i2c_setup_port(dev);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Port setup failed during device setup: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));
        return res;
    }

    // If addr_bit_len is not set (e.g. 0, which is invalid for i2c_addr_bit_len_t enum), default to
    // 7-bit. Modified to conditionally check for I2C_ADDR_BIT_LEN_10 based on hardware support
    if (dev->addr_bit_len != I2C_ADDR_BIT_LEN_7
#if SOC_I2C_SUPPORT_10BIT_ADDR
            && dev->addr_bit_len != I2C_ADDR_BIT_LEN_10
#endif
       )
    {
        ESP_LOGD(TAG, "[0x%02x at %d] addr_bit_len not explicitly set, defaulting to 7-bit.", dev->addr, dev->port);
        dev->addr_bit_len = I2C_ADDR_BIT_LEN_7;
    }

    // Only warn about address size if the device is actually using 10-bit addressing
    if (dev->addr_bit_len == I2C_ADDR_BIT_LEN_7 && dev->addr > 0x7F)
    {
        ESP_LOGW(TAG,
                 "[0x%02x at %d] Device address > 0x7F but addr_bit_len is 7-bit. Ensure address "
                 "is correct.",
                 dev->addr, dev->port);
    }

#if !defined(SOC_I2C_SUPPORT_10BIT_ADDR) || !SOC_I2C_SUPPORT_10BIT_ADDR
    // On platforms without 10-bit support, force 7-bit addressing regardless of user setting
    if (dev->addr_bit_len == I2C_ADDR_BIT_LEN_10)
    {
        ESP_LOGW(TAG, "[0x%02x at %d] 10-bit addressing not supported on this platform, forcing 7-bit mode", dev->addr, dev->port);
        dev->addr_bit_len = I2C_ADDR_BIT_LEN_7;
    }
#endif

    i2c_port_state_t *port_state = &i2c_ports[dev->port];

    // Check if i2cdev subsystem is initialized
    if (!port_state->lock)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] I2C subsystem not initialized - call i2cdev_init() first", dev->addr, dev->port);
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(port_state->lock, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)) != pdTRUE)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not take port mutex for device add", dev->addr, dev->port);
        return ESP_ERR_TIMEOUT;
    }

    // Decide under the port mutex whether the device still has to be added:
    // i2c_master_bus_add_device() does not detect duplicates, so checking the
    // handle before taking the mutex would let two tasks add the same device
    // twice and leaking the first handle.
    if (!dev->dev_handle)
    {
        if (!port_state->installed || !port_state->bus_handle)
        {
            ESP_LOGE(TAG, "[0x%02x at %d] Cannot add device, bus for port %d not ready!", dev->addr, dev->port, dev->port);
            xSemaphoreGive(port_state->lock);
            return ESP_ERR_INVALID_STATE;
        }

        ESP_LOGV(TAG, "[0x%02x at %d] Adding device to bus (Bus Handle: %p)...", dev->addr, dev->port, port_state->bus_handle);

        uint32_t effective_dev_speed = dev->cfg.master.clk_speed;
        if (effective_dev_speed == 0)
        {
            ESP_LOGW(TAG,
                     "[0x%02x at %d] Device speed (dev->cfg.master.clk_speed) is 0, using default: "
                     "%" PRIu32 " Hz",
                     dev->addr, dev->port, (uint32_t)I2C_DEFAULT_FREQ_HZ);
            effective_dev_speed = I2C_DEFAULT_FREQ_HZ;
        }

        i2c_device_config_t dev_config =
        {
            // Use the possibly modified addr_bit_len that respects hardware capabilities
            .dev_addr_length = dev->addr_bit_len,
            .device_address = dev->addr,
            .scl_speed_hz = effective_dev_speed,
            .flags.disable_ack_check = false,
        };

        res = i2c_master_bus_add_device(port_state->bus_handle, &dev_config, (i2c_master_dev_handle_t *)&dev->dev_handle);
        if (res == ESP_OK)
        {
            // Track the device for cleanup while the port mutex is held: the
            // registration must cover exactly the devices that have a handle.
            // A device that cannot be tracked would survive i2cdev_done() and
            // block bus deletion forever, so this failure is fatal.
            res = register_device(dev);
            if (res != ESP_OK)
            {
                ESP_LOGE(TAG, "[0x%02x at %d] Failed to register device: %s - removing it from the bus", dev->addr, dev->port, esp_err_to_name(res));
                esp_err_t rm_res = i2c_master_bus_rm_device((i2c_master_dev_handle_t)dev->dev_handle);
                if (rm_res != ESP_OK)
                {
                    ESP_LOGE(TAG, "[0x%02x at %d] Failed to remove untracked device handle: %s", dev->addr, dev->port, esp_err_to_name(rm_res));
                }
                dev->dev_handle = NULL;
            }
            else
            {
                ESP_LOGI(TAG, "[0x%02x at %d] Device added successfully (Device Handle: %p, Speed: %" PRIu32 " Hz).", dev->addr, dev->port, dev->dev_handle, effective_dev_speed);
            }
        }
        else
        {
            ESP_LOGE(TAG, "[0x%02x at %d] Failed to add device to bus: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));
            dev->dev_handle = NULL;
        }
    }
    else
    {
        ESP_LOGV(TAG, "[0x%02x at %d] Device handle %p already exists. Skipping add.", dev->addr, dev->port, dev->dev_handle);
        res = ESP_OK;
    }

    xSemaphoreGive(port_state->lock);

    ESP_LOGV(TAG, "[0x%02x at %d] Device context setup finished with res %d.", dev->addr, dev->port, res);
    return res;
}

// Helper function with retry mechanism for I2C operations
static esp_err_t i2c_do_operation_with_retry(i2c_dev_t *dev, esp_err_t (*i2c_func)(i2c_master_dev_handle_t, const void *, size_t, void *, size_t, int), const void *write_buffer, size_t write_size,
                                             void *read_buffer, size_t read_size)
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;
    esp_err_t res = ESP_FAIL;
    int retry = 0;
    int timeout_ms = CONFIG_I2CDEV_TIMEOUT;

    ESP_LOGV(TAG, "[0x%02x at %d] Performing I2C operation (timeout %d ms)...", dev->addr, dev->port, timeout_ms);

    while (retry <= I2C_MAX_RETRIES)
    {
        // Ensure the device is set up before each attempt: the bus may have
        // been installed after a previous failure, or the port may have been
        // reconfigured. An existing handle is reused as is.
        res = i2c_setup_device(dev);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "[0x%02x at %d] Device setup failed (Try %d): %d (%s). Retrying setup...", dev->addr, dev->port, retry, res, esp_err_to_name(res));
            // No point continuing this attempt if setup fails, but the loop will retry setup.
            vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_BASE_DELAY_MS * (1 << (retry))));
            retry++;
            continue;
        }
        if (!dev->dev_handle)
        {
            ESP_LOGE(TAG,
                     "[0x%02x at %d] Device handle is NULL after setup (Try %d)! Cannot perform "
                     "operation.",
                     dev->addr, dev->port, retry);
            // This indicates a persistent problem with adding the device to the bus.
            // No point retrying the i2c_func if handle is null.
            res = ESP_ERR_INVALID_STATE;
            vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_BASE_DELAY_MS * (1 << (retry))));
            retry++;
            continue;
        }

        ESP_LOGV(TAG, "[0x%02x at %d] Attempting I2C op (Try %d, Handle %p)", dev->addr, dev->port, retry, dev->dev_handle);
        res = i2c_func(dev->dev_handle, write_buffer, write_size, read_buffer, read_size, timeout_ms);

        if (res == ESP_OK)
        {
            ESP_LOGV(TAG, "[0x%02x at %d] I2C operation successful (Try %d).", dev->addr, dev->port, retry);
            return ESP_OK;
        }

        ESP_LOGW(TAG, "[0x%02x at %d] I2C op failed (Try %d, Handle %p): %d (%s).", dev->addr, dev->port, retry, dev->dev_handle, res, esp_err_to_name(res));

        // The device handle is deliberately left untouched here. It is created
        // in i2c_setup_device() and destroyed in i2c_dev_delete_mutex(), both
        // of which run under the port mutex and keep the per-port device list
        // consistent. Releasing it from a transfer error path would (a) race
        // with tasks still using the handle and (b) leave a stale entry in the
        // per-port device list. Errors are reported to the caller instead.

        retry++;
        if (retry <= I2C_MAX_RETRIES)
        {
            vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_BASE_DELAY_MS * (1 << retry))); // Exponential backoff
            ESP_LOGW(TAG, "[0x%02x at %d] Retrying operation...", dev->addr, dev->port);
        }
    }

    ESP_LOGE(TAG, "[0x%02x at %d] I2C operation failed after %d retries. Last error: %d (%s)", dev->addr, dev->port, I2C_MAX_RETRIES + 1, res, esp_err_to_name(res));
    return res;
}

// Wrapper functions for the I2C master API to use with the retry mechanism
// i2c_do_operation_with_retry() needs a unified function signature for all I2C operations
static esp_err_t i2c_master_transmit_wrapper(i2c_master_dev_handle_t handle, const void *write_buffer, size_t write_size, void *read_buffer, size_t read_size, int timeout_ms)
{
    return i2c_master_transmit(handle, write_buffer, write_size, timeout_ms);
}

static esp_err_t i2c_master_receive_wrapper(i2c_master_dev_handle_t handle, const void *write_buffer, size_t write_size, void *read_buffer, size_t read_size, int timeout_ms)
{
    return i2c_master_receive(handle, read_buffer, read_size, timeout_ms);
}

static esp_err_t i2c_master_transmit_receive_wrapper(i2c_master_dev_handle_t handle, const void *write_buffer, size_t write_size, void *read_buffer, size_t read_size, int timeout_ms)
{
    return i2c_master_transmit_receive(handle, write_buffer, write_size, read_buffer, read_size, timeout_ms);
}

esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
    if (!dev || !in_data || !in_size)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] i2c_dev_read called (out_size: %u, in_size: %u)", dev->addr, dev->port, out_size, in_size);

    esp_err_t result = i2c_do_operation_with_retry((i2c_dev_t *)dev, // Cast to non-const for i2c_setup_device internal modifications
                                                   out_data && out_size ? i2c_master_transmit_receive_wrapper : i2c_master_receive_wrapper, out_data, out_size, in_data, in_size);

    ESP_LOGV(TAG, "[0x%02x at %d] i2c_dev_read result: %s (%d)", dev->addr, dev->port, esp_err_to_name(result), result);
    return result;
}

esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;
    if (!(out_reg && out_reg_size) && !(out_data && out_size))
        return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] i2c_dev_write called (reg_size: %u, data_size: %u)", dev->addr, dev->port, out_reg_size, out_size);

    esp_err_t res;
    if (out_reg && out_reg_size && out_data && out_size)
    {
        size_t total_write_size = out_reg_size + out_size;

        // Check for overflow before proceeding
        if (total_write_size < out_reg_size || total_write_size < out_size)
        {
            ESP_LOGE(TAG, "[0x%02x at %d] Write size overflow: reg_size=%u + data_size=%u", dev->addr, dev->port, out_reg_size, out_size);
            return ESP_ERR_INVALID_ARG;
        }

        // Use stack for small buffers to avoid heap fragmentation
        if (total_write_size <= I2CDEV_MAX_STACK_ALLOC_SIZE)
        {
            // Use stack allocation for small buffers
            uint8_t stack_buf[I2CDEV_MAX_STACK_ALLOC_SIZE];
            memcpy(stack_buf, out_reg, out_reg_size);
            memcpy(stack_buf + out_reg_size, out_data, out_size);
            res = i2c_do_operation_with_retry((i2c_dev_t *)dev, i2c_master_transmit_wrapper, stack_buf, total_write_size, NULL, 0);
        }
        else
        {
            uint8_t *heap_buf = malloc(total_write_size);
            if (!heap_buf)
            {
                ESP_LOGE(TAG, "[0x%02x at %d] Failed to allocate %u bytes for write", dev->addr, dev->port, total_write_size);
                return ESP_ERR_NO_MEM;
            }
            memcpy(heap_buf, out_reg, out_reg_size);
            memcpy(heap_buf + out_reg_size, out_data, out_size);
            res = i2c_do_operation_with_retry((i2c_dev_t *)dev, i2c_master_transmit_wrapper, heap_buf, total_write_size, NULL, 0);
            free(heap_buf); // Free buffer regardless of operation result
        }
    }
    else if (out_reg && out_reg_size)
    {
        res = i2c_do_operation_with_retry((i2c_dev_t *)dev, i2c_master_transmit_wrapper, out_reg, out_reg_size, NULL, 0);
    }
    else if (out_data && out_size)
    {
        res = i2c_do_operation_with_retry((i2c_dev_t *)dev, i2c_master_transmit_wrapper, out_data, out_size, NULL, 0);
    }
    else
    {
        return ESP_ERR_INVALID_ARG; // Shouldn't reach here given the earlier check
    }

    ESP_LOGV(TAG, "[0x%02x at %d] i2c_dev_write result: %s (%d)", dev->addr, dev->port, esp_err_to_name(res), res);
    return res;
}

esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *data, size_t size)
{
    // Same contract as the legacy driver: a read without a destination buffer
    // or without a size is a caller error, not a silent no-op.
    if (!dev || !data || !size)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] i2c_dev_read_reg called (reg: 0x%02x, size: %u)", dev->addr, dev->port, reg, size);
    return i2c_dev_read(dev, &reg, 1, data, size);
}

esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *data, size_t size)
{
    // Same contract as the legacy driver: writing a register without data is a
    // caller error. Use i2c_dev_write(dev, &reg, 1, NULL, 0) to write a bare
    // register byte.
    if (!dev || !data || !size)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] i2c_dev_write_reg called (reg: 0x%02x, size: %u)", dev->addr, dev->port, reg, size);
    return i2c_dev_write(dev, &reg, 1, data, size);
}

esp_err_t i2c_dev_check_present(const i2c_dev_t *dev_const)
{
    if (!dev_const)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] Probing device presence...", dev_const->addr, dev_const->port);

    // Cast to non-const for i2c_setup_port (which may modify internal state)
    i2c_dev_t *dev = (i2c_dev_t *)dev_const;

    // Ensure the I2C port is set up before probing
    esp_err_t setup_res = i2c_setup_port(dev);
    if (setup_res != ESP_OK)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Failed to setup port for probe: %s", dev_const->addr, dev_const->port, esp_err_to_name(setup_res));
        return setup_res;
    }

    // Now probe using the initialized bus
    if (dev_const->port >= 0 && dev_const->port < I2C_NUM_MAX && i2c_ports[dev_const->port].lock)
    {
        if (xSemaphoreTake(i2c_ports[dev_const->port].lock, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)) == pdTRUE)
        {
            if (i2c_ports[dev_const->port].installed && i2c_ports[dev_const->port].bus_handle)
            {
                // Use ESP-IDF's built-in probe function - completely non-intrusive
                esp_err_t probe_res = i2c_master_probe(i2c_ports[dev_const->port].bus_handle, dev_const->addr, CONFIG_I2CDEV_TIMEOUT);
                xSemaphoreGive(i2c_ports[dev_const->port].lock);

                if (probe_res == ESP_OK)
                {
                    ESP_LOGV(TAG, "[0x%02x at %d] Device probe successful - device present", dev_const->addr, dev_const->port);
                    return ESP_OK;
                }
                else
                {
                    ESP_LOGV(TAG, "[0x%02x at %d] Device probe failed: %s", dev_const->addr, dev_const->port, esp_err_to_name(probe_res));
                    return probe_res;
                }
            }
            else
            {
                xSemaphoreGive(i2c_ports[dev_const->port].lock);
                ESP_LOGW(TAG, "[0x%02x at %d] Cannot probe - bus not ready on port %d", dev_const->addr, dev_const->port, dev_const->port);
                return ESP_ERR_INVALID_STATE;
            }
        }
        else
        {
            ESP_LOGE(TAG, "[0x%02x at %d] Could not take port mutex for probe", dev_const->addr, dev_const->port);
            return ESP_ERR_TIMEOUT;
        }
    }
    else
    {
        // The port range is already validated in i2c_setup_port(), so the only
        // way here is a port that was never initialized by i2cdev_init().
        ESP_LOGE(TAG, "[0x%02x at %d] Port is not initialized, call i2cdev_init() first", dev_const->addr, dev_const->port);
        return ESP_ERR_INVALID_STATE;
    }
}

// Compatibility wrapper for legacy code that still calls i2c_dev_probe
// The new driver implementation uses i2c_master_probe which doesn't need operation_type
esp_err_t i2c_dev_probe(const i2c_dev_t *dev, i2c_dev_type_t operation_type)
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] Legacy probe called (operation_type %d), redirecting to new implementation", dev->addr, dev->port, operation_type);

    return i2c_dev_check_present(dev);
}

// Clean up function to be called at application exit
esp_err_t i2cdev_done(void)
{
    esp_err_t result = ESP_OK;
    ESP_LOGV(TAG, "Cleaning up I2C subsystem (i2c_master)...");
    for (int i = 0; i < I2C_NUM_MAX; i++)
    {
        if (!i2c_ports[i].lock)
            continue;

        ESP_LOGV(TAG, "[Port %d] Cleaning up port...", i);
        if (xSemaphoreTake(i2c_ports[i].lock, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)) != pdTRUE)
        {
            // Another task owns the port. Neither its bus nor its mutex can be
            // touched safely; leave the port intact so that a later call can
            // finish the cleanup.
            ESP_LOGE(TAG, "[Port %d] Could not take port mutex for cleanup", i);
            if (result == ESP_OK)
                result = ESP_ERR_TIMEOUT;
            continue;
        }

        bool released = true;
        if (i2c_ports[i].installed)
        {
            ESP_LOGV(TAG, "[Port %d] Removing active devices before deleting bus...", i);
            // Remove all registered devices for this port from the bus. A
            // handle is dropped only after a successful removal: on failure
            // the device is still attached, and keeping the pointer allows a
            // retry instead of blocking bus deletion forever.
            for (i2c_dev_node_t *node = i2c_ports[i].devices; node; node = node->next)
            {
                i2c_dev_t *dev_ptr = node->dev;
                if (dev_ptr && dev_ptr->dev_handle)
                {
                    ESP_LOGV(TAG, "[Port %d] Removing device 0x%02x (Handle %p)", i, dev_ptr->addr, dev_ptr->dev_handle);
                    esp_err_t rm_res = i2c_master_bus_rm_device(dev_ptr->dev_handle);
                    if (rm_res != ESP_OK)
                    {
                        ESP_LOGE(TAG, "[Port %d] Failed to remove device 0x%02x handle: %d", i, dev_ptr->addr, rm_res);
                        if (result == ESP_OK)
                            result = rm_res; // Report first error
                        released = false;
                    }
                    else
                    {
                        dev_ptr->dev_handle = NULL;
                    }
                }
            }

            if (released)
            {
                ESP_LOGV(TAG, "[Port %d] Deleting master bus handle %p...", i, i2c_ports[i].bus_handle);
                esp_err_t del_res = i2c_del_master_bus(i2c_ports[i].bus_handle);
                if (del_res != ESP_OK)
                {
                    // The bus is still installed in the driver. Keep the
                    // handle: dropping it here would leak the port until
                    // reboot, because i2cdev would no longer know about it.
                    ESP_LOGE(TAG, "[Port %d] Failed to delete I2C bus during cleanup: %d", i, del_res);
                    if (result == ESP_OK)
                        result = del_res;
                    released = false;
                }
                else
                {
                    i2c_ports[i].installed = false;
                    i2c_ports[i].bus_handle = NULL;
                }
            }
        }

        xSemaphoreGive(i2c_ports[i].lock);

        if (!released)
        {
            ESP_LOGW(TAG, "[Port %d] Port was not fully released, keeping it for a later cleanup attempt", i);
            continue;
        }

        ESP_LOGV(TAG, "[Port %d] Deleting port mutex...", i);
        vSemaphoreDelete(i2c_ports[i].lock);
        i2c_ports[i].lock = NULL;
        // Clear the device list for this port
        free_device_list(&i2c_ports[i]);
        ESP_LOGV(TAG, "[Port %d] Cleanup complete.", i);
    } // end for loop
    ESP_LOGV(TAG, "I2C subsystem cleanup finished with result: %d", result);
    return result;
}

esp_err_t i2cdev_get_shared_handle(i2c_port_t port, void **bus_handle)
{
    if (port < 0 || port >= I2C_NUM_MAX || !bus_handle)
    {
        ESP_LOGE(TAG, "Invalid arguments: port=%d, bus_handle=%p", port, bus_handle);
        return ESP_ERR_INVALID_ARG;
    }

    i2c_port_state_t *port_state = &i2c_ports[port];

    // Check if i2cdev subsystem is initialized
    if (!port_state->lock)
    {
        ESP_LOGE(TAG, "[Port %d] I2C subsystem not initialized - call i2cdev_init() first", port);
        return ESP_ERR_INVALID_STATE;
    }

    // Take port mutex for thread-safe access
    if (xSemaphoreTake(port_state->lock, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)) != pdTRUE)
    {
        ESP_LOGE(TAG, "[Port %d] Could not take port mutex for get_shared_handle", port);
        return ESP_ERR_TIMEOUT;
    }

    // Check if bus is initialized
    if (!port_state->installed || !port_state->bus_handle)
    {
        xSemaphoreGive(port_state->lock);
        ESP_LOGE(TAG, "[Port %d] Bus not initialized - create an i2c_dev_t device first to initialize the bus", port);
        return ESP_ERR_INVALID_STATE;
    }

    // Return the bus handle
    *bus_handle = (void *)port_state->bus_handle;

    ESP_LOGI(TAG, "[Port %d] Shared bus handle retrieved: %p (SDA=%d, SCL=%d)",
             port, port_state->bus_handle, port_state->sda_pin_current,
             port_state->scl_pin_current);

    xSemaphoreGive(port_state->lock);
    return ESP_OK;
}
