i2cdev: ESP-IDF I2C master thread-safe utilities
================================================

One descriptor per I2C device, one bus per port, and a set of read and write
helpers on top of them. The same code runs on the legacy ``driver/i2c.h`` and on
the modern ``driver/i2c_master.h``, so a driver written against this library
does not have to care which one the project uses.

Features
--------

* A device is described by a plain ``i2c_dev_t`` structure: port, address, pins,
  clock speed. The fields keep the layout of the deprecated ``i2c_config_t``, so
  existing drivers work unchanged.
* The bus for a port is created on the first operation and shared by every
  device on that port. One port is bound to one pair of pins: an operation from
  a device that asks for other pins on the same port fails with
  ``ESP_ERR_INVALID_STATE``.
* Every device has a mutex, and every transaction is expected to be wrapped
  with the mutex macros, so several tasks may talk to the same device safely.
* Transfers are retried up to four times with 20, 40 and 80 ms of backoff, and
  the device is re-added to the bus before every attempt.
* ``i2cdev_get_shared_handle()`` hands the bus handle to native ESP-IDF
  components such as ``esp_lcd``, so both sides can use one bus.
* 10-bit addresses are supported where the target has
  ``SOC_I2C_SUPPORT_10BIT_ADDR``; elsewhere the address is sent as 7-bit and a
  warning is logged.

Two implementations
-------------------

The header is served by two source files, and the build system picks one:

.. list-table::
   :header-rows: 1

   * - Implementation
     - Underlying driver
     - When it is used
   * - ``i2cdev.c``
     - ``driver/i2c_master.h``
     - ESP-IDF v5.3 and later, by default
   * - ``i2cdev_legacy.c``
     - ``driver/i2c.h``
     - ESP-IDF older than v5.3, or ``CONFIG_I2CDEV_USE_LEGACY_DRIVER=y``

The public API is the same. The differences are in the details: the legacy
implementation is limited to ``CONFIG_I2CDEV_MAX_DEVICES_PER_PORT`` devices per
port and honours ``timeout_ticks``, while the new one keeps a dynamic device
list and ignores that field. ``CONFIG_I2CDEV_AUTO_ENABLE_PULLUPS`` affects the
new implementation only.

Quick start
-----------

.. code-block:: c

    #include <i2cdev.h>

    static i2c_dev_t sensor = {
        .port = 0,
        .addr = 0x48,
        .cfg = {
            .sda_io_num = GPIO_NUM_21,
            .scl_io_num = GPIO_NUM_22,
            .master.clk_speed = 400000,
        },
    };

    void app_main(void)
    {
        // Port mutexes. No bus is created here yet.
        ESP_ERROR_CHECK(i2cdev_init());

        // Per-device mutex. Still no bus access.
        ESP_ERROR_CHECK(i2c_dev_create_mutex(&sensor));

        // The bus and the device handle appear on the first real transfer.
        uint8_t id;
        ESP_ERROR_CHECK(i2c_dev_read_reg(&sensor, 0x0F, &id, 1));

        // Removes the device from the bus and deletes its mutex.
        ESP_ERROR_CHECK(i2c_dev_delete_mutex(&sensor));

        // Removes the buses and the port mutexes.
        ESP_ERROR_CHECK(i2cdev_done());
    }

Bus lifecycle
-------------

``i2cdev_init()`` creates only the per-port mutexes. The bus for a port is
created by the first operation on that port, and the pins asked for by that
first device become the pins of the port. From then on the bus serves every
device of the port until ``i2cdev_done()`` releases it, no matter which of them
is added or removed in between: ``i2c_dev_delete_mutex()`` removes the device,
not the bus.

``i2cdev_done()`` walks the ports and their devices, so the descriptors must
stay valid while they are registered: do not copy, move or free an
``i2c_dev_t`` that was used for an operation. Cleanup is best effort. If a bus
is still in use (for example, its handle was handed out and the other component
has not released it), the port is left intact and the call may be repeated.

Thread safety
-------------

The bus is serialised by the ESP-IDF driver itself, but a device is not: the
descriptor has a mutex, and every transaction on a device must be performed
under it.

.. code-block:: c

    esp_err_t read_temperature(i2c_dev_t *dev, float *value)
    {
        uint8_t raw[2];

        I2C_DEV_TAKE_MUTEX(dev);
        I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, REG_TEMP, raw, sizeof(raw)));
        I2C_DEV_GIVE_MUTEX(dev);

        *value = ((raw[0] << 8) | raw[1]) * 0.0625f;
        return ESP_OK;
    }

.. list-table::
   :header-rows: 1

   * - Macro
     - Behaviour
   * - ``I2C_DEV_TAKE_MUTEX(dev)``
     - Takes the mutex, returns on failure
   * - ``I2C_DEV_GIVE_MUTEX(dev)``
     - Releases the mutex, returns on failure
   * - ``I2C_DEV_CHECK(dev, call)``
     - Runs ``call``; on error releases the mutex and returns the error; on
       success the mutex stays taken
   * - ``I2C_DEV_CHECK_LOGE(dev, call, msg, ...)``
     - Same, and logs the failure through ``TAG``

``CONFIG_I2CDEV_NOLOCK`` turns the per-device mutex off: the macros become
no-ops and serialisation becomes the caller's job. Bus management, device
handles and teardown still happen, so devices are removed and buses released as
usual. Enable it only when the caller already serialises the device or when it
must be accessed from an interrupt.

Sharing the bus with native components
--------------------------------------

.. code-block:: c

    void *bus_handle;
    ESP_ERROR_CHECK(i2cdev_get_shared_handle(0, &bus_handle));
    // hand bus_handle to esp_lcd or any other component that accepts a bus

The handle belongs to i2cdev and must not be deleted: ``i2cdev_done()``
releases it. The bus lock of the ESP-IDF driver keeps the transactions apart.
Devices that the other component adds to the bus are unknown to i2cdev, so
removing them is the caller's job before ``i2cdev_done()`` is called.

Configuration
-------------

.. list-table::
   :header-rows: 1

   * - Option
     - Default
     - Meaning
   * - ``CONFIG_I2CDEV_USE_LEGACY_DRIVER``
     - ``n``
     - Use ``driver/i2c.h`` even on ESP-IDF v5.3 and later
   * - ``CONFIG_I2CDEV_AUTO_ENABLE_PULLUPS``
     - ``n``
     - Turn on internal pullups when both pullup flags are zero (new driver
       only)
   * - ``CONFIG_I2CDEV_DEFAULT_SDA_PIN``
     - ``21``
     - SDA pin used when ``sda_io_num`` is ``-1``
   * - ``CONFIG_I2CDEV_DEFAULT_SCL_PIN``
     - ``22``
     - SCL pin used when ``scl_io_num`` is ``-1``
   * - ``CONFIG_I2CDEV_MAX_DEVICES_PER_PORT``
     - ``8``
     - Devices per port, legacy driver only
   * - ``CONFIG_I2CDEV_TIMEOUT``
     - ``1000``
     - Timeout of a mutex wait and of a transaction, milliseconds
   * - ``CONFIG_I2CDEV_NOLOCK``
     - ``n``
     - Disable the per-device mutex

``CONFIG_I2CDEV_AUTO_ENABLE_PULLUPS`` is a convenience for boards without
external resistors: when it is enabled and neither ``sda_pullup_en`` nor
``scl_pullup_en`` is set in the descriptor, the internal pullups are turned on.
A board with proper 4.7 kΩ resistors does not need it, and the weak internal
pullups are not a substitute for them on a long or a fast bus.

Examples
--------

``examples/default`` scans the buses and lists the addresses that answer.

API reference
-------------

 .. doxygenfile:: i2cdev.h
