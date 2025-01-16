// toy_connector.h

#ifndef TOY_CONNECTOR_H
#define TOY_CONNECTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct ToyConnector ToyConnector;

/**
 * @brief Initialize the ToyConnector instance.
 *
 * This function initializes the BLE controller, Bluedroid stack, and registers necessary callbacks.
 *
 * @return Pointer to the initialized ToyConnector instance, or NULL on failure.
 */
ToyConnector* toy_connector_init(void);

/**
 * @brief Start the BLE pairing process.
 *
 * This function initiates the BLE pairing by starting the scanning process
 * and waits for the pairing to complete or timeout.
 *
 * @return true if pairing was successful, false otherwise.
 */
bool toy_connector_start_pairing_process(void);

/**
 * @brief Set the vibration intensity of the device.
 *
 * This function writes the desired vibration intensity to the RX characteristic
 * of the device. The intensity is mapped from a range of 0-100 to 0-255.
 *
 * @param toy Pointer to the initialized ToyConnector instance.
 * @param intensity Vibration intensity (0-100).
 */
void toy_connector_vibrate(ToyConnector* toy, uint8_t intensity);

/**
 * @brief Deinitialize the ToyConnector instance.
 *
 * This function disconnects from the device, unregisters callbacks,
 * disables the BLE stack, and frees allocated resources.
 *
 * @param toy Pointer to the initialized ToyConnector instance.
 */
void toy_connector_deinit(ToyConnector* toy);

#endif // TOY_CONNECTOR_H
