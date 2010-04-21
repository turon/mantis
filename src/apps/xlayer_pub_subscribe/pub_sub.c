#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "mos.h"
#include "printf.h"
#include "queue.h"
#include "pub_sub.h"

// Queue for storing pub/subscribe data.
queue_t dataQueue;

// Initialize publish/subscribe component and backend data structure (queue)
void pubSubInit(uint8_t *buffer, uint8_t size) {
	mos_queue_init(&dataQueue, buffer, size);
}

// Associate new data with key, fails if key already exists or no space left.
// Size is size in bytes.
uint8_t pubSubPublish(uint8_t key, uint8_t *dataPtr, uint8_t size) {
	uint8_t i = 0;
	uint8_t returnVal = 0;
	
	// Add key to queue
	mos_queue_add(&dataQueue, key);
	
	// Add size for key data to queue
	mos_queue_add(&dataQueue, size);
	
	// Add data for this key to queue
	for (i = 0; i < size; i++) {
		returnVal = mos_queue_add(&dataQueue, dataPtr[i]);
		
		if (returnVal != Q_OK) {
			return returnVal;
		}
	}
	
	return PS_OK;
}

// Read into dataPtr, data associated with the key
uint8_t pubSubRead(uint8_t key, uint8_t *dataPtr) {
	uint8_t queueLength = 0;
	uint8_t i = 0;
	uint8_t returnVal = 0;
	uint8_t queueByte = 0;
	uint8_t keyDataSize = 0;
	uint8_t dataPtrIndex = 0;
	uint8_t queueIndex = 0;
	
	queueLength = mos_queue_length(&dataQueue);
	
	// Find key in queue
	for (queueIndex = 0; queueIndex < queueLength; queueIndex++) {
		returnVal = mos_queue_peek(&dataQueue, queueIndex, &queueByte);
		
		if (returnVal != Q_OK) {
			return returnVal;
		}
		
		// Found key
		if (key == queueByte) {
			// Get size of data for key
			returnVal = mos_queue_peek(&dataQueue, queueIndex + 1, 
					&keyDataSize);
			
			if (returnVal != Q_OK) {
				return returnVal;
			}
			
			// Read key data into dataPtr
			queueIndex += 2; // Advance queueIndex to start of key data
			for (dataPtrIndex = 0; dataPtrIndex < keyDataSize; 
				dataPtrIndex++) {
				returnVal = mos_queue_peek(&dataQueue, queueIndex, 
						&(dataPtr[dataPtrIndex]));
				
				if (returnVal != Q_OK) {
					return returnVal;
				}
				
				queueIndex++;
			}
			
			return PS_OK;
		}
	}
	
	return PS_KEY_NOT_FOUND;
}

// Associate data in dataPtr with key
uint8_t pubSubWrite(uint8_t key, uint8_t *dataPtr) {
	uint8_t i = 0;
	uint8_t keyDataSize = 0;
	uint8_t queueLength = 0;
	uint8_t queueByte = 0;
	uint8_t returnVal = 0;
	uint8_t dataPtrIndex = 0;
	
	queueLength = mos_queue_length(&dataQueue);
	
	// Find key in queue
	for (i = 0; i < queueLength; i++) {
		returnVal = mos_queue_remove(&dataQueue, &queueByte);
		
		if (returnVal != Q_OK) {
			return returnVal;
		}
		
		// Found key
		if (key == queueByte) {
			// Get size of data for key
			returnVal = mos_queue_remove(&dataQueue, &keyDataSize);
			
			if (returnVal != Q_OK) {
				return returnVal;
			}
			
			// Write key and keyDataSize back to queue
			returnVal = mos_queue_add(&dataQueue, key);
			if (returnVal != Q_OK) {
				return returnVal;
			}
			returnVal = mos_queue_add(&dataQueue, keyDataSize);
			if (returnVal != Q_OK) {
				return returnVal;
			}
			
			// Write key data from dataPtr to queue
			for (dataPtrIndex = 0; dataPtrIndex < keyDataSize; 
				dataPtrIndex++) {
				returnVal = mos_queue_add(&dataQueue, dataPtr[dataPtrIndex]);
				if (returnVal != Q_OK) {
					return returnVal;
				}
				
				returnVal = mos_queue_remove(&dataQueue, 
						&(dataPtr[dataPtrIndex]));
				if (returnVal != Q_OK) {
					return returnVal;
				}
			}
			
			return PS_OK;
		}
		else {
			// Data removed from queue doesn't match key, so add it back to 
			// queue
			returnVal = mos_queue_add(&dataQueue, queueByte);
			if (returnVal != Q_OK) {
				return returnVal;
			}
		}
	}
	
	return PS_KEY_NOT_FOUND;
}

// Deletes key entry
uint8_t pubSubRemove(uint8_t key) {
	uint8_t i = 0;
	uint8_t keyDataSize = 0;
	uint8_t queueLength = 0;
	uint8_t queueByte = 0;
	uint8_t dataByte = 0;
	uint8_t keyDataIndex = 0;
	uint8_t returnVal = 0;

	queueLength = mos_queue_length(&dataQueue);
		
	// Find key in queue
	for (i = 0; i < queueLength; i++) {
		returnVal = mos_queue_remove(&dataQueue, &queueByte);
		if (returnVal != Q_OK) {
			return returnVal;
		}
		
		// Found key
		if (key == queueByte) {
			// Get size of data for key
			returnVal = mos_queue_remove(&dataQueue, &keyDataSize);
			
			if (returnVal != Q_OK) {
				return returnVal;
			}
			
			// Remove key data from queue
			for (keyDataIndex = 0; keyDataIndex < keyDataSize; 
			keyDataIndex++) {
				returnVal = mos_queue_remove(&dataQueue, &dataByte);
				if (returnVal != Q_OK) {
					return returnVal;
				}
			}
			
			return PS_OK;
		}
	}
	
	return PS_KEY_NOT_FOUND;
}
