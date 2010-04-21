#ifndef PUB_SUB_H_
#define PUB_SUB_H_

// Return codes
#define PS_OK 0
#define PS_KEY_NOT_FOUND 4

// Function prototypes
void pubSubInit(uint8_t *buffer, uint8_t size);
uint8_t pubSubPublish(uint8_t key, uint8_t *dataPtr, uint8_t size);
uint8_t pubSubRead(uint8_t key, uint8_t *dataPtr);
uint8_t pubSubWrite(uint8_t key, uint8_t *dataPtr);
uint8_t pubSubRemove(uint8_t key);

#endif /*PUB_SUB_H_*/
