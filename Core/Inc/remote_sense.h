#ifndef REMOTE_SENSE_H
#define REMOTE_SENSE_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

void RemoteSense_Init(ADC_HandleTypeDef *hadc);
void RemoteSense_Task(void);
void RemoteSense_Request(bool enable);
bool RemoteSense_IsClosed(void);
bool RemoteSense_IsWanted(void);
bool RemoteSense_IsLatched(void);
uint8_t RemoteSense_Code(void);
uint16_t RemoteSense_LocalMv(void);
uint16_t RemoteSense_RemotePMv(void);
uint16_t RemoteSense_RemoteNMv(void);

#endif /* REMOTE_SENSE_H */
