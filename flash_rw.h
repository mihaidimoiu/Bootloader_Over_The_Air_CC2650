/*
 * flash_rw.h
 *
 *  Created on: Jun 1, 2018
 *      Author: Mihai
 */

#ifndef FLASH_RW_H_
#define FLASH_RW_H_

void readFlashStaticData(uint32_t, uint8_t*, uint8_t);
int writeFlash(uint32_t, uint8_t*, uint8_t, bool);
int eraseFlashSector(uint32_t);
void readFlashNonCache(uint32_t, uint8_t*, uint8_t);
bool flashReadyFSM();
bool flashWriteProtect(uint32_t);

#endif /* FLASH_RW_H_ */
