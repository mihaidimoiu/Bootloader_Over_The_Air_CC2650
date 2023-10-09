/*
 * flash_rw.c
 *
 *  Created on: Jun 1, 2018
 *      Author: Mihai
 */

#include <stdbool.h>
#include <stdint.h>
#include "driverlib/flash.h"
#include "driverlib/vims.h"

/* Verifica daca automatul de stare al memoriei Flash
 * este gata sa scrie / citeasca.
 * Returneaza:
 * true = Automatul este gata
 * false = Automatul nu este gata
 */
bool flashReadyFSM(){
    if( FlashCheckFsmForReady() == FAPI_STATUS_FSM_READY){
        return true;
    }
    return false;
}

/* Verifica daca adresa unde se scrie / sterge
 * este protejata.
 * Se va returna:
 * true = Adresa nu este protejata
 * false = Adresa este protejata
 */
bool flashWriteProtect(uint32_t address){
    if( FlashProtectionGet(address) == FLASH_WRITE_PROTECT ){
        return false;
    }
    return true;
}
/* Sterge o pagina din flash de la adresa specifica
 * Adresa specifica este in hexa.
 * Coduri returnate :
 * -1 = Eroare la automatul de stare al flash-ului
 * -2 = Adresa este gresita
 *  0 = S-a sters pagina cu succes
 */
int eraseFlashSector(uint32_t address){
    CPUcpsid();
    uint32_t status = FlashSectorErase(address);
    if(status == FAPI_STATUS_FSM_ERROR){
        CPUcpsie();
        return -1;
    }
    if(status == FAPI_STATUS_INCORRECT_DATABUFFER_LENGTH){
        CPUcpsie();
        return -2;
    }
    if(status == FAPI_STATUS_SUCCESS){
        CPUcpsie();
        return 0;
    }
    return 0;
}


/* Scrie in memoria flash. Verifica daca locatia
 * este protejata iar daca nu este va scrie
 * dimensiunea specifica ca parametrul len.
 * Se vor oprii intreruperile pentru a nu perturba scrierea.
 * Returneaza:
 * -1 = Adresa unde se va scrie este protejata
 * -2 = Doar daca paramtetrul de stergere al paginii este "True"
 *      si nu s-a putut face stergerea paginii specificata la adresa
 * -3 = Daca nu s-a reusit scrie in flash la adresa specificata
 *  0 = Daca s-a facut cu succes scrierea
 */
int writeFlash(uint32_t address, uint8_t* buffer, uint8_t len, bool erase){
    if( !flashWriteProtect(address)){
        return -1;
    }

    if(erase == true)
        if(eraseFlashSector(address) < 0){
            return -2;
        }
    CPUcpsid();
    if(FlashProgram(buffer, address, len) != FAPI_STATUS_SUCCESS){
        return -3;
    }
    CPUcpsie();
    return 0;
}

//Citire date scrise dinamic din flash
/* Citeste din memoria flash, dinamic.
 * Se dezactiveaza modul cache, pentru a citi
 * din memorie ce s-a scris.
 * Se pune un pointer la adresa specificata si
 * se incremeneaza pointerul atat timp cat este specificat
 * in lungime data ca parametru dupa care se va activa
 * memoria cache.
 *
 * Cea ce se citeste se afla in *buffer_citire.
 */
void readFlashNonCache(uint32_t adresa, uint8_t *buffer_citire, uint8_t lungime_buffer)
{
    VIMSModeSet(VIMS_BASE, VIMS_MODE_DISABLED);
    while(VIMSModeGet(VIMS_BASE) != VIMS_MODE_DISABLED);
    uint8_t *citire = (uint8_t *) adresa;
    while(lungime_buffer--)
       *(buffer_citire++)=*(citire++);
    VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);
}

/* Asemanator cu mai sus doar ca nu se dezactiveaza
 * memoria cache si se citeste static.
 */
void readFlashStaticData(uint32_t adresa, uint8_t *buffer_citire, uint8_t lungime_buffer)
{
    uint8_t *citire = (uint8_t *) adresa;
    while(lungime_buffer--)
       *(buffer_citire++)=*(citire++);
}
