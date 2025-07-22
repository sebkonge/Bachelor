#pragma once
#include <string>

#include "ff.h"
#include "tf_card.h"

class SDClass{
public:
    SDClass(const char* fileName, const char* columSetup);

    bool mountSDCard();
    bool writeStringSDCard(std::string inputStr);

    void unmountSDCard();
private:
    const char* m_fileName;
    const char* m_columSetup;
    FATFS fs;
    FIL fil;
    FRESULT fr;
    UINT bw;

      // Konfigurer SPI til FatFs med default settings
    pico_fatfs_spi_config_t config = {
        spi0,
        CLK_SLOW_DEFAULT,
        CLK_FAST_DEFAULT,
        PIN_SPI0_MISO_DEFAULT,
        PIN_SPI0_CS_DEFAULT,
        PIN_SPI0_SCK_DEFAULT,
        PIN_SPI0_MOSI_DEFAULT,
        true // use internal pullup
    };    
};

SDClass::SDClass(const char* fileName, const char* columSetup): m_fileName(fileName), m_columSetup(columSetup){
    pico_fatfs_set_config(&config);
}


bool SDClass::mountSDCard(){
    bool retBool = false;

    for (int i = 0; i < 5; i++) { // prøver 5 gange
      fr = f_mount(&fs, "", 1);
      if (fr == FR_OK) {
        break;
      }
      printf("Mount fejlede (%d), prøver igen...\n", fr);
      pico_fatfs_reboot_spi();
    }

    if (fr == FR_OK) { // er den mounted ok
      fr = f_open(&fil, m_fileName,FA_WRITE | FA_CREATE_ALWAYS); 
      if (fr == FR_OK) { // kunne den skrive eller oprette filen
        f_write(&fil, m_columSetup, strlen(m_columSetup), &bw);
        f_close(&fil);
        retBool = true;
      } else {
        // Håndter andre fejl
        printf("Fejl: Kunne ikke åbne test.txt (%d)\n", fr);
      }
    }

    return retBool;
}


bool SDClass::writeStringSDCard(std::string inputStr){
    bool retBool = false;
    fr = f_open(&fil, m_fileName,FA_WRITE | FA_OPEN_APPEND); // Brug FA_OPEN_APPEND for at tilføje
    if (fr == FR_OK) {
        f_write(&fil, inputStr.c_str(), strlen(inputStr.c_str()), &bw);
        f_close(&fil);
        retBool = true;
    }
    else {
        f_unmount("");
    }

    return retBool;
}


void SDClass::unmountSDCard(){
    f_unmount("");
}


