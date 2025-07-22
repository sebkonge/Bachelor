#include <pico/stdio.h>
#include <pico/time.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <boards/pico.h>
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "pico/divider.h"


#define GPIO_ON 1
#define GPIO_OFF 0
#define LED_PIN PICO_DEFAULT_LED_PIN

#define PWM_PIN 8
#define PWM_FREQ 50 // PWM frekvensen er 50 Hz (1/f --> 1/50 = 20 ms)
#define UINT32SIZE 65535

uint32_t div_wrap_calc(){
    uint slice_8_A = pwm_gpio_to_slice_num(PWM_PIN);
    uint clk = clock_get_hz(clk_sys);

    uint countSize = (clk / PWM_FREQ) - 1;
    // wrap =  150 000 000 /50 -1 --> så skal wrap være 2999999
    // Da wrap kun er en 16 bit integer skal den regnes om

    // Så skal bruge en clock divider på 
    // 2999999/65535 = 45.77705 mindst
    uint minimumClockDiv = countSize/UINT32SIZE;

    // Går 2 integer op i clock divier
    uint div = minimumClockDiv + 2; // magisk tal for at sikre at clock divideren er fin størrelse
    
    pwm_set_clkdiv_int_frac(slice_8_A, div, 0);

    // wrap = ((150 000 000 /46.777 )/50) -1 = 65442.94182 ---> 65443
    uint32_t wrapSize = ((clk/div)/PWM_FREQ)-1;

    pwm_set_wrap(slice_8_A, wrapSize);
    printf("wrap: %d,  divider: %d", wrapSize, div);

    return wrapSize;
}


int main(){
    stdio_init_all();
    
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM); //sætter gpio pin til pwm funktion.

    uint slice_8_A = pwm_gpio_to_slice_num(PWM_PIN);
    uint channel = pwm_gpio_to_channel(PWM_PIN);
    
    uint clk = clock_get_hz(clk_sys);
    printf("slice: %d,  channel: %d,  clk speed: %d", slice_8_A, channel, clk);


    // wrap = ((150 000 000 /46.777 )/50) -1 = 65442.94182 ---> 65443
    // pwm_set_clkdiv_int_frac(slice_8_A, 46, 77);

    // uint16_t wrapSize = 65443;
    // pwm_set_wrap(slice_8_A, wrapSize);
    getchar();

    uint32_t wrapSize = div_wrap_calc();

    pwm_set_enabled(slice_8_A, true);  // Aktivér PWM

    float WrapStepSize = wrapSize*0.0005;
    uint32_t currentPWMValue = 0;
    char userInput;
    while (true) {
        userInput = getchar();
        if(userInput == 'i'){
            currentPWMValue +=WrapStepSize;
            printf("currentPWM value: %d \n",currentPWMValue);
            pwm_set_gpio_level(PWM_PIN, currentPWMValue);
        }
        if(userInput == 't'){
            currentPWMValue -=WrapStepSize;
            printf("currentPWM value: %d \n",currentPWMValue);
            pwm_set_gpio_level(PWM_PIN, currentPWMValue);
        }
    }
} 
/*

Pico 2 og pico w clock frekvens

Pico 2 og pico w clock pwm clock frekvens.

For pico 2
Wrap:
pwm_set_wrap (uint slice_num, uint16_t wrap)
Wrap value er den højeste værdi counterne tæller op til før den resetter til 0


Generer PWM signalet ved at sætte levelet hvor den skal tænde for duty cyclen.
Når counteren er under lvl så er outputtet high. Når counteren er over, så er outputtet low.
pwm_set_gpio_level (uint gpio, uint16_t level)


https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=1

Frekvens:
fc = frekvens af clk efter division

f = fc/(wrap+1)

Udregn wrap needed for at give en ønsket pwm frekvens
wrap = fc /f -1


Pico 2 clock frekvens er 150 000 000 hz
f = fc/(wrap+1) --> 150 000 000 / (65535+1) = 
Burde give en frekvens på 2288.81836 hz

Med en clock divider på 2 150/2 = 75 000 000
f = fc/(wrap+1) --> 75 000 000 / (65535+1) = 
Burde give en frekvens på 1144.40917 hz


regner wrap størrelse ud:
wrap =  150 000 000 /50 -1 --> så skal wrap være 2999999
Da wrap kun er en 16 bit integer skal den regnes om

Så skal brugen clock divider på 
2999999/65535 = 45.77705 mindst

wrap = (150 000 000 /45.777 )/50 -1 = 66872.54494 hvilket er for stort

Går 1 integer op i clock divier
wrap = ((150 000 000 /46.777 )/50) -1 = 65442.94182


*/
