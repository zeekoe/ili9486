#include <iostream>

#include "spi.h"
#include "display.h"

#include <memory.h>
#include <stdio.h>

// Data specific to the ILI9486 controller



void InitILI9486()
{
    // Do the initialization with a very low SPI bus speed, so that it will succeed even if the bus speed chosen by the user is too high.
    spi->clk = 34;
    __sync_synchronize();

    BEGIN_SPI_COMMUNICATION();
    {
        SPI_TRANSFER(0xB0/*Interface Mode Control*/, 0x00, 0x00/*DE polarity=High enable, PCKL polarity=data fetched at rising time, HSYNC polarity=Low level sync clock, VSYNC polarity=Low level sync clock*/);
        SPI_TRANSFER(0x11/*Sleep OUT*/);
        usleep(120*1000);

        const uint8_t pixelFormat = 0x55; /*DPI(RGB Interface)=16bits/pixel, DBI(CPU Interface)=16bits/pixel*/

        SPI_TRANSFER(0x3A/*Interface Pixel Format*/, 0x00, pixelFormat);

        SPI_TRANSFER(0x20/*Display Inversion OFF*/);

        SPI_TRANSFER(0xC0/*Power Control 1*/, 0x00, 0x09, 0x00, 0x09);
        SPI_TRANSFER(0xC1/*Power Control 2*/, 0x00, 0x41, 0x00, 0x00);
        SPI_TRANSFER(0xC2/*Power Control 3*/, 0x00, 0x33);
        SPI_TRANSFER(0xC5/*VCOM Control*/, 0x00, 0x00, 0x00, 0x36);

#define MADCTL_BGR_PIXEL_ORDER (1<<3)
#define MADCTL_COLUMN_ADDRESS_ORDER_SWAP (1<<6)
#define MADCTL_ROW_ADDRESS_ORDER_SWAP (1<<7)
#define MADCTL_ROTATE_180_DEGREES (MADCTL_COLUMN_ADDRESS_ORDER_SWAP | MADCTL_ROW_ADDRESS_ORDER_SWAP)

        uint8_t madctl = 0;
        madctl |= MADCTL_BGR_PIXEL_ORDER;
//    madctl ^= MADCTL_ROTATE_180_DEGREES;

        SPI_TRANSFER(0x36/*MADCTL: Memory Access Control*/, 0x00, madctl);
        SPI_TRANSFER(0xE0/*Positive Gamma Control*/, 0x00, 0x00, 0x00, 0x2C, 0x00, 0x2C, 0x00, 0x0B, 0x00, 0x0C, 0x00, 0x04, 0x00, 0x4C, 0x00, 0x64, 0x00, 0x36, 0x00, 0x03, 0x00, 0x0E, 0x00, 0x01, 0x00, 0x10, 0x00, 0x01, 0x00, 0x00);
        SPI_TRANSFER(0xE1/*Negative Gamma Control*/, 0x00, 0x0F, 0x00, 0x37, 0x00, 0x37, 0x00, 0x0C, 0x00, 0x0F, 0x00, 0x05, 0x00, 0x50, 0x00, 0x32, 0x00, 0x36, 0x00, 0x04, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x19, 0x00, 0x14, 0x00, 0x0F);
        SPI_TRANSFER(0xB6/*Display Function Control*/, 0, 0, 0, /*ISC=2*/2, 0, /*Display Height h=*/59); // Actual display height = (h+1)*8 so (59+1)*8=480

        SPI_TRANSFER(0x11/*Sleep OUT*/);
        usleep(120*1000);
        SPI_TRANSFER(0x29/*Display ON*/);
        SPI_TRANSFER(0x38/*Idle Mode OFF*/);
        SPI_TRANSFER(0x13/*Normal Display Mode ON*/);

        ClearScreen();
    }
    END_SPI_COMMUNICATION();

    // And speed up to the desired operation speed finally after init is done.
    usleep(10 * 1000); // Delay a bit before restoring CLK, or otherwise this has been observed to cause the display not init if done back to back after the clear operation above.
    spi->clk = SPI_BUS_CLOCK_DIVISOR;
}

void TurnBacklightOff()
{
#if defined(GPIO_TFT_BACKLIGHT) && defined(BACKLIGHT_CONTROL)
    SET_GPIO_MODE(GPIO_TFT_BACKLIGHT, 0x01); // Set backlight pin to digital 0/1 output mode (0x01) in case it had been PWM controlled
  CLEAR_GPIO(GPIO_TFT_BACKLIGHT); // And turn the backlight off.
#endif
}

void TurnBacklightOn()
{
#if defined(GPIO_TFT_BACKLIGHT) && defined(BACKLIGHT_CONTROL)
    SET_GPIO_MODE(GPIO_TFT_BACKLIGHT, 0x01); // Set backlight pin to digital 0/1 output mode (0x01) in case it had been PWM controlled
  SET_GPIO(GPIO_TFT_BACKLIGHT); // And turn the backlight on.
#endif
}

void TurnDisplayOff()
{
    TurnBacklightOff();
    QUEUE_SPI_TRANSFER(0x28/*Display OFF*/);
    QUEUE_SPI_TRANSFER(0x10/*Enter Sleep Mode*/);
    usleep(120*1000); // Sleep off can be sent 120msecs after entering sleep mode the earliest, so synchronously sleep here for that duration to be safe.
}

void TurnDisplayOn()
{
    TurnBacklightOff();
    QUEUE_SPI_TRANSFER(0x11/*Sleep Out*/);
    usleep(120 * 1000);
    QUEUE_SPI_TRANSFER(0x29/*Display ON*/);
    usleep(120 * 1000);
    TurnBacklightOn();
}

void DeinitSPIDisplay()
{
    ClearScreen();
    TurnDisplayOff();
}

int main() {
    InitILI9486();
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
