#include <stdio.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <linux/types.h>
#include <errno.h>
#include <wiringPi.h>

#define GPIO_OUT 0
#define GPIO_IN 1
#define MAX_PINS 51
static int iPinHandles[MAX_PINS];

#define DISPLAY_NATIVE_WIDTH 256
#define DISPLAY_NATIVE_HEIGHT 64

#define DISPLAY_WIDTH DISPLAY_NATIVE_HEIGHT
#define DISPLAY_HEIGHT DISPLAY_NATIVE_WIDTH

#define GPIO_SPI0_MOSI 19
#define GPIO_SPI0_CLK 23
#define GPIO_SPI0_CE0 24

#define GPIO_TFT_DATA_CONTROL 18 // overrides rotary encoder
#define GPIO_TFT_RESET_PIN 22 // overrides IR sensor, which is at 11 for us
#define GPIO_TFT_POWER 36 // was 38

#define SEND_CMD      1   // 3- and 4-wire - Display instruction (command)
#define SEND_DAT      2   // 3- and 4-wire - Display instruction (data)

#define MAXROWS      64
#define MAXCOLS     240 // 256?

int handle;
static struct spi_ioc_transfer xfer;

void drawRow(int y, const unsigned short *dataBufPtr);
int initSpi(int iChannel, int iSPIFreq);
int writeSpi(int iHandle, unsigned char *pBuf, int iLen);

void drawRow(int y, const unsigned short *dataBufPtr) {
    // todo
}

void displaySend(unsigned char sendType, unsigned char v);

void spiTransfer(char cmd, int num_args, ...) {
    va_list ap;

    unsigned char buf[1024] = {0}; // room for 1024 chars (or 1023 + a 0 byte for a string)
    unsigned char *origStr = buf;
    unsigned char *str = buf;
    *str++ = 0;
    *str++ = cmd;

//    digitalWrite(GPIO_TFT_DATA_CONTROL, 0);

//    printspi(origStr, 2, 1);
    writeSpi(handle, origStr, 2);

//    digitalWrite(GPIO_TFT_DATA_CONTROL, 1);

    str = origStr;

    va_start(ap, num_args);
    for (int i = 0; i < num_args; i++) {
        unsigned char x = va_arg(ap, int);
        *str = x;
        *str++;
    }
    va_end(ap);

    writeSpi(handle, origStr, num_args);

}

void initDisplay() {
    memset(iPinHandles, -1, sizeof(iPinHandles));

    wiringPiSetupPinType(WPI_PIN_PHYS);

    pinMode(GPIO_TFT_DATA_CONTROL, OUTPUT);
    pinMode(GPIO_TFT_RESET_PIN, OUTPUT);
    pinMode(GPIO_TFT_POWER, OUTPUT);
	pinMode(GPIO_SPI0_MOSI, OUTPUT);
	pinMode(GPIO_SPI0_CLK, OUTPUT);

    digitalWrite(GPIO_TFT_POWER, 0);
    digitalWrite(GPIO_TFT_RESET_PIN, 1);
    usleep(5 * 1000);

    usleep(100);
    digitalWrite(GPIO_TFT_RESET_PIN, 0);
    usleep(100);
    digitalWrite(GPIO_TFT_RESET_PIN, 1);
    usleep(5 * 1000);

//    handle = initSpi(0, 1500000);

    digitalWrite(GPIO_SPI0_CE0, 0);
    usleep(10);
    {
          displaySend(SEND_CMD, 0xFD); // Set Command Lock (MCU protection status)
          displaySend(SEND_DAT, 0x12); // = Reset

          displaySend(SEND_CMD, 0xB3); // Set Front Clock Divider / Oscillator Frequency
          displaySend(SEND_DAT, 0xD0); // = reset / 1100b

          displaySend(SEND_CMD, 0xCA); // Set MUX Ratio
          displaySend(SEND_DAT, 0x3F); // = 63d = 64MUX

          displaySend(SEND_CMD, 0xA2); // Set Display Offset
          displaySend(SEND_DAT, 0x00); // = RESET

          displaySend(SEND_CMD, 0xA1); // Set Display Start Line
          displaySend(SEND_DAT, 0x00); // = register 00h

          displaySend(SEND_CMD, 0xA0); // Set Re-map and Dual COM Line mode
          displaySend(SEND_DAT, 0x14); // = Reset except Enable Nibble Re-map, Scan from COM[N-1] to COM0, where N is the Multiplex ratio
          displaySend(SEND_DAT, 0x11); // = Reset except Enable Dual COM mode (MUX = 63)

          displaySend(SEND_CMD, 0xB5); // Set GPIO
          displaySend(SEND_DAT, 0x00); // = GPIO0, GPIO1 = HiZ, Input Disabled

          displaySend(SEND_CMD, 0xAB); // Function Selection
          displaySend(SEND_DAT, 0x01); // = reset = Enable internal VDD regulator

          displaySend(SEND_CMD, 0xB4); // Display Enhancement A
          displaySend(SEND_DAT, 0xA0); // = Enable external VSL
          displaySend(SEND_DAT, 0xB5); // = Normal (reset)

          displaySend(SEND_CMD, 0xC1); // Set Contrast Current
          displaySend(SEND_DAT, 0x7F); // = reset

          displaySend(SEND_CMD, 0xC7); // Master Contrast Current Control
          displaySend(SEND_DAT, 0x0F); // = no change

          displaySend(SEND_CMD, 0xB9); // Select Default Linear Gray Scale table

          displaySend(SEND_CMD, 0xB1); // Set Phase Length
          displaySend(SEND_DAT, 0xE2); // = Phase 1 period (reset phase length) = 5 DCLKs, Phase 2 period (first pre-charge phase length) = 14 DCLKs

          displaySend(SEND_CMD, 0xD1); // Display Enhancement B
          displaySend(SEND_DAT, 0xA2); // = Normal (reset)
          displaySend(SEND_DAT, 0x20); // n/a

          displaySend(SEND_CMD, 0xBB); // Set Pre-charge voltage
          displaySend(SEND_DAT, 0x1F); // = 0.60 x VCC

          displaySend(SEND_CMD, 0xB6); // Set Second Precharge Period
          displaySend(SEND_DAT, 0x08); // = 8 dclks [reset]

          displaySend(SEND_CMD, 0xBE); // Set VCOMH
          displaySend(SEND_DAT, 0x07); // = 0.86 x VCC

          displaySend(SEND_CMD, 0xA6); // Set Display Mode = Normal Display

          displaySend(SEND_CMD, 0xA9); // Exit Partial Display

          displaySend(SEND_CMD, 0xAF); // Set Sleep mode OFF (Display ON)

          usleep(10000);
    }
    digitalWrite(GPIO_SPI0_CE0, 1);
}

void deInitDisplay() {
    digitalWrite(GPIO_TFT_POWER, 1);
}

//--------------------------------------------------------------------------
//##########################################################################
//--------------------------------------------------------------------------
void displaySend(unsigned char sendType, unsigned char v)
{
  unsigned char i;

  digitalWrite(GPIO_SPI0_CE0, LOW);

  if (sendType == SEND_CMD)
  { // Send a command value
    digitalWrite(GPIO_TFT_DATA_CONTROL, LOW);
  }
  else if (sendType == SEND_DAT)
  { // Send a data value
    digitalWrite(GPIO_TFT_DATA_CONTROL, HIGH);
  }
//	usleep(10);

 // spiTransfer(v, 0);
    for(i=8;i>0;i--)
    { // Decrementing is faster
      if((v&0x80)>>7==1)
      {
        digitalWrite(GPIO_SPI0_MOSI, HIGH);
      }
      else
      {
        digitalWrite(GPIO_SPI0_MOSI, LOW);
      }
      v=v<<1;
      digitalWrite(GPIO_SPI0_CLK, LOW);
//	usleep(1);
      digitalWrite(GPIO_SPI0_CLK, HIGH);
//usleep(1);
    }

  digitalWrite(GPIO_SPI0_CE0, HIGH);
//	usleep(100);
}

//--------------------------------------------------------------------------
void Set_Column_Address(unsigned char a, unsigned char b)
{
  displaySend(SEND_CMD, 0x15); // Set Column Address
  displaySend(SEND_DAT, a);    //   Default => 0x00
  displaySend(SEND_DAT, b);    //   Default => 0x77
}

//--------------------------------------------------------------------------
void Set_Row_Address(unsigned char a, unsigned char b)
{
  displaySend(SEND_CMD, 0x75); // Set Row Address
  displaySend(SEND_DAT, a);    //   Default => 0x00
  displaySend(SEND_DAT, b);    //   Default => 0x7F
}

//--------------------------------------------------------------------------
void Set_Write_RAM()
{
  displaySend(SEND_CMD, 0x5C); // Enable MCU to Write into RAM
}

//--------------------------------------------------------------------------
void ClearDisplay()
{
  unsigned int i, j;

  // Turn off display while clearing (also hides noise at powerup)
  displaySend(SEND_CMD, 0xA4); // Set Display Mode = OFF

  Set_Column_Address(0x00,0x77);
  Set_Row_Address(0x00,0x7F);
  Set_Write_RAM();

  for(i=0;i<MAXROWS;i++)
  {
    for(j=0;j<MAXCOLS/2;j++)
    {
      displaySend(SEND_DAT, 0x00);
      displaySend(SEND_DAT, 0x00);
    }
    for(j=0;j<MAXCOLS/2;j++)
    {
      displaySend(SEND_DAT, 0x00);
      displaySend(SEND_DAT, 0x00);
    }
  }

  displaySend(SEND_CMD, 0xA6); // Set Display Mode = Normal Display
}

//--------------------------------------------------------------------------
void FillDisplay()
{
  unsigned int i, j;

  Set_Column_Address(0x00,0x77);
  Set_Row_Address(0x00,0x7F);
  Set_Write_RAM();

  for(i=0;i<MAXROWS;i++)
  {
    for(j=0;j<MAXCOLS/2;j++)
    {
      displaySend(SEND_DAT, 0xFF);
      displaySend(SEND_DAT, 0xFF);
    }
    for(j=0;j<MAXCOLS/2;j++)
    {
      displaySend(SEND_DAT, 0xFF);
      displaySend(SEND_DAT, 0xFF);
    }
  }
}

//--------------------------------------------------------------------------
void CheckerboardOdd()
{
  unsigned int i, j;

  Set_Column_Address(0x00,0x77);
  Set_Row_Address(0x00,0x7F);
  Set_Write_RAM();

  for(i=0;i<MAXROWS;i++)
  {
    for(j=0;j<MAXCOLS/2;j++)
    {
      displaySend(SEND_DAT, 0x0F);
      displaySend(SEND_DAT, 0x0F);
    }
    for(j=0;j<MAXCOLS/2;j++)
    {
      displaySend(SEND_DAT, j % 256);
      displaySend(SEND_DAT, j % 256);
    }
  }
}

//--------------------------------------------------------------------------
void CheckerboardEven()
{
  unsigned int i, j;

  Set_Column_Address(0x00,0x77);
  Set_Row_Address(0x00,0x7F);
  Set_Write_RAM();

  for(i=0;i<MAXROWS;i++)
  {
    for(j=0;j<MAXCOLS/2;j++)
    {
      displaySend(SEND_DAT, 0xF0);
      displaySend(SEND_DAT, 0xF0);
    }
    for(j=0;j<MAXCOLS/2;j++)
    {
      displaySend(SEND_DAT, 0x0F);
      displaySend(SEND_DAT, 0x0F);
    }
  }
}

int initSpi(int iChannel, int iSPIFreq)
{
    int rc, iSPIMode = SPI_MODE_0; // | SPI_NO_CS;
    char szName[32];
    int file_spi;
    int i = iSPIFreq;

    sprintf(szName,"/dev/spidev%d.0", iChannel);
    printf(szName);
    printf("\n");
    file_spi = open(szName, O_RDWR);
    if (file_spi < 0) {
        perror("Error opening SPI device");
        return -1; // Exit or handle error appropriately
    }
    rc = ioctl(file_spi, SPI_IOC_WR_MODE, &iSPIMode);
    if (rc < 0) {
        perror("Error setting SPI mode");
    }
    rc = ioctl(file_spi, SPI_IOC_WR_MAX_SPEED_HZ, &i);
    if (rc < 0) {
        perror("Error setting SPI speed");
    }
    memset(&xfer, 0, sizeof(xfer));
    xfer.speed_hz = iSPIFreq;
    xfer.cs_change = 0;
    xfer.delay_usecs = 0;
    xfer.bits_per_word = 8;

    if (file_spi < 0)
    {
        fprintf(stderr, "Failed to open the SPI bus\n");
        return -1;
    }
    return file_spi;
}

int writeSpi(int iHandle, unsigned char *pBuf, int iLen)
{
    int rc;
    xfer.rx_buf = 0;
    xfer.tx_buf = (unsigned long)pBuf;
    xfer.len = iLen;
    rc = ioctl(iHandle, SPI_IOC_MESSAGE(1), &xfer);
    return rc;
}


int main() {
do {
	printf("eerste");
	usleep(1000);
    initDisplay();
	printf("tweede\n");
    //displaySend(SEND_CMD, 0xA4); // Entire Display OFF, all pixels turns OFF in GS level 0
    //displaySend(SEND_CMD, 0xA5); // Entire Display ON, all pixels turns ON in GS level 15
    ClearDisplay();
    usleep(1000 * 1000);
printf("derde\n");
    CheckerboardOdd();
    usleep(1000 * 1000);
printf("vierde\n");
    CheckerboardEven();
    usleep(1000 * 1000);
    FillDisplay();
} while(1);
}
