#include <stdio.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <linux/types.h>
#include <errno.h>

#define GPIO_OUT 0
#define GPIO_IN 1
#define MAX_PINS 51
static int iPinHandles[MAX_PINS];

#define DISPLAY_SET_CURSOR_X 0x2A
#define DISPLAY_SET_CURSOR_Y 0x2B
#define DISPLAY_WRITE_PIXELS 0x2C

#define DISPLAY_NATIVE_WIDTH 320
#define DISPLAY_NATIVE_HEIGHT 480

#define DISPLAY_WIDTH DISPLAY_NATIVE_HEIGHT
#define DISPLAY_HEIGHT DISPLAY_NATIVE_WIDTH

#define SPI_BYTESPERPIXEL 2

/*
#define GPIO_SPI0_MOSI  10        // Pin P1-19, MOSI when SPI0 in use (display, touch)
#define GPIO_SPI0_MISO   9        // Pin P1-21, MISO when SPI0 in use (touch)
#define GPIO_SPI0_CLK   11        // Pin P1-23, CLK when SPI0 in use
#define GPIO_SPI0_CE0    8        // Pin P1-24, CE0 when SPI0 in use (display)
#define GPIO_SPI0_CE1    7        // Pin P1-26, CE1 when SPI0 in use (touch panel)
 */

#define GPIO_SPI0_MOSI 19
#define GPIO_SPI0_CLK 23
#define GPIO_SPI0_CE0 24

#define GPIO_TFT_DATA_CONTROL 18 // bcm 24
#define GPIO_TFT_RESET_PIN 22 // bcm 25
#define GPIO_BACKLIGHT 38

#define BEGIN_SPI_COMMUNICATION AIOWriteGPIO(GPIO_SPI0_CE0, 0);
#define END_SPI_COMMUNICATION AIOWriteGPIO(GPIO_SPI0_CE0, 1);

int handle;
static struct spi_ioc_transfer xfer;

void drawRow(int y, const unsigned short *dataBufPtr);
int AIOOpenSPI(int iChannel, int iSPIFreq);
int AIOWriteSPI(int iHandle, unsigned char *pBuf, int iLen);
int AIOWriteGPIO(int iPin, int iValue);
int AIOAddGPIO(int iPin, int iDirection);
void AIORemoveGPIO(int iPin);

void SPI_TRANSFER(char cmd, int num_args, ...) {
    va_list ap;

    unsigned char buf[1024] = {0}; // room for 1024 chars (or 1023 + a 0 byte for a string)
    unsigned char *origStr = buf;
    unsigned char *str = buf;
    *str++ = 0;
    *str++ = cmd;

    AIOWriteGPIO(GPIO_TFT_DATA_CONTROL, 0);

//    printspi(origStr, 2, 1);
    AIOWriteSPI(handle, origStr, 2);

    AIOWriteGPIO(GPIO_TFT_DATA_CONTROL, 1);

    str = origStr;

    va_start(ap, num_args);
    for (int i = 0; i < num_args; i++) {
        unsigned char x = va_arg(ap, int);
        *str = x;
        *str++;
    }
    va_end(ap);

//    printspi(origStr, num_args, 2);
    AIOWriteSPI(handle, origStr, num_args);

}

void initDisplay() {
    memset(iPinHandles, -1, sizeof(iPinHandles));

//    printf("GPIO_SPI0_CE0\n");
//    AIOAddGPIO(GPIO_SPI0_CE0, GPIO_OUT);
//    printf("GPIO_SPI0_MOSI\n");
//    AIOAddGPIO(GPIO_SPI0_MOSI, GPIO_OUT);
//    printf("GPIO_SPI0_CLK\n");
//    AIOAddGPIO(GPIO_SPI0_CLK, GPIO_OUT);
    printf("GPIO_TFT_DATA_CONTROL\n");
    AIOAddGPIO(GPIO_TFT_DATA_CONTROL, GPIO_OUT);
    printf("GPIO_TFT_RESET_PIN\n");
    AIOAddGPIO(GPIO_TFT_RESET_PIN, GPIO_OUT);
    AIOAddGPIO(GPIO_BACKLIGHT, GPIO_OUT);
    AIOWriteGPIO(GPIO_BACKLIGHT, 0);
    usleep(100 * 1000);



    // If a Reset pin is defined, toggle it briefly high->low->high to enable the device. Some devices do not have a reset pin, in which case compile with GPIO_TFT_RESET_PIN left undefined.
    printf("Resetting display at reset GPIO pin %d\n", GPIO_TFT_RESET_PIN);
    AIOWriteGPIO(GPIO_TFT_RESET_PIN, 1);
    usleep(120 * 1000);
    AIOWriteGPIO(GPIO_TFT_RESET_PIN, 0);
    usleep(120 * 1000);
    AIOWriteGPIO(GPIO_TFT_RESET_PIN, 1);
    usleep(120 * 1000);

    handle = AIOOpenSPI(2, 20000000);

//    unsigned char buf[1024] = {0}; // room for 1024 chars (or 1023 + a 0 byte for a string)
//    unsigned char *str = buf;
//    unsigned char c = 'a';
//    *str = c;
//
//    AIOWriteSPI(h, str, 1);

    // Do the initialization with a very low SPI bus speed, so that it will succeed even if the bus speed chosen by the user is too high.
//    __sync_synchronize();

    BEGIN_SPI_COMMUNICATION;
    {
        SPI_TRANSFER(0xB0/*Interface Mode Control*/, 2, 0x00,
                     0x00/*DE polarity=High enable, PCKL polarity=data fetched at rising time, HSYNC polarity=Low level sync clock, VSYNC polarity=Low level sync clock*/);
        SPI_TRANSFER(0x11/*Sleep OUT*/, 0);
        usleep(120 * 1000);

        const unsigned char pixelFormat = 0x55; /*DPI(RGB Interface)=16bits/pixel, DBI(CPU Interface)=16bits/pixel*/

        SPI_TRANSFER(0x3A/*Interface Pixel Format*/, 2, 0x00, pixelFormat);

        // Oddly, WaveShare 3.5" (B) seems to need Display Inversion ON, whereas WaveShare 3.5" (A) seems to need Display Inversion OFF for proper image. See https://github.com/juj/fbcp-ili9341/issues/8
        SPI_TRANSFER(0x20/*Display Inversion OFF*/, 0);

        SPI_TRANSFER(0xC0/*Power Control 1*/, 4, 0x00, 0x09, 0x00, 0x09);
        SPI_TRANSFER(0xC1/*Power Control 2*/, 4, 0x00, 0x41, 0x00, 0x00);
        SPI_TRANSFER(0xC2/*Power Control 3*/, 2, 0x00, 0x33);
        SPI_TRANSFER(0xC5/*VCOM Control*/, 4, 0x00, 0x00, 0x00, 0x36);

#define MADCTL_BGR_PIXEL_ORDER (1<<3)
#define MADCTL_ROW_COLUMN_EXCHANGE (1<<5)
#define MADCTL_COLUMN_ADDRESS_ORDER_SWAP (1<<6)
#define MADCTL_ROW_ADDRESS_ORDER_SWAP (1<<7)
#define MADCTL_ROTATE_180_DEGREES (MADCTL_COLUMN_ADDRESS_ORDER_SWAP | MADCTL_ROW_ADDRESS_ORDER_SWAP)

        unsigned char madctl = 0;
//        madctl |= MADCTL_BGR_PIXEL_ORDER;

        madctl |= MADCTL_ROW_COLUMN_EXCHANGE;
#ifdef DISPLAY_ROTATE_180_DEGREES
        madctl ^= MADCTL_ROTATE_180_DEGREES;
#endif

        SPI_TRANSFER(0x36/*MADCTL: Memory Access Control*/, 2, 0x00, madctl);
        SPI_TRANSFER(0xE0/*Positive Gamma Control*/, 30, 0x00, 0x00, 0x00, 0x2C, 0x00, 0x2C, 0x00, 0x0B, 0x00, 0x0C, 0x00,
                     0x04, 0x00, 0x4C, 0x00, 0x64, 0x00, 0x36, 0x00, 0x03, 0x00, 0x0E, 0x00, 0x01, 0x00, 0x10, 0x00,
                     0x01, 0x00, 0x00);
        SPI_TRANSFER(0xE1/*Negative Gamma Control*/, 30, 0x00, 0x0F, 0x00, 0x37, 0x00, 0x37, 0x00, 0x0C, 0x00, 0x0F, 0x00,
                     0x05, 0x00, 0x50, 0x00, 0x32, 0x00, 0x36, 0x00, 0x04, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x19, 0x00,
                     0x14, 0x00, 0x0F);
        SPI_TRANSFER(0xB6/*Display Function Control*/, 6, 0, 0, 0, /*ISC=2*/2, 0, /*Display Height h=*/
                     59); // Actual display height = (h+1)*8 so (59+1)*8=480
        SPI_TRANSFER(0x11/*Sleep OUT*/, 0);
        usleep(120 * 1000);
        SPI_TRANSFER(0x29/*Display ON*/, 0);
        SPI_TRANSFER(0x38/*Idle Mode OFF*/, 0);
        SPI_TRANSFER(0x13/*Normal Display Mode ON*/, 0);

        unsigned short dataBuf[1000];
        for(unsigned short i = 0; i < 1000; i += 1) {
            if (i < 20) {
                dataBuf[i] = 0xffff; // wit
            } else if (i < 40) {
                dataBuf[i] = 0x00ff; // lichtblauw
            } else if (i < 60) {
                dataBuf[i] = 0x0ff0; // paarsig
            } else if (i < 80) {
                dataBuf[i] = 0xff00; // rood
            } else if (i < 100) {
                dataBuf[i] = 0xf00f; // geel
            } else if (i < 120) {
                dataBuf[i] = 0xf000; // bruin
            } else if (i < 140) {
                dataBuf[i] = 0x0f00; // bruin
            } else if (i < 160) {
                dataBuf[i] = 0x00f0; // blauw
            } else if (i < 180) {
                dataBuf[i] = 0x000f; // groen
            } else if (i < 200) {
                dataBuf[i] = 0x0008;
            } else if (i < 220) {
                dataBuf[i] = 0x0880;
            } else if (i < 240) {
                dataBuf[i] = 0x8800;
            } else if (i < 260) {
                dataBuf[i] = 0x8008;
            } else {
                dataBuf[i] = 0xaaaa;
            }

        }
        unsigned short *dataBufPtr = dataBuf;


        for (int y = 0; y < DISPLAY_HEIGHT; ++y) {
            drawRow(y, dataBufPtr);

            if (y % 30 == 0 && y > 0) {
                dataBufPtr+=100;
            }
        }
        SPI_TRANSFER(DISPLAY_SET_CURSOR_X, 8, 0, 0, 0, 0, 0, (DISPLAY_WIDTH - 1) >> 8, 0, (DISPLAY_WIDTH - 1) & 0xFF);
        SPI_TRANSFER(DISPLAY_SET_CURSOR_Y, 8, 0, 0, 0, 0, 0, (DISPLAY_HEIGHT - 1) >> 8, 0, (DISPLAY_HEIGHT - 1) & 0xFF);
    }
    END_SPI_COMMUNICATION;
    usleep(1000*2000);
    AIOWriteGPIO(GPIO_BACKLIGHT, 1);
}

void drawRow(int y, const unsigned short *dataBufPtr) {
    SPI_TRANSFER(DISPLAY_SET_CURSOR_X, 8, 0, 0, 0, 0, 0, (DISPLAY_WIDTH - 1) >> 8, 0, (DISPLAY_WIDTH - 1) & 0xFF);
    SPI_TRANSFER(DISPLAY_SET_CURSOR_Y, 8, 0, (unsigned char) (y >> 8), 0, (unsigned char) (y & 0xFF), 0,
                 (DISPLAY_HEIGHT - 1) >> 8,
                 0, (DISPLAY_HEIGHT - 1) & 0xFF);

    unsigned char cmdBuf[2] = {0, DISPLAY_WRITE_PIXELS};

    AIOWriteGPIO(GPIO_TFT_DATA_CONTROL, 0);
    AIOWriteSPI(handle, cmdBuf, 2);
    AIOWriteGPIO(GPIO_TFT_DATA_CONTROL, 1);

    xfer.rx_buf = 0;
    xfer.tx_buf = (unsigned long) dataBufPtr;
    xfer.len = DISPLAY_WIDTH * SPI_BYTESPERPIXEL;

    ioctl(handle, SPI_IOC_MESSAGE(1), &xfer);
}

// Tinkerboard
static int iTinkerPins[] = {-1,-1,-1,252,-1,253,-1,17,161,-1,
                            160,164,184,166,-1,167,162,-1,163,257,
                            -1,256,171,254,255,-1,251,233,234,165,
                            -1,168,239,238,-1,185,223,224,187,-1,
                            188};

static int iRPIPins[] = {-1,-1,-1,2,-1,3,-1,4,14,-1,
                         15,17,18,27,-1,22,23,-1,24,10,
                         -1,9,25,11,8,-1,7,0,1,5,
                         -1,6,12,13,-1,19,16,26,20,-1,
                         21};

int AIOOpenSPI(int iChannel, int iSPIFreq)
{
    int rc, iSPIMode = SPI_MODE_0; // | SPI_NO_CS;
    char szName[32];
    int file_spi;
    int i = iSPIFreq;

    sprintf(szName,"/dev/spidev%d.0", iChannel);
    file_spi = open(szName, O_RDWR);
    rc = ioctl(file_spi, SPI_IOC_WR_MODE, &iSPIMode);
    if (rc < 0) fprintf(stderr, "Error setting SPI mode\n");
    rc = ioctl(file_spi, SPI_IOC_WR_MAX_SPEED_HZ, &i);
    if (rc < 0) fprintf(stderr, "Error setting SPI speed\n");
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
} /* AIOOpenSPI() */



int AIOWriteSPI(int iHandle, unsigned char *pBuf, int iLen)
{
    int rc;
    xfer.rx_buf = 0;
    xfer.tx_buf = (unsigned long)pBuf;
    xfer.len = iLen;
    rc = ioctl(iHandle, SPI_IOC_MESSAGE(1), &xfer);
    return rc;
} /* AIOWriteSPI() */


//
// Write a 0 or 1 to a GPIO output line
//
int AIOWriteGPIO(int iPin, int iValue)
{
    int rc, iGPIO;
    char szTemp[64];
    int *pPins;

    if (iPinHandles[iPin] == -1) // not open yet
    {
        pPins = iTinkerPins;
        iGPIO = pPins[iPin]; // convert to GPIO number
        sprintf(szTemp, "/sys/class/gpio/gpio%d/value", iGPIO);
        iPinHandles[iPin] = open(szTemp, O_WRONLY);
    }
    if (iValue) rc = write(iPinHandles[iPin], "1", 1);
    else rc = write(iPinHandles[iPin], "0", 1);
    if (rc < 0) // error
    { // do something
    }
    return 1;
} /* AIOWriteGPIO() */

int AIOAddGPIO(int iPin, int iDirection)
{
    char szName[64];
    int file_gpio, rc, iGPIO;
    int *pPins;

    pPins = iTinkerPins;

    file_gpio = open("/sys/class/gpio/export", O_WRONLY);
    if (file_gpio < 1) {
        fprintf(stderr, "Error opening /sys/class/gpio/export = %d\n", errno);
        return 0;
    }
    iGPIO = pPins[iPin];
    sprintf(szName, "%d", iGPIO);
    rc = write(file_gpio, szName, strlen(szName));
    close(file_gpio);
    usleep(200000); // allow time for udev to make the needed changes
    sprintf(szName, "/sys/class/gpio/gpio%d/direction", iGPIO);
    file_gpio = open(szName, O_WRONLY);
    if (file_gpio < 1) {
        fprintf(stderr, "error setting direction on GPIO pin %d\n", iGPIO);
        return 0;
    }
    if (iDirection == GPIO_OUT)
        rc = write(file_gpio, "out\n", 4);
    else
        rc = write(file_gpio, "in\n", 3);
    close(file_gpio);
    if (rc < 0)
    {
        fprintf(stderr, "Error setting mode on GPIO pin %d\n", iGPIO);
    }
    return 1;
} /* AIOAddGPIO() */