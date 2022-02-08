#ifndef KERNEL_MODULE
#include <stdio.h> // printf, stderr
#include <syslog.h> // syslog
#include <fcntl.h> // open, O_RDWR, O_SYNC
#include <sys/mman.h> // mmap, munmap
#include <bcm_host.h> // bcm_host_get_peripheral_address, bcm_host_get_peripheral_size, bcm_host_get_sdram_address
#endif

#include "config.h"
#include "spi.h"
#include "util.h"
#include "mem_alloc.h"

// Uncomment this to print out all bytes sent to the SPI bus
// #define DEBUG_SPI_BUS_WRITES

#ifdef DEBUG_SPI_BUS_WRITES
#define DEBUG_PRINT_WRITTEN_BYTE(byte) do { \
  printf("%02X", byte); \
  if ((writeCounter & 3) == 0) printf("\n"); \
  } while(0)
#else
#define DEBUG_PRINT_WRITTEN_BYTE(byte) ((void)0)
#endif

#ifdef CHIP_SELECT_LINE_NEEDS_REFRESHING_EACH_32BITS_WRITTEN
void ChipSelectHigh();
#define TOGGLE_CHIP_SELECT_LINE() if ((++writeCounter & 3) == 0) { ChipSelectHigh(); }
#else
#define TOGGLE_CHIP_SELECT_LINE() ((void)0)
#endif

static uint32_t writeCounter = 0;

#define WRITE_FIFO(word) do { \
  uint8_t w = (word); \
  spi->fifo = w; \
  TOGGLE_CHIP_SELECT_LINE(); \
  DEBUG_PRINT_WRITTEN_BYTE(w); \
  } while(0)

int mem_fd = -1;
volatile void *bcm2835 = 0;
volatile GPIORegisterFile *gpio = 0;
volatile SPIRegisterFile *spi = 0;

// Points to the system timer register. N.B. spec sheet says this is two low and high parts, in an 32-bit aligned (but not 64-bit aligned) address. Profiling shows
// that Pi 3 Model B does allow reading this as a u64 load, and even when unaligned, it is around 30% faster to do so compared to loading in parts "lo | (hi << 32)".
volatile uint64_t *systemTimerRegister = 0;

void DumpSPICS(uint32_t reg)
{
  PRINT_FLAG(BCM2835_SPI0_CS_CS);
  PRINT_FLAG(BCM2835_SPI0_CS_CPHA);
  PRINT_FLAG(BCM2835_SPI0_CS_CPOL);
  PRINT_FLAG(BCM2835_SPI0_CS_CLEAR_TX);
  PRINT_FLAG(BCM2835_SPI0_CS_CLEAR_RX);
  PRINT_FLAG(BCM2835_SPI0_CS_TA);
  PRINT_FLAG(BCM2835_SPI0_CS_DMAEN);
  PRINT_FLAG(BCM2835_SPI0_CS_INTD);
  PRINT_FLAG(BCM2835_SPI0_CS_INTR);
  PRINT_FLAG(BCM2835_SPI0_CS_ADCS);
  PRINT_FLAG(BCM2835_SPI0_CS_DONE);
  PRINT_FLAG(BCM2835_SPI0_CS_RXD);
  PRINT_FLAG(BCM2835_SPI0_CS_TXD);
  PRINT_FLAG(BCM2835_SPI0_CS_RXR);
  PRINT_FLAG(BCM2835_SPI0_CS_RXF);
  printf("SPI0 DLEN: %u\n", spi->dlen);
  printf("SPI0 CE0 register: %d\n", GET_GPIO(GPIO_SPI0_CE0) ? 1 : 0);
}

// Errata to BCM2835 behavior: documentation states that the SPI0 DLEN register is only used for DMA. However, even when DMA is not being utilized, setting it from
// a value != 0 or 1 gets rid of an excess idle clock cycle that is present when transmitting each byte. (by default in Polled SPI Mode each 8 bits transfer in 9 clocks)
// With DLEN=2 each byte is clocked to the bus in 8 cycles, observed to improve max throughput from 56.8mbps to 63.3mbps (+11.4%, quite close to the theoretical +12.5%)
// https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=181154
#define UNLOCK_FAST_8_CLOCKS_SPI() (spi->dlen = 2)

void WaitForPolledSPITransferToFinish()
{
  uint32_t cs;
  while (!(((cs = spi->cs) ^ BCM2835_SPI0_CS_TA) & (BCM2835_SPI0_CS_DONE | BCM2835_SPI0_CS_TA))) // While TA=1 and DONE=0
    if ((cs & (BCM2835_SPI0_CS_RXR | BCM2835_SPI0_CS_RXF)))
      spi->cs = BCM2835_SPI0_CS_CLEAR_RX | BCM2835_SPI0_CS_TA | DISPLAY_SPI_DRIVE_SETTINGS;

  if ((cs & BCM2835_SPI0_CS_RXD)) spi->cs = BCM2835_SPI0_CS_CLEAR_RX | BCM2835_SPI0_CS_TA | DISPLAY_SPI_DRIVE_SETTINGS;
}

void RunSPITask(SPITask *task)
{
  WaitForPolledSPITransferToFinish();

  uint8_t *tStart = task->PayloadStart();
  uint8_t *tEnd = task->PayloadEnd();
  const uint32_t payloadSize = tEnd - tStart;
  uint8_t *tPrefillEnd = tStart + MIN(15, payloadSize);

  // An SPI transfer to the display always starts with one control (command) byte, followed by N data bytes.
  CLEAR_GPIO(GPIO_TFT_DATA_CONTROL);

  WRITE_FIFO(0x00);
  WRITE_FIFO(task->cmd);

  while(!(spi->cs & (BCM2835_SPI0_CS_DONE))) /*nop*/;
  spi->fifo;
  spi->fifo;

  SET_GPIO(GPIO_TFT_DATA_CONTROL);

  {
    while(tStart < tPrefillEnd) WRITE_FIFO(*tStart++);
    while(tStart < tEnd)
    {
      uint32_t cs = spi->cs;
      if ((cs & BCM2835_SPI0_CS_TXD)) WRITE_FIFO(*tStart++);
// TODO:      else asm volatile("yield");
      if ((cs & (BCM2835_SPI0_CS_RXR|BCM2835_SPI0_CS_RXF))) spi->cs = BCM2835_SPI0_CS_CLEAR_RX | BCM2835_SPI0_CS_TA | DISPLAY_SPI_DRIVE_SETTINGS;
    }
  }
}

SharedMemory *spiTaskMemory = 0;
volatile uint64_t spiThreadIdleUsecs = 0;
volatile uint64_t spiThreadSleepStartTime = 0;
volatile int spiThreadSleeping = 0;
double spiUsecsPerByte;

SPITask *GetTask() // Returns the first task in the queue, called in worker thread
{
  uint32_t head = spiTaskMemory->queueHead;
  uint32_t tail = spiTaskMemory->queueTail;
  if (head == tail) return 0;
  SPITask *task = (SPITask*)(spiTaskMemory->buffer + head);
  if (task->cmd == 0) // Wrapped around?
  {
    spiTaskMemory->queueHead = 0;
    __sync_synchronize();
    if (tail == 0) return 0;
    task = (SPITask*)spiTaskMemory->buffer;
  }
  return task;
}

void DoneTask(SPITask *task) // Frees the first SPI task from the queue, called in worker thread
{
  __atomic_fetch_sub(&spiTaskMemory->spiBytesQueued, task->PayloadSize()+1, __ATOMIC_RELAXED);
  spiTaskMemory->queueHead = (uint32_t)((uint8_t*)task - spiTaskMemory->buffer) + sizeof(SPITask) + task->size;
  __sync_synchronize();
}

void ExecuteSPITasks()
{
#ifndef USE_DMA_TRANSFERS
  BEGIN_SPI_COMMUNICATION();
#endif
  {
    while(spiTaskMemory->queueTail != spiTaskMemory->queueHead)
    {
      SPITask *task = GetTask();
      if (task)
      {
        RunSPITask(task);
        DoneTask(task);
      }
    }
  }
#ifndef USE_DMA_TRANSFERS
  END_SPI_COMMUNICATION();
#endif
}

int InitSPI()
{

  // Memory map GPIO and SPI peripherals for direct access
  mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
  if (mem_fd < 0) FATAL_ERROR("can't open /dev/mem (run as sudo)");
  printf("bcm_host_get_peripheral_address: %p, bcm_host_get_peripheral_size: %u, bcm_host_get_sdram_address: %p\n", bcm_host_get_peripheral_address(), bcm_host_get_peripheral_size(), bcm_host_get_sdram_address());
  bcm2835 = mmap(NULL, bcm_host_get_peripheral_size(), (PROT_READ | PROT_WRITE), MAP_SHARED, mem_fd, bcm_host_get_peripheral_address());
  if (bcm2835 == MAP_FAILED) FATAL_ERROR("mapping /dev/mem failed");
  spi = (volatile SPIRegisterFile*)((uintptr_t)bcm2835 + BCM2835_SPI0_BASE);
  gpio = (volatile GPIORegisterFile*)((uintptr_t)bcm2835 + BCM2835_GPIO_BASE);
  systemTimerRegister = (volatile uint64_t*)((uintptr_t)bcm2835 + BCM2835_TIMER_BASE + 0x04); // Generates an unaligned 64-bit pointer, but seems to be fine.
  // TODO: On graceful shutdown, (ctrl-c signal?) close(mem_fd)

#if !defined(KERNEL_MODULE_CLIENT) || defined(KERNEL_MODULE_CLIENT_DRIVES)
  // By default all GPIO pins are in input mode (0x00), initialize them for SPI and GPIO writes
#ifdef GPIO_TFT_DATA_CONTROL
  SET_GPIO_MODE(GPIO_TFT_DATA_CONTROL, 0x01); // Data/Control pin to output (0x01)
#endif
  // The Pirate Audio hat ST7789 based display has Data/Control on the MISO pin, so only initialize the pin as MISO if the
  // Data/Control pin does not use it.
#if !defined(GPIO_TFT_DATA_CONTROL) || GPIO_TFT_DATA_CONTROL != GPIO_SPI0_MISO
  SET_GPIO_MODE(GPIO_SPI0_MISO, 0x04);
#endif
  SET_GPIO_MODE(GPIO_SPI0_MOSI, 0x04);
  SET_GPIO_MODE(GPIO_SPI0_CLK, 0x04);

#ifdef DISPLAY_NEEDS_CHIP_SELECT_SIGNAL
  // The Adafruit 1.65" 240x240 ST7789 based display is unique compared to others that it does want to see the Chip Select line go
  // low and high to start a new command. For that display we let hardware SPI toggle the CS line, and actually run TA<-0 and TA<-1
  // transitions to let the CS line live. For most other displays, we just set CS line always enabled for the display throughout
  // fbcp-ili9341 lifetime, which is a tiny bit faster.
  SET_GPIO_MODE(GPIO_SPI0_CE0, 0x04);
#ifdef DISPLAY_USES_CS1
  SET_GPIO_MODE(GPIO_SPI0_CE1, 0x04);
#endif
#else
  // Set the SPI 0 pin explicitly to output, and enable chip select on the line by setting it to low.
  // fbcp-ili9341 assumes exclusive access to the SPI0 bus, and exclusive presence of only one device on the bus,
  // which is (permanently) activated here.
  SET_GPIO_MODE(GPIO_SPI0_CE0, 0x01);
  CLEAR_GPIO(GPIO_SPI0_CE0);
#ifdef DISPLAY_USES_CS1
  SET_GPIO_MODE(GPIO_SPI0_CE1, 0x01);
#endif
#endif

  spi->cs = BCM2835_SPI0_CS_CLEAR | DISPLAY_SPI_DRIVE_SETTINGS; // Initialize the Control and Status register to defaults: CS=0 (Chip Select), CPHA=0 (Clock Phase), CPOL=0 (Clock Polarity), CSPOL=0 (Chip Select Polarity), TA=0 (Transfer not active), and reset TX and RX queues.
  spi->clk = SPI_BUS_CLOCK_DIVISOR; // Clock Divider determines SPI bus speed, resulting speed=256MHz/clk
#endif

  // Initialize SPI thread task buffer memory
#ifdef KERNEL_MODULE_CLIENT
  int driverfd = open("/proc/bcm2835_spi_display_bus", O_RDWR|O_SYNC);
  if (driverfd < 0) FATAL_ERROR("Could not open SPI ring buffer - kernel driver module not running?");
  spiTaskMemory = (SharedMemory*)mmap(NULL, SHARED_MEMORY_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED/* | MAP_NORESERVE | MAP_POPULATE | MAP_LOCKED*/, driverfd, 0);
  close(driverfd);
  if (spiTaskMemory == MAP_FAILED) FATAL_ERROR("Could not mmap SPI ring buffer!");
  printf("Got shared memory block %p, ring buffer head %p, ring buffer tail %p, shared memory block phys address: %p\n", (const char *)spiTaskMemory, spiTaskMemory->queueHead, spiTaskMemory->queueTail, spiTaskMemory->sharedMemoryBaseInPhysMemory);

#ifdef USE_DMA_TRANSFERS
  printf("DMA TX channel: %d, DMA RX channel: %d\n", spiTaskMemory->dmaTxChannel, spiTaskMemory->dmaRxChannel);
#endif

#else

#ifdef KERNEL_MODULE
  spiTaskMemory = (SharedMemory*)kmalloc(SHARED_MEMORY_SIZE, GFP_KERNEL | GFP_DMA);
  // TODO: Ideally we would be able to directly perform the DMA from the SPI ring buffer in 'spiTaskMemory'. However
  // that pointer is shared to userland, and it is proving troublesome to make it both userland-writable as well as cache-bypassing DMA coherent.
  // Therefore these two memory areas are separate for now, and we memcpy() from SPI ring buffer to the following intermediate 'dmaSourceMemory'
  // memory area to perform the DMA transfer. Is there a way to avoid this intermediate buffer? That would improve performance a bit.
  dmaSourceMemory = (SharedMemory*)dma_alloc_writecombine(0, SHARED_MEMORY_SIZE, &spiTaskMemoryPhysical, GFP_KERNEL);
  LOG("Allocated DMA memory: mem: %p, phys: %p", spiTaskMemory, (void*)spiTaskMemoryPhysical);
  memset((void*)spiTaskMemory, 0, SHARED_MEMORY_SIZE);
#else
  spiTaskMemory = (SharedMemory*)Malloc(SHARED_MEMORY_SIZE, "spi.cpp shared task memory");
#endif

  spiTaskMemory->queueHead = spiTaskMemory->queueTail = spiTaskMemory->spiBytesQueued = 0;
#endif

#ifdef USE_DMA_TRANSFERS
  InitDMA();
#endif

  // Enable fast 8 clocks per byte transfer mode, instead of slower 9 clocks per byte.
  UNLOCK_FAST_8_CLOCKS_SPI();

#if !defined(KERNEL_MODULE) && (!defined(KERNEL_MODULE_CLIENT) || defined(KERNEL_MODULE_CLIENT_DRIVES))
  printf("Initializing display\n");


#endif

  LOG("InitSPI done");
  return 0;
}

void DeinitSPI()
{
  DeinitSPIDisplay();
#ifdef USE_DMA_TRANSFERS
  DeinitDMA();
#endif

  spi->cs = BCM2835_SPI0_CS_CLEAR | DISPLAY_SPI_DRIVE_SETTINGS;

#ifndef KERNEL_MODULE_CLIENT
#ifdef GPIO_TFT_DATA_CONTROL
  SET_GPIO_MODE(GPIO_TFT_DATA_CONTROL, 0);
#endif
  SET_GPIO_MODE(GPIO_SPI0_CE1, 0);
  SET_GPIO_MODE(GPIO_SPI0_CE0, 0);
  SET_GPIO_MODE(GPIO_SPI0_MISO, 0);
  SET_GPIO_MODE(GPIO_SPI0_MOSI, 0);
  SET_GPIO_MODE(GPIO_SPI0_CLK, 0);
#endif

  if (bcm2835)
  {
    munmap((void*)bcm2835, bcm_host_get_peripheral_size());
    bcm2835 = 0;
  }

  if (mem_fd >= 0)
  {
    close(mem_fd);
    mem_fd = -1;
  }

#ifndef KERNEL_MODULE_CLIENT

#ifdef KERNEL_MODULE
  kfree(spiTaskMemory);
  dma_free_writecombine(0, SHARED_MEMORY_SIZE, dmaSourceMemory, spiTaskMemoryPhysical);
  spiTaskMemoryPhysical = 0;
#else
  free(spiTaskMemory);
#endif
#endif
  spiTaskMemory = 0;
}
