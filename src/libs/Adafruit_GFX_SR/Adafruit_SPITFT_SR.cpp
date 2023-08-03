/*!
 * @file Adafruit_SPITFT.cpp
 *
 * @mainpage Adafruit SPI TFT Displays (and some others)
 *
 * @section intro_sec Introduction
 *
 * Part of Adafruit's GFX graphics library. Originally this class was
 * written to handle a range of color TFT displays connected via SPI,
 * but over time this library and some display-specific subclasses have
 * mutated to include some color OLEDs as well as parallel-interfaced
 * displays. The name's been kept for the sake of older code.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!

 * @section dependencies Dependencies
 *
 * This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
 * Adafruit_GFX</a> being present on your system. Please make sure you have
 * installed the latest version before using this library.
 *
 * @section author Author
 *
 * Written by Limor "ladyada" Fried for Adafruit Industries,
 * with contributions from the open source community.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 */

#if !defined(__AVR_ATtiny85__) // Not for ATtiny, at all

#include "Adafruit_SPITFT_SR.h"

#if defined(__AVR__)
#define AVR_WRITESPI(x) hwspi._spi->transfer(x);
#endif

#if defined(PORT_IOBUS)
// On SAMD21, redefine digitalPinToPort() to use the slightly-faster
// PORT_IOBUS rather than PORT (not needed on SAMD51).
#undef digitalPinToPort
#define digitalPinToPort(P) (&(PORT_IOBUS->Group[g_APinDescription[P].ulPort]))
#endif // end PORT_IOBUS

#if defined(USE_SPI_DMA) && (defined(__SAMD51__) || defined(ARDUINO_SAMD_ZERO))
// #pragma message ("GFX DMA IS ENABLED. HIGHLY EXPERIMENTAL.")
#include "wiring_private.h" // pinPeripheral() function
#include <Adafruit_ZeroDMA.h>
#include <malloc.h>          // memalign() function
#define tcNum 2              // Timer/Counter for parallel write strobe PWM
#define wrPeripheral PIO_CCL // Use CCL to invert write strobe

// DMA transfer-in-progress indicator and callback
static volatile bool dma_busy = false;
static void dma_callback(Adafruit_ZeroDMA *dma) { dma_busy = false; }

#if defined(__SAMD51__)
// Timer/counter info by index #
static const struct
{
  Tc *tc;   // -> Timer/Counter base address
  int gclk; // GCLK ID
  int evu;  // EVSYS user ID
} tcList[] = {{TC0, TC0_GCLK_ID, EVSYS_ID_USER_TC0_EVU},
              {TC1, TC1_GCLK_ID, EVSYS_ID_USER_TC1_EVU},
              {TC2, TC2_GCLK_ID, EVSYS_ID_USER_TC2_EVU},
              {TC3, TC3_GCLK_ID, EVSYS_ID_USER_TC3_EVU},
#if defined(TC4)
              {TC4, TC4_GCLK_ID, EVSYS_ID_USER_TC4_EVU},
#endif
#if defined(TC5)
              {TC5, TC5_GCLK_ID, EVSYS_ID_USER_TC5_EVU},
#endif
#if defined(TC6)
              {TC6, TC6_GCLK_ID, EVSYS_ID_USER_TC6_EVU},
#endif
#if defined(TC7)
              {TC7, TC7_GCLK_ID, EVSYS_ID_USER_TC7_EVU}
#endif
};
#define NUM_TIMERS (sizeof tcList / sizeof tcList[0]) ///< # timer/counters
#endif                                                // end __SAMD51__

#endif // end USE_SPI_DMA

// Possible values for Adafruit_SPITFT.connection:
#define TFT_HARD_SPI 0              ///< Display interface = hardware SPI
#define TFT_SOFT_SPI 1              ///< Display interface = software SPI
#define TFT_PARALLEL 2              ///< Display interface = 8- or 16-bit parallel
#define TFT_HYBRID_HWSPI_PARALLEL 3 // Custom by Soldered

// CONSTRUCTORS ------------------------------------------------------------

// Custom constructor by Soldered
Adafruit_SPITFT::Adafruit_SPITFT(uint16_t w, uint16_t h, SPIClass *spiClass, int8_t cs, int8_t dc, int8_t wr, int8_t rd, int8_t rst)
    : Adafruit_GFX(w, h), connection(TFT_HYBRID_HWSPI_PARALLEL), _rst(rst), _cs(cs), _dc(dc)
{
  tft8._wr = wr;
  tft8._rd = rd;
  tft8.wide = true; // 16 bit
  hwspi._spi = spiClass;
}

// end constructors -------

// CLASS MEMBER FUNCTIONS --------------------------------------------------

// begin() and setAddrWindow() MUST be declared by any subclass.

/*!
    @brief  Configure microcontroller pins for TFT interfacing. Typically
            called by a subclass' begin() function.
    @param  freq     SPI frequency when using hardware SPI. If default (0)
                     is passed, will fall back on a device-specific value.
                     Value is ignored when using software SPI or parallel
                     connection.
    @param  spiMode  SPI mode when using hardware SPI. MUST be one of the
                     values SPI_MODE0, SPI_MODE1, SPI_MODE2 or SPI_MODE3
                     defined in SPI.h. Do NOT attempt to pass '0' for
                     SPI_MODE0 and so forth...the values are NOT the same!
                     Use ONLY the defines! (Pity it's not an enum.)
    @note   Another anachronistically-named function; this is called even
            when the display connection is parallel (not SPI). Also, this
            could probably be made private...quite a few class functions
            were generously put in the public section.
*/
void Adafruit_SPITFT::initSPI(uint32_t freq, uint8_t spiMode)
{

  if (!freq)
    freq = DEFAULT_SPI_FREQ; // If no freq specified, use default

  // Init basic control pins common to all connection types
  if (_cs >= 0)
  {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH); // Deselect
  }
  pinMode(_dc, OUTPUT);
  digitalWrite(_dc, HIGH); // Data mode

  if (connection == TFT_HARD_SPI)
  {

#if defined(SPI_HAS_TRANSACTION)
    hwspi.settings = SPISettings(freq, MSBFIRST, spiMode);
#else
    hwspi._freq = freq; // Save freq value for later
#endif
    hwspi._mode = spiMode; // Save spiMode value for later
    // Call hwspi._spi->begin() ONLY if this is among the 'established'
    // SPI interfaces in variant.h. For DIY roll-your-own SERCOM SPIs,
    // begin() and pinPeripheral() calls MUST be made in one's calling
    // code, BEFORE the screen-specific begin/init function is called.
    // Reason for this is that SPI::begin() makes its own calls to
    // pinPeripheral() based on g_APinDescription[n].ulPinType, which
    // on non-established SPI interface pins will always be PIO_DIGITAL
    // or similar, while we need PIO_SERCOM or PIO_SERCOM_ALT...it's
    // highly unique between devices and variants for each pin or
    // SERCOM so we can't make those calls ourselves here. And the SPI
    // device needs to be set up before calling this because it's
    // immediately followed with initialization commands. Blargh.
    if (
#if !defined(SPI_INTERFACES_COUNT)
        1
#else
#if SPI_INTERFACES_COUNT > 0
        (hwspi._spi == &SPI)
#endif
#if SPI_INTERFACES_COUNT > 1
        || (hwspi._spi == &SPI1)
#endif
#if SPI_INTERFACES_COUNT > 2
        || (hwspi._spi == &SPI2)
#endif
#if SPI_INTERFACES_COUNT > 3
        || (hwspi._spi == &SPI3)
#endif
#if SPI_INTERFACES_COUNT > 4
        || (hwspi._spi == &SPI4)
#endif
#if SPI_INTERFACES_COUNT > 5
        || (hwspi._spi == &SPI5)
#endif
#endif // end SPI_INTERFACES_COUNT
    )
    {
      hwspi._spi->begin();
    }
  }
  // Custom by Soldered
  else if (connection == TFT_HYBRID_HWSPI_PARALLEL)
  {
// First init the HW SPI like normal, for the shift registers
#if defined(SPI_HAS_TRANSACTION)
    hwspi.settings = SPISettings(freq, MSBFIRST, spiMode);
#else
    hwspi._freq = freq; // Save freq value for later
#endif
    hwspi._mode = spiMode; // Save spiMode value for later
    hwspi._spi->begin();

    // Then init the other pins required
    pinMode(tft8._wr, OUTPUT);
    digitalWrite(tft8._wr, HIGH);
    if (tft8._rd >= 0)
    {
      pinMode(tft8._rd, OUTPUT);
      digitalWrite(tft8._rd, HIGH);
    }
  }

  if (_rst >= 0)
  {
    // Toggle _rst low to reset
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(100);
    digitalWrite(_rst, LOW);
    delay(100);
    digitalWrite(_rst, HIGH);
    delay(200);
  }

#if defined(USE_SPI_DMA) && (defined(__SAMD51__) || defined(ARDUINO_SAMD_ZERO))
  if (((connection == TFT_HARD_SPI) || (connection == TFT_PARALLEL)) &&
      (dma.allocate() == DMA_STATUS_OK))
  { // Allocate channel
    // The DMA library needs to alloc at least one valid descriptor,
    // so we do that here. It's not used in the usual sense though,
    // just before a transfer we copy descriptor[0] to this address.
    if (dptr = dma.addDescriptor(NULL, NULL, 42, DMA_BEAT_SIZE_BYTE, false,
                                 false))
    {
      // Alloc 2 scanlines worth of pixels on display's major axis,
      // whichever that is, rounding each up to 2-pixel boundary.
      int major = (WIDTH > HEIGHT) ? WIDTH : HEIGHT;
      major += (major & 1);   // -> next 2-pixel bound, if needed.
      maxFillLen = major * 2; // 2 scanlines
      // Note to future self: if you decide to make the pixel buffer
      // much larger, remember that DMA transfer descriptors can't
      // exceed 65,535 bytes (not 65,536), meaning 32,767 pixels max.
      // Not that we have that kind of RAM to throw around right now.
      if ((pixelBuf[0] = (uint16_t *)malloc(maxFillLen * sizeof(uint16_t))))
      {
        // Alloc OK. Get pointer to start of second scanline.
        pixelBuf[1] = &pixelBuf[0][major];
        // Determine number of DMA descriptors needed to cover
        // entire screen when entire 2-line pixelBuf is used
        // (round up for fractional last descriptor).
        int numDescriptors = (WIDTH * HEIGHT + (maxFillLen - 1)) / maxFillLen;
        // DMA descriptors MUST be 128-bit (16 byte) aligned.
        // memalign() is considered obsolete but it's replacements
        // (aligned_alloc() or posix_memalign()) are not currently
        // available in the version of ARM GCC in use, but this
        // is, so here we are.
        if ((descriptor = (DmacDescriptor *)memalign(
                 16, numDescriptors * sizeof(DmacDescriptor))))
        {
          int dmac_id;
          volatile uint32_t *data_reg;

          if (connection == TFT_HARD_SPI)
          {
            // THIS IS AN AFFRONT TO NATURE, but I don't know
            // any "clean" way to get the sercom number from the
            // the SPIClass pointer (e.g. &SPI or &SPI1), which
            // is all we have to work with. SPIClass does contain
            // a SERCOM pointer but it is a PRIVATE member!
            // Doing an UNSPEAKABLY HORRIBLE THING here, directly
            // accessing the first 32-bit value in the SPIClass
            // structure, knowing that's (currently) where the
            // SERCOM pointer lives, but this ENTIRELY DEPENDS on
            // that structure not changing nor the compiler
            // rearranging things. Oh the humanity!

            if (*(SERCOM **)hwspi._spi == &sercom0)
            {
              dmac_id = SERCOM0_DMAC_ID_TX;
              data_reg = &SERCOM0->SPI.DATA.reg;
#if defined SERCOM1
            }
            else if (*(SERCOM **)hwspi._spi == &sercom1)
            {
              dmac_id = SERCOM1_DMAC_ID_TX;
              data_reg = &SERCOM1->SPI.DATA.reg;
#endif
#if defined SERCOM2
            }
            else if (*(SERCOM **)hwspi._spi == &sercom2)
            {
              dmac_id = SERCOM2_DMAC_ID_TX;
              data_reg = &SERCOM2->SPI.DATA.reg;
#endif
#if defined SERCOM3
            }
            else if (*(SERCOM **)hwspi._spi == &sercom3)
            {
              dmac_id = SERCOM3_DMAC_ID_TX;
              data_reg = &SERCOM3->SPI.DATA.reg;
#endif
#if defined SERCOM4
            }
            else if (*(SERCOM **)hwspi._spi == &sercom4)
            {
              dmac_id = SERCOM4_DMAC_ID_TX;
              data_reg = &SERCOM4->SPI.DATA.reg;
#endif
#if defined SERCOM5
            }
            else if (*(SERCOM **)hwspi._spi == &sercom5)
            {
              dmac_id = SERCOM5_DMAC_ID_TX;
              data_reg = &SERCOM5->SPI.DATA.reg;
#endif
#if defined SERCOM6
            }
            else if (*(SERCOM **)hwspi._spi == &sercom6)
            {
              dmac_id = SERCOM6_DMAC_ID_TX;
              data_reg = &SERCOM6->SPI.DATA.reg;
#endif
#if defined SERCOM7
            }
            else if (*(SERCOM **)hwspi._spi == &sercom7)
            {
              dmac_id = SERCOM7_DMAC_ID_TX;
              data_reg = &SERCOM7->SPI.DATA.reg;
#endif
            }
            dma.setPriority(DMA_PRIORITY_3);
            dma.setTrigger(dmac_id);
            dma.setAction(DMA_TRIGGER_ACTON_BEAT);

            // Initialize descriptor list.
            for (int d = 0; d < numDescriptors; d++)
            {
              // No need to set SRCADDR, DESCADDR or BTCNT --
              // those are done in the pixel-writing functions.
              descriptor[d].BTCTRL.bit.VALID = true;
              descriptor[d].BTCTRL.bit.EVOSEL = DMA_EVENT_OUTPUT_DISABLE;
              descriptor[d].BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_NOACT;
              descriptor[d].BTCTRL.bit.BEATSIZE = DMA_BEAT_SIZE_BYTE;
              descriptor[d].BTCTRL.bit.DSTINC = 0;
              descriptor[d].BTCTRL.bit.STEPSEL = DMA_STEPSEL_SRC;
              descriptor[d].BTCTRL.bit.STEPSIZE =
                  DMA_ADDRESS_INCREMENT_STEP_SIZE_1;
              descriptor[d].DSTADDR.reg = (uint32_t)data_reg;
            }
          }
          else
          { // Parallel connection

#if defined(__SAMD51__)
            int dmaChannel = dma.getChannel();
            // Enable event output, use EVOSEL output
            DMAC->Channel[dmaChannel].CHEVCTRL.bit.EVOE = 1;
            DMAC->Channel[dmaChannel].CHEVCTRL.bit.EVOMODE = 0;

            // CONFIGURE TIMER/COUNTER (for write strobe)

            Tc *timer = tcList[tcNum].tc; // -> Timer struct
            int id = tcList[tcNum].gclk;  // Timer GCLK ID
            GCLK_PCHCTRL_Type pchctrl;

            // Set up timer clock source from GCLK
            GCLK->PCHCTRL[id].bit.CHEN = 0; // Stop timer
            while (GCLK->PCHCTRL[id].bit.CHEN)
              ; // Wait for it
            pchctrl.bit.GEN = GCLK_PCHCTRL_GEN_GCLK0_Val;
            pchctrl.bit.CHEN = 1; // Enable
            GCLK->PCHCTRL[id].reg = pchctrl.reg;
            while (!GCLK->PCHCTRL[id].bit.CHEN)
              ; // Wait for it

            // Disable timer/counter before configuring it
            timer->COUNT8.CTRLA.bit.ENABLE = 0;
            while (timer->COUNT8.SYNCBUSY.bit.STATUS)
              ;

            timer->COUNT8.WAVE.bit.WAVEGEN = 2;    // NPWM
            timer->COUNT8.CTRLA.bit.MODE = 1;      // 8-bit
            timer->COUNT8.CTRLA.bit.PRESCALER = 0; // 1:1
            while (timer->COUNT8.SYNCBUSY.bit.STATUS)
              ;

            timer->COUNT8.CTRLBCLR.bit.DIR = 1; // Count UP
            while (timer->COUNT8.SYNCBUSY.bit.CTRLB)
              ;
            timer->COUNT8.CTRLBSET.bit.ONESHOT = 1; // One-shot
            while (timer->COUNT8.SYNCBUSY.bit.CTRLB)
              ;
            timer->COUNT8.PER.reg = 6; // PWM top
            while (timer->COUNT8.SYNCBUSY.bit.PER)
              ;
            timer->COUNT8.CC[0].reg = 2; // Compare
            while (timer->COUNT8.SYNCBUSY.bit.CC0)
              ;
            // Enable async input events,
            // event action = restart.
            timer->COUNT8.EVCTRL.bit.TCEI = 1;
            timer->COUNT8.EVCTRL.bit.EVACT = 1;

            // Enable timer
            timer->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
            while (timer->COUNT8.SYNCBUSY.bit.STATUS)
              ;

#if (wrPeripheral == PIO_CCL)
            // CONFIGURE CCL (inverts timer/counter output)

            MCLK->APBCMASK.bit.CCL_ = 1;         // Enable CCL clock
            CCL->CTRL.bit.ENABLE = 0;            // Disable to config
            CCL->CTRL.bit.SWRST = 1;             // Reset CCL registers
            CCL->LUTCTRL[tcNum].bit.ENABLE = 0;  // Disable LUT
            CCL->LUTCTRL[tcNum].bit.FILTSEL = 0; // No filter
            CCL->LUTCTRL[tcNum].bit.INSEL0 = 6;  // TC input
            CCL->LUTCTRL[tcNum].bit.INSEL1 = 0;  // MASK
            CCL->LUTCTRL[tcNum].bit.INSEL2 = 0;  // MASK
            CCL->LUTCTRL[tcNum].bit.TRUTH = 1;   // Invert in 0
            CCL->LUTCTRL[tcNum].bit.ENABLE = 1;  // Enable LUT
            CCL->CTRL.bit.ENABLE = 1;            // Enable CCL
#endif

            // CONFIGURE EVENT SYSTEM

            // Set up event system clock source from GCLK...
            // Disable EVSYS, wait for disable
            GCLK->PCHCTRL[EVSYS_GCLK_ID_0].bit.CHEN = 0;
            while (GCLK->PCHCTRL[EVSYS_GCLK_ID_0].bit.CHEN)
              ;
            pchctrl.bit.GEN = GCLK_PCHCTRL_GEN_GCLK0_Val;
            pchctrl.bit.CHEN = 1; // Re-enable
            GCLK->PCHCTRL[EVSYS_GCLK_ID_0].reg = pchctrl.reg;
            // Wait for it, then enable EVSYS clock
            while (!GCLK->PCHCTRL[EVSYS_GCLK_ID_0].bit.CHEN)
              ;
            MCLK->APBBMASK.bit.EVSYS_ = 1;

            // Connect Timer EVU to ch 0
            EVSYS->USER[tcList[tcNum].evu].reg = 1;
            // Datasheet recommends single write operation;
            // reg instead of bit. Also datasheet: PATH bits
            // must be zero when using async!
            EVSYS_CHANNEL_Type ev;
            ev.reg = 0;
            ev.bit.PATH = 2;                  // Asynchronous
            ev.bit.EVGEN = 0x22 + dmaChannel; // DMA channel 0+
            EVSYS->Channel[0].CHANNEL.reg = ev.reg;

            // Initialize descriptor list.
            for (int d = 0; d < numDescriptors; d++)
            {
              // No need to set SRCADDR, DESCADDR or BTCNT --
              // those are done in the pixel-writing functions.
              descriptor[d].BTCTRL.bit.VALID = true;
              // Event strobe on beat xfer:
              descriptor[d].BTCTRL.bit.EVOSEL = 0x3;
              descriptor[d].BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_NOACT;
              descriptor[d].BTCTRL.bit.BEATSIZE =
                  tft8.wide ? DMA_BEAT_SIZE_HWORD : DMA_BEAT_SIZE_BYTE;
              descriptor[d].BTCTRL.bit.SRCINC = 1;
              descriptor[d].BTCTRL.bit.DSTINC = 0;
              descriptor[d].BTCTRL.bit.STEPSEL = DMA_STEPSEL_SRC;
              descriptor[d].BTCTRL.bit.STEPSIZE =
                  DMA_ADDRESS_INCREMENT_STEP_SIZE_1;
              descriptor[d].DSTADDR.reg = (uint32_t)tft8.writePort;
            }
#endif      // __SAMD51
          } // end parallel-specific DMA setup

          lastFillColor = 0x0000;
          lastFillLen = 0;
          dma.setCallback(dma_callback);
          return; // Success!
                  // else clean up any partial allocation...
        }         // end descriptor memalign()
        free(pixelBuf[0]);
        pixelBuf[0] = pixelBuf[1] = NULL;
      }         // end pixelBuf malloc()
                // Don't currently have a descriptor delete function in
                // ZeroDMA lib, but if we did, it would be called here.
    }           // end addDescriptor()
    dma.free(); // Deallocate DMA channel
  }
#endif // end USE_SPI_DMA
}

/*!
    @brief  Allow changing the SPI clock speed after initialization
    @param  freq Desired frequency of SPI clock, may not be the
    end frequency you get based on what the chip can do!
*/
void Adafruit_SPITFT::setSPISpeed(uint32_t freq)
{
#if defined(SPI_HAS_TRANSACTION)
  hwspi.settings = SPISettings(freq, MSBFIRST, hwspi._mode);
#else
  hwspi._freq = freq;   // Save freq value for later
#endif
}

/*!
    @brief  Call before issuing command(s) or data to display. Performs
            chip-select (if required) and starts an SPI transaction (if
            using hardware SPI and transactions are supported). Required
            for all display types; not an SPI-specific function.
*/
void Adafruit_SPITFT::startWrite(void)
{
  SPI_BEGIN_TRANSACTION();
  if (_cs >= 0)
    SPI_CS_LOW();
}

/*!
    @brief  Call after issuing command(s) or data to display. Performs
            chip-deselect (if required) and ends an SPI transaction (if
            using hardware SPI and transactions are supported). Required
            for all display types; not an SPI-specific function.
*/
void Adafruit_SPITFT::endWrite(void)
{
  if (_cs >= 0)
    SPI_CS_HIGH();
  SPI_END_TRANSACTION();
}

// -------------------------------------------------------------------------
// Lower-level graphics operations. These functions require a chip-select
// and/or SPI transaction around them (via startWrite(), endWrite() above).
// Higher-level graphics primitives might start a single transaction and
// then make multiple calls to these functions (e.g. circle or text
// rendering might make repeated lines or rects) before ending the
// transaction. It's more efficient than starting a transaction every time.

/*!
    @brief  Draw a single pixel to the display at requested coordinates.
            Not self-contained; should follow a startWrite() call.
    @param  x      Horizontal position (0 = left).
    @param  y      Vertical position   (0 = top).
    @param  color  16-bit pixel color in '565' RGB format.
*/
void Adafruit_SPITFT::writePixel(int16_t x, int16_t y, uint16_t color)
{
  if ((x >= 0) && (x < _width) && (y >= 0) && (y < _height))
  {
    setAddrWindow(x, y, 1, 1);
    SPI_WRITE16(color);
  }
}

/*!
    @brief  Swap bytes in an array of pixels; converts little-to-big or
            big-to-little endian. Used by writePixels() below in some
            situations, but may also be helpful for user code occasionally.
    @param  src   Source address of 16-bit pixels buffer.
    @param  len   Number of pixels to byte-swap.
    @param  dest  Optional destination address if different than src --
                  otherwise, if NULL (default) or same address is passed,
                  pixel buffer is overwritten in-place.
*/
void Adafruit_SPITFT::swapBytes(uint16_t *src, uint32_t len, uint16_t *dest)
{
  if (!dest)
    dest = src; // NULL -> overwrite src buffer
  for (uint32_t i = 0; i < len; i++)
  {
    dest[i] = __builtin_bswap16(src[i]);
  }
}

/*!
    @brief  Issue a series of pixels from memory to the display. Not self-
            contained; should follow startWrite() and setAddrWindow() calls.
    @param  colors     Pointer to array of 16-bit pixel values in '565' RGB
                       format.
    @param  len        Number of elements in 'colors' array.
    @param  block      If true (default case if unspecified), function blocks
                       until DMA transfer is complete. This is simply IGNORED
                       if DMA is not enabled. If false, the function returns
                       immediately after the last DMA transfer is started,
                       and one should use the dmaWait() function before
                       doing ANY other display-related activities (or even
                       any SPI-related activities, if using an SPI display
                       that shares the bus with other devices).
    @param  bigEndian  If true, bitmap in memory is in big-endian order (most
                       significant byte first). By default this is false, as
                       most microcontrollers seem to be little-endian and
                       16-bit pixel values must be byte-swapped before
                       issuing to the display (which tend toward big-endian
                       when using SPI or 8-bit parallel). If an application
                       can optimize around this -- for example, a bitmap in a
                       uint16_t array having the byte values already ordered
                       big-endian, this can save time here, ESPECIALLY if
                       using this function's non-blocking DMA mode.
*/
void Adafruit_SPITFT::writePixels(uint16_t *colors, uint32_t len, bool block,
                                  bool bigEndian)
{

  if (!len)
    return; // Avoid 0-byte transfers

  // avoid paramater-not-used complaints
  (void)block;
  (void)bigEndian;

#if defined(ESP32)
  if (connection == TFT_HARD_SPI)
  {
    if (!bigEndian)
    {
      hwspi._spi->writePixels(colors, len * 2); // Inbuilt endian-swap
    }
    else
    {
      hwspi._spi->writeBytes((uint8_t *)colors, len * 2); // Issue bytes direct
    }
    return;
  }
#elif defined(ARDUINO_NRF52_ADAFRUIT) && \
    defined(NRF52840_XXAA) // Adafruit nRF52 use SPIM3 DMA at 32Mhz
  if (!bigEndian)
  {
    swapBytes(colors, len); // convert little-to-big endian for display
  }
  hwspi._spi->transfer(colors, NULL, 2 * len); // NULL RX to avoid overwrite
  if (!bigEndian)
  {
    swapBytes(colors, len); // big-to-little endian to restore pixel buffer
  }

  return;
#elif defined(ARDUINO_ARCH_RP2040)
  spi_inst_t *pi_spi = hwspi._spi == &SPI ? spi0 : spi1;

  if (!bigEndian)
  {
    // switch to 16-bit writes
    hw_write_masked(&spi_get_hw(pi_spi)->cr0, 15 << SPI_SSPCR0_DSS_LSB,
                    SPI_SSPCR0_DSS_BITS);
    spi_write16_blocking(pi_spi, colors, len);
    // switch back to 8-bit
    hw_write_masked(&spi_get_hw(pi_spi)->cr0, 7 << SPI_SSPCR0_DSS_LSB,
                    SPI_SSPCR0_DSS_BITS);
  }
  else
  {
    spi_write_blocking(pi_spi, (uint8_t *)colors, len * 2);
  }
  return;
#elif defined(USE_SPI_DMA) && \
    (defined(__SAMD51__) || defined(ARDUINO_SAMD_ZERO))
  if ((connection == TFT_HARD_SPI) || (connection == TFT_PARALLEL))
  {
    int maxSpan = maxFillLen / 2; // One scanline max
    uint8_t pixelBufIdx = 0;      // Active pixel buffer number
#if defined(__SAMD51__)
    if (connection == TFT_PARALLEL)
    {
      // Switch WR pin to PWM or CCL
      pinPeripheral(tft8._wr, wrPeripheral);
    }
#endif // end __SAMD51__
    if (!bigEndian)
    { // Normal little-endian situation...
      while (len)
      {
        int count = (len < maxSpan) ? len : maxSpan;

        // Because TFT and SAMD endianisms are different, must swap
        // bytes from the 'colors' array passed into a DMA working
        // buffer. This can take place while the prior DMA transfer
        // is in progress, hence the need for two pixelBufs.
        swapBytes(colors, count, pixelBuf[pixelBufIdx]);
        colors += count;

        // The transfers themselves are relatively small, so we don't
        // need a long descriptor list. We just alternate between the
        // first two, sharing pixelBufIdx for that purpose.
        descriptor[pixelBufIdx].SRCADDR.reg =
            (uint32_t)pixelBuf[pixelBufIdx] + count * 2;
        descriptor[pixelBufIdx].BTCTRL.bit.SRCINC = 1;
        descriptor[pixelBufIdx].BTCNT.reg = count * 2;
        descriptor[pixelBufIdx].DESCADDR.reg = 0;

        while (dma_busy)
          ; // Wait for prior line to finish

        // Move new descriptor into place...
        memcpy(dptr, &descriptor[pixelBufIdx], sizeof(DmacDescriptor));
        dma_busy = true;
        dma.startJob(); // Trigger SPI DMA transfer
        if (connection == TFT_PARALLEL)
          dma.trigger();
        pixelBufIdx = 1 - pixelBufIdx; // Swap DMA pixel buffers

        len -= count;
      }
    }
    else
    { // bigEndian == true
      // With big-endian pixel data, this can be handled as a single
      // DMA transfer using chained descriptors. Even full screen, this
      // needs only a relatively short descriptor list, each
      // transferring a max of 32,767 (not 32,768) pixels. The list
      // was allocated large enough to accommodate a full screen's
      // worth of data, so this won't run past the end of the list.
      int d, numDescriptors = (len + 32766) / 32767;
      for (d = 0; d < numDescriptors; d++)
      {
        int count = (len < 32767) ? len : 32767;
        descriptor[d].SRCADDR.reg = (uint32_t)colors + count * 2;
        descriptor[d].BTCTRL.bit.SRCINC = 1;
        descriptor[d].BTCNT.reg = count * 2;
        descriptor[d].DESCADDR.reg = (uint32_t)&descriptor[d + 1];
        len -= count;
        colors += count;
      }
      descriptor[d - 1].DESCADDR.reg = 0;

      while (dma_busy)
        ; // Wait for prior transfer (if any) to finish

      // Move first descriptor into place and start transfer...
      memcpy(dptr, &descriptor[0], sizeof(DmacDescriptor));
      dma_busy = true;
      dma.startJob(); // Trigger SPI DMA transfer
      if (connection == TFT_PARALLEL)
        dma.trigger();
    } // end bigEndian

    lastFillColor = 0x0000; // pixelBuf has been sullied
    lastFillLen = 0;
    if (block)
    {
      while (dma_busy)
        ; // Wait for last line to complete
#if defined(__SAMD51__) || defined(ARDUINO_SAMD_ZERO)
      if (connection == TFT_HARD_SPI)
      {
        // See SAMD51/21 note in writeColor()
        hwspi._spi->setDataMode(hwspi._mode);
      }
      else
      {
        pinPeripheral(tft8._wr, PIO_OUTPUT); // Switch WR back to GPIO
      }
#endif // end __SAMD51__ || ARDUINO_SAMD_ZERO
    }
    return;
  }
#endif // end USE_SPI_DMA

  // All other cases (bitbang SPI or non-DMA hard SPI or parallel),
  // use a loop with the normal 16-bit data write function:

  if (!bigEndian)
  {
    while (len--)
    {
      SPI_WRITE16(*colors++);
    }
  }
  else
  {
    // Well this is awkward. SPI_WRITE16() was designed for little-endian
    // hosts and big-endian displays as that's nearly always the typical
    // case. If the bigEndian flag was set, data is already in display's
    // order...so each pixel needs byte-swapping before being issued.
    // Rather than having a separate big-endian SPI_WRITE16 (adding more
    // bloat), it's preferred if calling function is smart and only uses
    // bigEndian where DMA is supported. But we gotta handle this...
    while (len--)
    {
      SPI_WRITE16(__builtin_bswap16(*colors++));
    }
  }
}

/*!
    @brief  Wait for the last DMA transfer in a prior non-blocking
            writePixels() call to complete. This does nothing if DMA
            is not enabled, and is not needed if blocking writePixels()
            was used (as is the default case).
*/
void Adafruit_SPITFT::dmaWait(void)
{
#if defined(USE_SPI_DMA) && (defined(__SAMD51__) || defined(ARDUINO_SAMD_ZERO))
  while (dma_busy)
    ;
#if defined(__SAMD51__) || defined(ARDUINO_SAMD_ZERO)
  if (connection == TFT_HARD_SPI)
  {
    // See SAMD51/21 note in writeColor()
    hwspi._spi->setDataMode(hwspi._mode);
  }
  else
  {
    pinPeripheral(tft8._wr, PIO_OUTPUT); // Switch WR back to GPIO
  }
#endif // end __SAMD51__ || ARDUINO_SAMD_ZERO
#endif
}

/*!
    @brief  Check if DMA transfer is active. Always returts false if DMA
            is not enabled.
    @return true if DMA is enabled and transmitting data, false otherwise.
*/
bool Adafruit_SPITFT::dmaBusy(void) const
{
#if defined(USE_SPI_DMA) && (defined(__SAMD51__) || defined(ARDUINO_SAMD_ZERO))
  return dma_busy;
#else
  return false;
#endif
}

/*!
    @brief  Issue a series of pixels, all the same color. Not self-
            contained; should follow startWrite() and setAddrWindow() calls.
    @param  color  16-bit pixel color in '565' RGB format.
    @param  len    Number of pixels to draw.
*/
void Adafruit_SPITFT::writeColor(uint16_t color, uint32_t len)
{

  if (!len)
    return; // Avoid 0-byte transfers

  uint8_t hi = color >> 8, lo = color;

  if (connection == TFT_HYBRID_HWSPI_PARALLEL)
  {
    while (len--)
    {
      AVR_WRITESPI(color);
      TFT_CS_STROBE();
      TFT_WR_STROBE();
    }
  }
}

/*!
    @brief  Draw a filled rectangle to the display. Not self-contained;
            should follow startWrite(). Typically used by higher-level
            graphics primitives; user code shouldn't need to call this and
            is likely to use the self-contained fillRect() instead.
            writeFillRect() performs its own edge clipping and rejection;
            see writeFillRectPreclipped() for a more 'raw' implementation.
    @param  x      Horizontal position of first corner.
    @param  y      Vertical position of first corner.
    @param  w      Rectangle width in pixels (positive = right of first
                   corner, negative = left of first corner).
    @param  h      Rectangle height in pixels (positive = below first
                   corner, negative = above first corner).
    @param  color  16-bit fill color in '565' RGB format.
    @note   Written in this deep-nested way because C by definition will
            optimize for the 'if' case, not the 'else' -- avoids branches
            and rejects clipped rectangles at the least-work possibility.
*/
void Adafruit_SPITFT::writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                    uint16_t color)
{
  if (w && h)
  { // Nonzero width and height?
    if (w < 0)
    {             // If negative width...
      x += w + 1; //   Move X to left edge
      w = -w;     //   Use positive width
    }
    if (x < _width)
    { // Not off right
      if (h < 0)
      {             // If negative height...
        y += h + 1; //   Move Y to top edge
        h = -h;     //   Use positive height
      }
      if (y < _height)
      { // Not off bottom
        int16_t x2 = x + w - 1;
        if (x2 >= 0)
        { // Not off left
          int16_t y2 = y + h - 1;
          if (y2 >= 0)
          { // Not off top
            // Rectangle partly or fully overlaps screen
            if (x < 0)
            {
              x = 0;
              w = x2 + 1;
            } // Clip left
            if (y < 0)
            {
              y = 0;
              h = y2 + 1;
            } // Clip top
            if (x2 >= _width)
            {
              w = _width - x;
            } // Clip right
            if (y2 >= _height)
            {
              h = _height - y;
            } // Clip bottom
            writeFillRectPreclipped(x, y, w, h, color);
          }
        }
      }
    }
  }
}

/*!
    @brief  Draw a horizontal line on the display. Performs edge clipping
            and rejection. Not self-contained; should follow startWrite().
            Typically used by higher-level graphics primitives; user code
            shouldn't need to call this and is likely to use the self-
            contained drawFastHLine() instead.
    @param  x      Horizontal position of first point.
    @param  y      Vertical position of first point.
    @param  w      Line width in pixels (positive = right of first point,
                   negative = point of first corner).
    @param  color  16-bit line color in '565' RGB format.
*/
void inline Adafruit_SPITFT::writeFastHLine(int16_t x, int16_t y, int16_t w,
                                            uint16_t color)
{
  if ((y >= 0) && (y < _height) && w)
  { // Y on screen, nonzero width
    if (w < 0)
    {             // If negative width...
      x += w + 1; //   Move X to left edge
      w = -w;     //   Use positive width
    }
    if (x < _width)
    { // Not off right
      int16_t x2 = x + w - 1;
      if (x2 >= 0)
      { // Not off left
        // Line partly or fully overlaps screen
        if (x < 0)
        {
          x = 0;
          w = x2 + 1;
        } // Clip left
        if (x2 >= _width)
        {
          w = _width - x;
        } // Clip right
        writeFillRectPreclipped(x, y, w, 1, color);
      }
    }
  }
}

/*!
    @brief  Draw a vertical line on the display. Performs edge clipping and
            rejection. Not self-contained; should follow startWrite().
            Typically used by higher-level graphics primitives; user code
            shouldn't need to call this and is likely to use the self-
            contained drawFastVLine() instead.
    @param  x      Horizontal position of first point.
    @param  y      Vertical position of first point.
    @param  h      Line height in pixels (positive = below first point,
                   negative = above first point).
    @param  color  16-bit line color in '565' RGB format.
*/
void inline Adafruit_SPITFT::writeFastVLine(int16_t x, int16_t y, int16_t h,
                                            uint16_t color)
{
  if ((x >= 0) && (x < _width) && h)
  { // X on screen, nonzero height
    if (h < 0)
    {             // If negative height...
      y += h + 1; //   Move Y to top edge
      h = -h;     //   Use positive height
    }
    if (y < _height)
    { // Not off bottom
      int16_t y2 = y + h - 1;
      if (y2 >= 0)
      { // Not off top
        // Line partly or fully overlaps screen
        if (y < 0)
        {
          y = 0;
          h = y2 + 1;
        } // Clip top
        if (y2 >= _height)
        {
          h = _height - y;
        } // Clip bottom
        writeFillRectPreclipped(x, y, 1, h, color);
      }
    }
  }
}

/*!
    @brief  A lower-level version of writeFillRect(). This version requires
            all inputs are in-bounds, that width and height are positive,
            and no part extends offscreen. NO EDGE CLIPPING OR REJECTION IS
            PERFORMED. If higher-level graphics primitives are written to
            handle their own clipping earlier in the drawing process, this
            can avoid unnecessary function calls and repeated clipping
            operations in the lower-level functions.
    @param  x      Horizontal position of first corner. MUST BE WITHIN
                   SCREEN BOUNDS.
    @param  y      Vertical position of first corner. MUST BE WITHIN SCREEN
                   BOUNDS.
    @param  w      Rectangle width in pixels. MUST BE POSITIVE AND NOT
                   EXTEND OFF SCREEN.
    @param  h      Rectangle height in pixels. MUST BE POSITIVE AND NOT
                   EXTEND OFF SCREEN.
    @param  color  16-bit fill color in '565' RGB format.
    @note   This is a new function, no graphics primitives besides rects
            and horizontal/vertical lines are written to best use this yet.
*/
inline void Adafruit_SPITFT::writeFillRectPreclipped(int16_t x, int16_t y,
                                                     int16_t w, int16_t h,
                                                     uint16_t color)
{
  setAddrWindow(x, y, w, h);
  writeColor(color, (uint32_t)w * h);
}

// -------------------------------------------------------------------------
// Ever-so-slightly higher-level graphics operations. Similar to the 'write'
// functions above, but these contain their own chip-select and SPI
// transactions as needed (via startWrite(), endWrite()). They're typically
// used solo -- as graphics primitives in themselves, not invoked by higher-
// level primitives (which should use the functions above for better
// performance).

/*!
    @brief  Draw a single pixel to the display at requested coordinates.
            Self-contained and provides its own transaction as needed
            (see writePixel(x,y,color) for a lower-level variant).
            Edge clipping is performed here.
    @param  x      Horizontal position (0 = left).
    @param  y      Vertical position   (0 = top).
    @param  color  16-bit pixel color in '565' RGB format.
*/
void Adafruit_SPITFT::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  // Clip first...
  if ((x >= 0) && (x < _width) && (y >= 0) && (y < _height))
  {
    // THEN set up transaction (if needed) and draw...
    startWrite();
    setAddrWindow(x, y, 1, 1);
    SPI_WRITE16(color);
    endWrite();
  }
}

/*!
    @brief  Draw a filled rectangle to the display. Self-contained and
            provides its own transaction as needed (see writeFillRect() or
            writeFillRectPreclipped() for lower-level variants). Edge
            clipping and rejection is performed here.
    @param  x      Horizontal position of first corner.
    @param  y      Vertical position of first corner.
    @param  w      Rectangle width in pixels (positive = right of first
                   corner, negative = left of first corner).
    @param  h      Rectangle height in pixels (positive = below first
                   corner, negative = above first corner).
    @param  color  16-bit fill color in '565' RGB format.
    @note   This repeats the writeFillRect() function almost in its entirety,
            with the addition of a transaction start/end. It's done this way
            (rather than starting the transaction and calling writeFillRect()
            to handle clipping and so forth) so that the transaction isn't
            performed at all if the rectangle is rejected. It's really not
            that much code.
*/
void Adafruit_SPITFT::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                               uint16_t color)
{
  if (w && h)
  { // Nonzero width and height?
    if (w < 0)
    {             // If negative width...
      x += w + 1; //   Move X to left edge
      w = -w;     //   Use positive width
    }
    if (x < _width)
    { // Not off right
      if (h < 0)
      {             // If negative height...
        y += h + 1; //   Move Y to top edge
        h = -h;     //   Use positive height
      }
      if (y < _height)
      { // Not off bottom
        int16_t x2 = x + w - 1;
        if (x2 >= 0)
        { // Not off left
          int16_t y2 = y + h - 1;
          if (y2 >= 0)
          { // Not off top
            // Rectangle partly or fully overlaps screen
            if (x < 0)
            {
              x = 0;
              w = x2 + 1;
            } // Clip left
            if (y < 0)
            {
              y = 0;
              h = y2 + 1;
            } // Clip top
            if (x2 >= _width)
            {
              w = _width - x;
            } // Clip right
            if (y2 >= _height)
            {
              h = _height - y;
            } // Clip bottom
            startWrite();
            writeFillRectPreclipped(x, y, w, h, color);
            endWrite();
          }
        }
      }
    }
  }
}

/*!
    @brief  Draw a horizontal line on the display. Self-contained and
            provides its own transaction as needed (see writeFastHLine() for
            a lower-level variant). Edge clipping and rejection is performed
            here.
    @param  x      Horizontal position of first point.
    @param  y      Vertical position of first point.
    @param  w      Line width in pixels (positive = right of first point,
                   negative = point of first corner).
    @param  color  16-bit line color in '565' RGB format.
    @note   This repeats the writeFastHLine() function almost in its
            entirety, with the addition of a transaction start/end. It's
            done this way (rather than starting the transaction and calling
            writeFastHLine() to handle clipping and so forth) so that the
            transaction isn't performed at all if the line is rejected.
*/
void Adafruit_SPITFT::drawFastHLine(int16_t x, int16_t y, int16_t w,
                                    uint16_t color)
{
  if ((y >= 0) && (y < _height) && w)
  { // Y on screen, nonzero width
    if (w < 0)
    {             // If negative width...
      x += w + 1; //   Move X to left edge
      w = -w;     //   Use positive width
    }
    if (x < _width)
    { // Not off right
      int16_t x2 = x + w - 1;
      if (x2 >= 0)
      { // Not off left
        // Line partly or fully overlaps screen
        if (x < 0)
        {
          x = 0;
          w = x2 + 1;
        } // Clip left
        if (x2 >= _width)
        {
          w = _width - x;
        } // Clip right
        startWrite();
        writeFillRectPreclipped(x, y, w, 1, color);
        endWrite();
      }
    }
  }
}

/*!
    @brief  Draw a vertical line on the display. Self-contained and provides
            its own transaction as needed (see writeFastHLine() for a lower-
            level variant). Edge clipping and rejection is performed here.
    @param  x      Horizontal position of first point.
    @param  y      Vertical position of first point.
    @param  h      Line height in pixels (positive = below first point,
                   negative = above first point).
    @param  color  16-bit line color in '565' RGB format.
    @note   This repeats the writeFastVLine() function almost in its
            entirety, with the addition of a transaction start/end. It's
            done this way (rather than starting the transaction and calling
            writeFastVLine() to handle clipping and so forth) so that the
            transaction isn't performed at all if the line is rejected.
*/
void Adafruit_SPITFT::drawFastVLine(int16_t x, int16_t y, int16_t h,
                                    uint16_t color)
{
  if ((x >= 0) && (x < _width) && h)
  { // X on screen, nonzero height
    if (h < 0)
    {             // If negative height...
      y += h + 1; //   Move Y to top edge
      h = -h;     //   Use positive height
    }
    if (y < _height)
    { // Not off bottom
      int16_t y2 = y + h - 1;
      if (y2 >= 0)
      { // Not off top
        // Line partly or fully overlaps screen
        if (y < 0)
        {
          y = 0;
          h = y2 + 1;
        } // Clip top
        if (y2 >= _height)
        {
          h = _height - y;
        } // Clip bottom
        startWrite();
        writeFillRectPreclipped(x, y, 1, h, color);
        endWrite();
      }
    }
  }
}

/*!
    @brief  Essentially writePixel() with a transaction around it. I don't
            think this is in use by any of our code anymore (believe it was
            for some older BMP-reading examples), but is kept here in case
            any user code relies on it. Consider it DEPRECATED.
    @param  color  16-bit pixel color in '565' RGB format.
*/
void Adafruit_SPITFT::pushColor(uint16_t color)
{
  startWrite();
  SPI_WRITE16(color);
  endWrite();
}

/*!
    @brief  Draw a 16-bit image (565 RGB) at the specified (x,y) position.
            For 16-bit display devices; no color reduction performed.
            Adapted from https://github.com/PaulStoffregen/ILI9341_t3
            by Marc MERLIN. See examples/pictureEmbed to use this.
            5/6/2017: function name and arguments have changed for
            compatibility with current GFX library and to avoid naming
            problems in prior implementation.  Formerly drawBitmap() with
            arguments in different order. Handles its own transaction and
            edge clipping/rejection.
    @param  x        Top left corner horizontal coordinate.
    @param  y        Top left corner vertical coordinate.
    @param  pcolors  Pointer to 16-bit array of pixel values.
    @param  w        Width of bitmap in pixels.
    @param  h        Height of bitmap in pixels.
*/
void Adafruit_SPITFT::drawRGBBitmap(int16_t x, int16_t y, uint16_t *pcolors,
                                    int16_t w, int16_t h)
{

  int16_t x2, y2;                 // Lower-right coord
  if ((x >= _width) ||            // Off-edge right
      (y >= _height) ||           // " top
      ((x2 = (x + w - 1)) < 0) || // " left
      ((y2 = (y + h - 1)) < 0))
    return; // " bottom

  int16_t bx1 = 0, by1 = 0, // Clipped top-left within bitmap
      saveW = w;            // Save original bitmap width value
  if (x < 0)
  { // Clip left
    w += x;
    bx1 = -x;
    x = 0;
  }
  if (y < 0)
  { // Clip top
    h += y;
    by1 = -y;
    y = 0;
  }
  if (x2 >= _width)
    w = _width - x; // Clip right
  if (y2 >= _height)
    h = _height - y; // Clip bottom

  pcolors += by1 * saveW + bx1; // Offset bitmap ptr to clipped top-left
  startWrite();
  setAddrWindow(x, y, w, h); // Clipped area
  while (h--)
  {                          // For each (clipped) scanline...
    writePixels(pcolors, w); // Push one (clipped) row
    pcolors += saveW;        // Advance pointer by one full (unclipped) line
  }
  endWrite();
}

// -------------------------------------------------------------------------
// Miscellaneous class member functions that don't draw anything.

/*!
    @brief  Invert the colors of the display (if supported by hardware).
            Self-contained, no transaction setup required.
    @param  i  true = inverted display, false = normal display.
*/
void Adafruit_SPITFT::invertDisplay(bool i)
{
  startWrite();
  writeCommand(i ? invertOnCommand : invertOffCommand);
  endWrite();
}

/*!
    @brief   Given 8-bit red, green and blue values, return a 'packed'
             16-bit color value in '565' RGB format (5 bits red, 6 bits
             green, 5 bits blue). This is just a mathematical operation,
             no hardware is touched.
    @param   red    8-bit red brightnesss (0 = off, 255 = max).
    @param   green  8-bit green brightnesss (0 = off, 255 = max).
    @param   blue   8-bit blue brightnesss (0 = off, 255 = max).
    @return  'Packed' 16-bit color value (565 format).
*/
uint16_t Adafruit_SPITFT::color565(uint8_t red, uint8_t green, uint8_t blue)
{
  return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}

/*!
@brief   Adafruit_SPITFT Send Command handles complete sending of commands and
data
@param   commandByte       The Command Byte
@param   dataBytes         A pointer to the Data bytes to send
@param   numDataBytes      The number of bytes we should send
*/
void Adafruit_SPITFT::sendCommand(uint8_t commandByte, uint8_t *dataBytes,
                                  uint8_t numDataBytes)
{
  SPI_BEGIN_TRANSACTION();
  if (_cs >= 0)
    SPI_CS_LOW();

  SPI_DC_LOW();          // Command mode
  spiWrite(commandByte); // Send the command byte

  SPI_DC_HIGH();
  for (int i = 0; i < numDataBytes; i++)
  {
    if ((connection == TFT_PARALLEL || connection == TFT_HYBRID_HWSPI_PARALLEL) && tft8.wide)
    {
      SPI_WRITE16(*(uint16_t *)dataBytes);
      dataBytes += 2;
    }
    else
    {
      spiWrite(*dataBytes); // Send the data bytes
      dataBytes++;
    }
  }

  if (_cs >= 0)
    SPI_CS_HIGH();
  SPI_END_TRANSACTION();
}

/*!
 @brief   Adafruit_SPITFT Send Command handles complete sending of commands and
 data
 @param   commandByte       The Command Byte
 @param   dataBytes         A pointer to the Data bytes to send
 @param   numDataBytes      The number of bytes we should send
 */
void Adafruit_SPITFT::sendCommand(uint8_t commandByte, const uint8_t *dataBytes,
                                  uint8_t numDataBytes)
{
  SPI_BEGIN_TRANSACTION();
  if (_cs >= 0)
    SPI_CS_LOW();

  SPI_DC_LOW();          // Command mode
  spiWrite(commandByte); // Send the command byte

  SPI_DC_HIGH();
  for (int i = 0; i < numDataBytes; i++)
  {
    if ((connection == TFT_HYBRID_HWSPI_PARALLEL) && tft8.wide)
    {
      SPI_WRITE16(*(uint16_t *)dataBytes);
      dataBytes += 2;
    }
    else
    {
      spiWrite(pgm_read_byte(dataBytes++));
    }
  }

  if (_cs >= 0)
    SPI_CS_HIGH();
  SPI_END_TRANSACTION();
}

/*!
 @brief  Adafruit_SPITFT sendCommand16 handles complete sending of
         commands and data for 16-bit parallel displays. Currently somewhat
         rigged for the NT35510, which has the odd behavior of wanting
         commands 16-bit, but subsequent data as 8-bit values, despite
         the 16-bit bus (high byte is always 0). Also seems to require
         issuing and incrementing address with each transfer.
 @param  commandWord   The command word (16 bits)
 @param  dataBytes     A pointer to the data bytes to send
 @param  numDataBytes  The number of bytes we should send
 */
void Adafruit_SPITFT::sendCommand16(uint16_t commandWord,
                                    const uint8_t *dataBytes,
                                    uint8_t numDataBytes)
{
  SPI_BEGIN_TRANSACTION();
  if (_cs >= 0)
    SPI_CS_LOW();

  if (numDataBytes == 0)
  {
    SPI_DC_LOW();             // Command mode
    SPI_WRITE16(commandWord); // Send the command word
    SPI_DC_HIGH();            // Data mode
  }
  for (int i = 0; i < numDataBytes; i++)
  {
    SPI_DC_LOW();             // Command mode
    SPI_WRITE16(commandWord); // Send the command word
    SPI_DC_HIGH();            // Data mode
    commandWord++;
    SPI_WRITE16((uint16_t)pgm_read_byte(dataBytes++));
  }

  if (_cs >= 0)
    SPI_CS_HIGH();
  SPI_END_TRANSACTION();
}

/*!
 @brief   Read 8 bits of data from display configuration memory (not RAM).
 This is highly undocumented/supported and should be avoided,
 function is only included because some of the examples use it.
 @param   commandByte
 The command register to read data from.
 @param   index
 The byte index into the command to read from.
 @return  Unsigned 8-bit data read from display register.
 */
/**************************************************************************/
uint8_t Adafruit_SPITFT::readcommand8(uint8_t commandByte, uint8_t index)
{
  uint8_t result;
  startWrite();
  SPI_DC_LOW(); // Command mode
  spiWrite(commandByte);
  SPI_DC_HIGH(); // Data mode
  do
  {
    result = spiRead();
  } while (index--); // Discard bytes up to index'th
  endWrite();
  return result;
}

/*!
 @brief   Read 16 bits of data from display register.
          For 16-bit parallel displays only.
 @param   addr  Command/register to access.
 @return  Unsigned 16-bit data.
 */
uint16_t Adafruit_SPITFT::readcommand16(uint16_t addr)
{
#if defined(USE_FAST_PINIO) // NOT SUPPORTED without USE_FAST_PINIO
  uint16_t result = 0;
  if ((connection == TFT_PARALLEL) && tft8.wide)
  {
    startWrite();
    SPI_DC_LOW(); // Command mode
    SPI_WRITE16(addr);
    SPI_DC_HIGH(); // Data mode
    TFT_RD_LOW();  // Read line LOW
#if defined(HAS_PORT_SET_CLR)
    *(volatile uint16_t *)tft8.dirClr = 0xFFFF;   // Input state
    result = *(volatile uint16_t *)tft8.readPort; // 16-bit read
    *(volatile uint16_t *)tft8.dirSet = 0xFFFF;   // Output state
#else                                             // !HAS_PORT_SET_CLR
    *(volatile uint16_t *)tft8.portDir = 0x0000;  // Input state
    result = *(volatile uint16_t *)tft8.readPort; // 16-bit read
    *(volatile uint16_t *)tft8.portDir = 0xFFFF;  // Output state
#endif                                            // end !HAS_PORT_SET_CLR
    TFT_RD_HIGH();                                // Read line HIGH
    endWrite();
  }
  return result;
#else
  (void)addr; // disable -Wunused-parameter warning
  return 0;
#endif // end !USE_FAST_PINIO
}

// -------------------------------------------------------------------------
// Lowest-level hardware-interfacing functions. Many of these are inline and
// compile to different things based on #defines -- typically just a few
// instructions. Others, not so much, those are not inlined.

/*!
    @brief  Start an SPI transaction if using the hardware SPI interface to
            the display. If using an earlier version of the Arduino platform
            (before the addition of SPI transactions), this instead attempts
            to set up the SPI clock and mode. No action is taken if the
            connection is not hardware SPI-based. This does NOT include a
            chip-select operation -- see startWrite() for a function that
            encapsulated both actions.
*/
inline void Adafruit_SPITFT::SPI_BEGIN_TRANSACTION(void)
{
  if (connection == TFT_HARD_SPI || connection == TFT_HYBRID_HWSPI_PARALLEL)
  {
#if defined(SPI_HAS_TRANSACTION)
    hwspi._spi->beginTransaction(hwspi.settings);
#else // No transactions, configure SPI manually...
#if defined(__AVR__) || defined(TEENSYDUINO) || defined(ARDUINO_ARCH_STM32F1)
    hwspi._spi->setClockDivider(SPI_CLOCK_DIV2);
#elif defined(__arm__)
    hwspi._spi->setClockDivider(11);
#elif defined(ESP8266) || defined(ESP32)
    hwspi._spi->setFrequency(hwspi._freq);
#elif defined(RASPI) || defined(ARDUINO_ARCH_STM32F1)
    hwspi._spi->setClock(hwspi._freq);
#endif
    hwspi._spi->setBitOrder(MSBFIRST);
    hwspi._spi->setDataMode(hwspi._mode);
#endif // end !SPI_HAS_TRANSACTION
  }
}

/*!
    @brief  End an SPI transaction if using the hardware SPI interface to
            the display. No action is taken if the connection is not
            hardware SPI-based or if using an earlier version of the Arduino
            platform (before the addition of SPI transactions). This does
            NOT include a chip-deselect operation -- see endWrite() for a
            function that encapsulated both actions.
*/
inline void Adafruit_SPITFT::SPI_END_TRANSACTION(void)
{
#if defined(SPI_HAS_TRANSACTION)
  if (connection == TFT_HARD_SPI || connection == TFT_HYBRID_HWSPI_PARALLEL)
  {
    hwspi._spi->endTransaction();
  }
#endif
}

/*!
    @brief  Issue a single 8-bit value to the display. Chip-select,
            transaction and data/command selection must have been
            previously set -- this ONLY issues the byte. This is another of
            those functions in the library with a now-not-accurate name
            that's being maintained for compatibility with outside code.
            This function is used even if display connection is parallel.
    @param  b  8-bit value to write.
*/
void Adafruit_SPITFT::spiWrite(uint8_t b)
{
  if (connection == TFT_HARD_SPI)
  {
#if defined(__AVR__)
    AVR_WRITESPI(b);
#elif defined(ESP8266) || defined(ESP32)
    hwspi._spi->write(b);
#elif defined(ARDUINO_ARCH_RP2040)
    spi_inst_t *pi_spi = hwspi._spi == &SPI ? spi0 : spi1;
    spi_write_blocking(pi_spi, &b, 1);
#else
    hwspi._spi->transfer(b);
#endif
  }
  else if (connection == TFT_SOFT_SPI)
  {
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (b & 0x80)
        SPI_MOSI_HIGH();
      else
        SPI_MOSI_LOW();
      SPI_SCK_HIGH();
      b <<= 1;
      SPI_SCK_LOW();
    }
  }
  else if (connection == TFT_HYBRID_HWSPI_PARALLEL)
  {
#if defined(__AVR__)
    AVR_WRITESPI(b);
    TFT_CS_STROBE();
    TFT_WR_STROBE();
#elif defined(ESP8266) || defined(ESP32)
    hwspi._spi->write16(0x00FF & b);
    TFT_CS_STROBE();
    TFT_WR_STROBE();
#endif
  }
}

/*!
    @brief  Write a single command byte to the display. Chip-select and
            transaction must have been previously set -- this ONLY sets
            the device to COMMAND mode, issues the byte and then restores
            DATA mode. There is no corresponding explicit writeData()
            function -- just use spiWrite().
    @param  cmd  8-bit command to write.
*/
void Adafruit_SPITFT::writeCommand(uint8_t cmd)
{
  SPI_DC_LOW();
  spiWrite(cmd);
  SPI_DC_HIGH();
}

/*!
    @brief   Read a single 8-bit value from the display. Chip-select and
             transaction must have been previously set -- this ONLY reads
             the byte. This is another of those functions in the library
             with a now-not-accurate name that's being maintained for
             compatibility with outside code. This function is used even if
             display connection is parallel.
    @return  Unsigned 8-bit value read (always zero if USE_FAST_PINIO is
             not supported by the MCU architecture).
*/
uint8_t Adafruit_SPITFT::spiRead(void)
{
  uint8_t b = 0;
  uint16_t w = 0;
  if (connection == TFT_HARD_SPI)
  {
    return hwspi._spi->transfer((uint8_t)0);
  }
  else
  { // TFT_PARALLEL
    if (tft8._rd >= 0)
    {
#if defined(USE_FAST_PINIO)
      TFT_RD_LOW(); // Read line LOW
#if defined(__AVR__)
      *tft8.portDir = 0x00; // Set port to input state
      w = *tft8.readPort;   // Read value from port
      *tft8.portDir = 0xFF; // Restore port to output
#else                       // !__AVR__
      if (!tft8.wide)
      {                      // 8-bit TFT connection
#if defined(HAS_PORT_SET_CLR)
        *tft8.dirClr = 0xFF; // Set port to input state
        w = *tft8.readPort;  // Read value from port
        *tft8.dirSet = 0xFF; // Restore port to output
#else  // !HAS_PORT_SET_CLR
        *tft8.portDir = 0x00;                        // Set port to input state
        w = *tft8.readPort;                          // Read value from port
        *tft8.portDir = 0xFF;                        // Restore port to output
#endif // end HAS_PORT_SET_CLR
      }
      else
      {                                             // 16-bit TFT connection
#if defined(HAS_PORT_SET_CLR)
        *(volatile uint16_t *)tft8.dirClr = 0xFFFF; // Input state
        w = *(volatile uint16_t *)tft8.readPort;    // 16-bit read
        *(volatile uint16_t *)tft8.dirSet = 0xFFFF; // Output state
#else  // !HAS_PORT_SET_CLR
        *(volatile uint16_t *)tft8.portDir = 0x0000; // Input state
        w = *(volatile uint16_t *)tft8.readPort;     // 16-bit read
        *(volatile uint16_t *)tft8.portDir = 0xFFFF; // Output state
#endif // end !HAS_PORT_SET_CLR
      }
      TFT_RD_HIGH();                                 // Read line HIGH
#endif // end !__AVR__
#else  // !USE_FAST_PINIO
      w = 0; // Parallel TFT is NOT SUPPORTED without USE_FAST_PINIO
#endif // end !USE_FAST_PINIO
    }
    return w;
  }
}

/*!
    @brief  Issue a single 16-bit value to the display. Chip-select,
            transaction and data/command selection must have been
            previously set -- this ONLY issues the word.
            Thus operates ONLY on 'wide' (16-bit) parallel displays!
    @param  w  16-bit value to write.
*/
void Adafruit_SPITFT::write16(uint16_t w)
{
  if (connection == TFT_PARALLEL)
  {
#if defined(USE_FAST_PINIO)
    if (tft8.wide)
      *(volatile uint16_t *)tft8.writePort = w;
#else
    (void)w; // disable -Wunused-parameter warning
#endif
    TFT_WR_STROBE();
  }
  else if (connection == TFT_HYBRID_HWSPI_PARALLEL)
  {
#if defined(__AVR__)
    AVR_WRITESPI(w >> 8);
    AVR_WRITESPI(w);
    TFT_CS_STROBE();
    TFT_WR_STROBE();
#elif defined(ESP8266) || defined(ESP32)
    hwspi._spi->write16(w);
    TFT_CS_STROBE();
    TFT_WR_STROBE();
#endif
  }
}

/*!
    @brief  Write a single command word to the display. Chip-select and
            transaction must have been previously set -- this ONLY sets
            the device to COMMAND mode, issues the byte and then restores
            DATA mode. This operates ONLY on 'wide' (16-bit) parallel
            displays!
    @param  cmd  16-bit command to write.
*/
void Adafruit_SPITFT::writeCommand16(uint16_t cmd)
{
  SPI_DC_LOW();
  write16(cmd);
  SPI_DC_HIGH();
}

/*!
    @brief   Read a single 16-bit value from the display. Chip-select and
             transaction must have been previously set -- this ONLY reads
             the byte. This operates ONLY on 'wide' (16-bit) parallel
             displays!
    @return  Unsigned 16-bit value read (always zero if USE_FAST_PINIO is
             not supported by the MCU architecture).
*/
uint16_t Adafruit_SPITFT::read16(void)
{
  uint16_t w = 0;
  if (connection == TFT_PARALLEL)
  {
    if (tft8._rd >= 0)
    {
#if defined(USE_FAST_PINIO)
      TFT_RD_LOW(); // Read line LOW
      if (tft8.wide)
      { // 16-bit TFT connection
#if defined(HAS_PORT_SET_CLR)
        *(volatile uint16_t *)tft8.dirClr = 0xFFFF; // Input state
        w = *(volatile uint16_t *)tft8.readPort;    // 16-bit read
        *(volatile uint16_t *)tft8.dirSet = 0xFFFF; // Output state
#else                                               // !HAS_PORT_SET_CLR
        *(volatile uint16_t *)tft8.portDir = 0x0000; // Input state
        w = *(volatile uint16_t *)tft8.readPort;     // 16-bit read
        *(volatile uint16_t *)tft8.portDir = 0xFFFF; // Output state
#endif                                              // end !HAS_PORT_SET_CLR
      }
      TFT_RD_HIGH(); // Read line HIGH
#else                // !USE_FAST_PINIO
      w = 0; // Parallel TFT is NOT SUPPORTED without USE_FAST_PINIO
#endif               // end !USE_FAST_PINIO
    }
  }
  return w;
}

/*!
    @brief  Set the software (bitbang) SPI MOSI line HIGH.
*/
inline void Adafruit_SPITFT::SPI_MOSI_HIGH(void)
{
#if defined(USE_FAST_PINIO)
#if defined(HAS_PORT_SET_CLR)
#if defined(KINETISK)
  *swspi.mosiPortSet = 1;
#else // !KINETISK
  *swspi.mosiPortSet = swspi.mosiPinMask;
#endif
#else  // !HAS_PORT_SET_CLR
  *swspi.mosiPort |= swspi.mosiPinMaskSet;
#endif // end !HAS_PORT_SET_CLR
#else  // !USE_FAST_PINIO
  // digitalWrite(swspi._mosi, HIGH);
#if defined(ESP32)
  for (volatile uint8_t i = 0; i < 1; i++)
    ;
#endif // end ESP32
#endif // end !USE_FAST_PINIO
}

/*!
    @brief  Set the software (bitbang) SPI MOSI line LOW.
*/
inline void Adafruit_SPITFT::SPI_MOSI_LOW(void)
{
#if defined(USE_FAST_PINIO)
#if defined(HAS_PORT_SET_CLR)
#if defined(KINETISK)
  *swspi.mosiPortClr = 1;
#else // !KINETISK
  *swspi.mosiPortClr = swspi.mosiPinMask;
#endif
#else  // !HAS_PORT_SET_CLR
  *swspi.mosiPort &= swspi.mosiPinMaskClr;
#endif // end !HAS_PORT_SET_CLR
#else  // !USE_FAST_PINIO
  // digitalWrite(swspi._mosi, LOW);
#if defined(ESP32)
  for (volatile uint8_t i = 0; i < 1; i++)
    ;
#endif // end ESP32
#endif // end !USE_FAST_PINIO
}

/*!
    @brief  Set the software (bitbang) SPI SCK line HIGH.
*/
inline void Adafruit_SPITFT::SPI_SCK_HIGH(void)
{
#if defined(USE_FAST_PINIO)
#if defined(HAS_PORT_SET_CLR)
#if defined(KINETISK)
  *swspi.sckPortSet = 1;
#else                                                // !KINETISK
  *swspi.sckPortSet = swspi.sckPinMask;
#if defined(__IMXRT1052__) || defined(__IMXRT1062__) // Teensy 4.x
  for (volatile uint8_t i = 0; i < 1; i++)
    ;
#endif
#endif
#else  // !HAS_PORT_SET_CLR
  *swspi.sckPort |= swspi.sckPinMaskSet;
#endif // end !HAS_PORT_SET_CLR
#else  // !USE_FAST_PINIO
  // digitalWrite(swspi._sck, HIGH);
#if defined(ESP32)
  for (volatile uint8_t i = 0; i < 1; i++)
    ;
#endif // end ESP32
#endif // end !USE_FAST_PINIO
}

/*!
    @brief  Set the software (bitbang) SPI SCK line LOW.
*/
inline void Adafruit_SPITFT::SPI_SCK_LOW(void)
{
#if defined(USE_FAST_PINIO)
#if defined(HAS_PORT_SET_CLR)
#if defined(KINETISK)
  *swspi.sckPortClr = 1;
#else                                                // !KINETISK
  *swspi.sckPortClr = swspi.sckPinMask;
#if defined(__IMXRT1052__) || defined(__IMXRT1062__) // Teensy 4.x
  for (volatile uint8_t i = 0; i < 1; i++)
    ;
#endif
#endif
#else  // !HAS_PORT_SET_CLR
  *swspi.sckPort &= swspi.sckPinMaskClr;
#endif // end !HAS_PORT_SET_CLR
#else  // !USE_FAST_PINIO
  // digitalWrite(swspi._sck, LOW);
#if defined(ESP32)
  for (volatile uint8_t i = 0; i < 1; i++)
    ;
#endif // end ESP32
#endif // end !USE_FAST_PINIO
}

/*!
    @brief   Read the state of the software (bitbang) SPI MISO line.
    @return  true if HIGH, false if LOW.
*/
inline bool Adafruit_SPITFT::SPI_MISO_READ(void)
{
#if defined(USE_FAST_PINIO)
#if defined(KINETISK)
  return *swspi.misoPort;
#else  // !KINETISK
  return *swspi.misoPort & swspi.misoPinMask;
#endif // end !KINETISK
#else  // !USE_FAST_PINIO
  // return digitalRead(swspi._miso);
  return false;
#endif // end !USE_FAST_PINIO
}

/*!
    @brief  Issue a single 16-bit value to the display. Chip-select,
            transaction and data/command selection must have been
            previously set -- this ONLY issues the word. Despite the name,
            this function is used even if display connection is parallel;
            name was maintaned for backward compatibility. Naming is also
            not consistent with the 8-bit version, spiWrite(). Sorry about
            that. Again, staying compatible with outside code.
    @param  w  16-bit value to write.
*/
void Adafruit_SPITFT::SPI_WRITE16(uint16_t w)
{
  // Custom by Soldered
  if (connection == TFT_HYBRID_HWSPI_PARALLEL)
  {
#if defined(__AVR__)
    AVR_WRITESPI(w >> 8);
    AVR_WRITESPI(w);
    TFT_CS_STROBE();
    TFT_WR_STROBE();
#elif defined(ESP8266) || defined(ESP32)
    hwspi._spi->write16(w);
    TFT_CS_STROBE();
    TFT_WR_STROBE();
#endif
  }
}

/*!
    @brief  Issue a single 32-bit value to the display. Chip-select,
            transaction and data/command selection must have been
            previously set -- this ONLY issues the longword. Despite the
            name, this function is used even if display connection is
            parallel; name was maintaned for backward compatibility. Naming
            is also not consistent with the 8-bit version, spiWrite().
            Sorry about that. Again, staying compatible with outside code.
    @param  l  32-bit value to write.
*/
void Adafruit_SPITFT::SPI_WRITE32(uint32_t l)
{
  if (connection == TFT_HARD_SPI)
  {
#if defined(__AVR__)
    AVR_WRITESPI(l >> 24);
    AVR_WRITESPI(l >> 16);
    AVR_WRITESPI(l >> 8);
    AVR_WRITESPI(l);
#elif defined(ESP8266) || defined(ESP32)
    hwspi._spi->write32(l);
#elif defined(ARDUINO_ARCH_RP2040)
    spi_inst_t *pi_spi = hwspi._spi == &SPI ? spi0 : spi1;
    l = __builtin_bswap32(l);
    spi_write_blocking(pi_spi, (uint8_t *)&l, 4);
#else
    hwspi._spi->transfer(l >> 24);
    hwspi._spi->transfer(l >> 16);
    hwspi._spi->transfer(l >> 8);
    hwspi._spi->transfer(l);
#endif
  }
  else if (connection == TFT_SOFT_SPI)
  {
    for (uint8_t bit = 0; bit < 32; bit++)
    {
      if (l & 0x80000000)
        SPI_MOSI_HIGH();
      else
        SPI_MOSI_LOW();
      SPI_SCK_HIGH();
      SPI_SCK_LOW();
      l <<= 1;
    }
  }
  // Custom by Soldered
  // Send data on MOSI pin, trigger the latch pin each 16 bits
  else if (connection == TFT_HYBRID_HWSPI_PARALLEL)
  {
#if defined(__AVR__)
    AVR_WRITESPI(l >> 24);
    AVR_WRITESPI(l >> 16);
    TFT_CS_STROBE();
    TFT_WR_STROBE();
    AVR_WRITESPI(l >> 8);
    AVR_WRITESPI(l);
    TFT_CS_STROBE();
    TFT_WR_STROBE();
#elif defined(ESP8266) || defined(ESP32)
    hwspi._spi->write16(l >> 16);
    TFT_CS_STROBE();
    TFT_WR_STROBE();
    hwspi._spi->write16(l & 0X00FF);
    TFT_CS_STROBE();
    TFT_WR_STROBE();
#endif
  }
}

/*!
    @brief  Set the WR line LOW, then HIGH. Used for parallel-connected
            interfaces when writing data.
*/
inline void Adafruit_SPITFT::TFT_WR_STROBE(void)
{
  digitalWrite(tft8._wr, LOW);
  digitalWrite(tft8._wr, HIGH);
}

/*!
    @brief  Set the CS line HIGH, then LOW. Used for parallel-connected
            interfaces when writing data.
*/
inline void Adafruit_SPITFT::TFT_CS_STROBE(void)
{
  SPI_CS_HIGH();
  SPI_CS_LOW();
}

/*!
    @brief  Set the RD line HIGH. Used for parallel-connected interfaces
            when reading data.
*/
inline void Adafruit_SPITFT::TFT_RD_HIGH(void)
{
  digitalWrite(tft8._rd, HIGH);
}

/*!
    @brief  Set the RD line LOW. Used for parallel-connected interfaces
            when reading data.
*/
inline void Adafruit_SPITFT::TFT_RD_LOW(void)
{
  digitalWrite(tft8._rd, LOW);
}

#endif // end __AVR_ATtiny85__