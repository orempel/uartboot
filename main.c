/***************************************************************************
 *   Copyright (C) 10/2014 by Olaf Rempel                                  *
 *   razzor@kopf-tisch.de                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; version 2 of the License,               *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

/* *********************************************************************** */
#if defined(CONFIG_ispprog)
/*
 * using ATmega16 @7.3728MHz:
 * Fuse H: 0xDA (512 words bootloader, jtag disabled)
 * Fuse L: 0xFF (ext. Crystal)
 */
#define F_CPU       7372800
#define BAUDRATE    19200
#define DEVCODE     0x74

/* 100 * 10ms => 1000ms */
//#define TIMEOUT     100

/* enter bootloader if PIND3 is high */
#define ISP_DDR     DDRD
#define ISP_PIN     PIND
#define ISP_PORT    PORTD
#define ISP_NUM     PIND3
#define ISP_POL     1

/* low active led on PORTB3 */
#define LED_DDR     DDRB
#define LED_PORT    PORTB
#define LED_NUM     PORTB3
#define LED_POL     0

/* *********************************************************************** */
#elif defined(CONFIG_flightctrl)
/*
 * using ATmega644 @20MHz:
 * Fuse E: 0xFD (2.7V BOD)
 * Fuse H: 0xDC (1024 words bootloader)
 * Fuse L: 0xFF (ext. Crystal)
 */
#define F_CPU       20000000
#define BAUDRATE    57600
#define DEVCODE     0x74    /* mega16 devcode */

/* 100 * 10ms => 1000ms */
#define TIMEOUT     100

/* enter bootloader if PIND3 is high */
//#define ISP_DDR     DDRD
//#define ISP_PIN     PIND
//#define ISP_PORT    PORTD
//#define ISP_NUM     PIND3
//#define ISP_POL     1

/* high active led on PORTB1 */
#define LED_DDR     DDRB
#define LED_PORT    PORTB
#define LED_NUM     PORTB1
#define LED_POL     1

/* support special MKBL commands */
#define SUPPORT_MKBL 1

/* *********************************************************************** */
#elif defined(CONFIG_funkstuff)
/*
 * using ATmega88 @8MHz:
 * Fuse E: 0xFA (2.7V BOD)
 * Fuse H: 0xDD (512 words bootloader)
 * Fuse L: 0xE2 (internal osc)
 */
#define F_CPU       8000000
#define BAUDRATE    19200
#define DEVCODE     0x76    /* mega8 devcode */

/* 100 * 10ms => 1000ms */
#define TIMEOUT     100

/* enter bootloader if PIND3 is high */
//#define ISP_DDR     DDRD
//#define ISP_PIN     PIND
//#define ISP_PORT    PORTD
//#define ISP_NUM     PIND3
//#define ISP_POL     1

/* low active led on PORTB3 */
//#define LED_DDR     DDRB
//#define LED_PORT    PORTB
//#define LED_NUM     PORTB3
//#define LED_POL     0

/* *********************************************************************** */
#elif defined(CONFIG_ispprog2)
/*
 * using ATmega328P @8MHz:
 * Fuse E: 0xFA (2.7V BOD)
 * Fuse H: 0xDC (512 words bootloader)
 * Fuse L: 0xE2 (internal osc)
 */
#define F_CPU       8000000
#define BAUDRATE    115200
#define DEVCODE     0x72    /* mega32 devcode */

/* 100 * 10ms => 1000ms */
#define TIMEOUT     100

/* enter bootloader if PINB1 is low */
#define ISP_DDR     DDRB
#define ISP_PIN     PINB
#define ISP_PORT    PORTB
#define ISP_NUM     PINB1
#define ISP_POL     0

/* high active led on PORTB0 */
#define LED_DDR     DDRB
#define LED_PORT    PORTB
#define LED_NUM     PORTB0
#define LED_POL     1

/* trim internal oscillator to get "good" baudrate */
#define OSCCAL_VALUE    0x80

/* *********************************************************************** */
#else
#error "unknown CONFIG"
#endif
/* *********************************************************************** */

/* needs F_CPU */
#include <util/delay.h>

#define UART_CALC_BAUDRATE(baudRate) ((uint32_t)(F_CPU) / ((uint32_t)(baudRate)*16) -1)

#if defined(ISP_NUM)
#if (ISP_POL == 0)
#define ISP_INIT()  { ISP_DDR &= ~(1<<ISP_NUM); ISP_PORT |= (1<<ISP_NUM); }
#define ISP_CHECK() (ISP_PIN & (1<<ISP_NUM))
#else
#define ISP_INIT()  { ISP_DDR &= ~(1<<ISP_NUM); ISP_PORT &= ~(1<<ISP_NUM); }
#define ISP_CHECK() !(ISP_PIN & (1<<ISP_NUM))
#endif
#endif /* defined(ISP_NUM) */

#if defined(LED_NUM)
#define LED_INIT()  LED_DDR |= (1<<LED_NUM);
#if (LED_POL == 0)
#define LED_ON()    LED_PORT &= ~(1<<LED_NUM)
#define LED_OFF()   LED_PORT |= (1<<LED_NUM)
#else
#define LED_ON()    LED_PORT |= (1<<LED_NUM)
#define LED_OFF()   LED_PORT &= ~(1<<LED_NUM)
#endif
#else
#define LED_INIT()
#define LED_ON()
#define LED_OFF()
#endif /* defined(LED_NUM) */


static uint8_t page_buf[SPM_PAGESIZE];
register uint16_t address asm("r2");


#if defined(__AVR_ATmega16__)
static void uartinit(void)
{
    /* Set baud rate */
    UBRRH = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
    UBRRL = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

    /* enable usart with 8n1 */
    UCSRB = (1<<TXEN) | (1<<RXEN);
    UCSRC = (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);
} /* uartinit */

static void sendchar(uint8_t data)
{
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = data;
} /* sendchar */

static uint8_t recvchar(void)
{
    loop_until_bit_is_set(UCSRA, RXC);
    return UDR;
} /* recvchar */

#if defined(TIMEOUT)
static uint8_t recvcheck(void)
{
    return (UCSRA & (1<<RXC));
} /* recvcheck */
#endif /* defined(TIMEOUT) */

#elif defined(__AVR_ATmega88__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega644P__)
static void uartinit(void)
{
    /* set baudrate */
    UBRR0H = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
    UBRR0L = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

    /* USART: rx/tx enable, 8n1 */
    UCSR0B = (1<<TXEN0) | (1<<RXEN0);
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
} /* uartinit */

static void sendchar(uint8_t data)
{
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = data;
} /* sendchar */

static uint8_t recvchar(void)
{
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
} /* recvchar */

#if defined(TIMEOUT)
static uint8_t recvcheck(void)
{
    return (UCSR0A & (1<<RXC0));
} /* recvcheck */
#endif /* defined(TIMEOUT) */
#endif


static void eraseFlash(void)
{
    address = 0;
    while (address < BOOTLOADER_START) {
        boot_page_erase(address);
        boot_spm_busy_wait();
        address += SPM_PAGESIZE;
    }
    boot_rww_enable();
} /* eraseFlash */


static void writeFlashPage(uint16_t size)
{
    uint16_t pagestart = address;
    uint8_t *tmp = page_buf;

    do {
        uint16_t data;
        data = *tmp++;
        data |= (*tmp++ << 8);
        boot_page_fill(address, data);

        address += 2;
        size -= 2;
    } while (size);

    boot_page_write(pagestart);
    boot_spm_busy_wait();
    boot_rww_enable();
} /* writeFlashPage */


static void writeEEpromPage(uint16_t size)
{
    uint8_t *tmp = page_buf;

    do {
        EEARL = address;
        EEARH = (address >> 8);
        EEDR = *tmp++;
        address++;

#if defined(__AVR_ATmega16__)
        EECR |= (1<<EEMWE);
        EECR |= (1<<EEWE);
#elif defined(__AVR_ATmega88__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega644P__)
        EECR |= (1<<EEMPE);
        EECR |= (1<<EEPE);
#else
#error writeEEpromPage() not implemented
#endif
        eeprom_busy_wait();

        size--;
    } while (size);
} /* writeEEpromPage */


static void readSendFlashPage(uint16_t size)
{
    do {
        sendchar(pgm_read_byte_near(address++));
        size--;
    } while (size);
} /* readSendFlashPage */


static void readSendEEpromPage(uint16_t size)
{
    do {
        EEARL = address;
        EEARH = (address >> 8);
        EECR |= (1<<EERE);
        address++;

        sendchar(EEDR);

        size--;
    } while (size);
} /* readSendEEpromPage */


#if defined(__AVR_ATmega88__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega644P__)
/*
 * For newer devices the watchdog timer remains active even after a
 * system reset. So disable it as soon as possible.
 * automagically called on startup
 */
void disable_wdt_timer(void) __attribute__((naked, section(".init3")));
void disable_wdt_timer(void)
{
    MCUSR = 0;
    WDTCSR = (1<<WDCE) | (1<<WDE);
    WDTCSR = (0<<WDE);
}
#endif

static void (*jump_to_app)(void) __attribute__ ((noreturn)) = 0x0000;

int main(void) __attribute__ ((noreturn));
int main(void)
{
#if defined(ISP_NUM)
    /* set as input, enable pullup/pulldown */
    ISP_INIT();

    /* check if bootloader is not selected */
    if (ISP_CHECK()) {
        jump_to_app();
    }
#endif /* defined(ISP_NUM) */

#if defined(LED_NUM)
    LED_DDR |= (1<<LED_NUM);
#endif /* defined(LED_NUM) */

#if defined(OSCCAL_VALUE)
    OSCCAL = OSCCAL_VALUE;
#endif /* defined(OSCCAL_VALUE) */

    /* initalize UART */
    uartinit();

    uint8_t val = 'S';

#if defined(TIMEOUT)
    uint8_t prev = 0x00;
    uint8_t boot_timeout = TIMEOUT;

    while (1) {
        if (recvcheck()) {
            val = recvchar();

#if defined(SUPPORT_MKBL)
            if ((prev == 0x1B) && (val == 0xAA || val == 'S')) {
                break;
            }
#else
            if ((prev == 0x1B) && (val == 'S')) {
                break;
            }
#endif /* defined(SUPPORT_MKBL) */

            prev = val;
        }

        _delay_ms(10);

        boot_timeout--;
        if (boot_timeout == 0x00) {
            LED_OFF();
            jump_to_app();
        }

#if defined(LED_NUM)
        if (!(boot_timeout & 0x03)) {
            LED_PORT ^= (1<<LED_NUM);
        }
#endif /* defined(LED_NUM) */
    }
#endif /* defined(TIMEOUT) */

    /* turn on LED */
    LED_ON();

    while (1) {
        uint8_t response = 0xFF;

        /* Autoincrement? */
        if (val == 'a') {
            response = 'Y';

        /* write address */
        } else if (val == 'A') {
            address = (recvchar() << 8);
            address |= recvchar();
            response = '\r';

        /* Buffer load support */
        } else if (val == 'b') {
            sendchar('Y');
            sendchar((SPM_PAGESIZE >> 8) & 0xFF);
            response = SPM_PAGESIZE & 0xFF;

        /* Start buffer load */
        } else if (val == 'B') {
            uint16_t size;
            uint16_t count;
            uint8_t *data = page_buf;

            size = (recvchar() << 8);
            size |= recvchar();
            val = recvchar();

            for (count = 0; count < SPM_PAGESIZE; count++) {
                *data++ = (count < size) ? recvchar() : 0xFF;
            }

            if ((val == 'F') && (address < BOOTLOADER_START)) {
                writeFlashPage(size);
                response = '\r';

            } else if ((val == 'E') && (address < E2END)) {
                writeEEpromPage(size);
                response = '\r';

            } else {
                response = 0x00;
            }

        /* Block read */
        } else if (val == 'g') {
            uint16_t size;

            size = (recvchar() << 8);
            size |= recvchar();
            val = recvchar();

            if (val == 'F') {
                readSendFlashPage(size);

            } else if (val == 'E') {
                readSendEEpromPage(size);
            }

        /* Chip erase */
        } else if (val == 'e') {
            eraseFlash();
            response = '\r';

        /* Exit upgrade */
        } else if (val == 'E') {
            wdt_enable(WDTO_250MS);
            response = '\r';

        /* Enter/Leave programming mode */
        } else if ((val == 'P') || (val == 'L')) {
            response = '\r';

        /* return programmer type: serial */
        } else if (val == 'p') {
            response = 'S';

        /* Return device type */
        } else if (val == 't') {
            sendchar(DEVCODE);
            response = 0x00;

        /* Clear and set LED ignored */
        } else if ((val == 'x') || (val == 'y')) {
            recvchar();
            response = '\r';

        /* Set device */
        } else if (val == 'T') {
            response = (recvchar() == DEVCODE) ? '\r' : 0x00;

        /* Return software identifier */
        } else if (val == 'S') {
            sendchar('A');
            sendchar('V');
            sendchar('R');
            sendchar('B');
            sendchar('O');
            sendchar('O');
            response = 'T';

#if defined(SUPPORT_MKBL)
        } else if (val == 0xAA) {
            sendchar('M');
            sendchar('K');
            sendchar('B');
            response = 'L';
#endif /* defined(SUPPORT_MKBL) */

        /* Return Software/Hardware Version */
        } else if ((val == 'V') || (val == 'v')) {
            sendchar('0');
            response = '8';

        /* Return Signature Bytes */
        } else if (val == 's') {
            sendchar(SIGNATURE_2);
            sendchar(SIGNATURE_1);
            response = SIGNATURE_0;

        } else if (val != 0x1B) {
            response = '?';
        }

        if (response != 0xFF) {
            sendchar(response);
        }

        val = recvchar();
    }
}
