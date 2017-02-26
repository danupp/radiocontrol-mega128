#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include "hd44780.h"
#include "build_settings.h"

#define fpga_addr 0x23
#define pll_addr 0x22

#define ROT1_A PE4
#define ROT1_B PE5
#define MENU_BUTTON PF2
#define MODE_BUTTON PF1
#define BAND_BUTTON PF0
#define VOLROT_A PE6
#define VOLROT_B PE7
#define VOL_BUTTON PF4
#define FREQ_BUTTON PE3
#define CLAR_POT PF3
#define MUTE PE0

#define LED_RED PD6
#define LED_GREEN PD5

#define LSB 0x01
#define LSBN 0x02
#define USB 0x03
#define USBN 0x04
#define CW 0x05
#define CWN 0x06
#define CWNN 0x07
#define AM 0x08
#define AMN 0x09
#define FM 0x0A

#define lcd_freq() \
  lcd_goto(0x04);  \
  sprintf(buffer,"%d,%06.2f     ",(int)floor(freq/1000+freq_offset),fmod(freq,1000));  \
  lcd_puts(buffer);  \
  lcd_goto(0x0f);  \
  if (clar > 10)  \
    sprintf(buffer,"+");  \
  else if (clar < -10)  \
    sprintf(buffer,"-");  \
  else  \
    sprintf(buffer," ");  \
  lcd_puts(buffer)


volatile uint8_t rot_flag, mode_flag, band_flag, timer_flag, vol_flag;
volatile uint16_t step_timer;
volatile uint8_t mode;
volatile uint8_t band;
volatile uint8_t tx_att;
volatile uint8_t band_timer = 255, vol_timer = 255;
volatile uint16_t bandf, tx, pll_n = 0;
volatile uint8_t rx_att;
volatile bool rffe_rx_att;

ISR(TIMER0_COMP_vect) {
  if (step_timer < 500)
    step_timer ++;
}

ISR(TIMER1_COMPA_vect) {
  timer_flag = 1;
  
}

ISR(INT4_vect) {  // falling level on INT4
  if(!(PINE & (1 << ROT1_A))) {
    _delay_us(20);
    if (!(PINE & (1 << ROT1_B))) {
      _delay_us(20);
      if (!(PINE & (1 << ROT1_A)) && !(PINE & (1 << ROT1_B))) {
	_delay_us(100);
	rot_flag = 0x02;
      }
    }
  }
}

ISR(INT5_vect) {  // falling level on INT5
  if (!(PINE & (1 << ROT1_B))) {
   _delay_us(20);
    if (!(PINE & (1 << ROT1_A))) {
      _delay_us(20);
      if (!(PINE & (1 << ROT1_A)) && !(PINE & (1 << ROT1_B))) {
	_delay_us(100);
	rot_flag = 0x01;
      }
    }
  }
}

ISR(INT6_vect) {  // falling level on INT6
  if(!(PINE & (1 << VOLROT_A))) {
    _delay_us(20);
    if (!(PINE & (1 << VOLROT_B))) {
      _delay_us(20);
      if (!(PINE & (1 << VOLROT_A)) && !(PINE & (1 << VOLROT_B))) {
	_delay_us(100);
	vol_flag = 0x02;
      }
    }
  }
}

ISR(INT7_vect) {  // falling level on INT7
  if (!(PINE & (1 << VOLROT_B))) {
    _delay_us(20);
    if (!(PINE & (1 << VOLROT_A))) {
      _delay_us(20);
      if (!(PINE & (1 << VOLROT_A)) && !(PINE & (1 << VOLROT_B))) {
	_delay_us(100);
	vol_flag = 0x01;
      }
    }
  }
}

void TWIinit(void) {
  TWBR = 0x45;
  TWCR = (1 << TWEN);
}

void TWIStart(void) {
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while ((TWCR & (1<<TWINT)) == 0);
}

void TWIStop(void) {
  TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void TWIWrite(uint8_t u8data) {
  TWDR = u8data;
  TWCR = (1<<TWINT)|(1<<TWEN);
  while ((TWCR & (1<<TWINT)) == 0);
}

uint8_t TWIReadACK(void) {
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
  while ((TWCR & (1<<TWINT)) == 0);
  return TWDR;
}

//read byte with NACK
uint8_t TWIReadNACK(void) {
  TWCR = (1<<TWINT)|(1<<TWEN);
  while ((TWCR & (1<<TWINT)) == 0);
  return TWDR;
}

uint8_t TWIGetStatus(void) {
  uint8_t status;
  //mask status
  status = TWSR & 0xF8;
  return status;
}

uint8_t TWI_write_byte(uint8_t addr, uint8_t data) {

  TWIStart();
  if (TWIGetStatus() != 0x08) {
    TWIStop();
    return 0x01;
  }
  TWIWrite(addr << 1);
  if (TWIGetStatus() != 0x18) {
    TWIStop();
    return 0x02;
  }  
  TWIWrite(data);
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x03;
  }
  TWIStop();
  return 0x00;
}

uint8_t TWI_read_byte(uint8_t addr, uint8_t* data) {

  TWIStart();
  if (TWIGetStatus() != 0x08) {
    TWIStop();
    return 0x01;
  }
  TWIWrite((addr << 1) | 0x01);
  if (TWIGetStatus() != 0x40) {
    TWIStop();
    return 0x02;
  }
  *data = TWIReadNACK();
  TWIStop();
  return 0x00;
}

uint8_t TWI_write_word(uint8_t addr, uint16_t data_word) {

  TWIStart();
  if (TWIGetStatus() != 0x08) {
    TWIStop();
    return 0x01;
  }
  TWIWrite(addr << 1);
  if (TWIGetStatus() != 0x18) {
    TWIStop();
    return 0x02;  
  }
  TWIWrite((uint8_t)((data_word & 0xFF00) >> 8));
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x03;
  }
  TWIWrite((uint8_t)(data_word & 0x00FF));
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x04;
  }
  TWIStop();
  return 0x00;
}

uint8_t TWI_write_3bytes(uint8_t addr, uint8_t byte1, uint8_t byte2, uint8_t byte3) {

  TWIStart();
  if (TWIGetStatus() != 0x08) {
    TWIStop();
    return 0x01;
  }
  TWIWrite(addr << 1);
  if (TWIGetStatus() != 0x18) {
    TWIStop();
    return 0x02;  
  }
  TWIWrite(byte1);
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x03;
  }
  TWIWrite(byte2);
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x04;
  }
  TWIWrite(byte3);
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x04;
  }
  TWIStop();
  return 0x00;
}

uint8_t TWI_write_4bytes(uint8_t addr, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {

  TWIStart();
  if (TWIGetStatus() != 0x08) {
    TWIStop();
    return 0x01;
  }
  TWIWrite(addr << 1);
  if (TWIGetStatus() != 0x18) {
    TWIStop();
    return 0x02;  
  }
  TWIWrite(byte1);
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x03;
  }
  TWIWrite(byte2);
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x04;
  }
  TWIWrite(byte3);
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x04;
  }
  TWIWrite(byte4);
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x04;
  }
  TWIStop();
  return 0x00;
}

uint8_t TWI_write_5bytes(uint8_t addr, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5) {

  TWIStart();
  if (TWIGetStatus() != 0x08) {
    TWIStop();
    return 0x01;
  }
  TWIWrite(addr << 1);
  if (TWIGetStatus() != 0x18) {
    TWIStop();
    return 0x02;  
  }
  TWIWrite(byte1);
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x03;
  }
  TWIWrite(byte2);
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x04;
  }
  TWIWrite(byte3);
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x04;
  }
  TWIWrite(byte4);
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x04;
  }
  TWIWrite(byte5);
  if (TWIGetStatus() != 0x28) {
    TWIStop();
    return 0x04;
  }
  TWIStop();
  return 0x00;
}

uint8_t updateVolume(uint8_t vol) {
  
  uint8_t addr, err;

  addr = 0x80; // Address for audio conf

  err = TWI_write_5bytes(fpga_addr, addr, 0x00, 0x00, 0x00, vol);
  if(err)
    return err;
  return 0x00;
}

uint16_t readClar(void) {
  return (ADC);
}

uint8_t updateRFFE(void) {
  uint8_t err;
  uint16_t rxatt;

  if (rx_att)
    rxatt = 0x0001;
  else
    rxatt = 0x0000;

  err = TWI_write_word(pll_addr,(pll_n & 0x0fff) | (tx << 15) | (bandf << 14) | (rxatt << 13));
  if (err)
    return err+5;
  return 0x00;
} 
  
uint8_t updateFreq(double freq, int8_t clar, uint8_t all_update) {
  //   char buffer[60];
  uint8_t byte1, byte2, byte3, byte4, byte5, err;
  uint16_t pll_n_ = pll_n;
  double freq_lo, ref_freq, ftw;
  static double ftw_toptop, ftw_topbot, ftw_bottop, ftw_botbot;

  /*  if (band < 200) { 
    freq_lo = (double)(freq_MHz*1000 + 45000 + freq_kHz);
    bandf = 0;
  }
  else {
    if (freq < 200*1e6) 
      freq_lo = (double)(freq_MHz*1000 + 21400 + freq_kHz);
    else
      freq_lo = (double)((freq_MHz - 1152)*1000 + 21400 + freq_kHz);
    bandf = 1;
    }*/

  if (all_update) {
    freq_lo = freq + (double)21400;
    bandf = 1;

    if (mode == USB) {
      freq_lo += (double)1.8;
    }
    else if (mode == CW) {
      freq_lo += (double)0.9;
    }
    else if (mode == CWN) {
      //freq_lo += (double)0.9;
    }
    else if (mode == LSB ) {
      freq_lo -= (double)1.8;
    }

    pll_n_ = (uint16_t)lround(freq_lo/(double)100);  // 100 kHz step
    ref_freq = 170*freq_lo/pll_n_;  // 170 ref divider value
    ftw = ref_freq*(double)279.62026666667; // 20MHz,25bit: 279.620266667 19.2MHz,22bit: 36.4088889
    ftw_toptop = floor(ftw/(double)16777216);
    ftw_topbot = floor((ftw-ftw_toptop*(double)16777216)/(double)65536);
    ftw_bottop = floor((ftw-ftw_toptop*(double)16777216-ftw_topbot*(double)65536)/(double)256);
    ftw_botbot = fmod(ftw,256);
  }

  clar = -(clar+1);  // For arch with LO above RF freq

  byte1 = 0xc0;
  byte2 = (uint8_t)(ftw_toptop) | (clar & 0xfe);
  byte3 = (uint8_t)ftw_topbot;
  byte4 = (uint8_t)ftw_bottop;
  byte5 = (uint8_t)ftw_botbot;

  err = TWI_write_5bytes(fpga_addr, byte1, byte2, byte3, byte4, byte5);
  if(err)
    return err;
 
  _delay_us(50);
  if (pll_n != pll_n_) {
    pll_n = pll_n_;
    err = updateRFFE();
    if (err)
      return err+5;
  }
  return 0x00;
}

uint8_t updateSettings (void) {

  uint8_t err;
  uint8_t byte1, byte2, byte3, byte4, byte5;

  switch (mode) {  
  case LSB:
    byte1 = 0b01111000; // Set USB (LO+)
    byte2 = (tx_att << 6)|(rx_att << 3)|0x01;
    byte3 = 0x08;
    byte4 = 0x00;
    byte5 = 0x00;
    break;
  case USB:
    byte1 = 0b01110000; // Set LSB (LO+)
    byte2 = (tx_att << 6)|(rx_att << 3)|0x01;
    byte3 = 0x08;
    byte4 = 0x00;
    byte5 = 0x00;
    break;
  case CW:
    byte1 = 0b01110000; // Set USB (LO+) 
    byte2 = (tx_att << 6)|(rx_att << 3)|0x01;
    byte3 = 0x08;
    byte4 = 0x00;
    byte5 = 0x00;
    break;
  case CWN:
    byte1 = 0b01100000; // Set narrow USB (LO+)
    byte2 = (tx_att << 6)|(rx_att << 3)|0x01;
    byte3 = 0x08;
    byte4 = 0x00;
    byte5 = 0x00;
    break;
  case AM:
    byte1 = 0b01000000;
    byte2 = (tx_att << 6)|(rx_att << 3)|0x01;
    byte3 = 0x08;
    byte4 = 0x00;
    byte5 = 0x00;
    break;
  case FM:
    byte1 = 0b01000001;
    byte2 = (tx_att << 6)|(rx_att << 3)|0x01;
    byte3 = 0x08;
    byte4 = 0x00;
    byte5 = 0x00;
  }
  

  err = TWI_write_5bytes(fpga_addr, byte1, byte2, byte3, byte4, byte5);
  if (err)
    return err;

  return 0x00;

}

void adcInit(void) {
  ADCSRA = (1 << ADEN) | (1 << ADFR) | (1 << ADPS2) | (1 << ADPS1); // Free run, /64 -> 77 kHz
  ADMUX = 0x43; // ADC3, AVCC ref
  ADCSRA |= (1 << ADSC); // Start conversion

}

void Timer0Init(void) {

  TCCR0 = 0x0f; // CTC mode, 1024 prescaler
  OCR0 = (uint8_t)10; // 1000000/(1024*10) = 100 Hz, 10ms
  TIMSK |= (1 << OCIE0); // Interrupt at compare match
}

void Timer1Init(void) {

  TCCR1A = 0x00;  
  TCCR1B = 0x0b; // CTC mode, 64 prescaler
  OCR1A = (uint16_t)1563; // 1000000/(64*1563) = 10 Hz, 100ms
  TIMSK |= (1 << OCIE1A); // Interrupt at compare match
}

int main(void)
{
  char buffer[60];
  double freq, freq_last;  // kHz part
  int16_t clarval, clarval_last;
  int8_t clar = 0;
  int freq_offset; // offset in MHz for display
  uint8_t err, data;
  uint8_t rssi, rssi_max=0, rssi_count=0;
  uint8_t last_dir;
  uint8_t tx_last = 255;
  uint8_t vol = 0x18;
  uint16_t steps;
  
  static const char string_intro_row1[] PROGMEM = "*** SM6VFZ";
  static const char string_intro_row2[] PROGMEM = "Starting radio";

  DDRA = 0xFF;
  DDRB = 0xFF;
  DDRC = 0xFF;
  DDRD = 0xFF;
  DDRE = ~(uint8_t)((1 << ROT1_A)|(1 << ROT1_B)|(1 << VOLROT_A)|(1 << VOLROT_B)|(1 << FREQ_BUTTON));
  DDRF = ~(uint8_t)((1 << VOL_BUTTON)|(1 << MODE_BUTTON)|(1 << BAND_BUTTON)|(1 << CLAR_POT)|(1 << MENU_BUTTON));
  DDRG = 0xFF;  
	
  PORTA = 0x00;
  PORTB = 0x00;
  PORTC = 0x00;
  PORTD = (uint8_t)(1 << PD2); // PD2 = I2C pull-up
  PORTE = (uint8_t)((1 << ROT1_A)|(1 << ROT1_B)|(1 << VOLROT_A)|(1 << VOLROT_B)|(1 << FREQ_BUTTON));
  PORTF = (uint8_t)((1 << VOL_BUTTON)|(1 << MODE_BUTTON)|(1 << BAND_BUTTON)|(1 << MODE_BUTTON));
  PORTG = 0x00;
	
  // turn off the analog comparator
  ACSR = 0x40U;
	
  // turn off SPI, TWI and USART0
  //PRR  = 0x86U;
	
  EICRA = 0x00;
  EICRB = (1<<ISC41)|(1<<ISC51)|(1<<ISC61)|(1<<ISC71);

  EIMSK = (1 << INTF4)|(1 << INTF5)|(1 << INTF6)|(1 << INTF7);

  wdt_disable();

  _delay_ms(500);

  TIMSK = 0;
  Timer0Init();
  Timer1Init();
  TWIinit();
  lcd_init();
  lcd_clrscr();
  adcInit();

  _delay_ms(200);

  strcpy_P(buffer, string_intro_row1);
  lcd_puts(buffer);
  lcd_goto(0x40);
  strcpy_P(buffer, string_intro_row2);
  lcd_puts(buffer);

  rot_flag = 0x01;
  mode_flag = 0x01;
  band_flag = 0x02;
  timer_flag = 0x00;

  band = 10;
  step_timer = 255;
  steps = 0;
  last_dir = 0x00;
  mode = 0x00;
  tx = 0x0000;
  rx_att = 0x00;
  tx_att = 0x00;
  rffe_rx_att = false;

  err = updateVolume(vol);
  
  _delay_ms(2000);

  lcd_clrscr();
  
  sei();

  while (1) {
    if (timer_flag) {

#ifdef CLARPOT
      clarval = readClar();
#endif
#ifndef CLARPOT
      clarval = 0;
#endif

      if ((clarval < clarval_last - 10) || (clarval > clarval_last + 10)) {
	clar = (int8_t)((clarval - 512) >> 2);
	err = updateFreq(freq,clar,0);
	if (err) {
	  sprintf(buffer,"Err %x         ",err);
	  lcd_goto(0x40);
	  lcd_puts(buffer);	
	}
	else {
	  lcd_freq();
	}
	clarval_last = clarval;
      }

      if (vol_timer < 255)
	vol_timer ++;

      // Band button

      if (band_timer < 9) { // Recently pressed ?
	band_timer ++;
	if ((band_timer < 9) && (PINF & (1 << BAND_BUTTON))) {
	  band_flag = 0x01; // short press
	  band_timer = 255;
	}
	if ((band_timer == 9) && (!(PINF & (1 << BAND_BUTTON)))) 
	  band_flag = 0x02; // long press
      }
      else if (!(PINF & (1 << BAND_BUTTON))) {  // New press
	band_timer = 0;
      }

      // Mode button

      if (!(PINF & (1 << MODE_BUTTON))) {
	mode_flag = 0x01;
      }
      
      // Update frequency if necessary:
      if (freq != freq_last) {
	lcd_freq();
	err = updateFreq(freq,clar,1);
	if (err) {
	  sprintf(buffer,"Err %x         ",err);
	  lcd_goto(0x40);
	  lcd_puts(buffer);	
	}
	freq_last = freq;
      }

      // Read RSSI and status:
      err = TWI_read_byte(fpga_addr, &data);
      if ((data & 0x80) && (tx_last != 1)) { // Shift to TX ?
	tx = 0x0001;
	updateRFFE();
	PORTD |= (1 << LED_RED);
	PORTD &= ~(1 << LED_GREEN);
	tx_last = 1;
      }
      else if (!(data & 0x80) && (tx_last != 0)) { // Shift to RX ?
	tx = 0x0000;
	updateRFFE();
	PORTD |= (1 << LED_GREEN);
	PORTD &= ~(1 << LED_RED);
	tx_last = 0;
      }

      rssi = (0x3f & data); 
      if (rssi_max < rssi) 
	rssi_max = rssi;
      rssi_count ++;
      if ((!rffe_rx_att && rssi_count > 2) || (rffe_rx_att && rssi_count > 2)) {
	rssi_count = 0;
	if (rssi_max < 4)
	  rssi = 0;
	else 
	  rssi = rssi_max - 4;
	rssi_max = 0;

	if (vol_timer < 10); // Don't print if vol info
	else if (rffe_rx_att && (rssi>9)) {
	  sprintf(buffer,"S---9+++   ");
	  lcd_goto(0x40);
	  lcd_puts(buffer);
	}
	else if (rffe_rx_att && (rssi < 10)) {
	  rffe_rx_att = false;
	  err = updateRFFE();
	}
	else if (rssi < 2) { 
	  sprintf(buffer,"S%d         ",rssi);
	  lcd_goto(0x40);
	  lcd_puts(buffer);
	}
	else if (rssi < 4) {
	  sprintf(buffer,"S-%d        ",rssi);
	  lcd_goto(0x40);
	  lcd_puts(buffer);
 	}
	else if (rssi < 6) {
	  sprintf(buffer,"S--%d       ",rssi);
	  lcd_goto(0x40);
	  lcd_puts(buffer);
	}
	else if (rssi < 8) {
	  sprintf(buffer,"S---%d      ",rssi);
	  lcd_goto(0x40);
	  lcd_puts(buffer);
	}
	else if (rssi < 10) {
	  sprintf(buffer,"S----%d     ",rssi);
	  lcd_goto(0x40);
	  lcd_puts(buffer);
	}
	else if (rssi < 12) {
	  sprintf(buffer,"S----9+    ");
	  lcd_goto(0x40);
	  lcd_puts(buffer);
	}
	else if (rssi < 13 ){
	  sprintf(buffer,"S----9++   ");
	  lcd_goto(0x40);
	  lcd_puts(buffer);
	}
	else {
	  rffe_rx_att = true;
	  err = updateRFFE();
	}
	rssi = 0;
      }
      timer_flag = 0;
    }
    else if (mode_flag) {
      switch (mode) {
      case LSB:
	mode = USB;
	if (!updateSettings()) { 
	  lcd_goto(0x00);
	  sprintf(buffer,"USB ");
	  lcd_puts(buffer);
	}
	break;
      case USB:
	mode = CW;
	if(!updateSettings()) {
	  lcd_goto(0x00);
	  sprintf(buffer,"CW  ");
	  lcd_puts(buffer);
	}
	break;
      case CW:
	mode = CWN;
	if(!updateSettings()) {
	  lcd_goto(0x00);
	  sprintf(buffer,"CWN ");
	  lcd_puts(buffer);
	}
	break;
      case CWN:
	if (band < 200) {
	  mode = AM;
	  if(!updateSettings()) {
	    lcd_goto(0x00);
	    sprintf(buffer,"AM  ");
	    lcd_puts(buffer);
	  }
	}
	else {
	  mode = FM;
	  if(!updateSettings()) {
	    lcd_goto(0x00);
	    sprintf(buffer,"FM  ");
	    lcd_puts(buffer);
	  }
	}	  
	break;
      case AM:
      case FM:
      default:
	mode = USB;
	if(!updateSettings()) { 
	  lcd_goto(0x00);
	  sprintf(buffer,"USB ");
	  lcd_puts(buffer);
	}
	break;
      }
      _delay_ms(250);
      err = updateFreq(freq,clar,1);  // To shift +/-
      mode_flag = 0x00;
    }

    if (band_flag) {
      if (band_flag == 0x02) { // long press
#ifdef TWOMETER_TRANSV
	if (band == 200) {
	  band = 201;
	  freq_offset = 1152;
	  freq = 144200;
	  tx_att = 0x01;
	  rx_att = 0x00;
	  updateSettings();
	}
	else if (band == 201) {
	  band = 202;
	  freq_offset = 5616;
	  freq = 144200;
	  tx_att = 0x01;
	  rx_att = 0x00;
	  updateSettings();
	}
	else {
	  band = 200;
	  freq_offset = 0;
	  freq = 144300;
	  tx_att = 0x00;
	  rx_att = 0x00;
	  updateSettings();
	  /*
	  band = 160;
	  freq_offset = 0;
	  freq = 1810;
	  tx_att = 0x00;
	  */
	}
#endif
      }
      else {
#ifdef HF_HAM
	if (freq<3500)
	  freq = 3510;
	else if(freq<7000)
	  freq = 7010;
	else if(freq<10100)
	  freq = 10110;
	else if(freq<14000)
	  freq = 14010;
	else if(freq<18068)
	  freq = 18078;
	else if(freq<21000)
	  freq = 21010;
	else if(freq<24890)
	  freq = 24900;
	else if(freq<28000)
	  freq = 28010;
	else if(freq<28500)
	  freq = 28600;
	else
	  freq = 1810;
#endif
#ifdef TWOMETER
      if(freq<144150)
	freq = 144300;
      else if(freq<144400)
	freq = 144450;
      else if(freq<145000)
	freq = 145500;
      else if(freq<150000)
	freq = 144050;
#endif
#ifdef TWOMETER_TRANSV
      if(freq<144700)
	freq = 144800;
      else if(freq<150000)
	freq = 144200;
#endif
      }

      lcd_freq();
      err = updateFreq(freq,clar,1);
      if (err) {
	sprintf(buffer,"Err %x         ",err);
	lcd_goto(0x40);
	lcd_puts(buffer);	
      }      
      err = updateSettings();
      if (err) {
	sprintf(buffer,"Err %x         ",err);
	lcd_goto(0x40);
	lcd_puts(buffer);	
      }      
      _delay_ms(200);
      band_flag = 0x00;
    }
    else if (rot_flag) {
      if (rot_flag != last_dir) {
	steps = 0;
	step_timer = 0;
	last_dir = rot_flag;
      }
      else if (rot_flag == 0x01) {
	if (step_timer < 30) {
	  step_timer = 0;
	  steps ++;
	}
	else {
	  step_timer = 0;
	  steps = 1;
	}
	
	if (steps > 500) {
	  if (mode == AM)
	    freq += 20;
	  else if (mode == FM)
	    freq += 50;
	  else
	    freq += 1;
	}
	else if (steps > 150) {
	  if (mode == AM)
	    freq += 5;
	  else if (mode == FM)
	    freq += 25;
	  else
	    freq += 0.5;
	}
	else {
	  if (mode == AM)
	    freq += 1;
	  else if (mode == FM)
	    freq += 2.5;
	  else
	    freq+=0.05;
	}
	last_dir = 0x01;
      }
      else if (rot_flag == 0x02) {
	if ((step_timer < 30) && (last_dir == rot_flag)) {
	  step_timer = 0;
	  steps ++;
	}
	else {
	  step_timer = 0;
	  steps = 1;
	}
	
	if (steps > 500) {
	  if (mode == AM)
	    freq -= 20;
	  else if (mode == FM)
	    freq -= 50;
	  else
	    freq -= 1;
	}
	else if (steps > 150) {
	  if (mode == AM)
	    freq -= 5;
	  else if (mode == FM)
	    freq -= 25;
	  else
	    freq -= 0.5;
	}
	else {
	  if (mode == AM)
	    freq -= 1;
	  else if (mode == FM)
	    freq -= 2.5;
	  else
	    freq-=0.05;
	}
      last_dir = 0x02;
      }
      //_delay_ms(30);

      rot_flag = 0x00;
    }
    else if (vol_flag) {
      if (vol_timer > 1) {
	if (vol_flag == 0x01) {
	  if(vol<31)
	    vol ++; // Higher means lower 
	  sprintf(buffer,"Vol %d    ", 31-vol);
	  lcd_goto(0x40);
	  lcd_puts(buffer);
	}
	else if (vol_flag == 0x02) {
	  if(vol>0)
	    vol --;
	  sprintf(buffer,"Vol %d    ", 31-vol);
	  lcd_goto(0x40);
	  lcd_puts(buffer);
	}

	err = updateVolume(vol);

	if (err) {
	  sprintf(buffer,"Err %x         ",err);
	  lcd_goto(0x40);
	  lcd_puts(buffer);	
	}      
	vol_timer = 0;
      }
      vol_flag = 0x00;
    }
    //sleep_mode();
  }
}
