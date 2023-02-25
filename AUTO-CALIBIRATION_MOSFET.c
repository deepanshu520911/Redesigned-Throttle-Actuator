#include <util/delay.h>

#include <avr/io.h>
#define ddrb (*(volatile char*)0x37)
#define portb (*(volatile char*)0x38)
#define pinb (*(volatile char*)0x36)
#define eearh (*(volatile char*)0x3F)
#define eearl (*(volatile char*)0x3E)
#define eedr (*(volatile char*)0x3D)
#define eecr (*(volatile char*)0x3C)
#define sreg (*(volatile char*)0x5F)

// #define ZERO 435
#define ZERO 485


uint16_t SPI_READ_AS5147D() {
  portb &= ~(1 << 2);  // CLOCK STARTS WITH LOGIC LOW

  uint8_t C = 0, D = 0, E = 0, F = 0, G = 0, H = 0, I = 0, J = 0, K = 0, L = 0, M = 0, N = 0, O = 0, P = 0;

  portb &= ~(1 << 3);  // AS5047D CS PIN LOW

//  portb = 0x36;
  portb = 0x26; // With mosfet to LDAC
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  D = pinb;
  D = pinb;
  D = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  E = pinb;
  E = pinb;
  E = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  F = pinb;
  F = pinb;
  F = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  G = pinb;
  G = pinb;
  G = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  H = pinb;
  H = pinb;
  H = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  I = pinb;
  I = pinb;
  I = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  J = pinb;
  J = pinb;
  J = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  K = pinb;
  K = pinb;
  K = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  L = pinb;
  L = pinb;
  L = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  M = pinb;
  M = pinb;
  M = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  N = pinb;
  N = pinb;
  N = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  O = pinb;
  O = pinb;
  O = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;
  portb = 0x36;
  P = pinb;
  P = pinb;
  P = pinb;
  portb = 0x32;
  C = pinb;
  C = pinb;
  C = pinb;

  portb |= (1 << 3);  // AS5047D CS PIN HIGH


  uint8_t ANGLE_VALUE1 = 0;
  uint8_t ANGLE_VALUE2 = 0;

  // ANGLE_VALUE1 |= ((C & 0x01) << 5);
  // ANGLE_VALUE1 |= ((D & 0x01) << 4);
  ANGLE_VALUE1 |= ((E & 0x01) << 3);
  ANGLE_VALUE1 |= ((F & 0x01) << 2);
  ANGLE_VALUE1 |= ((G & 0x01) << 1);
  ANGLE_VALUE1 |= ((H & 0x01) << 0);

  ANGLE_VALUE2 |= ((I & 0x01) << 7);
  ANGLE_VALUE2 |= ((J & 0x01) << 6);
  ANGLE_VALUE2 |= ((K & 0x01) << 5);
  ANGLE_VALUE2 |= ((L & 0x01) << 4);
  ANGLE_VALUE2 |= ((M & 0x01) << 3);
  ANGLE_VALUE2 |= ((N & 0x01) << 2);
  ANGLE_VALUE2 |= ((O & 0x01) << 1);
  ANGLE_VALUE2 |= ((P & 0x01) << 0);

  portb |= (1 << 2);  // CLOCK ENDS WITH LOGIC HIGH
  
  uint16_t value = (ANGLE_VALUE1 << 8) | ANGLE_VALUE2;
  
  return value;
}

void SPI_WRITE_DAC(uint8_t SPI_SETTING, uint16_t SPI_BYTE) {
  uint8_t A = 0, B = 0, C = 0, D = 0, E = 0, F = 0, G = 0, H = 0, I = 0, J = 0, K = 0, L = 0, M = 0, N = 0, O = 0, P = 0;

  A = 0x28;
  B = 0x2C;
  C = 0x28;
  D = 0x2C;
  E = 0x28 | ((SPI_SETTING >> 4) & 0x02);
  F = E | 0x04;
  G = 0x28 | ((SPI_SETTING >> 3) & 0x02);
  H = G | 0x04;
  I = 0x28;
  J = 0x2C;
  K = 0x28;
  L = 0x2C;
  M = 0x28;
  N = 0x2C;
  O = 0x28 | ((SPI_SETTING << 1) & 0x02);
  P = O | 0x04;

  portb &= ~(1 << 4);

  portb = A;
  portb = B;
  portb = C;
  portb = D;
  portb = E;
  portb = F;
  portb = G;
  portb = H;
  portb = I;
  portb = J;
  portb = K;
  portb = L;
  portb = M;
  portb = N;
  portb = O;
  portb = P;
  portb = 0x2E;

  A = 0x28 | ((SPI_BYTE >> 14) & 0x02);
  B = A | 0x04;
  C = 0x28 | ((SPI_BYTE >> 13) & 0x02);
  D = C | 0x04;
  E = 0x28 | ((SPI_BYTE >> 12) & 0x02);
  F = E | 0x04;
  G = 0x28 | ((SPI_BYTE >> 11) & 0x02);
  H = G | 0x04;
  I = 0x28 | ((SPI_BYTE >> 10) & 0x02);
  J = I | 0x04;
  K = 0x28 | ((SPI_BYTE >> 9) & 0x02);
  L = K | 0x04;
  M = 0x28 | ((SPI_BYTE >> 8) & 0x02);
  N = M | 0x04;
  O = 0x28 | ((SPI_BYTE >> 7) & 0x02);
  P = O | 0x04;

  portb = A;
  portb = B;
  portb = C;
  portb = D;
  portb = E;
  portb = F;
  portb = G;
  portb = H;
  portb = I;
  portb = J;
  portb = K;
  portb = L;
  portb = M;
  portb = N;
  portb = O;
  portb = P;
  portb = 0x2E;

  A = 0x28 | ((SPI_BYTE >> 6) & 0x02);
  B = A | 0x04;
  C = 0x28 | ((SPI_BYTE >> 5) & 0x02);
  D = C | 0x04;
  E = 0x28 | ((SPI_BYTE >> 4) & 0x02);
  F = E | 0x04;
  G = 0x28 | ((SPI_BYTE >> 3) & 0x02);
  H = G | 0x04;
  I = 0x28;
  J = 0x2C;
  K = 0x28;
  L = 0x2C;
  M = 0x28 | ((SPI_BYTE >> 0) & 0x02);
  N = M | 0x04;
  O = 0x28 | ((SPI_BYTE << 1) & 0x02);
  P = O | 0x04;

  portb = A;
  portb = B;
  portb = C;
  portb = D;
  portb = E;
  portb = F;
  portb = G;
  portb = H;
  portb = I;
  portb = J;
  portb = K;
  portb = L;
  portb = M;
  portb = N;
  portb = O;
  portb = P;
  portb = 0x2E;

  portb |= (1 << 4);
}

void EEPROM_WRITE(uint8_t Address, uint8_t Data) {
  /* Wait for completion of previous write */
  while (eecr & (1 << 1))
    ;
  /* Set up address and Data Registers */
  eearh = 0x00;
  eearl = Address;
  eedr = Data;
  /* Write logical one to EEMPE */
  eecr |= (1 << 2);
  /* Start eeprom write by setting EEPE */
  eecr |= (1 << 1);
}
uint8_t EEPROM_READ(uint8_t Address) {
  /* Wait for completion of previous write */
  while (eecr & (1 << 1))
    ;
  /* Set up address register */
  eearh = 0x00;
  eearl = Address;
  /* Start eeprom read by writing EERE */
  eecr |= (1 << 0);
  /* Return data from Data Register */
  return eedr;
}


int main() {

  ddrb = 0B00111110;
  portb = 0B00111110;
  //  portb |= (1 << 1); // DATA OUT PIN
  //  portb |= (1 << 2); // CLOCK PIN
  //  portb |= (1 << 4); // CHIP SELECT PIN DAC
  //  portb |= (1 << 3); // CHIP SELECT PIN AS5047D

//  SPI_WRITE_DAC(0x30, 0x0003);  // LDAC PIN DISABLED


  sreg &= ~(1 << 7);

  if ((EEPROM_READ(0x01)) != 0x03) {
    if (EEPROM_READ(0x01) != 0x01 && EEPROM_READ(0x01) != 0x02) {
      EEPROM_WRITE(0x01, 0x00);
    }
    if ((EEPROM_READ(0x01)) != 0x03) {
      _delay_ms(1000);

      uint16_t ANGLE1 = 0;

      ANGLE1 = SPI_READ_AS5147D();

      for (uint8_t i = 0; i < 20; ++i) {
        ANGLE1 = ((ANGLE1 + SPI_READ_AS5147D()) / 2);
        _delay_us(2000);
      }

      ANGLE1 = 0;
      for (uint8_t i = 0; i < 100; ++i) {
        ANGLE1 = ((ANGLE1 + SPI_READ_AS5147D()) / 2);
        _delay_us(2000);
      }

      ANGLE1 &= 0x0FFF;

      if (ANGLE1 > ZERO) {
        uint16_t CONSTANT = (4095 - ANGLE1 + ZERO);
        EEPROM_WRITE(0x02, (CONSTANT >> 8));
        EEPROM_WRITE(0x03, CONSTANT);
      }

      if (ANGLE1 < ZERO) {
        uint16_t CONSTANT = ZERO - ANGLE1;
        EEPROM_WRITE(0x02, (CONSTANT >> 8));
        EEPROM_WRITE(0x03, CONSTANT);
      }

      if (ANGLE1 == ZERO) {
        uint16_t CONSTANT = ZERO;
        EEPROM_WRITE(0x02, (CONSTANT >> 8));
        EEPROM_WRITE(0x03, CONSTANT);
      }
      EEPROM_WRITE(0x01, (0x01 + EEPROM_READ(0x01)));
    }
  }
  uint16_t ANGLE = 0;

  const uint16_t OFFSET = (EEPROM_READ(0x02) << 8) | EEPROM_READ(0x03);

//  const uint16_t OFFSET = 4095;

  while (1) {
    ANGLE = SPI_READ_AS5147D();
    ANGLE += OFFSET;
    ANGLE &= 0x0FFF;
    ANGLE <<= 4;

    SPI_WRITE_DAC(0x00, ANGLE);
    
    ANGLE >>= 4;
    ANGLE = 4095 - ANGLE;
    ANGLE <<= 4;

    SPI_WRITE_DAC(0x01, ANGLE); // With mosfet to LDAC
  }
}

