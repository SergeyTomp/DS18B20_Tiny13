#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// блок конфигурации условной компиляции
#define CA                          // тип LED-индикатора: при общем аноде - CA, при общем катоде - CC
#define MSBFIRST                    // порядок подачи на регистр сдвига: MSBFIRST - старшим битом вперёд, LSBFIRST - младшим битом вперёд
#define DRV_HIGH                    // вариант уровня управления включением сегмента контроллером: DRV_LOW - низний, DRV_HIGH - высокий

// блок конфигурации контроллера
#define ONEWIRE_PORTReg     PORTB   // порт работы с датчиком
#define ONEWIRE_DDRReg      DDRB    // регистр режима порта работы с датчиком
#define ONEWIRE_PINReg      PINB    // регистр чтения данных из датчика
#define VIZ_PORT            PORTB   // порт работы с дисплеем
#define VIZ_PORT_DDR        DDRB    // регистр режима порта работы с дисплеем
#define ONEWIRE_PIN         PB0     // пин МК, который соединен с линией данных датчика
#define ST_PIN              PB1     // пин МК, который соединен с 12 пином (ST) сдвигового регистра
#define DS_PIN              PB2     // пин MK, который соединен с 14 пином (DS) сдвигового регистра
#define SH_PIN              PB4     // пин МК, который соединен с 11 пином (SH) сдвигового регистра

// блок макросов
#define OneWire_setPinAsOutput  ONEWIRE_DDRReg |= (1<<ONEWIRE_PIN)                  // Set OneWire pin in Output mode
#define OneWire_setPinAsInput   ONEWIRE_DDRReg &= ~(1<<ONEWIRE_PIN)                 // Set OneWire pin in Input mode
#define OneWire_writePinLOW     ONEWIRE_PORTReg &=  ~(1<<ONEWIRE_PIN)               // Set LOW level on OneWire pin
#define OneWire_writePinHIGH    ONEWIRE_PORTReg |= (1<<ONEWIRE_PIN)                 // Set HIGH level on OneWire pin
#define OneWire_readPin         (( ONEWIRE_PINReg & (1<<ONEWIRE_PIN)) ? 1 : 0 )     // Read level from OneWire pin

volatile uint8_t ACTION = 1;        // флаг требования сеанса обмена с датчиком
volatile uint8_t COUNTER = 0;       // счётчик для активации прерываний

#if defined CA
#warning "LED display type is CA"
// массив символов для индикатора с ОА
uint8_t digitsArr[13] = {
        0b11000000, //0
        0b11111001, //1
        0b10100100, //2
        0b10110000, //3
        0b10011001, //4
        0b10010010, //5
        0b10000010, //6
        0b11111000, //7
        0b10000000, //8
        0b10010000, //9
        0b10111111, //-
        0b01111111, //.
        0b11111111  //off

};
#elif defined CC
#warning "LED display type is CC"
// массив символов для индикатора с ОК
uint8_t digitsArr[13] = {
        0b00111111, //0
        0b00000110, //1
        0b01011011, //2
        0b01001111, //3
        0b01100110, //4
        0b01101101, //5
        0b01111101, //6
        0b00000111, //7
        0b01111111, //8
        0b01101111, //9
        0b01000000, //-
        0b10000000,	//.
        0b00000000 	//off

};
#else
#error "LED display type not defined"
#endif

// command – битовое представление передаваемой цифры
inline static void shiftOut(uint8_t command) {
    VIZ_PORT = VIZ_PORT & ~_BV(SH_PIN); // SH-вход в низкий уровень из любого текущего, иначе первый бит может не записаться
    for (int i = 0; i < 8; i++) {
#if defined MSBFIRST	
#warning "Shift register byte load direction is MSBFIRST"
        VIZ_PORT = (command & 0b10000000) ? VIZ_PORT | _BV(DS_PIN) : VIZ_PORT & ~_BV(DS_PIN);
        command = command << 1;
#elif defined LSBFIRST	
#warning "Shift register byte load direction is LSBFIRST"
        VIZ_PORT = (command & 0b00000001) ? VIZ_PORT | _BV(DS_PIN) : VIZ_PORT & ~_BV(DS_PIN);
        command = command >> 1;
#else
#error "Shift register byte load direction not defined"
#endif
        VIZ_PORT = VIZ_PORT | _BV(SH_PIN);
        _delay_us(10);
        VIZ_PORT = VIZ_PORT & ~_BV(SH_PIN);
    }
}

inline static void showDigits(uint8_t pos, uint8_t digit) {
    VIZ_PORT = VIZ_PORT & ~_BV(ST_PIN);
    if (pos == 2) {    // если второй символ светодиодного индикатора, то нужно включить у него ещё и точку
#if defined CA
        shiftOut(digitsArr[digit] & digitsArr[11]);
#elif defined CC
        shiftOut( digitsArr[digit] | digitsArr[11] );
#else
#error "LED display type not defined to show decimal dot"
#endif
    } else {
        shiftOut(digitsArr[digit]);
    }
    VIZ_PORT = VIZ_PORT | _BV(ST_PIN);

    // гасим все секции (управляющие выводы могут быть в любом состоянии на этом шаге) и зажигаем нужную
#if defined DRV_LOW
#warning "LED segment is switched on by LOW level of controller pin"
    VIZ_PORT = VIZ_PORT | 0b00011110;
    VIZ_PORT = VIZ_PORT & ~_BV(pos);
#elif defined DRV_HIGH
#warning "LED segment is switched on by HIGH level of controller pin"
    VIZ_PORT = VIZ_PORT & 0b11100001;
    VIZ_PORT = VIZ_PORT | _BV(pos);
#else
#error "LED display switching level is not defined"
#endif
}

// Calculate CRC-8
//uint8_t crc8(const uint8_t * addr, uint8_t len){
//    uint8_t crc = 0;
//    while (len--) {
//        uint8_t inbyte = *addr++;
//        for (uint8_t i = 8; i; i--) {
//            uint8_t mix = (crc ^ inbyte) & 0x01;
//            crc >>= 1;
//            if (mix) crc ^= 0x8C;
//            inbyte >>= 1;
//        }
//    }
//    return crc;
//}

// Reset function
inline static uint8_t OneWire_reset(void) {

    // - Wait for line
    uint8_t Retries = 125;
    OneWire_setPinAsInput;
    do {
        if (--Retries == 0) return 0;
        _delay_us(2);
    } while (!OneWire_readPin);

    // - Drop line
    OneWire_writePinLOW;
    OneWire_setPinAsOutput;
    _delay_us(480);

    // - Listen for reply pulse
    OneWire_setPinAsInput;
    _delay_us(70);

    // - Read line state
    uint8_t State = !OneWire_readPin;
    _delay_us(410);
    return State;
}

// Write single bit
inline static void OneWire_writeBit(uint8_t Bit) {
    if (Bit & 1) {
        // Drop line
        OneWire_writePinLOW;
        OneWire_setPinAsOutput;
        // Write Bit-1
        _delay_us(10);
        OneWire_writePinHIGH;
        _delay_us(55);
    } else {
        // Drop line
        OneWire_writePinLOW;
        OneWire_setPinAsOutput;
        // Write Bit-0
        _delay_us(65);
        OneWire_writePinHIGH;
        _delay_us(5);
    }
}

// Read single bit
inline static uint8_t OneWire_readBit(void) {
    // Drop line
    OneWire_setPinAsOutput;
    OneWire_writePinLOW;
    _delay_us(3);

    // Wait for data
    OneWire_setPinAsInput;
    _delay_us(10);

    // Read bit into byte
    uint8_t Bit = OneWire_readPin;
    _delay_us(53);
    return Bit;
}

// Write byte
inline static void OneWire_writeByte(uint8_t byte) {

    // - Write each bit
    for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1) {
        OneWire_writeBit((bitMask & byte) ? 1 : 0);
    }
}

// Read byte
inline static uint8_t OneWire_readByte(void) {
    uint8_t byte = 0;

    // - Read all bits
    for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1) {
        // - Read & store bit into byte
        if (OneWire_readBit()) byte |= bitMask;
    }
    return byte;
}

// Read buffer
inline static void OneWire_read(uint8_t *buffer, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        buffer[i] = OneWire_readByte();
    }
}

// Write buffer
inline static void OneWire_write(uint8_t *buffer, uint8_t Size) {
    for (uint8_t i = 0; i < Size; i++) {
        OneWire_writeByte(buffer[i]);
    }
}

// STATE change
ISR(TIM0_OVF_vect) {

    if (!ACTION) {
        COUNTER++;
        if (COUNTER == 16) {
            ACTION = 1;
            COUNTER = 0;
        }
    }
}

int main(void) {

    int16_t tmp;            // временная переменная
    int8_t tempDigits[4];   // массив цифр температуры
    int8_t REQUESTED = 0;   // состояние сеанса обмена с датчиком: 0 - запроса конвертации не было, 1 - запрос сделан, нужен ответ
    uint8_t ROM[9];         // буфер чтения ответа датчика

    VIZ_PORT_DDR |= 0b00011110;

    TIMSK0 = 0b00000010;    // разрешаем прерывания по переполнению счётчика
    TCCR0B = 0b00000101;    // делитель для частоты счётчика 1024
    // вариант устаноки 0 во всех сегментах при включении, занимает 10 байт
    // memset(tempDigits, 0, sizeof(tempDigits));
    // вариант устаноки 8 во всех сегментах при включении, занимает 10 байт
    for (int8_t i = 0; i < 4; i++) {
        tempDigits[i] = 8;
    }
    sei();                  // разрешаем преывания глобально

    while (1) {

        if (ACTION) {
            switch (REQUESTED) {

                case 0:
                    // Start conversion
                    OneWire_reset();
                    OneWire_writeByte(0xCC);
                    OneWire_writeByte(0x44);
                    REQUESTED = 1;
                    break;

                case 1:
                    // Read ROM
                    OneWire_reset();
                    OneWire_writeByte(0xCC);
                    OneWire_writeByte(0xBE);
                    OneWire_read(ROM, sizeof(ROM));

                    //Check ROM CRC
//				    if( crc8( ROM, 8 ) != ROM[8] ) {
//					    REQUESTED = 0;
//					    break;
//				    }

                    tmp = 0;
                    tmp = ((tmp | ROM[1]) << 8) | ROM[0];   // получаем грязное значение регистров температуры в правильном порядке

                    if (ROM[1] & 0b1000000) {               // если в буфере отрицательное число, переводим из дополнительного кода в прямой и включаем "-"
                        tmp = (~tmp + 1);
                        tempDigits[3] = 10;
                    } else {
                        tempDigits[3] = 12;
                    }

                    // получаем число десятых долей умножением 4-х десятичных битов на 1/16 и на 10 (т.е. 5/8: (1/16 * 10 = 5/8))
                    tempDigits[0] = ((((tmp & 0x0F) << 2) + (tmp & 0x0F)) >> 3);
                    tmp = (tmp >> 4) & 0b01111111;          // дальше работаем только с целой частью.

                    tempDigits[2] = 0;
                    if (tmp < 10) {
                        tempDigits[1] = tmp;
                        tempDigits[2] = 12;
                    } else {                                // остаток от деления на 10 получаем прибавлением -10 в дополнительном коде и сбросом переполнения
                        while (tmp >= 10) {
                            tmp = (tmp + 0b11110110) & 0xFF;
                            tempDigits[2] += 1;
                        }
                        tempDigits[1] = tmp;

                        if (tempDigits[2] > 9) {            // если >= 100, в первой цифре будет 10...12, выделяем второй знак через -10, вместо "-" ставим 1
                            tempDigits[2] += 0b11110110;    // прибавление -10 в дополнительном коде
                            tempDigits[3] = 1;
                        }
                    };

                    REQUESTED = 0;
                    break;
            }
            ACTION = 0;
        }

        // вывод на дисплей
        for (uint8_t i = 1; i <= 4; i++) {
            showDigits(i, tempDigits[i - 1]);
            _delay_ms(5);
        }
    }

}