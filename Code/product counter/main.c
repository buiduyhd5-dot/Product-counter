/* Product Counter - C Code for ATmega16
 * Ma nguon duoc viet theo so do mach va yeu cau chuc nang.
 * Chu y:
 *  - Tat ca cac nut bam (TANG, GIAM, SETUP, START, STOP) va Sensor la Active LOW
 *    (duoc noi GND khi nhan, su dung dien tro keo len).
 *  - Motor duoc dieu khien bang PNP Transistor (Active LOW).
 */

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>



// ==================== PIN MAPPING (Theo so do mach) ====================

// Inputs (PC0, PB0-PB4) - Active LOW   <<< ??i t? PA0 ? PC0
#define SENSOR_PIN PINC
#define SENSOR_DDR DDRC
#define SENSOR_PORT PORTC
#define SENSOR_BIT PC0

#define BTN_PIN PINB
#define BTN_PORT PORTB
#define BTN_DDR DDRB
#define BTN_TANG PB0
#define BTN_GIAM PB1
#define BTN_SETUP PB2
#define BTN_START PB3
#define BTN_STOP PB4

// LED7 segment PA0-PA7   <<< ??i t? PC0-PC7 ? PA0-PA7
#define SEG_DDR DDRA
#define SEG_PORT PORTA

// Digit control PD1-PD4 (Theo so do: D1, D2, D3, D4)
#define DIGIT_DDR DDRD
#define DIGIT_PORT PORTD
#define S1_PIN PD0 // Hang chuc Setup
#define S2_PIN PD1 // Don vi Setup
#define C1_PIN PD2 // Hang chuc Dem
#define C2_PIN PD3 // Don vi Dem
#define DIGIT_MASK ((1 << S1_PIN) | (1 << S2_PIN) | (1 << C1_PIN) | (1 << C2_PIN))
const uint8_t DIGIT_PINS[4] = {S1_PIN, S2_PIN, C1_PIN, C2_PIN};

// Motor (PD5 - PNP Driver, Active LOW) & LED Red (PD6 - Active HIGH)
#define MOTOR_DDR DDRD
#define MOTOR_PORT PORTD
#define MOTOR_BIT PD5

#define LEDR_DDR DDRD
#define LEDR_PORT PORTD
#define LEDR_BIT PD6

// ==================== GLOBAL VARIABLES ====================
volatile uint8_t setup_value = 0;
volatile uint8_t run_count = 0;
volatile bool motor_running = false;
volatile uint8_t sensor_last = 1;
uint8_t display_buf[4] = {0, 0, 0, 0};

// ==================== BANG MA 7-SEG (Common Cathode) ====================
const uint8_t seg_code[10] = {
    ~0b00111111, // 0
    ~0b00000110, // 1
    ~0b01011011, // 2
    ~0b01001111, // 3
    ~0b01100110, // 4
    ~0b01101101, // 5
    ~0b01111101, // 6
    ~0b00000111, // 7
    ~0b01111111, // 8
    ~0b01101111  // 9
};

// ==================== CAP NHAT HIEN THI ====================
void update_display()
{
    cli();
    display_buf[0] = setup_value / 10;
    display_buf[1] = setup_value % 10;
    display_buf[2] = run_count / 10;
    display_buf[3] = run_count % 10;
    sei();
}

void show_digit(uint8_t pos)
{
    if (pos >= 4) return;

    DIGIT_PORT |= DIGIT_MASK;
    SEG_PORT = ~seg_code[display_buf[pos]];
    DIGIT_PORT &= ~(1 << DIGIT_PINS[pos]);
}

// Ngat Timer0 quet LED
ISR(TIMER0_OVF_vect)
{
    static uint8_t pos = 0;
    show_digit(pos);
    pos = (pos + 1) & 0x03;
}

// ==================== MOTOR ====================
void motor_start()
{
    motor_running = true;
    MOTOR_PORT &= ~(1 << MOTOR_BIT);
}

void motor_stop()
{
    motor_running = false;
    MOTOR_PORT |= (1 << MOTOR_BIT);
}

void led_red_on()  { LEDR_PORT |= (1 << LEDR_BIT); }
void led_red_off() { LEDR_PORT &= ~(1 << LEDR_BIT); }

// ==================== INIT ====================
void init_hw()
{
    SENSOR_DDR &= ~(1 << SENSOR_BIT);
    SENSOR_PORT |= (1 << SENSOR_BIT);

    BTN_DDR &= ~((1 << BTN_TANG) | (1 << BTN_GIAM) | (1 << BTN_SETUP) | (1 << BTN_START) | (1 << BTN_STOP));
    BTN_PORT |= ((1 << BTN_TANG) | (1 << BTN_GIAM) | (1 << BTN_SETUP) | (1 << BTN_START) | (1 << BTN_STOP));

    SEG_DDR = 0xFF;
    SEG_PORT = 0x00;

    DIGIT_DDR |= DIGIT_MASK;
    DIGIT_PORT &= ~DIGIT_MASK;

    MOTOR_DDR |= (1 << MOTOR_BIT);
    LEDR_DDR |= (1 << LEDR_BIT);
    motor_stop();
    led_red_off();

    TCCR0 = (1 << CS01);   // prescaler = 8
    TIMSK |= (1 << TOIE0);

    sei();
}

// ==================== DOC NUT (Active LOW) ====================
uint8_t read_button(uint8_t bit)
{
    if (!(BTN_PIN & (1 << bit)))
    {
        _delay_ms(15);
        if (!(BTN_PIN & (1 << bit)))
            return 1;
    }
    return 0;
}

// ==================== MAIN ====================
int main(void)
{
    init_hw();
    update_display();

    while (1)
    {
        // ====== BUTTON T?NG +5 ======
        if (read_button(BTN_TANG))
        {
            if (setup_value <= 94)
                setup_value += 5;
            else
                setup_value = 99;

            update_display();
            while (read_button(BTN_TANG));
        }

        // ====== BUTTON GI?M ======
        if (read_button(BTN_GIAM))
        {
            if (setup_value > 0)
                setup_value--;
            update_display();
            while (read_button(BTN_GIAM));
        }

        // ====== SETUP ======
        if (read_button(BTN_SETUP))
        {
            motor_stop();
            run_count = 0;
            led_red_off();
            update_display();
            while (read_button(BTN_SETUP));
        }

        // ====== START ======
        if (read_button(BTN_START))
        {
            if (setup_value > 0)
            {
                sensor_last = 1;
                update_display();
                led_red_off();
                motor_start();
            }
            else
            {
                led_red_on();
                _delay_ms(50);
                led_red_off();
            }
            while (read_button(BTN_START));
        }

        // ====== STOP ======
        if (read_button(BTN_STOP))
        {
            motor_stop();
            while (read_button(BTN_STOP));
        }
        // ====== SENSOR PROCESS ======
        uint8_t s = (SENSOR_PIN & (1 << SENSOR_BIT)) ? 1 : 0;

        if (motor_running)
        {
	        if (sensor_last == 1 && s == 0)
	        {
		        if (run_count < 99)
		        run_count++;

		        if (run_count >= setup_value)
		        {
			        led_red_on();
			        motor_stop();
		        }

		        update_display();
	        }
	        sensor_last = s;
        }
        else
        {
	        sensor_last = 1;
        }
	}
}
