#include "Driver_USART.h"
#include "PIN_LPC17xx.h"
#include "LPC17xx.h"
#include "GPIO_LPC17xx.h"
#include "Open1768_LCD.h"
#include "LCD_ILI9325.h"
#include "asciiLib.h"
#include "drawings.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

// ------------------------------------------
// Definitions and constants for DS18B20 sensors
// ------------------------------------------
// The DS18B20 scratchpad size is 9 bytes.
// The bytes include, among other things, the measured temperature, alarm thresholds, configuration, and CRC.
#define DS18B20_SCRATCHPAD_SIZE 9

// The ROM code size is 8 bytes: the unique 64-bit address of each sensor.
#define DS18B20_ROM_CODE_SIZE 8

// DS18B20 Commands:
#define DS18B20_SKIP_ROM 0xCC        // SKIP_ROM: bypasses addressing a specific sensor
#define DS18B20_CONVERT_T 0x44       // CONVERT_T: starts temperature conversion
#define DS18B20_READ_SCRATCHPAD 0xBE // READ_SCRATCHPAD: reads the 9-byte scratchpad
#define DS18B20_READ_ROM 0x33        // READ_ROM: reads the sensor's ROM address (if only one sensor is on the bus)
#define DS18B20_MATCH_ROM 0x55       // MATCH_ROM: addresses a specific device on the bus

// ------------------------------------------
// Global variables
// ------------------------------------------

// myTick is incremented in the SysTick interrupt; it is used for implementing microsecond delays.
volatile uint32_t myTick;

// Array for storing the ROM addresses of multiple sensors (max. 50 units)
uint8_t sensors[50][DS18B20_ROM_CODE_SIZE];
int num_sensors = 0; // Number of registered sensors

// Array for storing information about whether a sensor is connected
int sensorActive[50] = {0};

// ------------------------------------------
// Helper functions for UART
// ------------------------------------------

/**
 * @brief uart_getchar
 * Reads one character from UART0.
 * Waits until a byte of data is available in the receiver register.
 */
int uart_getchar(void)
{
    while (!(LPC_UART0->LSR & 0x01));
    return LPC_UART0->RBR;
}

/**
 * @brief send_string
 * Sends text via UART0 character by character.
 * Waits until the transmitter register is ready before sending the next character.
 */
void send_string(const char *string)
{
    for (int i = 0; i < (int)strlen(string); i++)
    {
        while (!(LPC_UART0->LSR & (1 << 5)));
        LPC_UART0->THR = string[i];
    }
}

/**
 * @brief wait_for_enter
 * Waits until the user presses ENTER (CR or LF) on the UART terminal.
 * A helper function for user interaction.
 */
void wait_for_enter()
{
    send_string("Press ENTER to continue...\n\r");
    while (1)
    {
        int c = uart_getchar();
        if (c == '\r' || c == '\n')
            break;
    }
}

// ------------------------------------------
// SysTick handling functions (for delays)
// ------------------------------------------

/**
 * @brief SysTick_Handler
 * Function called in the SysTick interrupt every 1 µs (configured in main).
 * Increments the variable myTick.
 */
void SysTick_Handler(void)
{
    myTick++;
}

/**
 * @brief delay
 * Implements a delay of t microseconds.
 * Waits until myTick reaches the value t.
 */
void delay(uint32_t t)
{
    myTick = 0;
    while (myTick < t);
}

// ------------------------------------------
// Low-level functions for handling the 1-Wire bus
// ------------------------------------------

/**
 * @brief wire_reset
 * Performs a reset of the 1-Wire bus.
 * Procedure:
 * - Hold the line low for ~480 µs,
 * - Release the line, wait ~70 µs,
 * - Read the line state - if a sensor is present, it will pull the line to 0.
 * Returns 1 if a sensor presence is detected; otherwise 0.
 */
int wire_reset(void)
{
    GPIO_PinWrite(0, 0, 0);
    delay(480);
    GPIO_PinWrite(0, 0, 1);
    delay(70);
    int rc = GPIO_PinRead(0, 0);
    delay(410);
    return (rc == 0) ? 1 : 0;
}

/**
 * @brief write_bit
 * Sends a single bit to the sensor:
 * - A '1' is sent by briefly pulling the line to 0
 * - A '0' is sent by holding the line low for a longer period
 * The timing is determined by the 1-Wire specification.
 */
void write_bit(int value)
{
    if (value)
    {
        GPIO_PinWrite(0, 0, 0);
        delay(6);
        GPIO_PinWrite(0, 0, 1);
        delay(64);
    }
    else
    {
        GPIO_PinWrite(0, 0, 0);
        delay(60);
        GPIO_PinWrite(0, 0, 1);
        delay(10);
    }
}

/**
 * @brief read_bit
 * Reads a single bit from the 1-Wire bus:
 * - The line is briefly pulled to 0 and then released
 * - After ~9 µs, the line state is checked
 * If the sensor transmits '1', the line will be high; if '0', low.
 */
int read_bit(void)
{
    int rc;
    GPIO_PinWrite(0, 0, 0);
    delay(6);
    GPIO_PinWrite(0, 0, 1);
    delay(9);
    rc = GPIO_PinRead(0, 0);
    delay(55);
    return rc;
}

/**
 * @brief wire_write
 * Sends a byte over the 1-Wire bus bit by bit, starting with the least significant bit.
 */
void wire_write(uint8_t byte)
{
    for (int i = 0; i < 8; i++)
    {
        write_bit(byte & 0x01);
        byte >>= 1;
    }
}

/**
 * @brief wire_read
 * Reads a byte from the 1-Wire bus bit by bit, starting with the least significant bit.
 */
uint8_t wire_read(void)
{
    uint8_t value = 0;
    for (int i = 0; i < 8; i++)
    {
        value >>= 1;
        if (read_bit())
            value |= 0x80; // 128 in hexadecimal; alternatively, 2^7 - setting the most significant bit to one (since we read from the least significant bit)
    }
    return value;
}

// ------------------------------------------
// Functions for calculating CRC (CRC-8) for DS18B20 data
// ------------------------------------------

/**
 * @brief byte_crc
 * Calculates the CRC-8 for a single byte according to the DS18B20 algorithm.
 */
uint8_t byte_crc(uint8_t crc, uint8_t byte)
{
    for (int i = 0; i < 8; i++)
    {
        uint8_t b = crc ^ byte;
        crc >>= 1;
        if (b & 0x01)
            crc ^= 0x8c;
        byte >>= 1;
    }
    return crc;
}

/**
 * @brief wire_crc
 * Calculates the CRC-8 for a sequence of bytes.
 * Used to verify the correctness of the data read from the sensor.
 */
uint8_t wire_crc(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++)
        crc = byte_crc(crc, data[i]);
    return crc;
}

// ------------------------------------------
// Higher-level functions for handling DS18B20 sensor addresses and commands
// ------------------------------------------

/**
 * @brief wire_read_address
 * Reads the ROM address (8 bytes) of a single sensor on the bus using the READ_ROM command.
 * Returns 1 if the reading and CRC are correct; otherwise, 0.
 * Works correctly if only one sensor is on the bus.
 */
int wire_read_address(uint8_t *rom_code)
{
    if (wire_reset() != 1)
        return 0;
    wire_write(DS18B20_READ_ROM);
    for (int i = 0; i < DS18B20_ROM_CODE_SIZE; i++)
        rom_code[i] = wire_read();
    uint8_t c = wire_crc(rom_code, 7);
    return (rom_code[7] == c);
}

/**
 * @brief wire_send_cmd
 * Sends a command to the sensor. If rom_code == NULL, uses SKIP_ROM.
 * If we want to communicate with a specific sensor, we would use MATCH_ROM (0x55).
 */
static int wire_send_cmd(const uint8_t *rom_code, uint8_t cmd)
{
    if (wire_reset() != 1)
        return 0;
    if (!rom_code)
    {
        wire_write(DS18B20_SKIP_ROM);
    }
    else
    {
        wire_write(DS18B20_MATCH_ROM);
        for (int i = 0; i < DS18B20_ROM_CODE_SIZE; i++)
            wire_write(rom_code[i]);
    }
    wire_write(cmd);
    return 1;
}

/**
 * @brief wire_start_measure
 * Starts a temperature measurement (CONVERT_T).
 * If rom_code == NULL, uses SKIP_ROM.
 */
int wire_start_measure(const uint8_t *rom_code)
{
    return wire_send_cmd(rom_code, DS18B20_CONVERT_T);
}

/**
 * @brief wire_read_scratchpad
 * Reads the scratchpad (9 bytes) from the sensor and checks the CRC.
 * Returns 1 if the reading is correct; otherwise, 0.
 */
static int wire_read_scratchpad(const uint8_t *rom_code, uint8_t *scratchpad)
{
    if (wire_send_cmd(rom_code, DS18B20_READ_SCRATCHPAD) != 1)
        return 0;
    for (int i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++)
        scratchpad[i] = wire_read();
    uint8_t c = wire_crc(scratchpad, 8);
    return (scratchpad[8] == c);
}

/**
 * @brief wire_get_temp
 * Reads the temperature from the sensor (after a prior wire_start_measure and waiting ~750 ms).
 * Returns the temperature in Celsius as a floating point number.
 * In case of an error, returns 85.
 */
float wire_get_temp(const uint8_t *rom_code)
{
    uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE];
    int16_t temp;
    if (!wire_read_scratchpad(rom_code, scratchpad))
        return 85.0f;
    memcpy(&temp, &scratchpad[0], sizeof(temp)); //pierwsze 2 bajty to temperatura
    return temp / 16.0f;
}

// ------------------------------------------
// Function for registering a new sensor
// ------------------------------------------

/**
 * @brief add_sensor
 * Prompts the user to connect ONE new sensor and press ENTER,
 * then reads its address and stores it in the sensors array.
 * Returns 1 if the addition was successful; otherwise, 0.
 */
int add_sensor()
{
    if (num_sensors >= 50)
    {
        send_string("Sensor limit reached!\n\r");
        return 0;
    }
    send_string("Connect ONE new sensor and press ENTER.\n\r");
    wait_for_enter();
    send_string("Reading new sensor's address...\n\r");
    uint8_t rom[8];
    if (wire_read_address(rom))
    {
        memcpy(sensors[num_sensors], rom, 8);
        num_sensors++;
		sensorActive[num_sensors - 1] = 1;
        send_string("Added sensor with address: ");
        for (int i = 0; i < 8; i++)
        {
            char buf[4];
            sprintf(buf, "%02X", rom[i]);
            send_string(buf);
        }
        send_string("\n\r");
        return 1;
    }
    else
    {
        send_string("No sensor detected or address read error!\n\r");
        return 0;
    }
}

int checkIfSensorPresent(const uint8_t* rom_code)
{
    // Reset the bus
    if (wire_reset() != 1)
    return 0;  // No device on the bus, or an error occurred

    // MATCH_ROM - targeting a specific sensor
    wire_write(DS18B20_MATCH_ROM);
    for (int i = 0; i < DS18B20_ROM_CODE_SIZE; i++)
        wire_write(rom_code[i]);

    // Send the READ_SCRATCHPAD command
    wire_write(DS18B20_READ_SCRATCHPAD);

    // Read 9 bytes of the scratchpad
    uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE];
    for (int i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++)
        scratchpad[i] = wire_read();

    // Check the CRC
    uint8_t c = wire_crc(scratchpad, 8);
    if (scratchpad[8] != c) {
        return 0; // CRC error -> sensor is not responding correctly
    }

    // Scratchpad returned a correct CRC, sensor is present
    return 1;
}

// Helper function to read the state of the button
// KEY1 is on pin P2.11
int read_key1(void)
{
    // Invert logic if active low or high
    // e.g., if pressed = 0:
    return (LPC_GPIO2->FIOPIN & (1 << 11)) == 0;
}

// ------------------------------------------
// main function - the entry point of the program
// ------------------------------------------

int main()
{
     // Configure SysTick - one tick per 1 µs
    SysTick_Config(SystemCoreClock / 1000000);

    // Configure UART0
    PIN_Configure(0, 2, 1, 2, 0);
    PIN_Configure(0, 3, 1, 0, 0);

    LPC_SC->PCONP |= (1 << 3);

    LPC_UART0->LCR = 3 | (1 << 7);
    LPC_UART0->DLL = 163;
    LPC_UART0->DLM = 0;
    LPC_UART0->LCR = 3;

    // Configure the pin for 1-Wire (P0.0)
    PIN_Configure(0, 0, 0, 0, 1);

    // Initialize the LCD
    lcdConfiguration();
    init_ILI9325();

    send_string("MULTIPLE DS18B20 SENSOR REGISTRATION\n\r");

    // Register sensors at startup
    while (1)
    {
        send_string("Do you want to add a new sensor? (Y/N)\n\r");
        int c = uart_getchar();
        if (c == 'Y' || c == 'y')
        {
            if (!add_sensor())
            {
                send_string("Failed to add sensor. Please try again.\n\r");
            }
        }
        else
        {
            break;
        }
    }

    if (num_sensors == 0)
    {
        send_string("No sensors added. Exiting.\n\r");
        while (1);
    }

    send_string("Sensors registered. Now connect ALL registered sensors to the bus.\n\r");
    wait_for_enter();

    send_string("I will read the temperature from each sensor in sequence.\n\r");

    ColorBackground(LCDBlueSea);
    DrawScale();
    int colours[] = {LCDCyan, LCDYellow, LCDGreen, LCDRed, LCDBlack};

    int current_sensor = 0;

    int prev_key_state = 0;
    int current_key_state = 0;

    int iter_per_sensor[50] = {0}; // Individual iteration index for each sensor

    while (1)
    {
    // Check the next sensor from the list
    if (sensorActive[current_sensor] == 1)
    {
        if (wire_start_measure(sensors[current_sensor]))
        {
            delay(750000);
            float temp = wire_get_temp(sensors[current_sensor]);
            // Treat 85.0 as an error
            if (fabsf(temp - 85.0f) < 0.0001f)
            {
                // Probably a CRC error or sensor missing
                send_string("Sensor disconnected! Stopping its operation.\n\r");
                sensorActive[current_sensor] = 0;
            }
            else
            {
                int temp_int = (int) temp;
                int temp_frac =  (int)((temp - temp_int) * 10000);

                char str[64];
                sprintf(str, "Sensor %d (address: ", current_sensor);
                send_string(str);
                for (int i = 0; i < 8; i++)
                {
                    char buf[4];
                    sprintf(buf, "%02X", sensors[current_sensor][i]);
                    send_string(buf);
                }
                sprintf(str, "): %d.%04d C\n\r", temp_int, temp_frac);
                send_string(str);

                int color_for_sensor = colours[current_sensor % (sizeof(colours) / sizeof(colours[0]))];
                display_graph(temp, iter_per_sensor[current_sensor], color_for_sensor);

                iter_per_sensor[current_sensor] += 3;
                if (iter_per_sensor[current_sensor] > 300)
                {
                    iter_per_sensor[current_sensor] = 0;
                    ColorBackground(LCDBlueSea);
                    DrawScale();
                }
            }
        }
        else
        {
            send_string("Sensor disconnected (start_measure FAIL). Stopping its operation.\n\r");
            sensorActive[current_sensor] = 0;
        }
    }
    else
    {
        // Sensor is inactive (disconnected) – checking if it has returned
        if (checkIfSensorPresent(sensors[current_sensor]))
        {
            // Sensor detected again
            send_string("Sensor has returned! Resuming service.\n\r");
            sensorActive[current_sensor] = 1;
        }
    }

    // Move to the next sensor
    current_sensor = (current_sensor + 1) % num_sensors;

        // Debouncing the KEY1 button
        if (read_key1())
        {
            delay(20000);
            if (read_key1())
            {
                current_key_state = 1;
            }
            else
            {
                current_key_state = 0;
            } 
        }
        else
        {
            current_key_state = 0;
        }

        // Detection of a rising edge of the button (0 -> 1)
        if (current_key_state == 1 && prev_key_state == 0)
        {
            // Button press detected
            send_string("KEY1 pressed, release the button...\n\r");

            // Wait for the user to release the button
            while (read_key1())
            {
                delay(50000);
            }

            send_string("KEY1 released.\n\r");
            
            // After pressing ENTER, add a new sensor
            if (!add_sensor())
            {
                send_string("Failed to add sensor. Please try again.\n\r");
            }
            else
            {
                send_string("New sensor added!\n\r");
                send_string("Now connect all the sensors!\n\r");
				wait_for_enter();
			}
        }

        prev_key_state = current_key_state;

        delay(1000000); // 1 second before the next measurement
    }
    
}