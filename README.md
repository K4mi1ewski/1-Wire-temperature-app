# DS18B20 Temperature Measurement Application

This application is built on the Open1768 microcontroller and runs in Keil uVision.
It measures temperature using DS18B20 sensors connected via the 1-Wire interface.

## How to Use

1. **Hardware Setup:**
   - **Sensors:** Connect DS18B20 sensors to the 1-Wire bus (pin P0.0). Each sensor must be wired with a data line, power (3.3V), and ground (GND). A 3.7 kΩ pull-up resistor is required on the data line.
   - **Microcontroller Connection:** Connect the Open1768 microcontroller to your computer via UART.
   - **LCD Display:** Ensure an LCD (compatible with Open1768_LCD and LCD_ILI9325 libraries) is connected for visual output.

2. **Flashing the Firmware:**
   - Open the project in Keil uVision.
   - Build the project ensuring all required libraries are included.
   - Flash the firmware to the microcontroller.

3. **Interacting with the Application:**
   - **UART Terminal:** Open a serial terminal application (e.g., MobaXterm) on your computer.
   - **Configure UART:** Set the appropriate COM port (e.g., COM5) and baud rate (9600 bps).
   - **Start Communication:** Reset the microcontroller to start transmitting data via UART.
   - **Sensor Registration:** Follow on-screen prompts in the terminal to register new sensors by connecting them one at a time (disconnect other sensors during registration) and pressing ENTER.
   - **Temperature Readings:** Once sensors are registered and all are connected, the application will cycle through them, measure temperatures, display data on the LCD, and output results to the UART terminal.

## Libraries Used

- **Driver_USART:** UART communication support.
- **PIN_LPC17xx**, **GPIO_LPC17xx**, **LPC17xx:** Libraries specific to the LPC17xx microcontroller.
- **Open1768_LCD** and **LCD_ILI9325:** Libraries for LCD display control.
- **asciiLib:** Library for handling ASCII characters.

## Additional Information

- The application continuously reads temperatures from the sensors, displays them graphically on the LCD, and outputs readings (sensor address and temperature) via UART.
- You can add new sensors on-the-fly by pressing the KEY1 button, which will prompt sensor registration in the terminal.
