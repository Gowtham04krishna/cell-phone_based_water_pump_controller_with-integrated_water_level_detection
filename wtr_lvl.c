#include "stm32f4xx.h"  // Device header

void GPIO_Config(void);

void delay_us(int us);

void interrupt_config(void);

void uart_config(void);

void UART_SendString (const char *str);

void UART_SendChar(char ch);

char UART_ReceiveChar(void);

void echo_generate(void);

void TIM2_Config(void);

int distance_cm;

volatile uint32_t start_time = 0;

volatile uint32_t end_time = 0;

int rec;

int check;

char receivedChar;

void relay(int check);

int main(void) 
{
	GPIO_Config();  // Initialize GPIO pins for TRIG and ECHO
	interrupt_config(); //configuring an interrupt for echo pin 
	TIM2_Config(); //timer configuration
	uart_config(); //configuring uart
	
	while (1) 
	{    
		
		//uart_config(); //configuring uart
		echo_generate(); //generating echo
			// Control the LEDs based on distance
			if(distance_cm >= 0 & distance_cm <= 200) 
			{		
					GPIOA->ODR |= (1U << 7);	 // Turn on LED connected to PA2 -> yellow1
					GPIOB->ODR &= ~(1U << 1);	 // Turn off LED connected to PA3 -> yellow2
					GPIOA->ODR &= ~(1U << 4);  // Turn off LED connected to PA4 -> green1
					GPIOA->ODR &= ~(1U << 5);  // Turn off LED connected to PA5 -> green2
					GPIOA->ODR &= ~(1U << 6);  // Turn off LED connected to PA6 -> red
					//GPIOB->MODER |= (1U << 0); //turn on relay module to power the pump  
											
					UART_SendString("LEVEL A\r\n");
				
					if (USART2->SR & USART_SR_RXNE) // Check if data is available in RX buffer
					{
						
						receivedChar = UART_ReceiveChar(); // Read received character
						
						if (receivedChar == 49)
						{
								check = 1;
								relay(check);
						} 
						else if(receivedChar == 48)
						{
								check = 0;
								relay(check);						
						}
					}
			}
			else if(distance_cm > 200 & distance_cm <= 400) 
			{		
					GPIOA->ODR |= (1U << 7);	 // Turn on LED connected to PA2 -> yellow1
					GPIOB->ODR |= (1U << 1);	 // Turn on LED connected to PA3 -> yellow2
					GPIOA->ODR &= ~(1U << 4);  // Turn off LED connected to PA4 -> green1
					GPIOA->ODR &= ~(1U << 5);  // Turn off LED connected to PA5 -> green2
					GPIOA->ODR &= ~(1U << 6);  // Turn off LED connected to PA6 -> red
					//GPIOB->MODER |= (1U << 0); //turn on relay module to power the pump
				
					UART_SendString("LEVEL B\r\n");
				
					if (USART2->SR & USART_SR_RXNE) // Check if data is available in RX buffer
					{
						
						receivedChar = UART_ReceiveChar(); // Read received character
						
						if (receivedChar == 49)
						{
								check = 1;
								relay(check);
						} 
						else if(receivedChar == 48)
						{
								check = 0;
								relay(check);						
						}
					}
			}
			else if(distance_cm > 400 & distance_cm <= 650) 
			{		
					GPIOA->ODR |= (1U << 7);	 // Turn on LED connected to PA2 -> yellow1
					GPIOB->ODR |= (1U << 1);	 // Turn on LED connected to PA3 -> yellow2
					GPIOA->ODR |= (1U << 4);   // Turn on LED connected to PA4 -> green1
					GPIOA->ODR &= ~(1U << 5);  // Turn off LED connected to PA5 -> green2
					GPIOA->ODR &= ~(1U << 6);  // Turn off LED connected to PA6 -> red
					//GPIOB->MODER |= (1U << 0); //turn on relay module to power the pump
					
					UART_SendString("LEVEL C\r\n");
				
					if (USART2->SR & USART_SR_RXNE) // Check if data is available in RX buffer
					{
						
						receivedChar = UART_ReceiveChar(); // Read received character
						
						if (receivedChar == 49)
						{
								check = 1;
								relay(check);
						} 
						else if(receivedChar == 48)
						{
								check = 0;
								relay(check);						
						}
					}
			}
			else if(distance_cm > 650 & distance_cm <= 1000) 
			{		
					GPIOA->ODR |= (1U << 7);	 // Turn on LED connected to PA2 -> yellow1
					GPIOB->ODR |= (1U << 1);	 // Turn on LED connected to PA3 -> yellow2
					GPIOA->ODR |= (1U << 4);   // Turn on LED connected to PA4 -> green1
					GPIOA->ODR |= (1U << 5);   // Turn off LED connected to PA5 -> green2
					GPIOA->ODR &= ~(1U << 6);  // Turn off LED connected to PA6 -> red
					//GPIOB->MODER |= (1U << 0); //turn on relay module to power the pump
				
					UART_SendString("LEVEL D\r\n");
				
					if (USART2->SR & USART_SR_RXNE) // Check if data is available in RX buffer
					{
						
						receivedChar = UART_ReceiveChar(); // Read received character
						
						if (receivedChar == 49)
						{
								check = 1;
								relay(check);
						} 
						else if(receivedChar == 48)
						{
								check = 0;
								relay(check);						
						}
					}
			}
			else if(distance_cm > 1000 & distance_cm <= 1500)
			{		
					GPIOA->ODR |= (1U << 7);	 // Turn on LED connected to PA2 -> yellow1
					GPIOB->ODR |= (1U << 1);	 // Turn on LED connected to PA3 -> yellow2
					GPIOA->ODR |= (1U << 4);  // Turn off LED connected to PA4 -> green1
					GPIOA->ODR |= (1U << 5);  // Turn off LED connected to PA5 -> green2
					GPIOA->ODR |= (1U << 6);  // Turn off LED connected to PA6 -> red
					//GPIOB->MODER |= (1U << 0); //turn on relay module
					
					UART_SendString("LEVEL E\r\n");
				
					if (USART2->SR & USART_SR_RXNE) // Check if data is available in RX buffer
					{
						
						receivedChar = UART_ReceiveChar(); // Read received character
						
						if (receivedChar == 49)
						{
								check = 1;
								relay(check);
						} 
						else if(receivedChar == 48)
						{
								check = 0;
								relay(check);						
						}
					}
			}
			else
			{
					GPIOA->ODR |= (1U << 7);	 // Turn on LED connected to PA2 -> yellow1
					GPIOB->ODR |= (1U << 1);	 // Turn on LED connected to PA3 -> yellow2
					GPIOA->ODR |= (1U << 4);  // Turn off LED connected to PA4 -> green1
					GPIOA->ODR |= (1U << 5);  // Turn off LED connected to PA5 -> green2
					GPIOA->ODR ^= (1U << 6);  // Turn off LED connected to PA6 -> red
					GPIOB->MODER &= ~(3U << 0);      // Clear mode bits for PB0 (00)
					
					UART_SendString("STOP!!\r\n");
					//USART2 -> CR1 = 0;
			}
						
			for (int i = 0; i < 100000; i++);  // Simple delay loop for debounce
	}
}

char UART_ReceiveChar(void)
{
	while (!(USART2->SR & USART_SR_RXNE)){}  // Wait until RX buffer is not empty
  return USART2->DR;
}

void relay(int check)
{
	if (check == 1)
	{
		GPIOB->MODER |= (1U << 0);
	}
	else if(check == 0)
	{
		GPIOB->MODER &= ~(3U << 0);
	}
}

// Function to configure GPIO pins
void GPIO_Config(void) 
{
	// Enable GPIOA,B clock
	RCC->AHB1ENR |= (1U << 0);
	RCC->AHB1ENR |= (1U << 1);

	// Set PA0 (TRIG) as output
	GPIOA->MODER |= (1 << 0);  // Set PA0 as output
	GPIOA->OTYPER &= ~(1 << 0);  // Output push-pull
	GPIOA->OSPEEDR |= (3 << 0);  // Set high speed

	// Set PA1 (ECHO) as input
	GPIOA->MODER |= (0U << 2);  // Set PA1 as input
	GPIOA->MODER |= (0U << 2);

	// Set PA4, PA5, PA6 as output for LEDs
	GPIOA->MODER |= (1 << 14);// Set PA7, PB1, PA4, PA5, PA6 as output
	GPIOB->MODER |= (1 << 2);
	GPIOA->MODER |= (1 << 8);  
	GPIOA->MODER |= (1 << 10);
	GPIOA->MODER |= (1 << 12);
	// Configure PB0 as output (General Purpose Output Mode)
	GPIOB->MODER &= ~(3U << 0);      // Clear mode bits for PB0 (00)
}


//configuring the interrupt pin pa1
void interrupt_config()
{
	RCC ->APB2ENR |= (1U << 14); //mapping the interrupts to the gpio pins
	
	SYSCFG -> EXTICR[0] = 0; //clearing the bits for safety
	SYSCFG -> EXTICR[0] |= (0U << 4); //for pin 1 of port a 
	SYSCFG -> EXTICR[0] |= (0U << 5);
	SYSCFG -> EXTICR[0] |= (0U << 6);
	SYSCFG -> EXTICR[0] |= (0U << 7);
	
	EXTI -> IMR |= (1U << 1); //unmasking the interrupt at pa1
	
	EXTI -> FTSR |= (1U << 1); //selecting the falling edge trigger for pa1
	
	EXTI -> RTSR |= (1U << 1); //selecting the rising edge trigger for pa1
	
	NVIC_EnableIRQ(EXTI1_IRQn); //enabling the nvic
}

//function to configure uart
void uart_config(void)
{
    // Enable clock for GPIOA
    RCC->AHB1ENR |= (1U << 0); // Clock to port A

    // Set PA2 (TX) to Alternate Function
    GPIOA->MODER &= ~(3U << (2 * 2)); // Clear mode bits for PA2
    GPIOA->MODER |= (2U << (2 * 2));  // Set PA2 to Alternate Function mode

    // Set PA3 (RX) to Alternate Function
    GPIOA->MODER &= ~(3U << (2 * 3)); // Clear mode bits for PA3
    GPIOA->MODER |= (2U << (2 * 3));  // Set PA3 to Alternate Function mode

    // Set the alternate function for PA2 and PA3 to AF7 (USART2)
    GPIOA->AFR[0] &= ~(0xF << (4 * 2)); // Clear AF bits for PA2
    GPIOA->AFR[0] |= (7 << (4 * 2));    // Set AF7 for PA2 (TX)

    GPIOA->AFR[0] &= ~(0xF << (4 * 3)); // Clear AF bits for PA3
    GPIOA->AFR[0] |= (7 << (4 * 3));    // Set AF7 for PA3 (RX)

    // Enable clock for USART2
    RCC->APB1ENR |= (1U << 17); // Clock to USART2

    // Set baud rate (assuming 84 MHz clock)
    USART2->BRR = 0x683; // Adjust this value based on your clock settings

    // Enable USART TX and RX
    USART2->CR1 |= (1U << 3); // TX enable
    USART2->CR1 |= (1U << 2); // RX enable

    // Enable USART
    USART2->CR1 |= (1U << 13); // USART enable
}


//send the string for moluculation
void UART_SendString (const char *str)
{
	while (*str) 
	{
		UART_SendChar(*str++);
  }
}

//seding character by character to the tx dr
void UART_SendChar(char ch)
{
	while( !(USART2 -> SR & (1U << 7)) ){} //UART and bluetooth
	USART2 -> DR = ch;
}


// Function to create a delay in microseconds using SysTick
void delay_us(int us) 
{
    // The STM32F401 runs at a core clock of 84 MHz, which gives a clock period of 11.9ns
    // So, 1 us = 84 clock cycles. Adjust for your clock frequency if it's different.
    volatile uint32_t count = 84 * us;  // 1 microsecond delay
	
    while (count--) 
		{
        __NOP();  // No operation (dummy instruction)
    }
}


// Function to measure distance using the ultrasonic sensor
void echo_generate(void) 
{
	// Send a 10us pulse to TRIG pin to start ultrasonic burst
	GPIOA->ODR |= (1 << 0);  // Set PA0 HIGH
	
	delay_us(10);            // Wait for 10 microseconds
	
	GPIOA->ODR &= ~(1 << 0);  // Set PA0 LOW
}



void EXTI1_IRQHandler(void) 
{
	if (EXTI->PR & (1U << 1)) 
	{  // Check if interrupt was caused by PA1
			EXTI->PR |= (1U << 1);  // Clear the pending interrupt flag

			if (GPIOA->IDR & (1 << 1)) {  // Rising edge detected (echo pulse started)
					TIM2->CNT = 0;  // Reset TIM2 counter
					TIM2->CR1 |= TIM_CR1_CEN;  // Start TIM2
			} 
			else 
			{  // Falling edge detected (echo pulse ended)
					TIM2->CR1 &= ~TIM_CR1_CEN;  // Stop TIM2
					end_time = TIM2->CNT;  // Capture end time (pulse duration in microseconds)

					// Convert pulse duration to distance in cm
					distance_cm = (end_time * 0.0343) / 2;  // Speed of sound = 0.0343 cm/us, divide by 2 for round trip
			}
	}
}


//setting the timmer
void TIM2_Config(void) 
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // Enable TIM2 clock

	TIM2->PSC = 84 - 1;  // Prescaler to slow down the clock (84 MHz / 84 = 1 MHz)
	
	TIM2->ARR = 0xFFFFFFFF;  // Set auto-reload to maximum (32-bit timer)
	
	TIM2->CNT = 0;  // Reset counter
	
	TIM2->CR1 |= TIM_CR1_URS;  // Update request source
}
