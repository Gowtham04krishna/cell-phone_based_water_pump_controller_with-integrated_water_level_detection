#include "stm32f4xx.h"  // Device header
#include <stdio.h>      // Include standard I/O for sprintf, snprintf, etc.

void GPIO_Config(void);
void delay_us(int us);
void interrupt_config(void);
void uart_config(void);
void UART_SendString(const char *str);
void UART_SendChar(char ch);
char UART_ReceiveChar(void);
void echo_generate(void);
void TIM2_Config(void);
void relay(int check);
void MPU_Config(void);
void trigger_usage_fault(void); // Function to intentionally trigger a usage fault
void UART_SendHex(uint32_t value); // Declare UART_SendHex function

int distance_cm;
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
int rec;
int check;
char receivedChar;

// Hard Fault Handler
void HardFault_Handler(void) {
    UART_SendString("Hard Fault Exception Occurred!\r\n");
    NVIC_SystemReset();  // Reset the microcontroller
}

// Bus Fault Handler
void BusFault_Handler(void) {
    UART_SendString("Bus Fault Exception Occurred!\r\n");
    
    uint32_t bfsr = SCB->CFSR & 0xFF;
    
    if (bfsr & (1 << 0)) {
        UART_SendString("  - Instruction Bus Error\r\n");
    }
    if (bfsr & (1 << 1)) {
        UART_SendString("  - Precise Data Bus Error\r\n");
    }
    if (bfsr & (1 << 2)) {
        UART_SendString("  - Imprecise Data Bus Error\r\n");
    }
    if (bfsr & (1 << 3)) {
        UART_SendString("  - Bus Fault on Exception Return\r\n");
    }
    if (bfsr & (1 << 5)) {
        UART_SendString("  - Bus Fault Address Register Valid\r\n");
        uint32_t fault_address = SCB->BFAR;
        UART_SendString("  - Fault Address: 0x");
        UART_SendHex(fault_address);
        UART_SendString("\r\n");
    }

    NVIC_SystemReset();  // Reset the microcontroller
}

// Memory Management Fault Handler
void MemManage_Handler(void) {
    UART_SendString("Memory Management Fault Occurred!\r\n");
    
    uint32_t mfsr = (SCB->CFSR >> 8) & 0xFF;  // Extract MFSR (bits 8-15 of CFSR)
    
    if (mfsr & (1 << 0)) {
        UART_SendString("  - Instruction Access Violation\r\n");
    }
    if (mfsr & (1 << 1)) {
        UART_SendString("  - Data Access Violation\r\n");
    }
    if (mfsr & (1 << 3)) {
        UART_SendString("  - Memory Management Fault on Exception Return\r\n");
    }
    if (mfsr & (1 << 4)) {
        UART_SendString("  - Memory Management Fault Address Register Valid\r\n");
        
        uint32_t fault_address = SCB->MMFAR;
        UART_SendString("  - Fault Address: 0x");
        UART_SendHex(fault_address);
        UART_SendString("\r\n");
    }

    NVIC_SystemReset();  // Reset the microcontroller
}

// Usage Fault Handler
void UsageFault_Handler(void) {
    UART_SendString("Usage Fault Occurred!\r\n");
    
    uint32_t ufsr = (SCB->CFSR >> 16) & 0xFF;  // Extract UFSR (bits 16-23 of CFSR)
    
    if (ufsr & (1 << 0)) {
        UART_SendString("  - Undefined Instruction\r\n");
    }
    if (ufsr & (1 << 1)) {
        UART_SendString("  - Illegal Unaligned Access\r\n");
    }
    if (ufsr & (1 << 2)) {
        UART_SendString("  - Invalid State on Instruction Execution\r\n");
    }
    if (ufsr & (1 << 3)) {
        UART_SendString("  - Error on Exception Return\r\n");
    }
    if (ufsr & (1 << 4)) {
        UART_SendString("  - Division by Zero\r\n");
    }

    NVIC_SystemReset();  // Reset the microcontroller
}

int main(void) {
    GPIO_Config();
    interrupt_config();
    TIM2_Config();
    uart_config();
    MPU_Config();  // Configure MPU for memory protection
    
    // Uncomment to test Usage Fault: trigger usage fault intentionally
    // trigger_usage_fault();

    while (1) {    
        echo_generate();

        // Control LEDs based on distance
        if(distance_cm >= 0 && distance_cm <= 200) {        
            GPIOA->ODR |= (1U << 7);
            GPIOB->ODR &= ~(1U << 1);
            GPIOA->ODR &= ~(1U << 4);
            GPIOA->ODR &= ~(1U << 5);
            GPIOA->ODR &= ~(1U << 6);
                
            UART_SendString("LEVEL A\r\n");

            if (USART2->SR & USART_SR_RXNE) {
                receivedChar = UART_ReceiveChar();
                check = (receivedChar == '1') ? 1 : 0;
                relay(check);
            }
        }
        else if(distance_cm > 200 && distance_cm <= 400) {        
            GPIOA->ODR |= (1U << 7);
            GPIOB->ODR |= (1U << 1);
            GPIOA->ODR &= ~(1U << 4);
            GPIOA->ODR &= ~(1U << 5);
            GPIOA->ODR &= ~(1U << 6);

            UART_SendString("LEVEL B\r\n");

            if (USART2->SR & USART_SR_RXNE) {
                receivedChar = UART_ReceiveChar();
                check = (receivedChar == '1') ? 1 : 0;
                relay(check);
            }
        }

        for (int i = 0; i < 100000; i++);  // Simple delay loop
    }
}

void MPU_Config(void) {
    // Disable the MPU
    MPU->CTRL = 0;

    // Configure region 0 as Execute Never (XN)
    MPU->RNR = 0;  // Select region 0
    MPU->RBAR = 0x20000000;  // Base address at start of SRAM
    MPU->RASR = (1 << 28) |  // Set XN bit to prevent execution
                (3 << 24) |  // Set full access permissions (privileged and unprivileged access)
                (0 << 19) |  // Region size: 4KB (log2(size) - 1)
                (1 << 0);    // Enable region

    // Enable the MPU with the default memory map as background
    MPU->CTRL = (1 << 2) |  // Enable background region
                (1 << 0);   // Enable MPU globally

    // Enable fault exceptions for memory management faults
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
}

// Function to intentionally trigger a usage fault
void trigger_usage_fault(void) {
    // Example: Division by zero
    volatile int a = 5;
    volatile int b = 0;
    volatile int c = a / b; // This will trigger a usage fault due to division by zero
    
    // Example: Unaligned access
    volatile uint32_t *ptr = (uint32_t*)0x20000001; // Unaligned address
    volatile uint32_t val = *ptr; // This will trigger a usage fault
}

char UART_ReceiveChar(void) {
    while (!(USART2->SR & USART_SR_RXNE));  // Wait until RX buffer is not empty
    return USART2->DR;
}

void relay(int check) {
    if (check == 1) {
        GPIOB->MODER |= (1U << 0);
    } else if(check == 0) {
        GPIOB->MODER &= ~(3U << 0);
    }
}

void GPIO_Config(void) {
    RCC->AHB1ENR |= (1U << 0); // Enable GPIOA clock
    RCC->AHB1ENR |= (1U << 1); // Enable GPIOB clock

    GPIOA->MODER |= (1 << 0);  // Set PA0 as output
    GPIOA->MODER |= (1U << 14) | (1U << 12) | (1U << 10) | (1U << 8) | (1U << 6); // LEDs
    GPIOB->MODER |= (1U << 2);  // Set PB1 as output for relay control
}

void delay_us(int us) {
    // Assuming 168 MHz CPU clock, adjust delay for your clock settings
    for (volatile int i = 0; i < us * 168; i++);
}

void interrupt_config(void) {
    RCC->APB2ENR |= (1U << 14);
    SYSCFG->EXTICR[0] |= (0U << 4);
    EXTI->IMR |= (1U << 1);
    EXTI->FTSR |= (1U << 1);
    EXTI->RTSR |= (1U << 1);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

void uart_config(void) {
    RCC->APB1ENR |= (1U << 17);
    RCC->AHB1ENR |= (1U << 0);

    // GPIOA configuration for UART
    GPIOA->MODER |= (1U << 5);
    GPIOA->MODER |= (1U << 7);
    GPIOA->AFR[0] |= (1U << 8);  // Set AF for PA2 (TX)
    GPIOA->AFR[0] |= (1U << 12); // Set AF for PA3 (RX)

    USART2->BRR = 0x683; // 9600 Baud Rate
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable UART
}

void echo_generate(void) {
    GPIOA->ODR |= (1 << 0);
    delay_us(10);
    GPIOA->ODR &= ~(1 << 0);
}

void TIM2_Config(void) {
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 84 - 1;  // Prescaler
    TIM2->ARR = 0xFFFFFFFF; // Auto-reload register
    TIM2->CNT = 0; // Reset counter
    TIM2->CR1 |= TIM_CR1_URS; // Update request source
}

void UART_SendString(const char *str) {
    while (*str) {
        UART_SendChar(*str++);
    }
}

void UART_SendChar(char ch) {
    while (!(USART2->SR & USART_SR_TXE));  // Wait until TX buffer is empty
    USART2->DR = ch;  // Transmit character
}

void UART_SendHex(uint32_t value) {
    char buffer[9];  // 8 hex digits + null terminator
    snprintf(buffer, sizeof(buffer), "0x%08X", value);
    UART_SendString(buffer);
}
