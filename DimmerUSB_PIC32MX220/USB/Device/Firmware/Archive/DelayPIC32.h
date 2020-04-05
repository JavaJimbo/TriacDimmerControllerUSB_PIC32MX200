/* DelayPIC32.h:
 * 
 * #pragma config FPLLMUL  = MUL_15        // PLL Multiplier for 220 - yields 60 Mhz
 * #pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
 * #pragma config FPLLIDIV = DIV_2         // PLL Input Divider
 * #pragma config FPLLODIV = DIV_1         // PLL Output Divider
 * #pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
 * 
 */

void Delay10us(long dwCount);
void DelayMs(long ms);
void DelayUs(unsigned short Count);






