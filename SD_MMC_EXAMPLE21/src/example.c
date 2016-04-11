/**
 * \file
 *
 * \brief SD/MMC card example with FatFs
 *
 * Copyright (c) 2012-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage SD/MMC/SDIO Card with FatFs Example
 *
 * \section Purpose
 *
 * This example shows how to implement the SD/MMC stack with the FatFS.
 * It will mount the file system and write a file in the card.
 *
 * The example outputs the information through the standard output (stdio).
 * To know the output used on the board, look in the conf_example.h file
 * and connect a terminal to the correct port.
 *
 * While using Xplained Pro evaluation kits, please attach I/O1 Xplained Pro
 * extension board to EXT1.
 *
 * \section Usage
 *
 * -# Build the program and download it into the board.
 * -# On the computer, open and configure a terminal application.
 * Refert to conf_example.h file.
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *    \code
	-- SD/MMC/SDIO Card Example on FatFs --
	-- Compiled: xxx xx xxxx xx:xx:xx --
	Please plug an SD, MMC or SDIO card in slot.
\endcode
 */

#include <asf.h>
#include "conf_example.h"
#include <string.h>


// DAC channel used
//#define DACC_CHANNEL        0 // (PB13)
#define DACC_CHANNEL        1 // (PB14)

//! DAC register base for test
#define DACC_BASE           DACC
//! DAC ID for test
#define DACC_ID             ID_DACC

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */

#define BUFF1_EMPTY 0
#define BUFF2_EMPTY 1
#define ACTIVE_BUFF 3
volatile uint8_t status_buf;
uint16_t number_elem_out;

#define BUFF_SIZE 4410
volatile BYTE Buff1_sound[BUFF_SIZE], Buff2_sound[BUFF_SIZE];

/** Analog control value */
#define DACC_ANALOG_CONTROL (DACC_ACR_IBCTLCH0(0x02) \
| DACC_ACR_IBCTLCH1(0x02) \
| DACC_ACR_IBCTLDACCORE(0x01))


volatile uint8_t g_file_name[30]; // Хранится имя воспроизводимого файла, для передачи в главный цикл
volatile uint8_t g_flag_play=0;   // Флаг необходимости запуска файла на воспроизведение
volatile uint8_t g_flag_stop=0;   // Флаг остановки воспроизведения


static void dac_init(void){
	/* Enable clock for DACC */
	sysclk_enable_peripheral_clock(DACC_ID);

	/* Reset DACC registers */
	dacc_reset(DACC_BASE);

	/* Half word transfer mode */
	dacc_set_transfer_mode(DACC_BASE, 0);

	/* Initialize timing, amplitude and frequency */
	dacc_set_timing(DACC_BASE, 0x08, 0, 0x10);

	/* Disable TAG and select output channel DACC_CHANNEL */
	dacc_set_channel_selection(DACC_BASE, DACC_CHANNEL);

	/* Enable output channel DACC_CHANNEL */
	dacc_enable_channel(DACC_BASE, DACC_CHANNEL);

	/* Set up analog current */
	dacc_set_analog_control(DACC_BASE, DACC_ANALOG_CONTROL);
}

/**
 * \brief SysTick IRQ handler.
 */
void SysTick_Handler(void)
{
	if(((status_buf&(1<<BUFF1_EMPTY))==0)&&((status_buf&(1<<BUFF2_EMPTY))==0))return;
	int16_t dac_val;
	
	if(status_buf&(1<<ACTIVE_BUFF))dac_val=(Buff2_sound[number_elem_out*2+1]<<8)+Buff2_sound[number_elem_out*2];
	else dac_val=(Buff1_sound[number_elem_out*2+1]<<8)+Buff1_sound[number_elem_out*2];
	number_elem_out++;
	if(number_elem_out>=(sizeof(Buff1_sound)/2)){
		number_elem_out=0;
		if(status_buf&(1<<ACTIVE_BUFF)){
			status_buf&=~(1<<ACTIVE_BUFF);
			status_buf&=~(1<<BUFF2_EMPTY);
		}
		else {
			status_buf|=(1<<ACTIVE_BUFF);
			status_buf&=~(1<<BUFF1_EMPTY);
		}
	}
		
	dac_val = dac_val>>4;
	dac_val = dac_val+2048;
	dacc_write_conversion_data(DACC_BASE, dac_val);
}

void play(char* file_name){
char test_file_name[30];
Ctrl_status status;
FRESULT res;
FATFS fs;
FIL file_object;

	sprintf(test_file_name, "0:%s", file_name);
	printf("Путь: %s",test_file_name);
	do {
		status = sd_mmc_test_unit_ready(0);
		if (CTRL_FAIL == status) {
			printf("Card install FAIL\n\r");
			printf("Please unplug and re-plug the card.\r");
			while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
			}
		}
	} while (CTRL_GOOD != status);
	
	printf("Mount disk...\n");
	memset(&fs, 0, sizeof(FATFS));
	res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
	if (FR_INVALID_DRIVE == res) {
		printf("[FAIL] res %d\n", res);
		goto main_end_of_test;
	}
	printf("[OK]\n");
	
	printf("Открытие файла на чтение...\n");
	res = f_open(&file_object,(char const *)test_file_name, FA_READ);
	if (res != FR_OK) {
		printf("[FAIL] res %d\n", res);
		goto main_end_of_test;
	}
	printf("[OK]\n");
	
	printf("Читаем из файла(в цикле, до тех пор пока есть данные)...\n");
	
	BYTE Buff[256];
	UINT temp=0;
	
	res = f_read(&file_object, Buff, 4, &temp);
	if (res != FR_OK) {
		printf("файл отказался читаться res %d\n", res);
		goto main_end_of_test;
	}
	if (temp != 4 || LD_DWORD(Buff) != FCC('R','I','F','F')) {
		printf("файл не тот\n");
		goto main_end_of_test;
	}
	
	printf("файл RIFF\n");
	
	uint8_t i;
	i=0;
	for(;;){
		res = f_read(&file_object, Buff, 1, &temp);
		if (res != FR_OK) {
			printf("[FAIL] res %d\n", res);
			goto main_end_of_test;
		}
		if(Buff[0]=='W'){
			res = f_read(&file_object, Buff, 3, &temp);
			if((Buff[0]=='A')&&(Buff[1]=='V')&&(Buff[2]=='E')){
				printf("файл WAVE\n");
				break;
			}
		}
		i++;
		if(i==20){
			printf("файл не WAVE\n");
			goto main_end_of_test;
		}
	}
	uint32_t size_sound=0;
	i=0;
	for(;;){
		res = f_read(&file_object, Buff, 1, &temp);
		if (res != FR_OK) {
			printf("[FAIL] res %d\n", res);
			goto main_end_of_test;
		}
		if(Buff[0]=='d'){
			f_read(&file_object, Buff, 3, &temp);
			if((Buff[0]=='a')&&(Buff[1]=='t')&&(Buff[2]=='a')){
				printf("раздел data найден\n");
				f_read(&file_object, Buff, 4, &temp);
				size_sound  = FCC(Buff[0],Buff[1],Buff[2],Buff[3]);
				printf("размер %08lX\n", size_sound);
				printf("длительность(по количеству байт в потоке) %u сек\n", (int16_t)((float)size_sound/44100.0/2.0));
				break;
			}
		}

		i++;
		if(i==200){
			printf("раздел data не найден\n");
			goto main_end_of_test;
		}
	}

	// Воспроизводим поток
	status_buf=0;
	do{
		if(g_flag_stop){g_flag_stop=0;break;}
		
		while(status_buf&(1<<BUFF1_EMPTY)); // Ждём пока опустеет буфер
		f_read(&file_object, Buff1_sound, sizeof(Buff1_sound), &temp);
		status_buf|=(1<<BUFF1_EMPTY);
		
		while(status_buf&(1<<BUFF2_EMPTY)); // Ждём пока опустеет буфер
		f_read(&file_object, Buff2_sound, sizeof(Buff2_sound), &temp);
		status_buf|=(1<<BUFF2_EMPTY);

	}while(sizeof(Buff1_sound)==temp);
	
	dacc_write_conversion_data(DACC_BASE, 0);
	printf("Test is successful.\r");
	
	main_end_of_test:
	f_close(&file_object);
}

#define RX_BUFFER_SIZE0 30

uint8_t rx_buffer0[RX_BUFFER_SIZE0];
uint8_t global_rx0_wr_index = 0;

void UART0_Handler() {
	uint32_t dw_status = uart_get_status(UART0);

	if(dw_status & UART_SR_RXRDY) {
		uint8_t received_byte;
		uart_read(UART0, &received_byte);
		rx_buffer0[global_rx0_wr_index++] = received_byte;
		
		if(global_rx0_wr_index >= RX_BUFFER_SIZE0-1){
			global_rx0_wr_index = 0;
		}
		if(received_byte == 0x0D){
			// Вытащить всё парсение в отдельную процедуру
			//пришло сообщение, парсим
			rx_buffer0[global_rx0_wr_index - 1] = 0; //ноль в конец сообщения
			global_rx0_wr_index = 0;
			
			// Получить имя и запустить файл
			if(strncmp("play=", rx_buffer0, 5) == 0){
				sprintf(g_file_name,"%s",rx_buffer0+5);
				g_flag_play=1;
			}
			// Получить имя и запустить файл
			if(strncmp("stop", rx_buffer0, 4) == 0){
				g_flag_stop=1;
			}
			
		}
		
	}
}

/**
 * \brief Application entry point.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{

 	const usart_serial_options_t usart_serial_options = {
 		.baudrate   = CONF_TEST_BAUDRATE,
 		.charlength = CONF_TEST_CHARLENGTH,
 		.paritytype = CONF_TEST_PARITY,
 		.stopbits   = CONF_TEST_STOPBITS,
 	};
 
 	irq_initialize_vectors();
 	cpu_irq_enable();
 
 	sysclk_init();
	board_init();
	
	dac_init();

	SysTick_Config(sysclk_get_cpu_hz() / 44100);
	
 	stdio_serial_init(CONF_TEST_USART, &usart_serial_options);

	// Включаем приём UART-а по прерываниям
	uart_enable_interrupt(UART0,UART_IER_RXRDY);
	NVIC_EnableIRQ(UART0_IRQn);
 
 	/* Initialize SD MMC stack */
 	sd_mmc_init();
 
 	printf("\x0C\n\r-- SD/MMC/SDIO Card Example on FatFs --\r");
 	printf("-- Compiled: %s %s --\r", __DATE__, __TIME__);
	
	
	
 	while (1) {
		 if(g_flag_play){
			 g_flag_play=0;
			 play(g_file_name);
		 }		 
 	}
}
