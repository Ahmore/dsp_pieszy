#include "gui.h"
#include "main.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "cmsis_os.h"
#include "arm_const_structs.h"


static TS_StateTypeDef  TS_State;

osMutexId(gui_print_id);

float32_t *fft_in_buff = (float32_t*)AUDIO_BUFFER_FFT_IN;
float32_t *fft_out_buff = (float32_t*)AUDIO_BUFFER_FFT_OUT;
arm_rfft_fast_instance_f32 fft_init;

volatile int16_t effect_type;

const char * effect_labels[] = { "None", "Echo", "Pitch increase", "FIR" };

void startTouchscreenTask(void const *arguments) {
	char buffer[30];
	uint16_t x, y;
	sprintf(&buffer, "Effect: %s", effect_labels[effect_type]);
	print_dbg(buffer);

	while(1) {
		BSP_TS_GetState(&TS_State);
		x = TS_State.touchX[0];
		y = TS_State.touchY[0];

		if (TS_State.touchDetected) {
			effect_type = (effect_type + 1) % EFFECTS;
			sprintf(&buffer, "Effect: %s", effect_labels[effect_type]);
			print_dbg(buffer);
			osDelay(100);
		}

	}
}

void startFFTTask(void const *arguments) {
	char buffer[30];
//	print_dbg("FFT task started");

//	arm_rfft_fast_init_f32(&fft_init, AUDIO_BLOCK_SAMPLES);

	while (1) {
		osDelay(50);
		//sprintf(&buffer, "RMS: %d", rms_value);
		//print_dbg(buffer);

		osMutexWait(gui_print_id, osWaitForever);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
		BSP_LCD_FillRect(BSP_LCD_GetXSize() - 32, 0, 32, BSP_LCD_GetYSize());
		BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
		BSP_LCD_SetBackColor(LCD_COLOR_DARKBLUE);
		uint32_t rms_height = (rms_value * BSP_LCD_GetYSize() / 4000);
		rms_height = rms_height > BSP_LCD_GetYSize() ?
						BSP_LCD_GetYSize() : rms_height;
		BSP_LCD_FillRect(BSP_LCD_GetXSize() - 32,
				BSP_LCD_GetYSize() - rms_height, 32, rms_height);


		osMutexRelease(gui_print_id);
//		osDelay(250);
//		int16_t* buffer = (int16_t*)AUDIO_BUFFER_INTERNAL;
//		for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
//			fft_in_buff[i] = (float32_t)buffer[i];
//	    }
//		arm_rfft_fast_f32(&fft_init, fft_in_buff, fft_out_buff, 0);
//	    arm_abs_f32(fft_out_buff, fft_out_buff, AUDIO_BLOCK_SAMPLES);
//
//		sprintf(&buffer, "FFT %d!", fft_out_buff[0]);
//		print_dbg(buffer);

	}
}
