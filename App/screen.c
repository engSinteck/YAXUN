/*
 * screen.c
 *
 *  Created on: Aug 14, 2023
 *      Author: rinaldo.santos
 */

#include "main.h"
#include "screen.h"
#include "../lvgl/lvgl.h"

LV_FONT_DECLARE(lv_font_7Seg_48);

extern uint32_t ADC_iron, ADC_air;
extern volatile uint32_t enc1_cnt, enc1_dir, enc1_btn;
extern volatile uint32_t enc2_cnt, enc2_dir, enc2_btn;
extern volatile uint32_t enc3_cnt, enc3_dir, enc3_btn;
extern uint32_t flag_iron, flag_air;
extern volatile uint8_t sw_air, sw_iron;
extern uint32_t target_speed;
extern uint32_t temp_iron, temp_air;
extern float temperature_iron, temperature_air;
extern float target_iron, target_air, temperature_K;
extern uint16_t pwm_iron;
extern char str_termopar[];
extern float vdda, vref;

static lv_obj_t * Tela_Yaxun;
static lv_obj_t * iron_temperature;
static lv_obj_t * air_temperature;
static lv_obj_t * preset_iron;
static lv_obj_t * preset_air;
static lv_obj_t * preset_speed;
static lv_obj_t * symbol_iron;
static lv_obj_t * symbol_air;
static lv_obj_t * frame_iron;
static lv_obj_t * frame_air;
static lv_timer_t * task_yaxun;

// Debug
static lv_obj_t * Tela_Debug;
static lv_obj_t * adc_iron;
static lv_obj_t * adc_air;
static lv_obj_t * enc_1;
static lv_obj_t * enc_2;
static lv_obj_t * enc_3;
static lv_obj_t * label_sw_iron;
static lv_obj_t * label_sw_air;
static lv_obj_t * label_temp_iron;
static lv_obj_t * label_temp_air;
static lv_obj_t * label_pwm_iron;
static lv_obj_t * label_termopar;
static lv_obj_t * label_power;
static lv_timer_t * task_debug;

void create_iron(void);
void create_air(void);
void update_yaxun_screen(lv_timer_t * timer);
void update_debug_screen(lv_timer_t * timer);

void screen_main(void)
{
	Tela_Yaxun = lv_obj_create(NULL);
	lv_obj_clear_flag(Tela_Yaxun, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_style_bg_color(Tela_Yaxun, lv_color_hex(0x000000), 0);
	lv_obj_set_style_bg_grad_color(Tela_Yaxun, lv_color_hex(0x000000), 0);

	create_iron();
	create_air();

    static uint32_t user_data = 10;
    task_yaxun = lv_timer_create(update_yaxun_screen, 200,  &user_data);

	lv_scr_load(Tela_Yaxun);
}

void create_iron(void)
{
	// Desenha Frame Iron
	frame_iron = lv_obj_create(Tela_Yaxun);
    lv_obj_set_size(frame_iron, 236, 84);
    lv_obj_set_style_radius(frame_iron, 2, 0);
    lv_obj_set_style_bg_color(frame_iron, lv_color_hex(0x0000FF), 0);
    lv_obj_set_style_bg_grad_color(frame_iron, lv_color_hex(0x0000FF), 0);
    lv_obj_set_style_border_color(frame_iron, lv_color_hex(0xAAA9AD), 0);
    lv_obj_set_style_bg_opa(frame_iron, LV_OPA_50, 0);
    lv_obj_clear_flag(frame_iron, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(frame_iron, 1, 20);
    // Label Temperature IRON
    iron_temperature = lv_label_create(frame_iron);
    lv_obj_set_style_text_font(iron_temperature, &lv_font_7Seg_48, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(iron_temperature, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(iron_temperature, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(iron_temperature, 1, 0);
    lv_obj_set_style_text_line_space(iron_temperature, 1, 0);
    lv_label_set_long_mode(iron_temperature, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(iron_temperature, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(iron_temperature, "%0.0f", temperature_iron);
    lv_obj_align_to(iron_temperature, frame_iron, LV_ALIGN_CENTER, 0, 8);	// Align
    // Label Preset IRON
    preset_iron = lv_label_create(frame_iron);
    lv_obj_set_style_text_font(preset_iron, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(preset_iron, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(preset_iron, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(preset_iron, 1, 0);
    lv_obj_set_style_text_line_space(preset_iron, 1, 0);
    lv_label_set_long_mode(preset_iron, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(preset_iron, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(preset_iron, "%0.0f", target_iron);
    lv_obj_align_to(preset_iron, frame_iron, LV_ALIGN_TOP_LEFT, -12, -14);	// Align
    // Symbol IRON
    symbol_iron = lv_label_create(frame_iron);
    lv_obj_set_style_text_font(symbol_iron, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(symbol_iron, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(symbol_iron, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(symbol_iron, 1, 0);
    lv_obj_set_style_text_line_space(symbol_iron, 1, 0);
    lv_label_set_long_mode(symbol_iron, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(symbol_iron, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text(symbol_iron, LV_SYMBOL_OK);
    lv_obj_align_to(symbol_iron, frame_iron, LV_ALIGN_TOP_RIGHT, 12, -14);	// Align
}

void create_air(void)
{
	// Desenha Frame Air
	frame_air = lv_obj_create(Tela_Yaxun);
    lv_obj_set_size(frame_air, 236, 84);
    lv_obj_set_style_radius(frame_air, 2, 0);
    lv_obj_set_style_bg_color(frame_air, lv_color_hex(0x0000FF), 0);
    lv_obj_set_style_bg_grad_color(frame_air, lv_color_hex(0x0000FF), 0);
    lv_obj_set_style_border_color(frame_air, lv_color_hex(0xAAA9AD), 0);
    lv_obj_set_style_bg_opa(frame_air, LV_OPA_50, 0);
    lv_obj_clear_flag(frame_air, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(frame_air, 1, 232);
    // Label Temperature Air
    air_temperature = lv_label_create(frame_air);
    lv_obj_set_style_text_font(air_temperature, &lv_font_7Seg_48, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(air_temperature, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(air_temperature, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(air_temperature, 1, 0);
    lv_obj_set_style_text_line_space(air_temperature, 1, 0);
    lv_label_set_long_mode(air_temperature, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(air_temperature, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(air_temperature, "%0.0f", temperature_air);
    lv_obj_align_to(air_temperature, frame_air, LV_ALIGN_CENTER, 0, 8);		// Align
    // Label Preset Air
    preset_air = lv_label_create(frame_air);
    lv_obj_set_style_text_font(preset_air, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(preset_air, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(preset_air, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(preset_air, 1, 0);
    lv_obj_set_style_text_line_space(preset_air, 1, 0);
    lv_label_set_long_mode(preset_air, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(preset_air, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(preset_air, "%0.0f", target_air);
    lv_obj_align_to(preset_air, frame_air, LV_ALIGN_TOP_LEFT, -12, -14);	// Align
    // Label Speed Air
    preset_speed = lv_label_create(frame_air);
    lv_obj_set_style_text_font(preset_speed, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(preset_speed, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(preset_speed, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(preset_speed, 1, 0);
    lv_obj_set_style_text_line_space(preset_speed, 1, 0);
    lv_label_set_long_mode(preset_speed, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(preset_speed, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(preset_speed, "%ld", target_speed);
    lv_obj_align_to(preset_speed, frame_air, LV_ALIGN_CENTER, 0, -30);	// Align
    // Symbol Air
    symbol_air = lv_label_create(frame_air);
    lv_obj_set_style_text_font(symbol_air, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(symbol_air, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(symbol_air, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(symbol_air, 1, 0);
    lv_obj_set_style_text_line_space(symbol_air, 1, 0);
    lv_label_set_long_mode(symbol_air, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(symbol_air, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text(symbol_air, LV_SYMBOL_OK);
    lv_obj_align_to(symbol_air, frame_air, LV_ALIGN_TOP_RIGHT, 12, -14);	// Align
}

void update_yaxun_screen(lv_timer_t * timer)
{
	// IRON
	lv_label_set_text_fmt(iron_temperature, "%0.0f", temperature_iron);
	lv_label_set_text_fmt(preset_iron, "%0.0f", target_iron);
	if(flag_iron == 0) {
		lv_label_set_text(symbol_iron, LV_SYMBOL_OK);
	}
	else if(flag_iron == 1) {
		lv_label_set_text(symbol_iron, LV_SYMBOL_UP);
	}
	else {
		lv_label_set_text(symbol_iron, LV_SYMBOL_DOWN);
	}
	// AIR
	lv_label_set_text_fmt(air_temperature, "%0.0f", temperature_iron);
	lv_label_set_text_fmt(preset_air, "%0.0f", target_air);
	lv_label_set_text_fmt(preset_speed, "%ld", target_speed);
	if(flag_air == 0) {
		lv_label_set_text(symbol_air, LV_SYMBOL_OK);
	}
	else if(flag_iron == 1) {
		lv_label_set_text(symbol_air, LV_SYMBOL_UP);
	}
	else {
		lv_label_set_text(symbol_air, LV_SYMBOL_DOWN);
	}
}


void screen_debug(void)
{
	Tela_Debug = lv_obj_create(NULL);
	lv_obj_clear_flag(Tela_Debug, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_style_bg_color(Tela_Debug, lv_color_hex(0x000000), 0);
	lv_obj_set_style_bg_grad_color(Tela_Debug, lv_color_hex(0x000000), 0);
	// Desenha Frame Debug
	lv_obj_t * frame_debug = lv_obj_create(Tela_Debug);
    lv_obj_set_size(frame_debug, 239, 319);
    lv_obj_set_style_radius(frame_debug, 2, 0);
    lv_obj_set_style_bg_color(frame_debug, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_grad_color(frame_debug, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_color(frame_debug, lv_color_hex(0xAAA9AD), 0);
    lv_obj_set_style_bg_opa(frame_debug, LV_OPA_50, 0);
    lv_obj_clear_flag(frame_debug, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(frame_debug, 0, 0);
	//
    adc_iron = lv_label_create(Tela_Debug);
    lv_obj_set_style_text_font(adc_iron, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(adc_iron, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(adc_iron, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(adc_iron, 1, 0);
    lv_obj_set_style_text_line_space(adc_iron, 1, 0);
    lv_label_set_long_mode(adc_iron, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(adc_iron, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(adc_iron, "ADC8: %ld - %0.1fmV", ADC_iron, (float)(ADC_iron * ((float)3300.0/4095)));
	lv_obj_set_pos(adc_iron, 10, 24);
	//
    adc_air = lv_label_create(Tela_Debug);
    lv_obj_set_style_text_font(adc_air, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(adc_air, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(adc_air, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(adc_air, 1, 0);
    lv_obj_set_style_text_line_space(adc_air, 1, 0);
    lv_label_set_long_mode(adc_air, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(adc_air, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(adc_air, "ADC9: %ld - %0.1fmV", ADC_air, (float)(ADC_air * ((float)3300.0/4095)));
	lv_obj_set_pos(adc_air, 10, 48);
	//
    enc_1 = lv_label_create(Tela_Debug);
    lv_obj_set_style_text_font(enc_1, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(enc_1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(enc_1, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(enc_1, 1, 0);
    lv_obj_set_style_text_line_space(enc_1, 1, 0);
    lv_label_set_long_mode(enc_1, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(enc_1, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(enc_1, "ENC1 - %ld  Dir: %ld  Btn: %ld", enc1_cnt, enc1_dir, enc1_btn);
	lv_obj_set_pos(enc_1, 10, 72);
	//
    enc_2 = lv_label_create(Tela_Debug);
    lv_obj_set_style_text_font(enc_2, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(enc_2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(enc_2, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(enc_2, 1, 0);
    lv_obj_set_style_text_line_space(enc_2, 1, 0);
    lv_label_set_long_mode(enc_2, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(enc_2, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(enc_2, "ENC2 - %ld  Dir: %ld Btn: %ld", enc2_cnt, enc2_dir, enc2_btn);
	lv_obj_set_pos(enc_2, 10, 96);
	//
    enc_3 = lv_label_create(Tela_Debug);
    lv_obj_set_style_text_font(enc_3, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(enc_3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(enc_3, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(enc_3, 1, 0);
    lv_obj_set_style_text_line_space(enc_3, 1, 0);
    lv_label_set_long_mode(enc_3, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(enc_3, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(enc_3, "ENC3 - %ld  Dir: %ld Btn: %ld ", enc3_cnt, enc3_dir, enc3_btn);
	lv_obj_set_pos(enc_3, 10, 120);

	label_temp_iron = lv_label_create(Tela_Debug);
    lv_obj_set_style_text_font(label_temp_iron, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label_temp_iron, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(label_temp_iron, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(label_temp_iron, 1, 0);
    lv_obj_set_style_text_line_space(label_temp_iron, 1, 0);
    lv_label_set_long_mode(label_temp_iron, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(label_temp_iron, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(label_temp_iron, "IRON °C: %0.1f", temperature_iron);
	lv_obj_set_pos(label_temp_iron, 10, 144);

	label_temp_air = lv_label_create(Tela_Debug);
	lv_obj_set_style_text_font(label_temp_air, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(label_temp_air, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(label_temp_air, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(label_temp_air, 1, 0);
	lv_obj_set_style_text_line_space(label_temp_air, 1, 0);
	lv_label_set_long_mode(label_temp_air, LV_LABEL_LONG_WRAP);          	// Break the long lines
	lv_label_set_recolor(label_temp_air, true);                         	// Enable re-coloring by commands in the text
    lv_label_set_text_fmt(label_temp_air, "AIR °C: %0.1f", temperature_air);
	lv_obj_set_pos(label_temp_air, 10, 168);

	label_pwm_iron = lv_label_create(Tela_Debug);
	lv_obj_set_style_text_font(label_pwm_iron, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(label_pwm_iron, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(label_pwm_iron, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(label_pwm_iron, 1, 0);
	lv_obj_set_style_text_line_space(label_pwm_iron, 1, 0);
	lv_label_set_long_mode(label_pwm_iron, LV_LABEL_LONG_WRAP);          	// Break the long lines
	lv_label_set_recolor(label_pwm_iron, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(label_pwm_iron, "PWM_IRON: %d", pwm_iron);
	lv_obj_set_pos(label_pwm_iron, 10, 192);

	label_sw_iron = lv_label_create(Tela_Debug);
    lv_obj_set_style_text_font(label_sw_iron, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label_sw_iron, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(label_sw_iron, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(label_sw_iron, 1, 0);
    lv_obj_set_style_text_line_space(label_sw_iron, 1, 0);
    lv_label_set_long_mode(label_sw_iron, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(label_sw_iron, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(label_sw_iron, "SW_IRON: %d", sw_iron);
	lv_obj_set_pos(label_sw_iron, 10, 216);

	label_sw_air = lv_label_create(Tela_Debug);
    lv_obj_set_style_text_font(label_sw_air, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label_sw_air, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(label_sw_air, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(label_sw_air, 1, 0);
    lv_obj_set_style_text_line_space(label_sw_air, 1, 0);
    lv_label_set_long_mode(label_sw_air, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(label_sw_air, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(label_sw_air, "SW_AIR: %d", sw_air);
	lv_obj_set_pos(label_sw_air, 10, 240);

	label_termopar = lv_label_create(Tela_Debug);
    lv_obj_set_style_text_font(label_termopar, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label_termopar, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(label_termopar, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(label_termopar, 1, 0);
    lv_obj_set_style_text_line_space(label_termopar, 1, 0);
    lv_label_set_long_mode(label_termopar, LV_LABEL_LONG_WRAP);          	// Break the long lines
    lv_label_set_recolor(label_termopar, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(label_termopar, "%s", str_termopar);
	lv_obj_set_pos(label_termopar, 10, 264);

	label_power = lv_label_create(Tela_Debug);
	lv_obj_set_style_text_font(label_power, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(label_power, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(label_power, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(label_power, 1, 0);
	lv_obj_set_style_text_line_space(label_power, 1, 0);
	lv_label_set_long_mode(label_power, LV_LABEL_LONG_WRAP);          	// Break the long lines
	lv_label_set_recolor(label_power, true);                         	// Enable re-coloring by commands in the text
	lv_label_set_text_fmt(label_power, "Vcca: %0.2f  Vref %0.2f", vdda, vref);
	lv_obj_set_pos(label_power, 10, 288);

    static uint32_t user_data = 10;
    task_debug = lv_timer_create(update_debug_screen, 250,  &user_data);

	lv_scr_load(Tela_Debug);
}

void update_debug_screen(lv_timer_t * timer)
{
	lv_label_set_text_fmt(adc_iron, "ADC8: %ld - %0.1fmV", ADC_iron, (float)(ADC_iron * ((float)3300.0/4095)));
	lv_label_set_text_fmt(adc_air, "ADC9: %ld - %0.1fmV", ADC_air, (float)(ADC_air * ((float)3300.0/4095)));

	lv_label_set_text_fmt(label_temp_iron, "IRON °C: %0.1f", temperature_iron);
	lv_label_set_text_fmt(label_temp_air, "AIR °C: %0.1f", temperature_air);

	lv_label_set_text_fmt(enc_1, "ENC1 - %ld  Dir: %ld Btn: %ld", enc1_cnt, enc1_dir, enc1_btn);
	lv_label_set_text_fmt(enc_2, "ENC2 - %ld  Dir: %ld Btn: %ld", enc2_cnt, enc2_dir, enc2_btn);
	lv_label_set_text_fmt(enc_3, "ENC3 - %ld  Dir: %ld Btn: %ld", enc3_cnt, enc3_dir, enc3_btn);

	lv_label_set_text_fmt(label_pwm_iron, "PWM_IRON: %d", pwm_iron);
	lv_label_set_text_fmt(label_sw_iron, "SW_IRON: %d", sw_iron);
	lv_label_set_text_fmt(label_sw_air,  "SW_AIR: %d",  sw_air);

	lv_label_set_text_fmt(label_termopar, "%s", str_termopar);
}


