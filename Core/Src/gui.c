#include "gui.h"
#include "app.h"
#include "debug_uart.h"
#include "logo.h"
#include "main.h"
#include "psu_gui_api.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define GUI_FILTER_ALPHA 0.5f
#define BTN_DEBOUNCE_MS 25u
#define BTN_POLL_FALLBACK_LOCKOUT_MS 80u
#define GUI_ENCODER_LONG_PRESS_MS 1200u
#define GUI_V_SNAP_TO_SET 0.01f
#define GUI_V_DEADBAND 0.02f
#define GUI_MODE_AUTOFOLLOW_MS 1200u
#define GUI_REDRAW_PERIOD_MS 40u
#define GUI_ENCODER_TIMER_COUNTS_PER_STEP 4
#define GUI_ENCODER_DIRECTION (-1)

static float v_out = 0.0f;
static float i_out = 0.0f;
static float p_out = 0.0f;
static float v_in = 0.0f;
static float pd_voltage_v = 0.0f;
static float pd_current_a = 0.0f;
static float pd_power_w = 0.0f;
static uint32_t pd_active_rdo_raw = 0U;
static uint8_t pd_contract_valid = 0U;

static float v_target_local = 0.0f;

volatile uint8_t onoff_click = 0;
volatile uint8_t aux1_click = 0;
volatile uint8_t aux2_click = 0;

static volatile uint32_t onoff_last_ms = 0;
static volatile uint32_t aux1_last_ms = 0;
static volatile uint32_t aux2_last_ms = 0;
static GPIO_PinState onoff_pressed_state = GPIO_PIN_RESET;
static uint32_t last_manual_focus_ms = 0;
static uint8_t control_focus_pending = 0;
static PSU_GuiControlMode_t last_control_mode = PSU_GUI_CONTROL_MODE_CV;

static inline void debounce_set_flag_when_pressed(volatile uint8_t *flag,
                                                  volatile uint32_t *last_ms,
                                                  GPIO_TypeDef *port,
                                                  uint16_t pin,
                                                  GPIO_PinState pressed_state)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - *last_ms) < BTN_DEBOUNCE_MS) {
    return;
  }

  if (HAL_GPIO_ReadPin(port, pin) != pressed_state) {
    return;
  }

  *last_ms = now;
  *flag = 1;
}

static inline float GUI_FilterVout(float prev, float in) {
  float cand = in;

  if (fabsf(cand - v_target_local) <= GUI_V_SNAP_TO_SET) {
    return v_target_local;
  }

  if (fabsf(cand - prev) < GUI_V_DEADBAND) {
    return prev;
  }

  return prev + GUI_FILTER_ALPHA * (cand - prev);
}

static PSU_GuiControlMode_t GUI_GetDisplayControlMode(void) {
  PSU_GuiControlMode_t mode = PSU_GuiGetControlMode();

  if (mode == PSU_GUI_CONTROL_MODE_CC) {
    return PSU_GUI_CONTROL_MODE_CC;
  }

  return PSU_GUI_CONTROL_MODE_CV;
}

static uint8_t enc_synced = 0;
static uint8_t enc_gpio_synced = 0;
static uint8_t enc_gpio_last = 0;
static int8_t enc_gpio_accum = 0;
static uint8_t enc_timer_ready = 0;
static uint16_t enc_timer_last = 0;
static int32_t enc_timer_accum = 0;

extern TIM_HandleTypeDef htim4;

/* ================== KONFIGURACJA ================== */
static const float EDIT_STEPS[] = {10.0f, 1.0f, 0.1f, 0.01f};
#define EDIT_STEP_COUNT (sizeof(EDIT_STEPS) / sizeof(EDIT_STEPS[0]))

/* ================== MAKRA POMOCNICZE ================== */
#define INT_P(x) ((int)(x))
#define FRAC_P(x) ((int)(((x) - (int)(x)) * 100))

/* ================== ZMIENNE STANU ================== */
typedef enum {
  GUI_VIEW = 0,
  GUI_EDIT_V,
  GUI_EDIT_I,
  GUI_SETTINGS
} gui_mode_t;

static gui_mode_t gui_mode = GUI_VIEW;
static bool output_enabled = false;
static uint8_t gui_force_redraw = 1u;

static float i_target_local = 1.0f;
static PSU_GuiUsbMode_t settings_usb_mode = PSU_GUI_USB_MODE_AUTO;

/* 0–3: V (10,1,0.1,0.01), 4–7: I (10,1,0.1,0.01) */
static uint8_t edit_pos = 0;

static uint8_t GUI_ReadEncoderAB(void);

static void GUI_SyncEncoder(void) {
  enc_gpio_last = GUI_ReadEncoderAB();
  enc_gpio_accum = 0;
  enc_gpio_synced = 1;

  if (enc_timer_ready != 0u) {
    enc_timer_last = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    enc_timer_accum = 0;
  }
}

static void GUI_InitInputGpio(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = ENC_BTN_Pin;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ENC_BTN_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BTN_ON_OFF_Pin;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_ON_OFF_GPIO_Port, &GPIO_InitStruct);
  onoff_pressed_state =
      (HAL_GPIO_ReadPin(BTN_ON_OFF_GPIO_Port, BTN_ON_OFF_Pin) == GPIO_PIN_SET) ?
      GPIO_PIN_RESET :
      GPIO_PIN_SET;

  GPIO_InitStruct.Pull = GPIO_PULLUP;

  GPIO_InitStruct.Pin = BTN_AUX1_Pin;
  HAL_GPIO_Init(BTN_AUX1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BTN_AUX2_Pin;
  HAL_GPIO_Init(BTN_AUX2_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

static void GUI_InitEncoderTimer(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*
   * CubeMX owns TIM4/PB6/PB7, but the GUI starts it here. Keep PB6/PB7 in AF
   * mode; reconfiguring them as plain GPIO would disable the hardware encoder.
   */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;

  GPIO_InitStruct.Pin = ENC_A_Pin;
  HAL_GPIO_Init(ENC_A_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ENC_B_Pin;
  HAL_GPIO_Init(ENC_B_GPIO_Port, &GPIO_InitStruct);

  /* Use both encoder channels even if CubeMX generated TI1-only mode. */
  htim4.Instance->SMCR &= ~TIM_SMCR_SMS;
  htim4.Instance->SMCR |= TIM_ENCODERMODE_TI12;

  __HAL_TIM_SET_COUNTER(&htim4, 0x8000u);
  enc_timer_last = 0x8000u;
  enc_timer_accum = 0;

  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) == HAL_OK) {
    enc_timer_ready = 1u;
  } else {
    enc_timer_ready = 0u;
  }
}

static uint8_t GUI_GetUnitsCursorForMode(PSU_GuiControlMode_t mode) {
  return (mode == PSU_GUI_CONTROL_MODE_CC) ? 5u : 1u;
}

static void GUI_HoldManualFocus(void) {
  last_manual_focus_ms = HAL_GetTick();
}

static void GUI_SetEditSelection(uint8_t new_edit_pos) {
  edit_pos = new_edit_pos % 8u;
  gui_mode = (edit_pos < 4u) ? GUI_EDIT_V : GUI_EDIT_I;
  gui_force_redraw = 1u;
}

static void GUI_ApplyAutoFocus(PSU_GuiControlMode_t mode) {
  GUI_SetEditSelection(GUI_GetUnitsCursorForMode(mode));
  GUI_SyncEncoder();
}

/* ================== PROSTE BUTTON HANDLING ================== */
typedef struct {
  uint8_t stable;
  uint8_t last_raw;
  uint8_t active_level;
  uint8_t long_reported;
  uint32_t last_change_ms;
  uint32_t pressed_since_ms;
} btn_t;

typedef enum {
  BTN_EVENT_NONE = 0,
  BTN_EVENT_SHORT_PRESS,
  BTN_EVENT_LONG_PRESS
} btn_event_t;

static void BTN_Init(btn_t *b, uint8_t initial_raw, uint8_t active_level) {
  uint8_t raw = initial_raw ? 1u : 0u;

  b->stable = raw;
  b->last_raw = raw;
  b->active_level = active_level ? 1u : 0u;
  b->long_reported = 0u;
  b->last_change_ms = HAL_GetTick();
  b->pressed_since_ms = HAL_GetTick();
}

void BTN_enc_handle() {
  /* The encoder button is polled so short and long presses can be
   * distinguished. EXTI remains enabled for compatibility with the board. */
}

void BTN_onoff_handle() {
  debounce_set_flag_when_pressed(&onoff_click, &onoff_last_ms,
                                 BTN_ON_OFF_GPIO_Port, BTN_ON_OFF_Pin, onoff_pressed_state);
}

void BTN_aux1_handle() {
  debounce_set_flag_when_pressed(&aux1_click, &aux1_last_ms,
                                 BTN_AUX1_GPIO_Port, BTN_AUX1_Pin, GPIO_PIN_RESET);
}

void BTN_aux2_handle() {
  debounce_set_flag_when_pressed(&aux2_click, &aux2_last_ms,
                                 BTN_AUX2_GPIO_Port, BTN_AUX2_Pin, GPIO_PIN_RESET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ENC_BTN_Pin) {
    BTN_enc_handle();
  } else if (GPIO_Pin == BTN_ON_OFF_Pin) {
    BTN_onoff_handle();
  } else if (GPIO_Pin == BTN_AUX1_Pin) {
    BTN_aux1_handle();
  } else if (GPIO_Pin == BTN_AUX2_Pin) {
    BTN_aux2_handle();
  }
}

void BTN_reset() {
  onoff_click = 0;
  aux1_click = 0;
  aux2_click = 0;
}

static btn_event_t BTN_EncoderEvent(btn_t *b, uint8_t raw_state) {
  uint8_t raw = raw_state ? 1u : 0u;
  uint32_t now = HAL_GetTick();

  if (raw != b->last_raw) {
    b->last_raw = raw;
    b->last_change_ms = now;
  }

  if ((uint32_t)(now - b->last_change_ms) >= BTN_DEBOUNCE_MS) {
    if (raw != b->stable) {
      b->stable = raw;
      if (b->stable == b->active_level) {
        b->pressed_since_ms = now;
        b->long_reported = 0u;
      } else if (b->long_reported == 0u) {
        return BTN_EVENT_SHORT_PRESS;
      }
    }
  }

  if ((b->stable == b->active_level) &&
      (b->long_reported == 0u) &&
      ((uint32_t)(now - b->pressed_since_ms) >=
       GUI_ENCODER_LONG_PRESS_MS)) {
    b->long_reported = 1u;
    return BTN_EVENT_LONG_PRESS;
  }

  return BTN_EVENT_NONE;
}

static uint8_t BTN_Click(btn_t *b, uint8_t raw_state,
                         volatile uint32_t *last_event_ms) {
  uint8_t raw = raw_state ? 1u : 0u;
  uint32_t now = HAL_GetTick();

  if (raw != b->last_raw) {
    b->last_raw = raw;
    b->last_change_ms = now;
  }

  if ((now - b->last_change_ms) < BTN_DEBOUNCE_MS) {
    return 0;
  }

  if (raw != b->stable) {
    b->stable = raw;
    if (b->stable == b->active_level) {
      if ((uint32_t)(now - *last_event_ms) < BTN_POLL_FALLBACK_LOCKOUT_MS) {
        return 0;
      }
      *last_event_ms = now;
      return 1;
    }
  }

  return 0;
}

/* ================== FUNKCJE GRAFICZNE ================== */
void GUI_FillRect(int x, int y, int w, int h, SSD1306_COLOR color) {
  for (int i = 0; i < h; i++) {
    ssd1306_Line(x, y + i, x + w, y + i, color);
  }
}

void GUI_DrawProgressBar(int x, int y, int w, int h, int percent) {
  if (percent > 100)
    percent = 100;
  if (percent < 0)
    percent = 0;

  ssd1306_DrawRectangle(x, y, x + w, y + h, White);

  int fill_w = (w * percent) / 100;
  if (fill_w > 0) {
    GUI_FillRect(x + 1, y + 2, fill_w, h - 3, White);
  }
}

static void GUI_DrawDigitUnderline(int base_x, int y_line, uint8_t step_idx,
                                   SSD1306_COLOR color) {
  if (step_idx > 3)
    step_idx = 3;

  static const uint8_t digit_x_offset[4] = {0, 10, 10 + 10 + 5,
                                            10 + 10 + 5 + 10};

  int x0 = base_x + digit_x_offset[step_idx];
  int x1 = x0 + 7;

  ssd1306_Line(x0, y_line, x1, y_line, color);
}

/* ================== OUTPUT ENABLE (HW) ================== */
static void GUI_SetOutputEnabled(bool en) {
  if (!en) {
    output_enabled = false;
    gui_force_redraw = 1u;

    PSU_GuiSetTargetVoltage(0.0f);
    PSU_GuiSetTargetCurrent(i_target_local); /* limit prądu zostaje */
    PSU_Stop();
    return;
  }

  output_enabled = true;
  gui_force_redraw = 1u;

  PSU_GuiSetTargetVoltage(v_target_local);
  PSU_GuiSetTargetCurrent(i_target_local);
  PSU_Start();
}

static uint8_t GUI_ReadEncoderAB(void) {
  uint8_t a = (HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin) == GPIO_PIN_SET) ? 1u : 0u;
  uint8_t b = (HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin) == GPIO_PIN_SET) ? 1u : 0u;

  return (uint8_t)((a << 1) | b);
}

static int32_t GUI_GetEncoderGpioDelta(void) {
  static const int8_t quad_table[16] = {
      0, -1, 1, 0,
      1, 0, 0, -1,
      -1, 0, 0, 1,
      0, 1, -1, 0,
  };
  uint8_t state = GUI_ReadEncoderAB();

  if (!enc_gpio_synced) {
    enc_gpio_last = state;
    enc_gpio_accum = 0;
    enc_gpio_synced = 1;
    return 0;
  }

  if (state == enc_gpio_last) {
    return 0;
  }

  int8_t step = quad_table[(uint8_t)((enc_gpio_last << 2) | state)];
  enc_gpio_last = state;

  if (step == 0) {
    enc_gpio_accum = 0;
    return 0;
  }

  enc_gpio_accum += step;

  if (enc_gpio_accum >= 4) {
    enc_gpio_accum = 0;
    return 1;
  }

  if (enc_gpio_accum <= -4) {
    enc_gpio_accum = 0;
    return -1;
  }

  return 0;
}

static int32_t GUI_GetEncoderTimerDelta(void) {
  int32_t raw_delta;
  int32_t steps;
  uint16_t cnt;

  if (enc_timer_ready == 0u) {
    return 0;
  }

  cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  raw_delta = GUI_ENCODER_DIRECTION * (int16_t)(cnt - enc_timer_last);
  enc_timer_last = cnt;

  if (raw_delta == 0) {
    return 0;
  }

  enc_timer_accum += raw_delta;
  steps = enc_timer_accum / GUI_ENCODER_TIMER_COUNTS_PER_STEP;

  if (steps != 0) {
    enc_timer_accum -= steps * GUI_ENCODER_TIMER_COUNTS_PER_STEP;
  }

  return steps;
}

static char *GUI_UsbModeText(PSU_GuiUsbMode_t mode) {
  switch (mode) {
    case PSU_GUI_USB_MODE_SINK_ONLY:
      return "SINK ONLY";
    case PSU_GUI_USB_MODE_SOURCE_ONLY:
      return "SOURCE ONLY";
    case PSU_GUI_USB_MODE_AUTO:
    default:
      return "AUTO (DRP)";
  }
}

static void GUI_OpenSettings(void) {
  settings_usb_mode = PSU_GuiGetUsbMode();
  gui_mode = GUI_SETTINGS;
  GUI_SyncEncoder();
  gui_force_redraw = 1u;
}

static void GUI_CloseSettings(bool apply) {
  if (apply) {
    (void)PSU_GuiSetUsbMode(settings_usb_mode);
  }
  gui_mode = GUI_VIEW;
  edit_pos = GUI_GetUnitsCursorForMode(GUI_GetDisplayControlMode());
  GUI_SyncEncoder();
  gui_force_redraw = 1u;
}

/* ================== LOGIKA ENKODERA ================== */
static void GUI_HandleEncoder(void) {
  int32_t delta = GUI_GetEncoderTimerDelta();

  if ((delta == 0) && (enc_timer_ready == 0u)) {
    delta = GUI_ENCODER_DIRECTION * GUI_GetEncoderGpioDelta();
  }

  if (delta == 0) {
    return;
  }

  if (gui_mode != GUI_VIEW) {
    GUI_HoldManualFocus();
  }
  gui_force_redraw = 1u;

  if (gui_mode == GUI_SETTINGS) {
    int32_t selected = (int32_t)settings_usb_mode + delta;

    selected %= 3;
    if (selected < 0) {
      selected += 3;
    }
    settings_usb_mode = (PSU_GuiUsbMode_t)selected;
    return;
  }

  if (edit_pos < 4) {
    float step = EDIT_STEPS[edit_pos];
    v_target_local += (float)delta * step;
    if (v_target_local < 0.0f)
      v_target_local = 0.0f;
    if (v_target_local > 30.0f)
      v_target_local = 30.0f;

    /* zawsze aktualizuj PSU target */
    PSU_GuiSetTargetVoltage(v_target_local);

  } else if (edit_pos < 8) {
    float step = EDIT_STEPS[edit_pos - 4];
    i_target_local += (float)delta * step;

    if (i_target_local < 0.0f)
      i_target_local = 0.0f;
    if (i_target_local > 5.8f)
      i_target_local = 5.8f;

    PSU_GuiSetTargetCurrent(i_target_local);
  }
}

static void GUI_PollPdContract(void) {
  float new_voltage_v = 0.0f;
  float new_current_a = 0.0f;
  float new_power_w = 0.0f;
  uint32_t new_rdo_raw = 0U;
  uint8_t new_valid = PSU_GuiGetPdContract(&new_voltage_v,
                                            &new_current_a,
                                            &new_power_w,
                                            &new_rdo_raw);

  if ((new_valid != pd_contract_valid) ||
      (new_rdo_raw != pd_active_rdo_raw) ||
      (new_power_w != pd_power_w)) {
    gui_force_redraw = 1u;
  }

  pd_contract_valid = new_valid;
  pd_active_rdo_raw = new_rdo_raw;
  pd_voltage_v = new_voltage_v;
  pd_current_a = new_current_a;
  pd_power_w = new_power_w;
}

/* ================== GŁÓWNY EKRAN (128x128) ================== */
/* UWAGA: wygląda tak jak u Ciebie, nie ruszam układu/fontów */
static void GUI_DrawMain(void) {
  char buf[32];
  ssd1306_Fill(Black);
  PSU_GuiControlMode_t control_mode = GUI_GetDisplayControlMode();
  const bool psu_output_enabled = (PSU_IsRunning() != 0U);

  output_enabled = psu_output_enabled;

  float v_out_raw = PSU_GuiGetOutputVoltage();
  float i_out_raw = PSU_GuiGetOutputCurrent();
  float v_in_raw = PSU_GuiGetInputVoltage();

  // Napięcie na ekranie jest lekko wygładzone dla czytelności.
  v_out = GUI_FilterVout(v_out, v_out_raw);
  i_out = i_out_raw;
  v_in = v_in_raw;
  p_out = v_out * i_out;

  float v_display = v_target_local;
  float i_display = i_target_local;

  int pwm_percent = (int)(PSU_GuiGetDuty() * 100.0f);

  ssd1306_SetCursor(2, 2);
  snprintf(buf, sizeof(buf), "IN: %d.%dV", INT_P(v_in), FRAC_P(v_in) / 10);
  ssd1306_WriteString(buf, Font_6x8, White);

  ssd1306_SetCursor(2, 13);
  {
#if (PSU_GUI_PD_DEBUG != 0U)
    static uint32_t last_pd_draw_log_ms = 0U;
#endif
    if (pd_contract_valid != 0U) {
      snprintf(buf, sizeof(buf), "PD %uV %uW",
               (unsigned int)(pd_voltage_v + 0.5f),
               (unsigned int)(pd_power_w + 0.5f));
    } else {
      snprintf(buf, sizeof(buf), "PD --");
    }

#if (PSU_GUI_PD_DEBUG != 0U)
    if ((uint32_t)(HAL_GetTick() - last_pd_draw_log_ms) >= 1000U) {
      Debug_Printf("[GUI-DRAW] top_pd_text=\"%s\" valid=%u RDO=0x%08lX",
                   buf,
                   (unsigned int)pd_contract_valid,
                   (unsigned long)pd_active_rdo_raw);
      last_pd_draw_log_ms = HAL_GetTick();
    }
#endif
  }
  ssd1306_WriteString(buf, Font_6x8, White);

  ssd1306_SetCursor(72, 13);
  {
    float transfer_power_w = 0.0f;

    if (PSU_GuiGetTransferPower(&transfer_power_w) != 0U) {
      int transfer_power_rounded_w = (int)lroundf(transfer_power_w);

      if (transfer_power_rounded_w >= 0) {
        snprintf(buf, sizeof(buf), "P:+%dW", transfer_power_rounded_w);
      } else {
        snprintf(buf, sizeof(buf), "P:%dW", transfer_power_rounded_w);
      }
    } else {
      snprintf(buf, sizeof(buf), "P:--");
    }
  }
  ssd1306_WriteString(buf, Font_6x8, White);

  ssd1306_SetCursor(70, 3);
  ssd1306_WriteString("PWM", Font_6x8, White);
  GUI_DrawProgressBar(90, 3, 36, 6, pwm_percent);

  ssd1306_Line(0, 23, 127, 23, White);

  int x_center = (128 - (6 * 16)) / 2;

  if (!psu_output_enabled) {
    int x_off = (128 - (3 * 16)) / 2;
    ssd1306_SetCursor(x_off, 35);
    ssd1306_WriteString("OFF", Font_16x24, White);
  } else {
    snprintf(buf, sizeof(buf), "%02d.%02dV", INT_P(v_out), FRAC_P(v_out));
    ssd1306_SetCursor(x_center, 35);
    ssd1306_WriteString(buf, Font_16x24, White);
  }

  ssd1306_SetCursor(8, 65);
  snprintf(buf, sizeof(buf), "%d.%02d A", INT_P(i_out), FRAC_P(i_out));
  ssd1306_WriteString(buf, Font_16x15, White);

  ssd1306_SetCursor(70, 65);
  snprintf(buf, sizeof(buf), "%2d.%01d W", INT_P(p_out), FRAC_P(p_out) / 10);
  ssd1306_WriteString(buf, Font_16x15, White);

  ssd1306_Line(0, 85, 127, 85, White);
  ssd1306_Line(64, 85, 64, 128, White);

  const bool active_v_box =
      (gui_mode == GUI_VIEW) ? (control_mode != PSU_GUI_CONTROL_MODE_CC)
                             : (gui_mode == GUI_EDIT_V);
  const bool active_i_box =
      (gui_mode == GUI_VIEW) ? (control_mode == PSU_GUI_CONTROL_MODE_CC)
                             : (gui_mode == GUI_EDIT_I);

  SSD1306_COLOR  txt_v;
  SSD1306_COLOR  txt_i;

  if (active_v_box) {
    txt_v = Black;
    GUI_FillRect(0, 86, 63, 42, White);
  } else {
    
    txt_v = White;
  }

  if (active_i_box) {
    txt_i = Black;
    GUI_FillRect(65, 86, 63, 42, White);
  } else {
    txt_i = White;
  }

  ssd1306_SetCursor(8, 91);
  ssd1306_WriteString("SET [V]", Font_7x10, txt_v);

  ssd1306_SetCursor(8, 108);
  snprintf(buf, sizeof(buf), "%02d.%02d", INT_P(v_display), FRAC_P(v_display));
  ssd1306_WriteString(buf, Font_16x15, txt_v);

  if (active_v_box) {
    uint8_t v_cursor =
        (gui_mode == GUI_EDIT_V) ? edit_pos : GUI_GetUnitsCursorForMode(control_mode);
    GUI_DrawDigitUnderline(10, 123, v_cursor, txt_v);
  }

  ssd1306_SetCursor(72, 91);
  ssd1306_WriteString("SET [A]", Font_7x10, txt_i);

  ssd1306_SetCursor(72, 108);
  snprintf(buf, sizeof(buf), "%02d.%02d", INT_P(i_display), FRAC_P(i_display));
  ssd1306_WriteString(buf, Font_16x15, txt_i);

  if (active_i_box) {
    uint8_t i_cursor = (gui_mode == GUI_EDIT_I)
                           ? (edit_pos - 4u)
                           : (GUI_GetUnitsCursorForMode(control_mode) - 4u);
    GUI_DrawDigitUnderline(74, 123, i_cursor, txt_i);
  }

  ssd1306_UpdateScreen();
}

static void GUI_DrawSettings(void) {
  char buf[24];
  PSU_GuiUsbMode_t applied_mode = PSU_GuiGetUsbMode();
  uint8_t option;

  ssd1306_Fill(Black);
  ssd1306_SetCursor(24, 3);
  ssd1306_WriteString("USTAWIENIA", Font_7x10, White);
  ssd1306_Line(0, 16, 127, 16, White);

  ssd1306_SetCursor(5, 22);
  ssd1306_WriteString("TRYB PORTU USB-C", Font_6x8, White);

  for (option = 0u; option < 3u; ++option) {
    int y = 36 + ((int)option * 20);
    SSD1306_COLOR text_color = White;

    if ((PSU_GuiUsbMode_t)option == settings_usb_mode) {
      GUI_FillRect(4, y - 3, 120, 15, White);
      text_color = Black;
    }

    ssd1306_SetCursor(10, y);
    ssd1306_WriteString(GUI_UsbModeText((PSU_GuiUsbMode_t)option),
                        Font_7x10, text_color);
  }

  (void)snprintf(buf, sizeof(buf), "AKT: %s", GUI_UsbModeText(applied_mode));
  ssd1306_SetCursor(5, 98);
  ssd1306_WriteString(buf, Font_6x8, White);
  ssd1306_SetCursor(5, 110);
  ssd1306_WriteString("KLIK=ZAPISZ", Font_6x8, White);
  ssd1306_SetCursor(5, 120);
  ssd1306_WriteString("DLUGO=ANULUJ", Font_6x8, White);

  ssd1306_UpdateScreen();
}

/* ================== API GLOWNE ================== */
void GUI_Init(void) {
  GUI_InitInputGpio();
  GUI_InitEncoderTimer();

  enc_synced = 0;
  enc_gpio_synced = 0;
  GUI_SyncEncoder();

  ssd1306_Init();
  ssd1306_Fill(Black);

  ssd1306_DrawBitmap(0, 0, logo, 128, 128, White);
  ssd1306_UpdateScreen();

  uint32_t logo_ms = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - logo_ms) < 100u) {
    ssd1306_Service();
  }

  /* startowe nastawy */
  v_target_local = 0.0f;
  i_target_local = 1.0f;

  PSU_GuiSetTargetVoltage(v_target_local);
  PSU_GuiSetTargetCurrent(i_target_local);

  gui_mode = GUI_VIEW;
  edit_pos = GUI_GetUnitsCursorForMode(GUI_GetDisplayControlMode());
  last_manual_focus_ms = 0u;
  control_focus_pending = 0u;
  last_control_mode = GUI_GetDisplayControlMode();

  /* HARD OFF na starcie */
  GUI_SetOutputEnabled(false);
}

void GUI_Process(void) {
  static uint32_t last_draw_ms = 0;
  ssd1306_Service();

  uint32_t now = HAL_GetTick();
  PSU_GuiControlMode_t control_mode = GUI_GetDisplayControlMode();
  bool psu_output_enabled = (PSU_IsRunning() != 0U);
  btn_event_t encoder_event;

  if (output_enabled != psu_output_enabled) {
    output_enabled = psu_output_enabled;
    gui_force_redraw = 1u;
  }

  GUI_PollPdContract();

  static btn_t btn_enc;
  static btn_t btn_onoff;
  static btn_t btn_aux1;
  static btn_t btn_aux2;
  static uint8_t inited = 0;

  if (!inited) {
    BTN_Init(&btn_enc, HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin), 1u);
    BTN_Init(&btn_onoff,
             HAL_GPIO_ReadPin(BTN_ON_OFF_GPIO_Port, BTN_ON_OFF_Pin),
             (onoff_pressed_state == GPIO_PIN_SET) ? 1u : 0u);
    BTN_Init(&btn_aux1, HAL_GPIO_ReadPin(BTN_AUX1_GPIO_Port, BTN_AUX1_Pin), 0u);
    BTN_Init(&btn_aux2, HAL_GPIO_ReadPin(BTN_AUX2_GPIO_Port, BTN_AUX2_Pin), 0u);
    inited = 1;
  }

  encoder_event = BTN_EncoderEvent(
      &btn_enc, HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin));
  if (BTN_Click(&btn_onoff, HAL_GPIO_ReadPin(BTN_ON_OFF_GPIO_Port, BTN_ON_OFF_Pin),
                &onoff_last_ms)) {
    onoff_click = 1;
  }
  if (BTN_Click(&btn_aux1, HAL_GPIO_ReadPin(BTN_AUX1_GPIO_Port, BTN_AUX1_Pin),
                &aux1_last_ms)) {
    aux1_click = 1;
  }
  if (BTN_Click(&btn_aux2, HAL_GPIO_ReadPin(BTN_AUX2_GPIO_Port, BTN_AUX2_Pin),
                &aux2_last_ms)) {
    aux2_click = 1;
  }

    /* Pierwszy cykl: zsynchronizuj enkoder i nie licz delty, bo potrafi strzelić */
  if (!enc_synced) {
    GUI_SyncEncoder();
    enc_synced = 1;

    /* Dla pewności: startowe nastawy, żeby GUI nie pokazało śmieci */
    v_target_local = 0.0f;
    i_target_local = 1.0f;
    PSU_GuiSetTargetVoltage(v_target_local);
    PSU_GuiSetTargetCurrent(i_target_local);

    output_enabled = false;
    gui_force_redraw = 1u;
    last_manual_focus_ms = now;
    control_focus_pending = 0u;
    last_control_mode = control_mode;
    edit_pos = GUI_GetUnitsCursorForMode(control_mode);

    /* I tyle. Wracamy, żeby nic nie ruszyć w tym cyklu. */
    return;
  }

  GUI_HandleEncoder();

  if (encoder_event == BTN_EVENT_LONG_PRESS) {
    if (gui_mode == GUI_SETTINGS) {
      GUI_CloseSettings(false);
    } else {
      GUI_OpenSettings();
    }
  } else if (encoder_event == BTN_EVENT_SHORT_PRESS) {
    if (gui_mode == GUI_SETTINGS) {
      GUI_CloseSettings(true);
    } else {
      GUI_SetOutputEnabled(!output_enabled);
    }
  }

  /* Dedykowany ON/OFF pozostaje aktywny także w ustawieniach. */
  if (onoff_click) {
    GUI_SetOutputEnabled(!output_enabled);
    psu_output_enabled = (PSU_IsRunning() != 0U);
    output_enabled = psu_output_enabled;
    onoff_click = 0;
  }

  /* Edycja pozycji cyfry */
  if (gui_mode == GUI_SETTINGS) {
    aux1_click = 0;
    aux2_click = 0;
    control_focus_pending = 0u;
  } else if (gui_mode == GUI_VIEW) {
    edit_pos = GUI_GetUnitsCursorForMode(control_mode);
    control_focus_pending = 0u;

    if (aux1_click || aux2_click) {
      /* Edycja ma wracac do nastawy z GUI, nie do chwilowej wartosci rampy. */
      v_target_local = PSU_GuiGetTargetVoltage();
      i_target_local = PSU_GuiGetTargetCurrent();

      GUI_ApplyAutoFocus(control_mode);
      GUI_HoldManualFocus();
      gui_force_redraw = 1u;
      aux1_click = 0;
      aux2_click = 0;
    }
  } else {
    if (aux1_click) {
      GUI_SetEditSelection((uint8_t)(edit_pos + 1u));
      GUI_HoldManualFocus();
      gui_force_redraw = 1u;
      aux1_click = 0;
    }
    if (aux2_click) {
      GUI_SetEditSelection((uint8_t)(edit_pos + 7u));
      GUI_HoldManualFocus();
      gui_force_redraw = 1u;
      aux2_click = 0;
    }

    if (control_mode != last_control_mode) {
      control_focus_pending = 1u;
    }

    if (control_focus_pending &&
        ((uint32_t)(now - last_manual_focus_ms) >= GUI_MODE_AUTOFOLLOW_MS)) {
      GUI_ApplyAutoFocus(control_mode);
      control_focus_pending = 0u;
    }
  }

  last_control_mode = control_mode;

  if ((gui_force_redraw == 0u) &&
      ((HAL_GetTick() - last_draw_ms) < GUI_REDRAW_PERIOD_MS)) {
    return;
  }

  last_draw_ms = HAL_GetTick();
  gui_force_redraw = 0u;

  if (gui_mode == GUI_SETTINGS) {
    GUI_DrawSettings();
  } else {
    GUI_DrawMain();
  }
}
