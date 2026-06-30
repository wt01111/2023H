#include "signal_separation.h"
#include "signal_separation_config.h"

#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"

#ifndef ARM_MATH_CM7
#define ARM_MATH_CM7
#endif
#include "arm_math.h"
#include "stm32h7xx_hal_dac_ex.h"

#include <stdarg.h>
#include <stdio.h>

typedef enum
{
  WAVE_SINE = 0,
  WAVE_TRIANGLE = 1
} WaveType;

typedef enum
{
  CORDIC_MODE_NONE = 0,
  CORDIC_MODE_SINE,
  CORDIC_MODE_PHASE
} CordicMode;

typedef struct
{
  uint32_t freq_hz;
  uint8_t freq_index;
  WaveType wave;
  float32_t amp_adc;
  float32_t amp_dac;
  uint32_t phase_q32;
} SignalComponent;

typedef struct
{
  uint32_t nominal_step;
  int32_t step_corr;
  int64_t integrator;
  uint32_t phase_ref;
  uint64_t sample_ref;
  int32_t last_error;
} NcoState;

__ALIGNED(32) static uint16_t adc_dma_buf[SIGSEP_ADC_DMA_LEN];
__ALIGNED(32) static uint16_t dac1_dma_buf[SIGSEP_DAC_FRAME_LEN];
__ALIGNED(32) static uint16_t dac2_dma_buf[SIGSEP_DAC_FRAME_LEN];

static int16_t sine_lut[SIGSEP_SINE_LUT_SIZE];
static float32_t step_sin[SIGSEP_MAX_BIN + 1U];
static float32_t step_cos[SIGSEP_MAX_BIN + 1U];
static float32_t identify_amp_acc[SIGSEP_FREQ_COUNT];
static SignalComponent active_comp[2];
static NcoState nco_state[2];
static CordicMode cordic_mode = CORDIC_MODE_NONE;

static volatile int32_t adc_ready_offset = -1;
static volatile uint32_t adc_frame_count = 0;
static volatile uint32_t adc_frame_overrun = 0;
static volatile uint64_t adc_sample_count = 0;
static volatile uint64_t adc_ready_sample_start = 0;
static volatile uint64_t dac_sample_count = 0;
static volatile uint64_t dac_ready_play_sample[2];
static volatile uint32_t dac_ready_mask = 0;
static volatile uint32_t dac_half_overrun = 0;
static volatile uint8_t separation_identified = 0;
static uint32_t identify_frame_count = 0;

static void Debug_Printf(const char *fmt, ...);
static void Startup_Print(void);
static void Print_SeparationResult(const SignalComponent comp[2]);
static const char *WaveName(WaveType wave);
static void Timer2_SetSampleRate(uint32_t sample_rate_hz);
static uint32_t Timer2_GetClockHz(void);

static void CORDIC_PrepareTables(void);
static void CORDIC_SelectSine(void);
static void CORDIC_SelectPhase(void);
static float32_t CORDIC_SinFromPhase(uint32_t phase);
static uint32_t CORDIC_PhaseFromIQ(float32_t i_part, float32_t q_part);

static void Fill_DacMidscale(void);
static void Fill_DacHalf(uint32_t half_index, uint64_t play_sample);
static void Build_DacSamples(uint16_t *dst, const SignalComponent *comp, uint32_t start_phase,
                             uint32_t phase_step, uint32_t len);

static uint8_t Analyze_Frame(const uint16_t *samples, SignalComponent out[2]);
static float32_t Frame_Mean(const uint16_t *samples);
static void Measure_Component(const uint16_t *samples, float32_t mean, uint32_t freq_hz,
                              float32_t *amp, uint32_t *phase_q32);
static float32_t Measure_AmplitudeOnly(const uint16_t *samples, float32_t mean, uint32_t freq_hz);
static float32_t Correlate_Amplitude(const uint16_t *samples, float32_t mean, uint32_t freq_hz);
static WaveType Detect_WaveType(const uint16_t *samples, float32_t mean, uint32_t freq_hz, uint32_t other_hz,
                                float32_t fundamental_amp);

static void Nco_Init(uint32_t ch, const SignalComponent *comp, uint64_t sample_start);
static uint32_t Nco_Step(uint32_t ch);
static uint32_t Nco_PhaseAtSample(uint32_t ch, uint64_t sample);
static int32_t Nco_UpdateLock(uint32_t ch, uint32_t measured_phase, uint64_t frame_start_sample);
static void Nco_FollowCommonSource(uint32_t follower_ch, uint32_t master_ch,
                                   int32_t master_phase_error, uint64_t frame_start_sample);
static int32_t Scale_PhaseErrorByFreq(int32_t phase_error, uint32_t dst_hz, uint32_t src_hz);
static int32_t Scale_StepCorrection(uint32_t follower_ch, uint32_t master_ch);
static uint32_t PhaseOffsetDeg_To_Q32(int32_t deg);
static uint32_t PhaseStep_Q32(uint32_t freq_hz);

static void Debug_Printf(const char *fmt, ...)
{
  char buf[128];
  va_list args;
  int len;

  va_start(args, fmt);
  len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (len < 0)
  {
    return;
  }
  if ((uint32_t)len >= sizeof(buf))
  {
    len = (int)sizeof(buf) - 1;
  }

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, SIGSEP_UART_TIMEOUT_MS);
}

static void Startup_Print(void)
{
  Debug_Printf("\r\nsignal separation start\r\n");
}

static const char *WaveName(WaveType wave)
{
  return (wave == WAVE_TRIANGLE) ? "tri" : "sin";
}

static void Print_SeparationResult(const SignalComponent comp[2])
{
  Debug_Printf("A: %luHz %s | B: %luHz %s\r\n",
               (unsigned long)comp[0].freq_hz,
               WaveName(comp[0].wave),
               (unsigned long)comp[1].freq_hz,
               WaveName(comp[1].wave));
}

static uint32_t Timer2_GetClockHz(void)
{
  uint32_t clk = HAL_RCC_GetPCLK1Freq();

  if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != RCC_D2CFGR_D2PPRE1_DIV1)
  {
    clk *= 2U;
  }

  return clk;
}

static void Timer2_SetSampleRate(uint32_t sample_rate_hz)
{
  uint32_t tim_clk = Timer2_GetClockHz();
  uint32_t arr;

  if ((sample_rate_hz == 0U) || (tim_clk < sample_rate_hz))
  {
    Error_Handler();
  }

  arr = (tim_clk / sample_rate_hz) - 1U;
  __HAL_TIM_DISABLE(&htim2);
  __HAL_TIM_SET_PRESCALER(&htim2, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  htim2.Instance->EGR = TIM_EVENTSOURCE_UPDATE;
}

static void CORDIC_SelectSine(void)
{
  CORDIC_ConfigTypeDef cfg;

  if (cordic_mode == CORDIC_MODE_SINE)
  {
    return;
  }

  cfg.Function = CORDIC_FUNCTION_SINE;
  cfg.Precision = CORDIC_PRECISION_6CYCLES;
  cfg.Scale = CORDIC_SCALE_0;
  cfg.NbWrite = CORDIC_NBWRITE_1;
  cfg.NbRead = CORDIC_NBREAD_1;
  cfg.InSize = CORDIC_INSIZE_32BITS;
  cfg.OutSize = CORDIC_OUTSIZE_32BITS;
  if (HAL_CORDIC_Configure(&hcordic, &cfg) != HAL_OK)
  {
    Error_Handler();
  }
  cordic_mode = CORDIC_MODE_SINE;
}

static void CORDIC_SelectPhase(void)
{
  CORDIC_ConfigTypeDef cfg;

  if (cordic_mode == CORDIC_MODE_PHASE)
  {
    return;
  }

  cfg.Function = CORDIC_FUNCTION_PHASE;
  cfg.Precision = CORDIC_PRECISION_6CYCLES;
  cfg.Scale = CORDIC_SCALE_0;
  cfg.NbWrite = CORDIC_NBWRITE_2;
  cfg.NbRead = CORDIC_NBREAD_1;
  cfg.InSize = CORDIC_INSIZE_32BITS;
  cfg.OutSize = CORDIC_OUTSIZE_32BITS;
  if (HAL_CORDIC_Configure(&hcordic, &cfg) != HAL_OK)
  {
    Error_Handler();
  }
  cordic_mode = CORDIC_MODE_PHASE;
}

static float32_t CORDIC_SinFromPhase(uint32_t phase)
{
  int32_t in = (int32_t)phase;
  int32_t out = 0;

  CORDIC_SelectSine();
  if (HAL_CORDIC_Calculate(&hcordic, &in, &out, 1U, 10U) != HAL_OK)
  {
    Error_Handler();
  }

  return ((float32_t)out) / 2147483648.0f;
}

static uint32_t CORDIC_PhaseFromIQ(float32_t i_part, float32_t q_part)
{
  float32_t abs_i = (i_part >= 0.0f) ? i_part : -i_part;
  float32_t abs_q = (q_part >= 0.0f) ? q_part : -q_part;
  float32_t scale = (abs_i > abs_q) ? abs_i : abs_q;
  int32_t in[2];
  int32_t out = 0;

  if (scale < 1.0f)
  {
    return 0U;
  }

  in[0] = (int32_t)((i_part / scale) * 1073741824.0f);
  in[1] = (int32_t)((q_part / scale) * 1073741824.0f);

  CORDIC_SelectPhase();
  if (HAL_CORDIC_Calculate(&hcordic, in, &out, 1U, 10U) != HAL_OK)
  {
    Error_Handler();
  }

  return (uint32_t)out;
}

static void CORDIC_PrepareTables(void)
{
  uint32_t freq_hz;
  uint32_t phase_step;
  uint32_t i;

  step_sin[0] = 0.0f;
  step_cos[0] = 1.0f;
  for (i = 1U; i <= SIGSEP_MAX_BIN; i++)
  {
    freq_hz = i * SIGSEP_FREQ_STEP_HZ;
    phase_step = (uint32_t)(((uint64_t)freq_hz * 4294967296ULL) / SIGSEP_SAMPLE_RATE_HZ);
    step_sin[i] = CORDIC_SinFromPhase(phase_step);
    step_cos[i] = CORDIC_SinFromPhase(phase_step + 0x40000000UL);
  }

  for (i = 0U; i < SIGSEP_SINE_LUT_SIZE; i++)
  {
    uint32_t phase = i << SIGSEP_SINE_LUT_SHIFT;
    float32_t s = CORDIC_SinFromPhase(phase);
    sine_lut[i] = (int16_t)(s * 32767.0f);
  }
}

static void Fill_DacMidscale(void)
{
  uint32_t i;

  for (i = 0U; i < SIGSEP_DAC_FRAME_LEN; i++)
  {
    dac1_dma_buf[i] = SIGSEP_DAC_MID;
    dac2_dma_buf[i] = SIGSEP_DAC_MID;
  }
}

static float32_t Frame_Mean(const uint16_t *samples)
{
  float32_t mean = 0.0f;
  uint32_t i;

  for (i = 0U; i < SIGSEP_FRAME_LEN; i++)
  {
    mean += (float32_t)samples[i];
  }

  return mean / (float32_t)SIGSEP_FRAME_LEN;
}

static void Measure_Component(const uint16_t *samples, float32_t mean, uint32_t freq_hz,
                              float32_t *amp, uint32_t *phase_q32)
{
  uint32_t bin = freq_hz / SIGSEP_FREQ_STEP_HZ;
  float32_t s = 0.0f;
  float32_t c = 1.0f;
  float32_t sum_s = 0.0f;
  float32_t sum_c = 0.0f;
  float32_t mag = 0.0f;
  float32_t x;
  float32_t next_c;
  uint32_t i;

  if ((bin == 0U) || (bin > SIGSEP_MAX_BIN) || ((freq_hz % SIGSEP_FREQ_STEP_HZ) != 0U))
  {
    *amp = 0.0f;
    *phase_q32 = 0U;
    return;
  }

  for (i = 0U; i < SIGSEP_FRAME_LEN; i++)
  {
    x = ((float32_t)samples[i]) - mean;
    sum_s += x * s;
    sum_c += x * c;

    next_c = (c * step_cos[bin]) - (s * step_sin[bin]);
    s = (s * step_cos[bin]) + (c * step_sin[bin]);
    c = next_c;
  }

  (void)arm_sqrt_f32((sum_s * sum_s) + (sum_c * sum_c), &mag);
  *amp = (2.0f * mag) / (float32_t)SIGSEP_FRAME_LEN;
  *phase_q32 = CORDIC_PhaseFromIQ(sum_s, sum_c);
}

static float32_t Correlate_Amplitude(const uint16_t *samples, float32_t mean, uint32_t freq_hz)
{
  return Measure_AmplitudeOnly(samples, mean, freq_hz);
}

static float32_t Measure_AmplitudeOnly(const uint16_t *samples, float32_t mean, uint32_t freq_hz)
{
  uint32_t bin = freq_hz / SIGSEP_FREQ_STEP_HZ;
  float32_t s = 0.0f;
  float32_t c = 1.0f;
  float32_t sum_s = 0.0f;
  float32_t sum_c = 0.0f;
  float32_t mag = 0.0f;
  float32_t x;
  float32_t next_c;
  uint32_t i;

  if ((bin == 0U) || (bin > SIGSEP_MAX_BIN) || ((freq_hz % SIGSEP_FREQ_STEP_HZ) != 0U))
  {
    return 0.0f;
  }

  for (i = 0U; i < SIGSEP_FRAME_LEN; i++)
  {
    x = ((float32_t)samples[i]) - mean;
    sum_s += x * s;
    sum_c += x * c;

    next_c = (c * step_cos[bin]) - (s * step_sin[bin]);
    s = (s * step_cos[bin]) + (c * step_sin[bin]);
    c = next_c;
  }

  (void)arm_sqrt_f32((sum_s * sum_s) + (sum_c * sum_c), &mag);
  return (2.0f * mag) / (float32_t)SIGSEP_FRAME_LEN;
}

static WaveType Detect_WaveType(const uint16_t *samples, float32_t mean, uint32_t freq_hz, uint32_t other_hz,
                                float32_t fundamental_amp)
{
  float32_t h3_amp = 0.0f;
  float32_t h5_amp = 0.0f;

  if (fundamental_amp < SIGSEP_MIN_VALID_ADC_AMP)
  {
    return WAVE_SINE;
  }

  if ((freq_hz * 3U) <= (SIGSEP_SAMPLE_RATE_HZ / 2U))
  {
    h3_amp = Correlate_Amplitude(samples, mean, freq_hz * 3U);
  }
  if ((freq_hz * 5U) <= (SIGSEP_SAMPLE_RATE_HZ / 2U))
  {
    h5_amp = Correlate_Amplitude(samples, mean, freq_hz * 5U);
  }

  if ((other_hz == (freq_hz * 3U)) && (h5_amp > (fundamental_amp * SIGSEP_TRI_H5_RATIO)))
  {
    return WAVE_TRIANGLE;
  }
  if ((other_hz != (freq_hz * 3U)) && (h3_amp > (fundamental_amp * SIGSEP_TRI_H3_RATIO)))
  {
    return WAVE_TRIANGLE;
  }

  return WAVE_SINE;
}

static uint8_t Analyze_Frame(const uint16_t *samples, SignalComponent out[2])
{
  float32_t mean;
  float32_t amp[SIGSEP_FREQ_COUNT];
  uint32_t phase[SIGSEP_FREQ_COUNT];
  uint32_t freq_hz;
  uint32_t best0 = 0U;
  uint32_t best1 = 1U;
  uint32_t t;
  uint32_t i;

  mean = Frame_Mean(samples);

  for (i = 0U; i < SIGSEP_FREQ_COUNT; i++)
  {
    freq_hz = SIGSEP_FREQ_MIN_HZ + (i * SIGSEP_FREQ_STEP_HZ);
    Measure_Component(samples, mean, freq_hz, &amp[i], &phase[i]);
    identify_amp_acc[i] += amp[i];
  }

  identify_frame_count++;
  if (identify_frame_count < SIGSEP_IDENTIFY_FRAMES)
  {
    return 0U;
  }

  for (i = 0U; i < SIGSEP_FREQ_COUNT; i++)
  {
    amp[i] = identify_amp_acc[i] / (float32_t)identify_frame_count;
    identify_amp_acc[i] = 0.0f;
  }
  identify_frame_count = 0U;

  if (amp[best1] > amp[best0])
  {
    best0 = 1U;
    best1 = 0U;
  }

  for (i = 2U; i < SIGSEP_FREQ_COUNT; i++)
  {
    if (amp[i] > amp[best0])
    {
      best1 = best0;
      best0 = i;
    }
    else if (amp[i] > amp[best1])
    {
      best1 = i;
    }
  }

  if (best0 > best1)
  {
    t = best0;
    best0 = best1;
    best1 = t;
  }

  out[0].freq_index = (uint8_t)best0;
  out[0].freq_hz = SIGSEP_FREQ_MIN_HZ + (best0 * SIGSEP_FREQ_STEP_HZ);
  out[0].amp_adc = amp[best0];
  out[0].amp_dac = out[0].amp_adc * SIGSEP_ADC_TO_DAC_SCALE;
  out[0].phase_q32 = phase[best0];

  out[1].freq_index = (uint8_t)best1;
  out[1].freq_hz = SIGSEP_FREQ_MIN_HZ + (best1 * SIGSEP_FREQ_STEP_HZ);
  out[1].amp_adc = amp[best1];
  out[1].amp_dac = out[1].amp_adc * SIGSEP_ADC_TO_DAC_SCALE;
  out[1].phase_q32 = phase[best1];

  for (i = 0U; i < 2U; i++)
  {
    if (out[i].amp_dac < SIGSEP_DEFAULT_DAC_AMP)
    {
      out[i].amp_dac = SIGSEP_DEFAULT_DAC_AMP;
    }
    if (out[i].amp_dac > SIGSEP_MAX_DAC_AMP)
    {
      out[i].amp_dac = SIGSEP_MAX_DAC_AMP;
    }
  }

  out[0].wave = Detect_WaveType(samples, mean, out[0].freq_hz, out[1].freq_hz, out[0].amp_adc);
  out[1].wave = Detect_WaveType(samples, mean, out[1].freq_hz, out[0].freq_hz, out[1].amp_adc);

  return 1U;
}

static void Build_DacSamples(uint16_t *dst, const SignalComponent *comp, uint32_t start_phase,
                             uint32_t phase_step, uint32_t len)
{
  uint32_t phase = start_phase;
  uint32_t q;
  uint32_t frac;
  int32_t y_q15;
  int32_t amp = (int32_t)(comp->amp_dac + 0.5f);
  int32_t code;
  uint32_t i;

  for (i = 0U; i < len; i++)
  {
    if (comp->wave == WAVE_TRIANGLE)
    {
      q = phase >> 30;
      frac = (phase & 0x3FFFFFFFUL) >> 15;

      if (q == 0U)
      {
        y_q15 = (int32_t)frac;
      }
      else if (q == 1U)
      {
        y_q15 = 32767 - (int32_t)frac;
      }
      else if (q == 2U)
      {
        y_q15 = -(int32_t)frac;
      }
      else
      {
        y_q15 = -32767 + (int32_t)frac;
      }
    }
    else
    {
      y_q15 = sine_lut[phase >> SIGSEP_SINE_LUT_SHIFT];
    }

    code = (int32_t)SIGSEP_DAC_MID + ((amp * y_q15) >> 15);
    if (code < 0)
    {
      code = 0;
    }
    if (code > (int32_t)SIGSEP_DAC_MAX)
    {
      code = (int32_t)SIGSEP_DAC_MAX;
    }

    dst[i] = (uint16_t)code;
    phase += phase_step;
  }
}

static uint32_t PhaseStep_Q32(uint32_t freq_hz)
{
  return (uint32_t)(((uint64_t)freq_hz * 4294967296ULL) / SIGSEP_SAMPLE_RATE_HZ);
}

static uint32_t PhaseOffsetDeg_To_Q32(int32_t deg)
{
  return (uint32_t)(((int64_t)deg * 4294967296LL) / 360LL);
}

static void Nco_Init(uint32_t ch, const SignalComponent *comp, uint64_t sample_start)
{
  nco_state[ch].nominal_step = PhaseStep_Q32(comp->freq_hz);
  nco_state[ch].step_corr = 0;
  nco_state[ch].integrator = 0;
  nco_state[ch].phase_ref = comp->phase_q32;
  nco_state[ch].sample_ref = sample_start;
  nco_state[ch].last_error = 0;
}

static uint32_t Nco_Step(uint32_t ch)
{
  return nco_state[ch].nominal_step + (uint32_t)nco_state[ch].step_corr;
}

static uint32_t Nco_PhaseAtSample(uint32_t ch, uint64_t sample)
{
  uint64_t delta = 0U;

  if (sample >= nco_state[ch].sample_ref)
  {
    delta = sample - nco_state[ch].sample_ref;
  }

  return nco_state[ch].phase_ref + (uint32_t)((uint64_t)Nco_Step(ch) * delta);
}

static int32_t Nco_UpdateLock(uint32_t ch, uint32_t measured_phase, uint64_t frame_start_sample)
{
  uint32_t predicted_phase = Nco_PhaseAtSample(ch, frame_start_sample);
  int32_t phase_error = (int32_t)(measured_phase - predicted_phase);
  int64_t integrator = ((nco_state[ch].integrator *
                         (int64_t)SIGSEP_PLL_INTEGRATOR_LEAK_NUM) /
                        65536LL) +
                       (int64_t)phase_error;
  int64_t correction;
  int64_t integrator_limit;
  int32_t max_corr;

  max_corr = (int32_t)(nco_state[ch].nominal_step / SIGSEP_PLL_MAX_CORR_DIV);
  if (max_corr < 1)
  {
    max_corr = 1;
  }

  integrator_limit = (int64_t)max_corr *
                     (int64_t)SIGSEP_FRAME_LEN *
                     (int64_t)SIGSEP_PLL_STEP_KI_DIV;
  if (integrator_limit > SIGSEP_PLL_INTEGRATOR_LIMIT)
  {
    integrator_limit = SIGSEP_PLL_INTEGRATOR_LIMIT;
  }

  if (integrator > integrator_limit)
  {
    integrator = integrator_limit;
  }
  if (integrator < -integrator_limit)
  {
    integrator = -integrator_limit;
  }

  correction = ((int64_t)phase_error / (int64_t)(SIGSEP_FRAME_LEN * SIGSEP_PLL_STEP_KP_DIV)) +
               (integrator / (int64_t)(SIGSEP_FRAME_LEN * SIGSEP_PLL_STEP_KI_DIV));

  if (correction > (int64_t)max_corr)
  {
    correction = max_corr;
  }
  if (correction < -(int64_t)max_corr)
  {
    correction = -(int64_t)max_corr;
  }

  nco_state[ch].integrator = integrator;
  nco_state[ch].step_corr = (int32_t)correction;
  nco_state[ch].phase_ref = predicted_phase +
                            (uint32_t)(phase_error /
                                       (int32_t)(1UL << SIGSEP_PLL_PHASE_KP_SHIFT));
  nco_state[ch].sample_ref = frame_start_sample;
  nco_state[ch].last_error = phase_error;

  return phase_error;
}

static int32_t Scale_PhaseErrorByFreq(int32_t phase_error, uint32_t dst_hz, uint32_t src_hz)
{
  int64_t scaled;

  if (src_hz == 0U)
  {
    return 0;
  }

  scaled = ((int64_t)phase_error * (int64_t)dst_hz) / (int64_t)src_hz;
  return (int32_t)((uint32_t)scaled);
}

static int32_t Scale_StepCorrection(uint32_t follower_ch, uint32_t master_ch)
{
  int64_t scaled;
  int32_t max_corr;

  if (nco_state[master_ch].nominal_step == 0U)
  {
    return 0;
  }

  scaled = ((int64_t)nco_state[master_ch].step_corr *
            (int64_t)nco_state[follower_ch].nominal_step) /
           (int64_t)nco_state[master_ch].nominal_step;

  max_corr = (int32_t)(nco_state[follower_ch].nominal_step / SIGSEP_PLL_MAX_CORR_DIV);
  if (scaled > (int64_t)max_corr)
  {
    scaled = max_corr;
  }
  if (scaled < -(int64_t)max_corr)
  {
    scaled = -(int64_t)max_corr;
  }

  return (int32_t)scaled;
}

static void Nco_FollowCommonSource(uint32_t follower_ch, uint32_t master_ch,
                                   int32_t master_phase_error, uint64_t frame_start_sample)
{
  uint32_t predicted_phase = Nco_PhaseAtSample(follower_ch, frame_start_sample);
  int32_t follower_phase_error = Scale_PhaseErrorByFreq(master_phase_error,
                                                        active_comp[follower_ch].freq_hz,
                                                        active_comp[master_ch].freq_hz);

  nco_state[follower_ch].integrator = 0;
  nco_state[follower_ch].step_corr = Scale_StepCorrection(follower_ch, master_ch);
  nco_state[follower_ch].phase_ref = predicted_phase +
                                     (uint32_t)(follower_phase_error /
                                                (int32_t)(1UL << SIGSEP_PLL_PHASE_KP_SHIFT));
  nco_state[follower_ch].sample_ref = frame_start_sample;
  nco_state[follower_ch].last_error = follower_phase_error;
}

static void Fill_DacHalf(uint32_t half_index, uint64_t play_sample)
{
  uint32_t dst_offset = half_index * SIGSEP_DAC_HALF_LEN;
  uint32_t phase0 = Nco_PhaseAtSample(0U, play_sample) + PhaseOffsetDeg_To_Q32(SIGSEP_DAC1_PHASE_OFFSET_DEG);
  uint32_t phase1 = Nco_PhaseAtSample(1U, play_sample) + PhaseOffsetDeg_To_Q32(SIGSEP_DAC2_PHASE_OFFSET_DEG);

  Build_DacSamples(&dac1_dma_buf[dst_offset], &active_comp[0], phase0, Nco_Step(0U), SIGSEP_DAC_HALF_LEN);
  Build_DacSamples(&dac2_dma_buf[dst_offset], &active_comp[1], phase1, Nco_Step(1U), SIGSEP_DAC_HALF_LEN);
}

void SignalSeparation_Start(void)
{
  Timer2_SetSampleRate(SIGSEP_SAMPLE_RATE_HZ);
  CORDIC_PrepareTables();
  Fill_DacMidscale();

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_DMNGT, ADC_CONVERSIONDATA_DMA_CIRCULAR);

  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, SIGSEP_DAC_MID) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, SIGSEP_DAC_MID) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dac1_dma_buf, SIGSEP_DAC_FRAME_LEN, DAC_ALIGN_12B_R) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t *)dac2_dma_buf, SIGSEP_DAC_FRAME_LEN, DAC_ALIGN_12B_R) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, SIGSEP_ADC_DMA_LEN) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  Startup_Print();
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
}

void SignalSeparation_Task(void)
{
  int32_t offset;
  uint64_t ready_sample_start;
  uint64_t ready_play_sample[2];
  uint32_t ready_mask;
  SignalComponent result[2];
  float32_t mean;
  float32_t measured_amp[2];
  uint32_t measured_phase[2];
  float32_t target_amp_dac[2];
  uint8_t smooth_amp = 0U;
  uint32_t master_ch = (SIGSEP_PHASE_MASTER_CH == 0U) ? 0U : 1U;
  uint32_t follower_ch = master_ch ^ 1U;
  int32_t master_phase_error;

  __disable_irq();
  offset = adc_ready_offset;
  adc_ready_offset = -1;
  ready_sample_start = adc_ready_sample_start;
  ready_mask = dac_ready_mask;
  ready_play_sample[0] = dac_ready_play_sample[0];
  ready_play_sample[1] = dac_ready_play_sample[1];
  __enable_irq();

  if (separation_identified != 0U)
  {
    if ((ready_mask & 0x03U) == 0x03U)
    {
      __disable_irq();
      dac_ready_mask &= ~0x03U;
      __enable_irq();
      Fill_DacHalf(0U, ready_play_sample[0]);
    }
    if ((ready_mask & 0x0CU) == 0x0CU)
    {
      __disable_irq();
      dac_ready_mask &= ~0x0CU;
      __enable_irq();
      Fill_DacHalf(1U, ready_play_sample[1]);
    }
  }

  if (offset >= 0)
  {
    if (separation_identified == 0U)
    {
      if (Analyze_Frame(&adc_dma_buf[offset], result) == 0U)
      {
        return;
      }
      active_comp[0] = result[0];
      active_comp[1] = result[1];
      Nco_Init(0U, &active_comp[0], ready_sample_start);
      Nco_Init(1U, &active_comp[1], ready_sample_start);
      Print_SeparationResult(active_comp);
      separation_identified = 1U;
    }
    else
    {
      mean = Frame_Mean(&adc_dma_buf[offset]);
#if (SIGSEP_COMMON_SOURCE_LOCK != 0U)
      Measure_Component(&adc_dma_buf[offset], mean, active_comp[master_ch].freq_hz,
                        &measured_amp[master_ch], &measured_phase[master_ch]);
      measured_amp[follower_ch] = Measure_AmplitudeOnly(&adc_dma_buf[offset], mean,
                                                        active_comp[follower_ch].freq_hz);
      measured_phase[follower_ch] = Nco_PhaseAtSample(follower_ch, ready_sample_start);
#else
      Measure_Component(&adc_dma_buf[offset], mean, active_comp[0].freq_hz,
                        &measured_amp[0], &measured_phase[0]);
      Measure_Component(&adc_dma_buf[offset], mean, active_comp[1].freq_hz,
                        &measured_amp[1], &measured_phase[1]);
#endif
      active_comp[0].phase_q32 = measured_phase[0];
      active_comp[1].phase_q32 = measured_phase[1];
      active_comp[0].amp_adc += (measured_amp[0] - active_comp[0].amp_adc) / (float32_t)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
      active_comp[1].amp_adc += (measured_amp[1] - active_comp[1].amp_adc) / (float32_t)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
      smooth_amp = 1U;
#if (SIGSEP_COMMON_SOURCE_LOCK != 0U)
      master_phase_error = Nco_UpdateLock(master_ch, measured_phase[master_ch], ready_sample_start);
      Nco_FollowCommonSource(follower_ch, master_ch, master_phase_error, ready_sample_start);
#else
      (void)Nco_UpdateLock(0U, measured_phase[0], ready_sample_start);
      (void)Nco_UpdateLock(1U, measured_phase[1], ready_sample_start);
#endif
    }

    target_amp_dac[0] = active_comp[0].amp_adc * SIGSEP_ADC_TO_DAC_SCALE;
    target_amp_dac[1] = active_comp[1].amp_adc * SIGSEP_ADC_TO_DAC_SCALE;
    if (target_amp_dac[0] < SIGSEP_DEFAULT_DAC_AMP)
    {
      target_amp_dac[0] = SIGSEP_DEFAULT_DAC_AMP;
    }
    if (target_amp_dac[1] < SIGSEP_DEFAULT_DAC_AMP)
    {
      target_amp_dac[1] = SIGSEP_DEFAULT_DAC_AMP;
    }
    if (target_amp_dac[0] > SIGSEP_MAX_DAC_AMP)
    {
      target_amp_dac[0] = SIGSEP_MAX_DAC_AMP;
    }
    if (target_amp_dac[1] > SIGSEP_MAX_DAC_AMP)
    {
      target_amp_dac[1] = SIGSEP_MAX_DAC_AMP;
    }
    if (smooth_amp != 0U)
    {
      active_comp[0].amp_dac += (target_amp_dac[0] - active_comp[0].amp_dac) / (float32_t)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
      active_comp[1].amp_dac += (target_amp_dac[1] - active_comp[1].amp_dac) / (float32_t)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
    }
    else
    {
      active_comp[0].amp_dac = target_amp_dac[0];
      active_comp[1].amp_dac = target_amp_dac[1];
    }
  }
}

void SignalSeparation_RestartIdentify(void)
{
  uint32_t i;

  __disable_irq();
  separation_identified = 0U;
  adc_ready_offset = -1;
  dac_ready_mask = 0U;
  __enable_irq();

  identify_frame_count = 0U;
  for (i = 0U; i < SIGSEP_FREQ_COUNT; i++)
  {
    identify_amp_acc[i] = 0.0f;
  }

  Fill_DacMidscale();
}

uint8_t SignalSeparation_GetFrequencies(uint32_t *freq0_hz, uint32_t *freq1_hz)
{
  if (separation_identified == 0U)
  {
    return 0U;
  }

  if (freq0_hz != NULL)
  {
    *freq0_hz = active_comp[0].freq_hz;
  }
  if (freq1_hz != NULL)
  {
    *freq1_hz = active_comp[1].freq_hz;
  }

  return 1U;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_frame_count++;
    adc_sample_count += SIGSEP_FRAME_LEN;
    if (adc_ready_offset < 0)
    {
      adc_ready_offset = 0;
      adc_ready_sample_start = adc_sample_count - SIGSEP_FRAME_LEN;
    }
    else
    {
      adc_frame_overrun++;
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_frame_count++;
    adc_sample_count += SIGSEP_FRAME_LEN;
    if (adc_ready_offset < 0)
    {
      adc_ready_offset = (int32_t)SIGSEP_FRAME_LEN;
      adc_ready_sample_start = adc_sample_count - SIGSEP_FRAME_LEN;
    }
    else
    {
      adc_frame_overrun++;
    }
  }
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    /* CH1 is the DAC sample-time master; CH2 uses the same TIM2 trigger. */
    dac_sample_count += SIGSEP_DAC_HALF_LEN;
    if (separation_identified == 0U)
    {
      return;
    }
    dac_ready_play_sample[0] = dac_sample_count + SIGSEP_DAC_HALF_LEN;
    if ((dac_ready_mask & 0x01U) != 0U)
    {
      dac_half_overrun++;
    }
    dac_ready_mask |= 0x01U;
  }
}

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
  if ((hdac->Instance == DAC1) && (separation_identified != 0U))
  {
    /* CH2 only confirms that this half-buffer is no longer being read. */
    if ((dac_ready_mask & 0x02U) != 0U)
    {
      dac_half_overrun++;
    }
    dac_ready_mask |= 0x02U;
  }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    /* CH1 is the DAC sample-time master; CH2 uses the same TIM2 trigger. */
    dac_sample_count += SIGSEP_DAC_HALF_LEN;
    if (separation_identified == 0U)
    {
      return;
    }
    dac_ready_play_sample[1] = dac_sample_count + SIGSEP_DAC_HALF_LEN;
    if ((dac_ready_mask & 0x04U) != 0U)
    {
      dac_half_overrun++;
    }
    dac_ready_mask |= 0x04U;
  }
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
  if ((hdac->Instance == DAC1) && (separation_identified != 0U))
  {
    /* CH2 only confirms that this half-buffer is no longer being read. */
    if ((dac_ready_mask & 0x08U) != 0U)
    {
      dac_half_overrun++;
    }
    dac_ready_mask |= 0x08U;
  }
}
