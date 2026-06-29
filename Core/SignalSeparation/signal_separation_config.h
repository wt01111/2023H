#ifndef SIGNAL_SEPARATION_CONFIG_H
#define SIGNAL_SEPARATION_CONFIG_H

#define SIGSEP_SAMPLE_RATE_HZ          2500000U
#define SIGSEP_FRAME_LEN               500U
#define SIGSEP_ADC_DMA_LEN             (SIGSEP_FRAME_LEN * 2U)
#define SIGSEP_DAC_FRAME_LEN           2000U
#define SIGSEP_DAC_HALF_LEN            (SIGSEP_DAC_FRAME_LEN / 2U)

#define SIGSEP_SINE_LUT_BITS           10U
#define SIGSEP_SINE_LUT_SIZE           (1U << SIGSEP_SINE_LUT_BITS)
#define SIGSEP_SINE_LUT_SHIFT          (32U - SIGSEP_SINE_LUT_BITS)

#define SIGSEP_FREQ_MIN_HZ             10000U
#define SIGSEP_FREQ_STEP_HZ            5000U
#define SIGSEP_FREQ_COUNT              19U
#define SIGSEP_MAX_BIN                 100U
#define SIGSEP_IDENTIFY_FRAMES         4U

#define SIGSEP_DAC_MID                 2048U
#define SIGSEP_DAC_MAX                 4095U
#define SIGSEP_ADC_TO_DAC_SCALE        (4095.0f / 65535.0f)
#define SIGSEP_DEFAULT_DAC_AMP         700.0f
#define SIGSEP_MAX_DAC_AMP             1850.0f

/* Positive value advances this DAC channel relative to its locked input component. */
#define SIGSEP_DAC1_PHASE_OFFSET_DEG   0
#define SIGSEP_DAC2_PHASE_OFFSET_DEG   0

/*
 * When both separated components come from one coherent source, lock only one
 * phase loop and derive the other channel's correction from the same time error.
 * Channel 0 is A', channel 1 is B'.
 */
#define SIGSEP_COMMON_SOURCE_LOCK      1U
#define SIGSEP_PHASE_MASTER_CH         0U

#define SIGSEP_MIN_VALID_ADC_AMP       120.0f
#define SIGSEP_TRI_H3_RATIO            0.060f
#define SIGSEP_TRI_H5_RATIO            0.025f

#define SIGSEP_PLL_PHASE_KP_SHIFT      2U
#define SIGSEP_PLL_STEP_KP_DIV         20U
#define SIGSEP_PLL_STEP_KI_DIV         200U
#define SIGSEP_PLL_MAX_CORR_DIV        2000U
#define SIGSEP_PLL_INTEGRATOR_LIMIT    8589934592LL
#define SIGSEP_PLL_INTEGRATOR_LEAK_NUM 65535U

#define SIGSEP_AMP_SMOOTH_SHIFT        3U
#define SIGSEP_UART_TIMEOUT_MS         100U

#endif
