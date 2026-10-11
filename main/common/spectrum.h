/* Shared on-device spectrum protocol. Acquisition callbacks return normalized
 * IQ10 words (I in bits 0..9, Q in 10..19) in stable CPU-owned memory. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef bool (*spectrum_acquire_fn)(unsigned n, unsigned rate,
                                   const uint32_t **words, unsigned *elapsed_us);
bool spectrum_command(const char *line, unsigned frequency_mhz, spectrum_acquire_fn acquire);

bool spectrum_fft_init(void);

/* Snapshot and continuous backends run exclusively. Reuse their large buffers
 * on SRAM-constrained targets; acquiring this workspace invalidates the cached
 * snapshot window so a later transport switch rebuilds it. */
typedef struct {
    int16_t *fft, *window;
    float *power;
    uint8_t *frame;
} spectrum_workspace_t;
spectrum_workspace_t spectrum_workspace(void);

/* Optional observer of each completed SPEC frame (power codes in natural FFT
 * order, valid only during the call). Runs on the capture task: keep it short. */
typedef void (*spectrum_sink_fn)(const uint8_t *codes, unsigned bins, unsigned rate_hz,
                                 unsigned frequency_mhz);
void spectrum_set_sink(spectrum_sink_fn sink);

/* One local spectrum without any transport: averages `averages` FFTs and writes
 * `bins` power codes in natural FFT order to out. Must not overlap spectrum_command. */
bool spectrum_local(unsigned rate, unsigned bins, unsigned averages,
                    spectrum_acquire_fn acquire, uint8_t *out, unsigned *rate_hz);
