#ifndef BTRACK_CONFIG_H
#define BTRACK_CONFIG_H

#if !defined(USE_FFTW) && !defined(USE_KISSFFT)
#error "Need to specify an FFT library"
#endif

#ifdef USE_FFTW
#include "fftw3.h"
#endif

#ifdef USE_KISS_FFT
#include "kiss_fft.h"
#endif

#endif // BTRACK_CONFIG_H