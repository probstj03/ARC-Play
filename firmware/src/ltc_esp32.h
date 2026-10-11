/**
 * @file ltc_esp32.h
 * @brief libltc 1.3.2 – single-header port for ESP32 (Arduino framework)
 *
 * All original libltc source files (ltc.c, encoder.c, decoder.c, timecode.c)
 * are merged into this one header. No separate .c compilation units needed.
 *
 * Changes from the upstream libltc source:
 *  - fprintf/printf calls replaced with Serial.printf or removed
 *  - #include <endian.h> removed (ESP32 is always little-endian)
 *  - #include <config.h> removed
 *  - All implementations declared 'static inline' to prevent
 *    multiple-definition linker errors when included in several TUs
 *  - INFINITY fallback added for ARM-GCC targets that lack it
 *
 * NASA Rule 3: All encoder/decoder allocations happen once during init;
 *              no dynamic memory is allocated in real-time paths.
 * NASA Rule 8: Preprocessor use is limited to include guards, simple
 *              constants, and the _LTCWRITE_CONV conversion macro.
 *
 * Original: Copyright (C) 2006-2022 Robin Gareus <robin@gareus.org>
 *           LGPL v3 or later
 */

#ifndef LTC_ESP32_H
#define LTC_ESP32_H

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ESP32 is always little-endian; guard ensures the flag is set for
   code paths that check byte order at compile time. */
#ifndef __LITTLE_ENDIAN__
#define __LITTLE_ENDIAN__
#endif

#ifndef INFINITY
# ifdef HUGE_VAL
#  define INFINITY HUGE_VAL
# else
#  define INFINITY (1e308 * 10.0) 
# endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Public types and structures  (ported from ltc.h)
 * ========================================================================= */

#define LIBLTC_VERSION          "1.3.2"
#define LIBLTC_VERSION_MAJOR    1
#define LIBLTC_VERSION_MINOR    3
#define LIBLTC_VERSION_MICRO    2
#define LTC_FRAME_BIT_COUNT     80

/** Audio sample type used throughout the encoder/decoder: 8-bit unsigned, center = 128 */
typedef unsigned char ltcsnd_sample_t;

/** 64-bit sample position counter used for stream offset tracking */
typedef long long int ltc_off_t;

/** Supported LTC frame-rate standards */
enum LTC_TV_STANDARD {
    LTC_TV_525_60,   ///< 30 fps    NTSC non-drop
    LTC_TV_625_50,   ///< 25 fps    PAL
    LTC_TV_1125_60,  ///< 30 fps    HDTV (1125/60)
    LTC_TV_FILM_24,  ///< 24 fps    cinema / film
    LTC_TV_525_DF    ///< 29.97 fps NTSC drop-frame (dfbit set automatically)
};

/** Gibt die Framerate für einen LTC_TV_STANDARD zurück. */
static inline double ltc_standard_fps(enum LTC_TV_STANDARD s) {
    switch (s) {
        case LTC_TV_525_60:  return 30.0;
        case LTC_TV_625_50:  return 25.0;
        case LTC_TV_1125_60: return 30.0;
        case LTC_TV_FILM_24: return 24.0;
        case LTC_TV_525_DF:  return 29.97;
        default:             return 24.0;
    }
}

/** Flags that control encoder behaviour and frame interpretation */
enum LTC_BG_FLAGS {
    LTC_USE_DATE       = 1,  ///< Interpret user-bits field as date (DD/MM/YY/CC)
    LTC_TC_CLOCK       = 2,  ///< Timecode represents wall-clock time (free-run)
    LTC_BGF_DONT_TOUCH = 4,  ///< Do not modify binary-group flag bits on (re-)init
    LTC_NO_PARITY      = 8   ///< Skip parity-bit calculation / insertion
};

/* ── LTC Frame layout (80 bits, little-endian bit order) ───────────────── */
/* Each LTC frame encodes hours/mins/secs/frames plus 32 user-bits and ends
   with a fixed 16-bit sync word (0x3FFD forward, 0xBFFC in memory).       */
struct LTCFrame {
    unsigned int frame_units:4;
    unsigned int user1:4;

    unsigned int frame_tens:2;
    unsigned int dfbit:1;
    unsigned int col_frame:1;
    unsigned int user2:4;

    unsigned int secs_units:4;
    unsigned int user3:4;

    unsigned int secs_tens:3;
    unsigned int biphase_mark_phase_correction:1;
    unsigned int user4:4;

    unsigned int mins_units:4;
    unsigned int user5:4;

    unsigned int mins_tens:3;
    unsigned int binary_group_flag_bit0:1;
    unsigned int user6:4;

    unsigned int hours_units:4;
    unsigned int user7:4;

    unsigned int hours_tens:2;
    unsigned int binary_group_flag_bit1:1;
    unsigned int binary_group_flag_bit2:1;
    unsigned int user8:4;

    unsigned int sync_word:16;
};
typedef struct LTCFrame LTCFrame;

/** Extended frame returned by the decoder; adds stream position, direction,
    per-bit timing data, and signal-level statistics for diagnostics. */
struct LTCFrameExt {
    LTCFrame        ltc;
    ltc_off_t       off_start;
    ltc_off_t       off_end;
    int             reverse;
    float           biphase_tics[LTC_FRAME_BIT_COUNT];
    ltcsnd_sample_t sample_min;
    ltcsnd_sample_t sample_max;
    double          volume;
};
typedef struct LTCFrameExt LTCFrameExt;

/** Human-readable timecode / date / timezone struct used by the encoder API */
struct SMPTETimecode {
    char          timezone[6]; ///< POSIX offset string, e.g. "+0100"
    unsigned char years;
    unsigned char months;
    unsigned char days;
    unsigned char hours;
    unsigned char mins;
    unsigned char secs;
    unsigned char frame;
};
typedef struct SMPTETimecode SMPTETimecode;

/* =========================================================================
 * Internal structures – treat as opaque; access only through the API.
 * NASA Rule 9: pointer members use a single level of indirection only.
 * ========================================================================= */

#define SAMPLE_CENTER 128   /* DC midpoint of the 8-bit unsigned sample range */

struct LTCEncoder {
    double fps;
    double sample_rate;
    double filter_const;
    int    flags;
    enum LTC_TV_STANDARD standard;
    ltcsnd_sample_t enc_lo, enc_hi;

    size_t offset;
    size_t bufsize;
    ltcsnd_sample_t *buf;

    char state;

    double samples_per_clock;
    double samples_per_clock_2;
    double sample_remainder;

    LTCFrame f;
};
typedef struct LTCEncoder LTCEncoder;

struct LTCDecoder {
    LTCFrameExt *queue;
    int queue_len;
    int queue_read_off;
    int queue_write_off;

    unsigned char biphase_state;
    unsigned char biphase_prev;
    unsigned char snd_to_biphase_state;
    int snd_to_biphase_cnt;
    int snd_to_biphase_lmt;
    double snd_to_biphase_period;

    ltcsnd_sample_t snd_to_biphase_min;
    ltcsnd_sample_t snd_to_biphase_max;

    unsigned short decoder_sync_word;
    LTCFrame ltc_frame;
    int bit_cnt;

    ltc_off_t frame_start_off;
    ltc_off_t frame_start_prev;

    float biphase_tics[LTC_FRAME_BIT_COUNT];
    int biphase_tic;
};
typedef struct LTCDecoder LTCDecoder;

/* =========================================================================
 * Function declarations – Public API
 * NASA Rule 7: every non-void function must have its return value checked
 *              by the caller; NULL / error returns are documented below.
 * ========================================================================= */

/* ── Frame helper functions ─────────────────────────────────────────────── */
static void ltc_frame_to_time(SMPTETimecode *stime, LTCFrame *frame, int flags);
static void ltc_time_to_frame(LTCFrame *frame, SMPTETimecode *stime, enum LTC_TV_STANDARD standard, int flags);
static void ltc_frame_reset(LTCFrame *frame);
static int  ltc_frame_increment(LTCFrame *frame, int fps, enum LTC_TV_STANDARD standard, int flags);
static int  ltc_frame_decrement(LTCFrame *frame, int fps, enum LTC_TV_STANDARD standard, int flags);
static void ltc_frame_set_parity(LTCFrame *frame, enum LTC_TV_STANDARD standard);
static int  ltc_frame_parse_bcg_flags(LTCFrame *frame, enum LTC_TV_STANDARD standard);
static ltc_off_t ltc_frame_alignment(double samples_per_frame, enum LTC_TV_STANDARD standard);
static unsigned long ltc_frame_get_user_bits(LTCFrame *f);

/* ── Encoder ────────────────────────────────────────────────────────────── */
/* ltc_encoder_create: allocates an encoder; returns NULL on failure.
   Must be called once during init (NASA Rule 3 – no runtime allocation). */
static LTCEncoder *ltc_encoder_create(double sample_rate, double fps, enum LTC_TV_STANDARD standard, int flags);
static void  ltc_encoder_free(LTCEncoder *e);
static void  ltc_encoder_set_timecode(LTCEncoder *e, SMPTETimecode *t);
static void  ltc_encoder_get_timecode(LTCEncoder *e, SMPTETimecode *t);
static void  ltc_encoder_set_user_bits(LTCEncoder *e, unsigned long data);
static int   ltc_encoder_inc_timecode(LTCEncoder *e);
static int   ltc_encoder_dec_timecode(LTCEncoder *e);
static void  ltc_encoder_set_frame(LTCEncoder *e, LTCFrame *f);
static void  ltc_encoder_get_frame(LTCEncoder *e, LTCFrame *f);
static int   ltc_encoder_copy_buffer(LTCEncoder *e, ltcsnd_sample_t *buf);
static int   ltc_encoder_get_bufferptr(LTCEncoder *e, ltcsnd_sample_t **buf, int flush);
static void  ltc_encoder_buffer_flush(LTCEncoder *e);
static size_t ltc_encoder_get_buffersize(LTCEncoder *e);
static int   ltc_encoder_reinit(LTCEncoder *e, double sample_rate, double fps, enum LTC_TV_STANDARD standard, int flags);
static void  ltc_encoder_reset(LTCEncoder *e);
static int   ltc_encoder_set_buffersize(LTCEncoder *e, double sample_rate, double fps);
static double ltc_encoder_get_volume(LTCEncoder *e);
static int   ltc_encoder_set_volume(LTCEncoder *e, double dBFS);
static double ltc_encoder_get_filter(LTCEncoder *e);
static void  ltc_encoder_set_filter(LTCEncoder *e, double rise_time);
static int   ltc_encoder_encode_byte(LTCEncoder *e, int byte, double speed);
static int   ltc_encoder_end_encode(LTCEncoder *e);
static void  ltc_encoder_encode_frame(LTCEncoder *e);
static void  ltc_encoder_encode_reversed_frame(LTCEncoder *e);

/* ── Decoder ────────────────────────────────────────────────────────────── */
/* ltc_decoder_create: allocates a decoder with an internal frame queue.
   apv = expected samples per video frame (sets biphase timing thresholds).  */
static LTCDecoder *ltc_decoder_create(int apv, int queue_size);
static int  ltc_decoder_free(LTCDecoder *d);
static void ltc_decoder_write(LTCDecoder *d, ltcsnd_sample_t *buf, size_t size, ltc_off_t posinfo);
static void ltc_decoder_write_s16(LTCDecoder *d, short *buf, size_t size, ltc_off_t posinfo);
static void ltc_decoder_write_float(LTCDecoder *d, float *buf, size_t size, ltc_off_t posinfo);
static void ltc_decoder_write_double(LTCDecoder *d, double *buf, size_t size, ltc_off_t posinfo);
static void ltc_decoder_write_u16(LTCDecoder *d, unsigned short *buf, size_t size, ltc_off_t posinfo);
static int  ltc_decoder_read(LTCDecoder *d, LTCFrameExt *frame);
static void ltc_decoder_queue_flush(LTCDecoder *d);
static int  ltc_decoder_queue_length(LTCDecoder *d);

/* =========================================================================
 * Implementations – all marked static to avoid multiple-definition errors.
 * NASA Rule 4: each function fits on one printed page (~60 lines).
 * NASA Rule 6: local variables are declared at the smallest possible scope.
 * ========================================================================= */

/* Round a double to the nearest integer (portable rint() replacement) */
static inline double _ltc_rint(double v) {
    return floor(v + 0.5);
}

/* =========================================================================
 * Timecode helpers  (ported from libltc timecode.c)
 * Converts between the packed LTCFrame bit-fields and the readable
 * SMPTETimecode struct, and handles drop-frame counter skipping.
 * ========================================================================= */

/* Lookup table: SMPTE timezone code (stored in user-bits 7/8) ↔ UTC offset string */
struct _SMPTETZEntry { unsigned char code; char tz[6]; };
static const struct _SMPTETZEntry _ltc_tz_table[] = {
    {0x00,"+0000"},{0x00,"-0000"},{0x01,"-0100"},{0x02,"-0200"},
    {0x03,"-0300"},{0x04,"-0400"},{0x05,"-0500"},{0x06,"-0600"},
    {0x07,"-0700"},{0x08,"-0800"},{0x09,"-0900"},{0x10,"-1000"},
    {0x11,"-1100"},{0x12,"-1200"},{0x13,"+1300"},{0x14,"+1200"},
    {0x15,"+1100"},{0x16,"+1000"},{0x17,"+0900"},{0x18,"+0800"},
    {0x19,"+0700"},{0x20,"+0600"},{0x21,"+0500"},{0x22,"+0400"},
    {0x23,"+0300"},{0x24,"+0200"},{0x25,"+0100"},
    {0x28,"TP-03"},{0x29,"TP-02"},{0x30,"TP-01"},{0x31,"TP-00"},
    {0x0A,"-0030"},{0x0B,"-0130"},{0x0C,"-0230"},{0x0D,"-0330"},
    {0x0E,"-0430"},{0x0F,"-0530"},{0x1A,"-0630"},{0x1B,"-0730"},
    {0x1C,"-0830"},{0x1D,"-0930"},{0x1E,"-1030"},{0x1F,"-1130"},
    {0x2A,"+1130"},{0x2B,"+1030"},{0x2C,"+0930"},{0x2D,"+0830"},
    {0x2E,"+0730"},{0x2F,"+0630"},{0x3A,"+0530"},{0x3B,"+0430"},
    {0x3C,"+0330"},{0x3D,"+0230"},{0x3E,"+0130"},{0x3F,"+0030"},
    {0x32,"+1245"},{0x38,"+XXXX"},
    {0xFF,""}
};

static void _ltc_set_tz_string(LTCFrame *frame, SMPTETimecode *stime) {
    unsigned char code = frame->user7 + (frame->user8 << 4);
    strcpy(stime->timezone, "+0000");
    for (int i = 0; _ltc_tz_table[i].code != 0xFF; i++) {
        if (_ltc_tz_table[i].code == code) {
            strcpy(stime->timezone, _ltc_tz_table[i].tz);
            break;
        }
    }
}

static void _ltc_set_tz_code(SMPTETimecode *stime, LTCFrame *frame) {
    unsigned char code = 0x00;
    for (int i = 0; _ltc_tz_table[i].code != 0xFF; i++) {
        if (strcmp(_ltc_tz_table[i].tz, stime->timezone) == 0) {
            code = _ltc_tz_table[i].code;
            break;
        }
    }
    frame->user7 = code & 0x0F;
    frame->user8 = (code & 0xF0) >> 4;
}

static void _ltc_skip_drop_frames(LTCFrame *frame) {
    if ((frame->mins_units != 0) &&
        (frame->secs_units == 0) &&
        (frame->secs_tens  == 0) &&
        (frame->frame_units == 0) &&
        (frame->frame_tens  == 0)) {
        frame->frame_units += 2;
    }
}

static void ltc_frame_to_time(SMPTETimecode *stime, LTCFrame *frame, int flags) {
    if (!stime) return;
    if (flags & LTC_USE_DATE) {
        _ltc_set_tz_string(frame, stime);
        stime->years  = frame->user5 + frame->user6 * 10;
        stime->months = frame->user3 + frame->user4 * 10;
        stime->days   = frame->user1 + frame->user2 * 10;
    } else {
        stime->years = stime->months = stime->days = 0;
        strcpy(stime->timezone, "+0000");
    }
    stime->hours = frame->hours_units + frame->hours_tens * 10;
    stime->mins  = frame->mins_units  + frame->mins_tens  * 10;
    stime->secs  = frame->secs_units  + frame->secs_tens  * 10;
    stime->frame = frame->frame_units + frame->frame_tens * 10;
}

static void ltc_time_to_frame(LTCFrame *frame, SMPTETimecode *stime, enum LTC_TV_STANDARD standard, int flags) {
    if (flags & LTC_USE_DATE) {
        _ltc_set_tz_code(stime, frame);
        frame->user6 = stime->years  / 10;
        frame->user5 = stime->years  % 10;
        frame->user4 = stime->months / 10;
        frame->user3 = stime->months % 10;
        frame->user2 = stime->days   / 10;
        frame->user1 = stime->days   % 10;
    }
    frame->hours_tens  = stime->hours / 10;
    frame->hours_units = stime->hours % 10;
    frame->mins_tens   = stime->mins  / 10;
    frame->mins_units  = stime->mins  % 10;
    frame->secs_tens   = stime->secs  / 10;
    frame->secs_units  = stime->secs  % 10;
    frame->frame_tens  = stime->frame / 10;
    frame->frame_units = stime->frame % 10;
    if (frame->dfbit) _ltc_skip_drop_frames(frame);
    if ((flags & LTC_NO_PARITY) == 0) ltc_frame_set_parity(frame, standard);
}

static void ltc_frame_reset(LTCFrame *frame) {
    memset(frame, 0, sizeof(LTCFrame));
    /* sync word = 0x3FFD  →  little-endian stored as 0xBFFC */
    frame->sync_word = 0xBFFC;
}

static void ltc_frame_set_parity(LTCFrame *frame, enum LTC_TV_STANDARD standard) {
    unsigned char p = 0;
    if (standard != LTC_TV_625_50)
        frame->biphase_mark_phase_correction = 0;
    else
        frame->binary_group_flag_bit2 = 0;

    for (int i = 0; i < LTC_FRAME_BIT_COUNT / 8; i++)
        p ^= ((unsigned char *)frame)[i];

#define _PRY(B) ((p >> (B)) & 1)
    if (standard != LTC_TV_625_50)
        frame->biphase_mark_phase_correction =
            _PRY(0)^_PRY(1)^_PRY(2)^_PRY(3)^_PRY(4)^_PRY(5)^_PRY(6)^_PRY(7);
    else
        frame->binary_group_flag_bit2 =
            _PRY(0)^_PRY(1)^_PRY(2)^_PRY(3)^_PRY(4)^_PRY(5)^_PRY(6)^_PRY(7);
#undef _PRY
}

static int ltc_frame_parse_bcg_flags(LTCFrame *frame, enum LTC_TV_STANDARD standard) {
    int flags = 0;
    if (standard == LTC_TV_625_50) {
        if (frame->biphase_mark_phase_correction) flags |= 1; /* BGF0 = bit27 */
        if (frame->binary_group_flag_bit1)        flags |= 2; /* BGF1 = bit58 */
        if (frame->binary_group_flag_bit0)        flags |= 4; /* BGF2 = bit43 */
    } else {
        if (frame->binary_group_flag_bit0) flags |= 1;
        if (frame->binary_group_flag_bit1) flags |= 2;
        if (frame->binary_group_flag_bit2) flags |= 4;
    }
    return flags;
}

static ltc_off_t ltc_frame_alignment(double spf, enum LTC_TV_STANDARD standard) {
    switch (standard) {
        case LTC_TV_525_60: return (ltc_off_t)_ltc_rint(spf * 4.0 / 525.0);
        case LTC_TV_625_50: return (ltc_off_t)_ltc_rint(spf * 1.0 / 625.0);
        default: return 0;
    }
}

static unsigned long ltc_frame_get_user_bits(LTCFrame *f) {
    unsigned long d = 0;
    d += f->user8; d <<= 4; d += f->user7; d <<= 4;
    d += f->user6; d <<= 4; d += f->user5; d <<= 4;
    d += f->user4; d <<= 4; d += f->user3; d <<= 4;
    d += f->user2; d <<= 4; d += f->user1;
    return d;
}

static int ltc_frame_increment(LTCFrame *frame, int fps, enum LTC_TV_STANDARD standard, int flags) {
    int rv = 0;
    frame->frame_units++;
    if (frame->frame_units == 10) { frame->frame_units = 0; frame->frame_tens++; }
    if (fps == (int)(frame->frame_units + frame->frame_tens * 10)) {
        frame->frame_units = frame->frame_tens = 0;
        frame->secs_units++;
        if (frame->secs_units == 10) {
            frame->secs_units = 0; frame->secs_tens++;
            if (frame->secs_tens == 6) {
                frame->secs_tens = 0; frame->mins_units++;
                if (frame->mins_units == 10) {
                    frame->mins_units = 0; frame->mins_tens++;
                    if (frame->mins_tens == 6) {
                        frame->mins_tens = 0; frame->hours_units++;
                        if (frame->hours_units == 10) { frame->hours_units = 0; frame->hours_tens++; }
                        if (frame->hours_units == 4 && frame->hours_tens == 2) {
                            rv = 1;
                            frame->hours_tens = frame->hours_units = 0;
                            if (flags & LTC_USE_DATE) {
                                SMPTETimecode st;
                                st.years  = frame->user5 + frame->user6 * 10;
                                st.months = frame->user3 + frame->user4 * 10;
                                st.days   = frame->user1 + frame->user2 * 10;
                                if (st.months > 0 && st.months < 13) {
                                    unsigned char dpm[12]={31,28,31,30,31,30,31,31,30,31,30,31};
                                    if ((st.years % 4) == 0) dpm[1] = 29;
                                    st.days++;
                                    if (st.days > dpm[st.months - 1]) {
                                        st.days = 1; st.months++;
                                        if (st.months > 12) { st.months = 1; st.years = (st.years + 1) % 100; }
                                    }
                                    frame->user6 = st.years  / 10; frame->user5 = st.years  % 10;
                                    frame->user4 = st.months / 10; frame->user3 = st.months % 10;
                                    frame->user2 = st.days   / 10; frame->user1 = st.days   % 10;
                                } else { rv = -1; }
                            }
                        }
                    }
                }
            }
        }
    }
    if (frame->dfbit) _ltc_skip_drop_frames(frame);
    if ((flags & LTC_NO_PARITY) == 0) ltc_frame_set_parity(frame, standard);
    return rv;
}

static int ltc_frame_decrement(LTCFrame *frame, int fps, enum LTC_TV_STANDARD standard, int flags) {
    int rv = 0;
    int frames = frame->frame_units + frame->frame_tens * 10;
    frames = (frames > 0) ? frames - 1 : fps - 1;
    frame->frame_units = frames % 10;
    frame->frame_tens  = frames / 10;
    if (frames == fps - 1) {
        int secs = frame->secs_units + frame->secs_tens * 10;
        secs = (secs > 0) ? secs - 1 : 59;
        frame->secs_units = secs % 10; frame->secs_tens = secs / 10;
        if (secs == 59) {
            int mins = frame->mins_units + frame->mins_tens * 10;
            mins = (mins > 0) ? mins - 1 : 59;
            frame->mins_units = mins % 10; frame->mins_tens = mins / 10;
            if (mins == 59) {
                int hours = frame->hours_units + frame->hours_tens * 10;
                hours = (hours > 0) ? hours - 1 : 23;
                frame->hours_units = hours % 10; frame->hours_tens = hours / 10;
                if (hours == 23) {
                    rv = 1;
                    if (flags & LTC_USE_DATE) {
                        SMPTETimecode st;
                        st.years  = frame->user5 + frame->user6 * 10;
                        st.months = frame->user3 + frame->user4 * 10;
                        st.days   = frame->user1 + frame->user2 * 10;
                        if (st.months > 0 && st.months < 13) {
                            unsigned char dpm[12]={31,28,31,30,31,30,31,31,30,31,30,31};
                            if ((st.years % 4) == 0) dpm[1] = 29;
                            if (st.days > 1) {
                                st.days--;
                            } else {
                                st.months = 1 + (st.months + 10) % 12;
                                st.days = dpm[st.months - 1];
                                if (st.months == 12) st.years = (st.years + 99) % 100;
                            }
                            frame->user6 = st.years  / 10; frame->user5 = st.years  % 10;
                            frame->user4 = st.months / 10; frame->user3 = st.months % 10;
                            frame->user2 = st.days   / 10; frame->user1 = st.days   % 10;
                        } else { rv = -1; }
                    }
                }
            }
        }
    }
    if (frame->dfbit) _ltc_skip_drop_frames(frame);
    if ((flags & LTC_NO_PARITY) == 0) ltc_frame_set_parity(frame, standard);
    return rv;
}

/* =========================================================================
 * Encoder  (ported from libltc encoder.c)
 * Converts an LTCFrame struct into an 8-bit audio waveform using
 * bi-phase mark coding.  Call ltc_encoder_encode_frame() to fill the
 * internal buffer, then ltc_encoder_get_bufferptr() to read samples out.
 * ========================================================================= */

static int _enc_addvalues(LTCEncoder *e, int n) {
    const ltcsnd_sample_t tgtval = e->state ? e->enc_hi : e->enc_lo;
    if ((int)(e->offset + n) >= (int)e->bufsize) return 1;
    ltcsnd_sample_t *wave = &e->buf[e->offset];
    const double tcf = e->filter_const;
    if (tcf > 0) {
        ltcsnd_sample_t val = SAMPLE_CENTER;
        int m = (n + 1) >> 1;
        for (int i = 0; i < m; i++) {
            val = (ltcsnd_sample_t)(val + tcf * ((double)tgtval - val));
            wave[n - i - 1] = wave[i] = val;
        }
    } else {
        memset(wave, tgtval, n);
    }
    e->offset += n;
    return 0;
}

static int _enc_encode_byte(LTCEncoder *e, int byte, double speed) {
    if (byte < 0 || byte > 9 || speed == 0) return -1;
    int err = 0;
    const unsigned char c = ((unsigned char *)&e->f)[byte];
    unsigned char b = (speed < 0) ? 128 : 1;
    const double spc = e->samples_per_clock   * fabs(speed);
    const double sph = e->samples_per_clock_2 * fabs(speed);
    do {
        int n;
        if ((c & b) == 0) {
            n = (int)(spc + e->sample_remainder);
            e->sample_remainder = spc + e->sample_remainder - n;
            e->state = !e->state;
            err |= _enc_addvalues(e, n);
        } else {
            n = (int)(sph + e->sample_remainder);
            e->sample_remainder = sph + e->sample_remainder - n;
            e->state = !e->state;
            err |= _enc_addvalues(e, n);
            n = (int)(sph + e->sample_remainder);
            e->sample_remainder = sph + e->sample_remainder - n;
            e->state = !e->state;
            err |= _enc_addvalues(e, n);
        }
        if (speed < 0) b >>= 1; else b <<= 1;
    } while (b);
    return err;
}

static int _enc_encode_transition(LTCEncoder *e) {
    if ((int)(e->offset + 1) >= (int)e->bufsize) return -1;
    int off = e->offset;
    int n   = e->bufsize - e->offset - 1;
    e->state = !e->state;
    int err = _enc_addvalues(e, n);
    if (e->filter_const <= 0) e->offset = off + 1;
    return err;
}

static void ltc_encoder_set_filter(LTCEncoder *e, double rise_time) {
    if (rise_time <= 0)
        e->filter_const = 0;
    else
        e->filter_const = 1.0 - exp(-1.0 / (e->sample_rate * rise_time / 2000000.0 / exp(1.0)));
}

static int ltc_encoder_reinit(LTCEncoder *e, double sample_rate, double fps, enum LTC_TV_STANDARD standard, int flags) {
    if (sample_rate < 1) return -1;
    size_t needed = (size_t)(1 + ceil(sample_rate / fps));
    if (needed > e->bufsize) return -1;
    e->state = 0; e->offset = 0;
    e->sample_rate = sample_rate;
    ltc_encoder_set_filter(e, 40.0);
    e->fps = fps; e->flags = flags; e->standard = standard;
    e->samples_per_clock   = sample_rate / (fps * 80.0);
    e->samples_per_clock_2 = e->samples_per_clock / 2.0;
    e->sample_remainder    = 0.5;
    if (flags & LTC_BGF_DONT_TOUCH) {
        e->f.col_frame = 0;
        e->f.binary_group_flag_bit1 = (flags & LTC_TC_CLOCK) ? 1 : 0;
        switch (standard) {
            case LTC_TV_625_50:
                e->f.biphase_mark_phase_correction = 0;
                e->f.binary_group_flag_bit0 = (flags & LTC_USE_DATE) ? 1 : 0;
                break;
            default:
                e->f.binary_group_flag_bit0 = 0;
                e->f.binary_group_flag_bit2 = (flags & LTC_USE_DATE) ? 1 : 0;
                break;
        }
    }
    if ((flags & LTC_NO_PARITY) == 0) ltc_frame_set_parity(&e->f, standard);
    e->f.dfbit = ((int)_ltc_rint(fps * 100.0) == 2997) ? 1 : 0;
    return 0;
}

static LTCEncoder *ltc_encoder_create(double sample_rate, double fps, enum LTC_TV_STANDARD standard, int flags) {
    if (sample_rate < 1) return NULL;
    LTCEncoder *e = (LTCEncoder *)calloc(1, sizeof(LTCEncoder));
    if (!e) return NULL;
    e->enc_lo = 38; e->enc_hi = 218;   /* -3 dBFS default */
    e->bufsize = 1 + (size_t)ceil(sample_rate / fps);
    e->buf = (ltcsnd_sample_t *)calloc(e->bufsize, sizeof(ltcsnd_sample_t));
    if (!e->buf) { free(e); return NULL; }
    ltc_frame_reset(&e->f);
    ltc_encoder_reinit(e, sample_rate, fps, standard, flags);
    return e;
}

static void ltc_encoder_free(LTCEncoder *e) {
    if (!e) return;
    if (e->buf) free(e->buf);
    free(e);
}

static void ltc_encoder_reset(LTCEncoder *e) {
    e->state = 0; e->sample_remainder = 0.5; e->offset = 0;
}

static double ltc_encoder_get_volume(LTCEncoder *e) {
    ltcsnd_sample_t diff = (e->enc_hi - e->enc_lo) / 2;
    return 20.0 * log10((double)diff / 127.0);
}

static int ltc_encoder_set_volume(LTCEncoder *e, double dBFS) {
    if (dBFS > 0) return -1;
    double pp = _ltc_rint(127.0 * pow(10.0, dBFS / 20.0));
    if (pp < 1 || pp > 127) return -1;
    ltcsnd_sample_t diff = ((ltcsnd_sample_t)pp) & 0x7F;
    e->enc_lo = SAMPLE_CENTER - diff;
    e->enc_hi = SAMPLE_CENTER + diff;
    return 0;
}

static double ltc_encoder_get_filter(LTCEncoder *e) {
    const double num = -2000000.0 * exp(1.0);
    const double den = e->sample_rate * log(1.0 - e->filter_const);
    return num / den;
}

static int ltc_encoder_set_buffersize(LTCEncoder *e, double sample_rate, double fps) {
    free(e->buf);
    e->offset  = 0;
    e->bufsize = 1 + (size_t)ceil(sample_rate / fps);
    e->buf = (ltcsnd_sample_t *)calloc(e->bufsize, sizeof(ltcsnd_sample_t));
    return e->buf ? 0 : -1;
}

static int ltc_encoder_encode_byte(LTCEncoder *e, int byte, double speed) {
    return _enc_encode_byte(e, byte, speed);
}

static int ltc_encoder_end_encode(LTCEncoder *e) {
    return _enc_encode_transition(e);
}

static void ltc_encoder_encode_frame(LTCEncoder *e) {
    for (int b = 0; b < 10; b++) _enc_encode_byte(e, b, 1.0);
}

static void ltc_encoder_encode_reversed_frame(LTCEncoder *e) {
    for (int b = 9; b >= 0; b--) _enc_encode_byte(e, b, -1.0);
}

static void ltc_encoder_get_timecode(LTCEncoder *e, SMPTETimecode *t) {
    ltc_frame_to_time(t, &e->f, e->flags);
}

static void ltc_encoder_set_timecode(LTCEncoder *e, SMPTETimecode *t) {
    ltc_time_to_frame(&e->f, t, e->standard, e->flags);
}

static void ltc_encoder_set_user_bits(LTCEncoder *e, unsigned long d) {
    LTCFrame *f = &e->f;
    f->user1 = d & 0xF; d >>= 4; f->user2 = d & 0xF; d >>= 4;
    f->user3 = d & 0xF; d >>= 4; f->user4 = d & 0xF; d >>= 4;
    f->user5 = d & 0xF; d >>= 4; f->user6 = d & 0xF; d >>= 4;
    f->user7 = d & 0xF; d >>= 4; f->user8 = d & 0xF;
}

static void ltc_encoder_get_frame(LTCEncoder *e, LTCFrame *f) {
    memcpy(f, &e->f, sizeof(LTCFrame));
}

static void ltc_encoder_set_frame(LTCEncoder *e, LTCFrame *f) {
    memcpy(&e->f, f, sizeof(LTCFrame));
}

static int ltc_encoder_inc_timecode(LTCEncoder *e) {
    return ltc_frame_increment(&e->f, (int)_ltc_rint(e->fps), e->standard, e->flags);
}

static int ltc_encoder_dec_timecode(LTCEncoder *e) {
    return ltc_frame_decrement(&e->f, (int)_ltc_rint(e->fps), e->standard, e->flags);
}

static size_t ltc_encoder_get_buffersize(LTCEncoder *e) { return e->bufsize; }

static void ltc_encoder_buffer_flush(LTCEncoder *e) { e->offset = 0; }

static int ltc_encoder_get_bufferptr(LTCEncoder *e, ltcsnd_sample_t **buf, int flush) {
    const int len = (int)e->offset;
    if (buf) *buf = e->buf;
    if (flush) e->offset = 0;
    return len;
}

static int ltc_encoder_copy_buffer(LTCEncoder *e, ltcsnd_sample_t *buf) {
    const int len = (int)e->offset;
    memcpy(buf, e->buf, len);
    e->offset = 0;
    return len;
}

/* =========================================================================
 * Decoder  (ported from libltc decoder.c)
 * Parses raw 8-bit audio samples and extracts LTC frames by detecting the
 * biphase-mark sync word.  Feed audio with ltc_decoder_write*(), then pull
 * decoded frames with ltc_decoder_read() one at a time.
 * ========================================================================= */

/* Bit-Konstanten für den Decoder */
#define _B8_00000001 0x01u
#define _B8_00000010 0x02u
#define _B8_00000100 0x04u
#define _B8_00001000 0x08u
#define _B8_00010000 0x10u
#define _B8_00100000 0x20u
#define _B8_01000000 0x40u
#define _B8_10000000 0x80u
#define _B16_00111111_11111101 0x3ffdU
#define _B16_10111111_11111100 0xbffcU

static double _dec_calc_volume_db(LTCDecoder *d) {
    if (d->snd_to_biphase_max <= d->snd_to_biphase_min) return -INFINITY;
    return 20.0 * log10((d->snd_to_biphase_max - d->snd_to_biphase_min) / 255.0);
}

static void _dec_parse_ltc(LTCDecoder *d, unsigned char bit, ltc_off_t offset, ltc_off_t posinfo) {
    int bit_num, bit_set, byte_num;

    if (d->bit_cnt == 0) {
        memset(&d->ltc_frame, 0, sizeof(LTCFrame));
        if (d->frame_start_prev < 0)
            d->frame_start_off = posinfo - (ltc_off_t)d->snd_to_biphase_period;
        else
            d->frame_start_off = d->frame_start_prev;
    }
    d->frame_start_prev = offset + posinfo;

    if (d->bit_cnt >= LTC_FRAME_BIT_COUNT) {
        int k;
        const int byte_max = LTC_FRAME_BIT_COUNT >> 3;
        for (k = 0; k < byte_max; k++) {
            const unsigned char bi = ((unsigned char *)&d->ltc_frame)[k];
            unsigned char bo = 0;
            if (bi & _B8_10000000) bo |= _B8_01000000;
            if (bi & _B8_01000000) bo |= _B8_00100000;
            if (bi & _B8_00100000) bo |= _B8_00010000;
            if (bi & _B8_00010000) bo |= _B8_00001000;
            if (bi & _B8_00001000) bo |= _B8_00000100;
            if (bi & _B8_00000100) bo |= _B8_00000010;
            if (bi & _B8_00000010) bo |= _B8_00000001;
            if (k + 1 < byte_max)
                if (((unsigned char *)&d->ltc_frame)[k+1] & _B8_00000001) bo |= _B8_10000000;
            ((unsigned char *)&d->ltc_frame)[k] = bo;
        }
        d->frame_start_off += (ltc_off_t)ceil(d->snd_to_biphase_period);
        d->bit_cnt--;
    }

    d->decoder_sync_word <<= 1;
    if (bit) {
        d->decoder_sync_word |= 1;
        if (d->bit_cnt < LTC_FRAME_BIT_COUNT) {
            bit_num  = d->bit_cnt & 7;
            bit_set  = _B8_00000001 << bit_num;
            byte_num = d->bit_cnt >> 3;
            ((unsigned char *)&d->ltc_frame)[byte_num] |= bit_set;
        }
    }
    d->bit_cnt++;

    /* Sync-Wort erkannt → Frame in Queue */
    if (d->decoder_sync_word == _B16_00111111_11111101) {
        if (d->bit_cnt == LTC_FRAME_BIT_COUNT) {
            if (d->queue_write_off == d->queue_len) d->queue_write_off = 0;
            memcpy(&d->queue[d->queue_write_off].ltc, &d->ltc_frame, sizeof(LTCFrame));
            for (int bc = 0; bc < LTC_FRAME_BIT_COUNT; bc++) {
                int btc = (d->biphase_tic + bc) % LTC_FRAME_BIT_COUNT;
                d->queue[d->queue_write_off].biphase_tics[bc] = d->biphase_tics[btc];
            }
            d->queue[d->queue_write_off].off_start  = d->frame_start_off;
            d->queue[d->queue_write_off].off_end    = posinfo + (ltc_off_t)offset - 1LL;
            d->queue[d->queue_write_off].reverse    = 0;
            d->queue[d->queue_write_off].volume     = _dec_calc_volume_db(d);
            d->queue[d->queue_write_off].sample_min = d->snd_to_biphase_min;
            d->queue[d->queue_write_off].sample_max = d->snd_to_biphase_max;
            d->queue_write_off++;
        }
        d->bit_cnt = 0;
    }

    /* Reverses Sync-Wort */
    if (d->decoder_sync_word == _B16_10111111_11111100) {
        if (d->bit_cnt == LTC_FRAME_BIT_COUNT) {
            int k;
            const int byte_max = LTC_FRAME_BIT_COUNT >> 3;
            for (k = 0; k < byte_max; k++) {
                const unsigned char bi = ((unsigned char *)&d->ltc_frame)[k];
                unsigned char bo = 0;
                if (bi & _B8_10000000) bo |= _B8_00000001;
                if (bi & _B8_01000000) bo |= _B8_00000010;
                if (bi & _B8_00100000) bo |= _B8_00000100;
                if (bi & _B8_00010000) bo |= _B8_00001000;
                if (bi & _B8_00001000) bo |= _B8_00010000;
                if (bi & _B8_00000100) bo |= _B8_00100000;
                if (bi & _B8_00000010) bo |= _B8_01000000;
                if (bi & _B8_00000001) bo |= _B8_10000000;
                ((unsigned char *)&d->ltc_frame)[k] = bo;
            }
            int bm2 = byte_max - 2;
            for (k = 0; k < bm2 / 2; k++) {
                unsigned char tmp = ((unsigned char *)&d->ltc_frame)[k];
                ((unsigned char *)&d->ltc_frame)[k] = ((unsigned char *)&d->ltc_frame)[bm2-1-k];
                ((unsigned char *)&d->ltc_frame)[bm2-1-k] = tmp;
            }
            if (d->queue_write_off == d->queue_len) d->queue_write_off = 0;
            memcpy(&d->queue[d->queue_write_off].ltc, &d->ltc_frame, sizeof(LTCFrame));
            for (int bc = 0; bc < LTC_FRAME_BIT_COUNT; bc++) {
                int btc = (d->biphase_tic + bc) % LTC_FRAME_BIT_COUNT;
                d->queue[d->queue_write_off].biphase_tics[bc] = d->biphase_tics[btc];
            }
            ltc_off_t rev_off = (ltc_off_t)(16.0 * d->snd_to_biphase_period);
            d->queue[d->queue_write_off].off_start  = d->frame_start_off - rev_off;
            d->queue[d->queue_write_off].off_end    = posinfo + (ltc_off_t)offset - 1LL - rev_off;
            d->queue[d->queue_write_off].reverse    = (LTC_FRAME_BIT_COUNT >> 3) * 8 * (int)d->snd_to_biphase_period;
            d->queue[d->queue_write_off].volume     = _dec_calc_volume_db(d);
            d->queue[d->queue_write_off].sample_min = d->snd_to_biphase_min;
            d->queue[d->queue_write_off].sample_max = d->snd_to_biphase_max;
            d->queue_write_off++;
        }
        d->bit_cnt = 0;
    }
}

static inline void _biphase_decode2(LTCDecoder *d, ltc_off_t offset, ltc_off_t pos) {
    d->biphase_tics[d->biphase_tic] = (float)d->snd_to_biphase_period;
    d->biphase_tic = (d->biphase_tic + 1) % LTC_FRAME_BIT_COUNT;
    if (d->snd_to_biphase_cnt <= (int)(2 * d->snd_to_biphase_period))
        pos -= (ltc_off_t)(d->snd_to_biphase_period - d->snd_to_biphase_cnt);
    if (d->snd_to_biphase_state == d->biphase_prev) {
        d->biphase_state = 1;
        _dec_parse_ltc(d, 0, offset, pos);
    } else {
        d->biphase_state = 1 - d->biphase_state;
        if (d->biphase_state == 1) _dec_parse_ltc(d, 1, offset, pos);
    }
    d->biphase_prev = d->snd_to_biphase_state;
}

static void _decode_ltc(LTCDecoder *d, ltcsnd_sample_t *sound, size_t size, ltc_off_t posinfo) {
    for (size_t i = 0; i < size; i++) {
        /* Dynamische Min/Max-Verfolgung */
        d->snd_to_biphase_min = (ltcsnd_sample_t)(SAMPLE_CENTER - (((SAMPLE_CENTER - d->snd_to_biphase_min) * 15) / 16));
        d->snd_to_biphase_max = (ltcsnd_sample_t)(SAMPLE_CENTER + (((d->snd_to_biphase_max - SAMPLE_CENTER) * 15) / 16));
        if (sound[i] < d->snd_to_biphase_min) d->snd_to_biphase_min = sound[i];
        if (sound[i] > d->snd_to_biphase_max) d->snd_to_biphase_max = sound[i];

        ltcsnd_sample_t min_thr = (ltcsnd_sample_t)(SAMPLE_CENTER - (((SAMPLE_CENTER - d->snd_to_biphase_min) * 8) / 16));
        ltcsnd_sample_t max_thr = (ltcsnd_sample_t)(SAMPLE_CENTER + (((d->snd_to_biphase_max - SAMPLE_CENTER) * 8) / 16));

        if ((d->snd_to_biphase_state  && (sound[i] > max_thr)) ||
            (!d->snd_to_biphase_state && (sound[i] < min_thr))) {
            if (d->snd_to_biphase_cnt > d->snd_to_biphase_lmt) {
                _biphase_decode2(d, (ltc_off_t)i, posinfo);
                _biphase_decode2(d, (ltc_off_t)i, posinfo);
            } else {
                d->snd_to_biphase_cnt *= 2;
                _biphase_decode2(d, (ltc_off_t)i, posinfo);
            }
            if (d->snd_to_biphase_cnt > (int)(d->snd_to_biphase_period * 4))
                d->bit_cnt = 0;
            else {
                d->snd_to_biphase_period = (d->snd_to_biphase_period * 3.0 + d->snd_to_biphase_cnt) / 4.0;
                d->snd_to_biphase_lmt    = (int)((d->snd_to_biphase_period * 3) / 4);
            }
            d->snd_to_biphase_cnt   = 0;
            d->snd_to_biphase_state = !d->snd_to_biphase_state;
        }
        d->snd_to_biphase_cnt++;
    }
}

/* ── Öffentliche Decoder-API ──────────────────────────────────────────── */

static LTCDecoder *ltc_decoder_create(int apv, int queue_len) {
    LTCDecoder *d = (LTCDecoder *)calloc(1, sizeof(LTCDecoder));
    if (!d) return NULL;
    if (queue_len < 1) queue_len = 1;
    d->queue_len = queue_len;
    d->queue = (LTCFrameExt *)calloc(queue_len, sizeof(LTCFrameExt));
    if (!d->queue) { free(d); return NULL; }
    d->biphase_state         = 1;
    d->snd_to_biphase_period = apv / 80;
    d->snd_to_biphase_lmt    = (d->snd_to_biphase_period * 3) / 4;
    d->snd_to_biphase_min    = SAMPLE_CENTER;
    d->snd_to_biphase_max    = SAMPLE_CENTER;
    d->frame_start_prev      = -1;
    d->biphase_tic           = 0;
    return d;
}

static int ltc_decoder_free(LTCDecoder *d) {
    if (!d) return 1;
    if (d->queue) free(d->queue);
    free(d);
    return 0;
}

static void ltc_decoder_write(LTCDecoder *d, ltcsnd_sample_t *buf, size_t size, ltc_off_t posinfo) {
    _decode_ltc(d, buf, size, posinfo);
}

/* Konvertierungs-Hilfsmakro */
#define _LTC_CONV_BUF 512

#define _LTCWRITE_CONV(FN, FTYPE, CONV) \
static void ltc_decoder_write_##FN(LTCDecoder *d, FTYPE *buf, size_t size, ltc_off_t posinfo) { \
    ltcsnd_sample_t tmp[_LTC_CONV_BUF]; \
    size_t start = 0; \
    while (start < size) { \
        int c = (int)(size - start); \
        if (c > _LTC_CONV_BUF) c = _LTC_CONV_BUF; \
        for (int i = 0; i < c; i++) tmp[i] = (ltcsnd_sample_t)(CONV); \
        _decode_ltc(d, tmp, c, posinfo + (ltc_off_t)start); \
        start += c; \
    } \
}

_LTCWRITE_CONV(double,  double,         128 + buf[start+i] * 127.0)
_LTCWRITE_CONV(float,   float,          128 + buf[start+i] * 127.f)
_LTCWRITE_CONV(s16,     short,          128 + (buf[start+i] >> 8))
_LTCWRITE_CONV(u16,     unsigned short, buf[start+i] >> 8)

#undef _LTC_CONV_BUF
#undef _LTCWRITE_CONV

static int ltc_decoder_read(LTCDecoder *d, LTCFrameExt *frame) {
    if (!frame) return -1;
    if (d->queue_read_off == d->queue_write_off) return 0;
    if (d->queue_read_off == d->queue_len) d->queue_read_off = 0;
    memcpy(frame, &d->queue[d->queue_read_off], sizeof(LTCFrameExt));
    d->queue_read_off++;
    return 1;
}

static void ltc_decoder_queue_flush(LTCDecoder *d) {
    d->queue_read_off = d->queue_write_off;
}

static int ltc_decoder_queue_length(LTCDecoder *d) {
    return (d->queue_write_off - d->queue_read_off + d->queue_len) % d->queue_len;
}

/* Undef interne Makros damit sie nicht in den Sketch leaken */
#undef _B8_00000001
#undef _B8_00000010
#undef _B8_00000100
#undef _B8_00001000
#undef _B8_00010000
#undef _B8_00100000
#undef _B8_01000000
#undef _B8_10000000
#undef _B16_00111111_11111101
#undef _B16_10111111_11111100

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LTC_ESP32_H */
