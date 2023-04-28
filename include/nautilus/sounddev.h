/*
 * This file is part of the Nautilus AeroKernel developed
 * by the Hobbes and V3VEE Projects with funding from the
 * United States National  Science Foundation and the Department of Energy.
 *
 * The V3VEE Project is a joint project between Northwestern University
 * and the University of New Mexico.  The Hobbes Project is a collaboration
 * led by Sandia National Laboratories that includes several national
 * laboratories and universities. You can find out more at:
 * http://www.v3vee.org  and
 * http://xstack.sandia.gov/hobbes
 *
 * Copyright (c) 2016, Peter Dinda <pdinda@northwestern.edu>
 * Copyright (c) 2015, The V3VEE Project  <http://www.v3vee.org>
 *                     The Hobbes Project <http://xstack.sandia.gov/hobbes>
 * All rights reserved.
 *
 * Author: XXX
 *
 * This is free software.  You are permitted to use,
 * redistribute, and modify it as specified in the file "LICENSE.txt".
 */

#ifndef __SOUND_DEV
#define __SOUND_DEV

#include <nautilus/dev.h>

typedef enum
{
    NK_SOUND_DEV_STATUS_SUCCESS = 0,
    NK_SOUND_DEV_STATUS_ERROR
} nk_sound_dev_status_t;

typedef enum
{
    NK_SOUND_DEV_SCALE_LINEAR,
    NK_SOUND_DEV_SCALE_LOGARITHMIC
} nk_sound_dev_scale_t;

struct nk_sound_dev_params
{
    uint32_t sample_rate;
    uint8_t sample_resolution;
    uint8_t num_of_channels;
    nk_sound_dev_scale_t scale;
};

struct nk_sound_dev_stream
{
    uint8_t stream_id;
    struct nk_sound_dev_params params;
};

struct nk_sound_dev_int
{
    // this must be first so it derives cleanly
    // from nk_dev_int
    struct nk_dev_int dev_int;

    // ===================================
    // sounddev-specific interface
    // an interface either succeeds (returns zero) or fails (returns -1)
    // ===================================

    // ==========
    //   Layers
    // ==========
    //
    // Application
    //     |
    //     V     <---
    // nk_sound_dev  |
    //     |         |  This sound device
    //     V         |  abstraction layers
    //  streams      |
    //     |     <---
    //     V
    // specific sound
    // device driver

    // interface to learn about parameters supported by the sound device driver
    int (*get_available_sample_rates)(void *state, uint32_t rates[]);
    int (*get_available_sample_resolution)(void *state, uint8_t resolutions[]);
    int (*get_available_num_of_channels)(void *state, uint8_t channels[]);
    int (*get_available_scale)(void *state, nk_sound_dev_scale_t scales[]);

    // interface to open/close streams
    struct nk_sound_dev_stream *(*open_stream)(void *state, struct nk_sound_dev_params *params);
    int (*close_stream)(void *state, struct nk_sound_dev_stream *stream);

    // interface to write/read streams
    int (*write_to_stream)(void *state,
                           struct nk_sound_dev_stream *stream,
                           uint8_t *src,
                           uint64_t len,
                           void (*callback)(nk_sound_dev_status_t status, void *context),
                           void *context);
    int (*read_from_stream)(void *state,
                            struct nk_sound_dev_stream *stream,
                            uint8_t *dst,
                            uint64_t len,
                            void (*callback)(nk_sound_dev_status_t status, void *context),
                            void *context);

    // interface to retrieve parameters of the given stream
    int (*get_stream_params)(void *state, struct nk_sound_dev_stream *stream, struct nk_sound_dev_params *p);
};

struct nk_sound_dev
{
    // must be first member
    struct nk_dev dev;
};

int nk_sound_dev_init();
int nk_sound_dev_deinit();

struct nk_sound_dev *nk_sound_dev_register(char *name, uint64_t flags, struct nk_sound_dev_int *inter, void *state);
int nk_sound_dev_unregister(struct nk_sound_dev *);

struct nk_sound_dev *nk_sound_dev_find(char *name);

int nk_sound_dev_get_available_sample_rates(struct nk_sound_dev *dev, uint32_t rates[]);
int nk_sound_dev_get_available_sample_resolution(struct nk_sound_dev *dev, uint8_t resolutions[]);
int nk_sound_dev_get_available_num_of_channels(struct nk_sound_dev *dev, uint8_t channels[]);
int nk_sound_dev_get_available_scale(struct nk_sound_dev *dev, nk_sound_dev_scale_t scales[]);

struct nk_sound_dev_stream *nk_sound_dev_open_stream(struct nk_sound_dev *dev, struct nk_sound_dev_params *params);
int nk_sound_dev_close_stream(struct nk_sound_dev *dev, struct nk_sound_dev_stream *stream);

int nk_sound_dev_write_to_stream(struct nk_sound_dev *dev,
                                 struct nk_sound_dev_stream *stream,
                                 uint8_t *src,
                                 uint64_t len,
                                 nk_dev_request_type_t type,
                                 void (*callback)(nk_sound_dev_status_t status,
                                                  void *state),
                                 void *state);

int nk_sound_dev_read_to_stream(struct nk_sound_dev *dev,
                                struct nk_sound_dev_stream *stream,
                                uint8_t *dst,
                                uint64_t len,
                                nk_dev_request_type_t type,
                                void (*callback)(nk_sound_dev_status_t status,
                                                 void *state),
                                void *state);

int nk_sound_dev_get_stream_params(struct nk_sound_dev *dev, struct nk_sound_dev_stream *stream, struct nk_sound_dev_params *p);

#endif
