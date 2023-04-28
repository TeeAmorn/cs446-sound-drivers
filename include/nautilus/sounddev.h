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
    NK_SOUND_DEV_SCALE_LINEAR,
    NK_SOUND_DEV_SCALE_LOGARITHMIC
} nk_sound_dev_scale_t;

struct nk_sound_dev_characteristics
{
    uint32_t sample_rate;
    uint8_t sample_resolution;
    uint8_t num_of_channels;
    nk_sound_dev_scale_t scale;
};

typedef enum
{
    NK_SOUND_DEV_STATUS_SUCCESS = 0,
    NK_SOUND_DEV_STATUS_ERROR
} nk_sound_dev_status_t;

struct nk_sound_dev_params
{
    uint32_t sample_rate;
    uint8_t sample_resolution;
    uint8_t num_of_channels;
    nk_sound_dev_scale_t scale;
};

struct nk_sound_dev_int
{
    // this must be first so it derives cleanly
    // from nk_dev_int
    struct nk_dev_int dev_int;

    // sounddev-specific interface - set to zero if not available
    // an interface either succeeds (returns zero) or fails (returns -1)
    int (*get_characteristics)(void *state, struct nk_sound_dev_characteristics *c);
    // send/receive are non-blocking always.  -1 on error, otherwise return 0
    // callback can be null
    int (*write_sound)(void *state, uint8_t *src, uint64_t len, void (*callback)(nk_sound_dev_status_t status, void *context), void *context);
    int (*read_sound)(void *state, uint8_t *dst, uint64_t len, void (*callback)(nk_sound_dev_status_t status, void *context), void *context);
    int (*set_params)(void *state, struct nk_sound_dev_params *p);
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

int nk_sound_dev_get_characteristics(struct nk_sound_dev *dev, struct nk_sound_dev_characteristics *c);

int nk_sound_dev_write(struct nk_sound_dev *dev,
                       uint8_t *src,
                       uint64_t len,
                       nk_dev_request_type_t type,
                       void (*callback)(nk_sound_dev_status_t status,
                                        void *state), // for callback reqs
                       void *state);                  // for callback reqs

int nk_sound_dev_read(struct nk_sound_dev *dev,
                      uint8_t *dest,
                      uint64_t len,
                      nk_dev_request_type_t type,
                      void (*callback)(nk_sound_dev_status_t status,
                                       void *state), // for callback reqs
                      void *state);                  // for callback reqs

int nk_sound_dev_set_params(struct nk_sound_dev *dev, struct nk_sound_dev_params *p);

#endif
