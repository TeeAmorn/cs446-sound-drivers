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

#include <nautilus/nautilus.h>
#include <nautilus/dev.h>
#include <nautilus/sounddev.h>

#ifndef NAUT_CONFIG_DEBUG_SOUNDDEV
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

#define ERROR(fmt, args...) ERROR_PRINT("sounddev: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("sounddev: " fmt, ##args)
#define INFO(fmt, args...) INFO_PRINT("sounddev: " fmt, ##args)

#if 0
static spinlock_t state_lock;

#define STATE_LOCK_CONF uint8_t _state_lock_flags
#define STATE_LOCK() _state_lock_flags = spin_lock_irq_save(&state_lock)
#define STATE_UNLOCK() spin_unlock_irq_restore(&state_lock, _state_lock_flags);

#endif

int nk_sound_dev_init()
{
    INFO("init\n");
    return 0;
}

int nk_sound_dev_deinit()
{
    INFO("deinit\n");
    return 0;
}

struct nk_sound_dev *nk_sound_dev_register(char *name, uint64_t flags, struct nk_sound_dev_int *inter, void *state)
{
    INFO("register device %s\n", name);
    return (struct nk_sound_dev *)nk_dev_register(name, NK_DEV_SOUND, flags, (struct nk_dev_int *)inter, state);
}

int nk_sound_dev_unregister(struct nk_sound_dev *d)
{
    INFO("unregister device %s\n", d->dev.name);
    return nk_dev_unregister((struct nk_dev *)d);
}

struct nk_sound_dev *nk_sound_dev_find(char *name)
{
    DEBUG("find %s\n", name);
    struct nk_dev *d = nk_dev_find(name);
    if (d->type != NK_DEV_SOUND)
    {
        DEBUG("%s not found\n", name);
        return 0;
    }
    else
    {
        DEBUG("%s found\n", name);
        return (struct nk_sound_dev *)d;
    }
}

int nk_sound_dev_get_avaiable_modes(struct nk_sound_dev *dev, struct nk_sound_dev_params params[]) {
    struct nk_dev *d = (struct nk_dev *)(&(dev->dev));
    struct nk_sound_dev_int *di = (struct nk_sound_dev_int *)(d->interface);
    return di->get_avaiable_modes(d->state, params); 
}

struct nk_sound_dev_stream *nk_sound_dev_open_stream(struct nk_sound_dev *dev, nk_sound_dev_stream_t stream_type, struct nk_sound_dev_params *params)
{
    struct nk_dev *d = (struct nk_dev *)(&(dev->dev));
    struct nk_sound_dev_int *di = (struct nk_sound_dev_int *)(d->interface);
    return di->open_stream(d->state, stream_type, params);
}

int nk_sound_dev_close_stream(struct nk_sound_dev *dev, struct nk_sound_dev_stream *stream)
{
    struct nk_dev *d = (struct nk_dev *)(&(dev->dev));
    struct nk_sound_dev_int *di = (struct nk_sound_dev_int *)(d->interface);
    return di->close_stream(d->state, stream);
}

struct op
{
    int completed;
    nk_sound_dev_status_t status;
    struct nk_sound_dev *dev;
};

static void generic_write_callback(nk_sound_dev_status_t status, void *context)
{
    struct op *o = (struct op *)context;
    DEBUG("generic write callback (status = 0x%lx) for %p\n", status, context);
    o->status = status;
    o->completed = 1;
    nk_dev_signal((struct nk_dev *)o->dev);
}

static void generic_read_callback(nk_sound_dev_status_t status, void *context)
{
    struct op *o = (struct op *)context;
    DEBUG("generic read callback (status = 0x%lx) for %p\n", status, context);
    o->status = status;
    o->completed = 1;
    nk_dev_signal((struct nk_dev *)o->dev);
}

static int generic_cond_check(void *state)
{
    struct op *o = (struct op *)state;
    return o->completed;
}

int nk_sound_dev_write_to_stream(struct nk_sound_dev *dev,
                                 struct nk_sound_dev_stream *stream,
                                 uint8_t *src,
                                 uint64_t len,
                                 nk_dev_request_type_t type,
                                 void (*callback)(nk_sound_dev_status_t status, void *state),
                                 void *state)
{
    if (stream->type == NK_SOUND_DEV_INPUT_STREAM) {
        DEBUG("write sound not possible, stream is of type NK_SOUND_DEV_INPUT_STREAM\n");
        return -1;
    }

    struct nk_dev *d = (struct nk_dev *)(&(dev->dev));
    struct nk_sound_dev_int *di = (struct nk_sound_dev_int *)(d->interface);
    DEBUG("write sound on %s (len=%lu, type=%lx)\n", d->name, len, type);
    switch (type)
    {
    case NK_DEV_REQ_CALLBACK:
        if (!di->write_to_stream)
        {
            DEBUG("write sound not possible\n");
            return -1;
        }
        else
        {
            return di->write_to_stream(d->state, stream, src, len, callback, state);
        }
        break;
    case NK_DEV_REQ_BLOCKING:
    case NK_DEV_REQ_NONBLOCKING:
        if (!di->write_to_stream)
        {
            DEBUG("write sound not possible\n");
            return -1;
        }
        else
        {
            volatile struct op o;

            o.completed = 0;
            o.status = 0;
            o.dev = dev;

            if (type == NK_DEV_REQ_NONBLOCKING)
            {
                if (di->write_to_stream(d->state, stream, src, len, 0, 0))
                {
                    ERROR("failed to write sound\n");
                    return -1;
                }
                else
                {
                    DEBUG("started writing sound\n");
                    return 0;
                }
            }
            else
            {
                if (di->write_to_stream(d->state, stream, src, len, generic_write_callback, (void *)&o))
                {
                    ERROR("failed to write sound\n");
                    return -1;
                }
                else
                {
                    DEBUG("started writing sound, waiting for completion\n");
                    while (!o.completed)
                    {
                        nk_dev_wait((struct nk_dev *)dev, generic_cond_check, (void *)&o);
                    }
                    DEBUG("write sound completed\n");
                    return o.status;
                }
            }
        }
        break;
    default:
        return -1;
    }
}

int nk_sound_dev_read_to_stream(struct nk_sound_dev *dev,
                                struct nk_sound_dev_stream *stream,
                                uint8_t *dst,
                                uint64_t len,
                                nk_dev_request_type_t type,
                                void (*callback)(nk_sound_dev_status_t status, void *state),
                                void *state)
{
    if (stream->type == NK_SOUND_DEV_OUTPUT_STREAM) {
        DEBUG("write sound not possible, stream is of type NK_SOUND_DEV_OUTPUT_STREAM\n");
        return -1;
    }
    
    struct nk_dev *d = (struct nk_dev *)(&(dev->dev));
    struct nk_sound_dev_int *di = (struct nk_sound_dev_int *)(d->interface);
    DEBUG("read sound on %s (len=%lu, type=%lx)\n", d->name, len, type);

    switch (type)
    {
    case NK_DEV_REQ_CALLBACK:
        if (!di->read_from_stream)
        {
            DEBUG("read sound not possible\n");
            return -1;
        }
        else
        {
            return di->read_from_stream(d->state, stream, dst, len, callback, state);
        }
        break;
    case NK_DEV_REQ_BLOCKING:
    case NK_DEV_REQ_NONBLOCKING:
        if (!di->read_from_stream)
        {
            DEBUG("read sound not possible\n");
            return -1;
        }
        else
        {
            volatile struct op o;

            o.completed = 0;
            o.status = 0;
            o.dev = dev;

            if (type == NK_DEV_REQ_NONBLOCKING)
            {
                if (di->read_from_stream(d->state, stream, dst, len, 0, 0))
                {
                    ERROR("failed to read sound\n");
                    return -1;
                }
                else
                {
                    DEBUG("started reading sound\n");
                    return 0;
                }
            }
            else
            {
                if (di->read_from_stream(d->state, stream, dst, len, generic_read_callback, (void *)&o))
                {
                    ERROR("failed to read sound\n");
                    return -1;
                }
                else
                {
                    DEBUG("started reading sound, waiting for completion\n");
                    while (!o.completed)
                    {
                        nk_dev_wait((struct nk_dev *)d, generic_cond_check, (void *)&o);
                    }
                    DEBUG("read sound completed\n");
                    return o.status;
                }
            }
        }
        break;
    default:
        return -1;
    }
}

int nk_sound_dev_get_stream_params(struct nk_sound_dev *dev, struct nk_sound_dev_stream *stream, struct nk_sound_dev_params *p)
{
    struct nk_dev *d = (struct nk_dev *)(&(dev->dev));
    struct nk_sound_dev_int *di = (struct nk_sound_dev_int *)(d->interface);

    DEBUG("set sound parameters of %s\n", d->name);
    return di->get_stream_params(d->state, stream, p);
}