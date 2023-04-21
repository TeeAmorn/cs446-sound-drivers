# Devices

## Initialization

The snippets in this subsection show how specific device drivers and device abstraction layers are currently being initialized and registered.

1. The entry point is in `void init(...)` inside `src/arch/x64/init.c`

```c
void
init (unsigned long mbd,
      unsigned long magic)
{
    struct naut_info * naut = &nautilus_info;

    // ...snippets...

    nk_dev_init();
    nk_char_dev_init();
    nk_block_dev_init();
    nk_net_dev_init();
    nk_gpu_dev_init();

    // ...snippets...

    pci_init(naut);

    // ...snippets...

#ifdef NAUT_CONFIG_E1000_PCI
    e1000_pci_init(naut);
#endif

#ifdef NAUT_CONFIG_E1000E_PCI
    e1000e_pci_init(naut);
#endif

    // ...snippets...
}
```

2. The device abstraction layers, in which various device drivers rely on, are initialized first. The `pci` subsystem, which scans for all devices on the PCI bus, is then initialized via `pci_init(naut)`.

3. The `e1000_pci_init` function is an example of when a device driver gets initialized. Here, we are initialing the network interface card driver for Intel's E1000 cards. In this case, the device driver uses the network abstraction layer `nk_net_dev`.

## Sound Abstraction Layers

```c
// what is the purpose of this?
typedef enum {
  NK_SOUND_DEV_STATUS_SUCCESS = 0,
    NK_SOUND_DEV_STATUS_ERROR
}
nk_sound_dev_status_t
```

```c
struct nk_sound_dev_characteristics {
    // currently none - discuss with Dinda
}
```

```c
struct nk_sound_dev_int {
  // this must be first so it derives cleanly
  // from nk_dev int
  struct nk_dev_int dev_int;

  // sounddev-specific interface - set to zero if not available
  // an interface either succeeds (returns zero) or fails (returns -1)
  // in any case, it returns immediately
  int( * get_characteristics)(void * state, struct nk_sound_dev_characteristics * c);
  int( * read)(void * state, uint8_t * dest, ...);
  int( * write)(void * state, uint8_t * src, ...);
}
```

```c
struct nk_sound_dev {
  // must be first member
  struct nk_dev dev;
}
```

```c
int nk_sound_dev_init() {
  INFO("init\n");
  return 0;
}
```

```c
int nk_sound_dev_deinit() {
  INFO("deinit\n");
  return 0;
}
```

```c
struct nk_sound_dev * nk_sound_dev_register(char * name, uint64_t flags, struct nk_sound_dev_int * inter, void * state) {
  INFO("register device %s\n", name);
  // have to define NK_DEV_SOUND IN include/nautilus/dev.h
  return (struct nk_sound_dev * ) nk_dev_register(name, NK_DEV_SOUND, flags, (struct nk_dev_int * ) inter, state);
}
```

```c
int nk_sound_dev_unregister(struct nk_sound_dev * d) {
  INFO("unregister device %s\n", d -> dev.name);
  return nk_dev_unregister((struct nk_dev * ) d);
}
```

```c
struct nk_sound_dev * nk_sound_dev_find(char * name) {
  DEBUG("find %s\n", name);
  struct nk_dev * d = nk_dev_find(name);
  if (d -> type != NK_DEV_SOUND) {
    DEBUG("%s not found\n", name);
    return 0;
  } else {
    DEBUG("%s found\n", name);
    return (struct nk_sound_dev * ) d;
  }
}
```

```c
int nk_sound_dev_get_characteristics(struct nk_sound_dev * dev, struct nk_sound_dev_characteristics * c) {
  struct nk_dev * d = (struct nk_dev * )( & (dev -> dev));
  struct nk_sound_dev_int * di = (struct nk_sound_dev_int * )(d -> interface);

  DEBUG("get characteristics of %s\n", d -> name);
  return di -> get_characteristics(d -> state, c);
}
```

```c
// TODO!
int nk_sound_dev_read( ... );
```

```c
// TODO!
int nk_sound_dev_write( ... );
```
