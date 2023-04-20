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
typedef enum {
    NK_SOUND_DEV_STATUS_SUCCESS=0,
    NK_SOUND_DEV_STATUS_ERROR
} nk_sound_dev_status_t;
```

```c
struct nk_sound_dev_characteristics { ... };
```

```c
struct nk_sound_dev_int { ... };
```

```c
struct nk_sound_dev { ... };
```

```c
int nk_sound_dev_init();
```

```c
int nk_sound_dev_deinit();
```

```c
struct nk_sound_dev * nk_sound_dev_register( ... );
```

```c
int nk_sound_dev_unregister( ... );
```

```c
struct nk_sound_dev * nk_sound_dev_find( ... );
```

```c
int nk_sound_dev_get_characteristics( ... );
```

```c
int nk_sound_dev_read( ... );
```

```c
int nk_sound_dev_write( ... );
```
