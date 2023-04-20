# Devices

## Initialization

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

## Abstraction Layers
