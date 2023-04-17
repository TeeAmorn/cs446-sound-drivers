# QEMU

The following instructions have only been tested on Fedora. 

## Installation

1. Install QEMU:

```
sudo dnf install qemu
```

2. Create a new directory called `qemu`:
```
mkdir ~/qemu
```

3. Download and store the Fedora ISO, `fedora.iso`, in the `qemu` directory. The ISO can be obtained directly from the official Fedora page.

4. Create a virtual image for your system inside the `qemu` directory. Here, we are creating an image whose storage format is _copy-on-write_ and has a maximum size of 10G.
```
qemu-img create -f qcow2 image.img 10G
```

5. Start the emulator by running the following command from inside the `qemu` directory. Your directory should contain 2 files at this point: `fedora.iso` and `image.img`.
```
qemu-system-x86_64 -enable-kvm -cdrom fedora.iso -boot menu=on -drive file=image.img -m 4G -audiodev id=pa,driver=pa -device intel-hda -device hda-duplex
```

The instructions above should hook up the audio frontend (the OS, Fedora in our case, inside QEMU) with the backend (the side facing our host system, which is also Fedora). This allows sound from inside the QEMU to be played to our host system.
