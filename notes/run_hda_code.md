# Run HDA Code

The following instructions were tested on Macbooks (intel and M1 ships) and a virtual machine (Ubantu 16.04). While the HDA code works find on a Linux virtual machine, it cannot play sound on Macbooks. Professor Dinda suspects that the HDA code seeems to only support specific qemu versions. Please see this [Piazza](https://piazza.com/class/lfmvv2o1psc5fp/post/64) post for more detail.

## Instructions

1. `cd` into Nautilus folder that has the HDA code

2. Copy Professor Dinda's default config -- this will ensure we turn on the debug mode

```
cp ~pdinda/HANDOUT/cs446-nautilus-config .config
```

3. Compile the code and make an ISO image

```
make -j
make isoimage
```

4. `scp` the iso file (`nautilus.iso`) to your machine

5. Run the following command on your machine

```
qemu-system-x86_64 -smp 2 -m 2048 -vga std -serial stdio -device intel-hda -device hda-duplex -cdrom nautilus.iso
```

6. Run the following commands to play the different sounds

```
play 500 5
play_piano
play_champions
```
