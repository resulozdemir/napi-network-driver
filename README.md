# NAPI Network Driver

An example **Linux kernel network driver** implementing the **NAPI (New API)** polling model.  The code is heavily commented and intended for students exploring kernel-level networking.

---

## Highlights

* **NAPI integration** – registers a `struct napi_struct`, uses `napi_schedule()` / `napi_complete()` for efficient RX handling under high load.
* **DMA ring buffers** – demonstrates coherent DMA allocation for TX/RX packet transfer.
* **Multicast & VLAN support** – shows how to program hardware filters and maintain a VLAN table.
* **Interrupt handling** – unified ISR processes RX/TX/error bits and wakes the network queue.
* **Config-time hooks** – implements `ndo_change_mtu`, `ndo_set_config`, etc.

---

## Build Instructions

1. Install kernel headers matching your running kernel.
2. Create a simple `Makefile`:
   ```makefile
   obj-m += Network-Driver.o
   all:
       make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
   clean:
       make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
   ```
3. Build:
   ```bash
   make
   ```
4. Load the module (replace IRQ / I/O addresses as needed):
   ```bash
   sudo insmod Network-Driver.ko
   ```
5. Check dmesg for init logs and confirm interface appears with `ip link`.

---

## Repository Layout

```
Network-Driver.c   # Kernel module source
README.md          # Documentation
```

---

## Disclaimer

This driver **does not interact with real hardware**; many register offsets are placeholders (`0xYOUR_*`). It serves only as an educational template.  
License: GPL-2.0
