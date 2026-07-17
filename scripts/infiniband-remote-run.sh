#!/bin/sh
ssh ib2 ./qemu-pci.sh \
    -machine q35,nvdimm=on \
    -bios RELEASEX64_OVMF.fd \
    -boot d \
    -rtc base=localtime \
    -serial stdio \
    -nic model=rtl8139,id=rtl8139,hostfwd=udp::1797-:1797,hostfwd=tcp::1797-:1797 \
    -object filter-dump,id=filter1,netdev=rtl8139,file=rtl8139.dump \
    -snapshot -hda http://juliusmac.local/d3os.img \
    -vnc :1
