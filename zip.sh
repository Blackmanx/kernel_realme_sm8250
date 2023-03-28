cp out/arch/arm64/boot/Image ./anykernel/

cd anykernel && zip -r Bigshot-kernel-$(date +"%d-%m-%Y"-%H%M).zip * && mv Bigshot-kernel-$(date +"%d-%m-%Y-%H%M").zip ../out/

cd ..
