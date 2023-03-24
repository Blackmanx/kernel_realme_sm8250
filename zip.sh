cp out/arch/arm64/boot/Image ~/Documents/kernel/Files

cp out/arch/arm64/boot/dtbo.img ~/Documents/kernel/Files

cd ~/Documents/kernel/Files && zip -r Bigshot-kernel-$(date +"%d-%m-%Y"-%H%M).zip * && mv Bigshot-kernel-$(date +"%d-%m-%Y-%H%M").zip ~/Documents/kernel/

cd ~/Documents/kernel-T
