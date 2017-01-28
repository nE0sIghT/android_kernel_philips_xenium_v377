include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/mms400/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/mms400/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  melfas_mms400.o
obj-y	+=  melfas_mms400_cmd.o
obj-y	+=  melfas_mms400_mod.o
obj-y	+=  melfas_mms400_test.o
obj-y	+=  melfas_mms438_fw_update.o
