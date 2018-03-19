#
# Makefile for the qti specific video device drivers
# based on V4L2.
#
obj-$(CONFIG_MSM_VIDC_V4L2) += vidc/
obj-$(CONFIG_MSM_VIDC_3X_V4L2) += vidc_3x/
obj-y += sde/
obj-$(CONFIG_SPECTRA_CAMERA) += camera/
obj-$(CONFIG_MSMB_CAMERA) += camera_v2/
obj-y += broadcast/
obj-$(CONFIG_DVB_MPQ) += dvb/
obj-$(CONFIG_QCA402X) += qca402/
ifeq ($(CONFIG_MACH_XIAOMI_LAND),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_v2-land/
else
obj-$(CONFIG_MSMB_CAMERA) += camera_v2/
endif
obj-$(CONFIG_MSM_AIS) += ais/
obj-y += sde/
