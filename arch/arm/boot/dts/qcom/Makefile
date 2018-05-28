dtb-$(CONFIG_ARCH_MSM8996) += msm8996-v3-pmi8996-cdp.dtb \
	msm8996pro-pmi8996-mtp.dtb \
	msm8996-v3-pmi8996-mtp-zs570kl-SR1.dtb

ifeq ($(CONFIG_ARM64),y)
always          := $(dtb-y)
subdir-y        := $(dts-dirs)
else
targets += dtbs
targets += $(addprefix ../, $(dtb-y))

$(obj)/../%.dtb: $(src)/%.dts FORCE
        $(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/,$(dtb-y))
endif
clean-files := *.dtb
