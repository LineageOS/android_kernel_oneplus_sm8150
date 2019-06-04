for make audio is work normal, please help add audio_tfa9894.ko to your BoardConfig and product.mk
as below patch show. Thanks

diff --git a/BoardConfig.mk b/BoardConfig.mk
index 695d3e2..d1fa101 100644
--- a/BoardConfig.mk
+++ b/BoardConfig.mk
@@ -108,6 +108,7 @@ BOARD_VENDOR_KERNEL_MODULES := \
     $(KERNEL_MODULES_OUT)/audio_mbhc.ko \
     $(KERNEL_MODULES_OUT)/audio_wcd934x.ko \
     $(KERNEL_MODULES_OUT)/audio_wcd9360.ko \
+    $(KERNEL_MODULES_OUT)/audio_tfa9894.ko \
     $(KERNEL_MODULES_OUT)/audio_wcd_spi.ko \
     $(KERNEL_MODULES_OUT)/audio_native.ko \
     $(KERNEL_MODULES_OUT)/audio_machine_msmnile.ko \
diff --git a/msmnile.mk b/msmnile.mk
index 1a8413b..9c2d719 100644
--- a/msmnile.mk
+++ b/msmnile.mk
@@ -156,6 +156,7 @@ AUDIO_DLKM += audio_wcd_spi.ko
 AUDIO_DLKM += audio_native.ko
 AUDIO_DLKM += audio_machine_msmnile.ko
 AUDIO_DLKM += audio_wcd934x.ko
+AUDIO_DLKM += audio_tfa9894.ko
 PRODUCT_PACKAGES += $(AUDIO_DLKM)

 PRODUCT_PACKAGES += fs_config_files
