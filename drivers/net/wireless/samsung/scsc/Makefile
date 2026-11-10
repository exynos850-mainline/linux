# SPDX-License-Identifier: GPL-2.0-only
ccflags-y += -I$(src)/include -I$(src)/core

obj-$(CONFIG_SCSC_WLBT) += scsc_mx.o
scsc_mx-y := \
	core/cpacket_buffer.o \
	core/fw_panic_record.o \
	core/fwhdr.o \
	core/fwimage.o \
	core/gdb_transport.o \
	core/mifintrbit.o \
	core/mifmboxman.o \
	core/miframman.o \
	core/mifstream.o \
	core/mx140_file.o \
	core/mxfwconfig.o \
	core/mxlog.o \
	core/mxlog_transport.o \
	core/mxman.o \
	core/mxmgmt_transport.o \
	core/mxsyserr.o \
	core/panicmon.o \
	core/platform_mif_exynos850.o \
	core/platform_mif_module.o \
	core/scsc_mx_impl.o \
	core/scsc_mx_module.o \
	core/scsc_service.o \
	core/suspendmon.o

obj-$(CONFIG_SCSC_WLAN) += scsc_wlan.o
scsc_wlan-y := \
	wlan/ba.o \
	wlan/cac.o \
	wlan/cfg80211_ops.o \
	wlan/cm_if.o \
	wlan/debug.o \
	wlan/debug_frame.o \
	wlan/dev.o \
	wlan/hip.o \
	wlan/hip4.o \
	wlan/mbulk.o \
	wlan/mgt.o \
	wlan/mib.o \
	wlan/mib_text_convert.o \
	wlan/mlme.o \
	wlan/netif.o \
	wlan/reg_info.o \
	wlan/rx.o \
	wlan/sap_dbg.o \
	wlan/sap_ma.o \
	wlan/sap_mlme.o \
	wlan/sap_test.o \
	wlan/scsc_wifi_fcq.o \
	wlan/traffic_monitor.o \
	wlan/tx.o
