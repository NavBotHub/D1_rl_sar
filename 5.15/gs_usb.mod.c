#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x24d702c7, "module_layout" },
	{ 0xc9edd5c5, "can_change_mtu" },
	{ 0x25cc9e82, "usb_deregister" },
	{ 0x379d1f1c, "usb_register_driver" },
	{ 0x6cee23fd, "alloc_can_skb" },
	{ 0xf12d9387, "can_fd_dlc2len" },
	{ 0x7cace5a, "alloc_canfd_skb" },
	{ 0x9e5e8fa, "can_rx_offload_irq_finish" },
	{ 0x40729c4a, "alloc_can_err_skb" },
	{ 0x91bdaa4c, "netif_tx_wake_queue" },
	{ 0xf3322143, "can_rx_offload_get_echo_skb" },
	{ 0x6e95ac0a, "can_free_echo_skb" },
	{ 0x6047ede6, "can_fd_len2dlc" },
	{ 0x666c3935, "consume_skb" },
	{ 0x9f32536, "can_put_echo_skb" },
	{ 0x4829a47e, "memcpy" },
	{ 0x54740038, "kfree_skb_reason" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xb0ed58a8, "usb_unanchor_urb" },
	{ 0x4a1ce9a1, "netdev_err" },
	{ 0x1ea45baf, "netif_device_detach" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0xffeedf6a, "delayed_work_timer_fn" },
	{ 0x862258db, "timecounter_init" },
	{ 0xc4f0da12, "ktime_get_with_offset" },
	{ 0x79a0e532, "usb_free_urb" },
	{ 0xb3142616, "usb_submit_urb" },
	{ 0x14ca4fe6, "usb_anchor_urb" },
	{ 0xdfcff954, "usb_alloc_urb" },
	{ 0x6bf76fe5, "can_rx_offload_enable" },
	{ 0x2ebe9537, "open_candev" },
	{ 0x9fa7184a, "cancel_delayed_work_sync" },
	{ 0xf7dd6035, "close_candev" },
	{ 0xb68879d6, "napi_disable" },
	{ 0x31909811, "cpu_hwcap_keys" },
	{ 0x14b89635, "arm64_const_caps_ready" },
	{ 0xb2fcb56d, "queue_delayed_work_on" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x6f9e763b, "timecounter_read" },
	{ 0xe64faa0d, "can_rx_offload_queue_tail" },
	{ 0x45438c23, "can_rx_offload_queue_sorted" },
	{ 0xbc3f2cb0, "timecounter_cyc2time" },
	{ 0x37a0cba, "kfree" },
	{ 0x962c8ae1, "usb_kill_anchored_urbs" },
	{ 0x4c9f0cc0, "unregister_candev" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x93c7edeb, "usb_find_common_endpoints" },
	{ 0x6c2e5b8, "free_candev" },
	{ 0x91d4d08, "can_rx_offload_del" },
	{ 0x26bae506, "register_candev" },
	{ 0x4bba31f7, "can_rx_offload_add_manual" },
	{ 0xe2d5255a, "strcmp" },
	{ 0x845d17cc, "_dev_info" },
	{ 0x1708e5a8, "alloc_candev_mqs" },
	{ 0x8feb96b0, "netdev_info" },
	{ 0xc3dc8e2e, "ethtool_op_get_ts_info" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xdcb764ad, "memset" },
	{ 0xc42c119c, "_dev_err" },
	{ 0xc3690fc, "_raw_spin_lock_bh" },
	{ 0xe46021ca, "_raw_spin_unlock_bh" },
	{ 0x1da43160, "usb_control_msg_recv" },
	{ 0x3ea1b6e4, "__stack_chk_fail" },
	{ 0xa77e59bd, "usb_control_msg_send" },
	{ 0x1fdc7df2, "_mcount" },
};

MODULE_INFO(depends, "can-dev");

MODULE_ALIAS("usb:v1D50p606Fd*dc*dsc*dp*ic*isc*ip*in00*");
MODULE_ALIAS("usb:v1209p2323d*dc*dsc*dp*ic*isc*ip*in00*");
MODULE_ALIAS("usb:v1CD2p606Fd*dc*dsc*dp*ic*isc*ip*in00*");
MODULE_ALIAS("usb:v16D0p10B8d*dc*dsc*dp*ic*isc*ip*in00*");
MODULE_ALIAS("usb:v16D0p0F30d*dc*dsc*dp*ic*isc*ip*in00*");
