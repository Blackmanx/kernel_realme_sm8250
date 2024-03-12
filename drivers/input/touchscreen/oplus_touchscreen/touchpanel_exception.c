// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include "touchpanel_exception.h"

#include <linux/err.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <asm/current.h>
#include <linux/version.h>

#include "touchpanel_common.h"

#ifdef TPD_DEVICE
#undef TPD_DEVICE
#define TPD_DEVICE "touchpanel"
#else
#define TPD_DEVICE "touchpanel"
#endif

#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)

static inline void *tp_kzalloc(size_t size, gfp_t flags)
{
	void *p;

	p = kzalloc(size, flags);

	if (!p) {
		TPD_INFO("%s: Failed to allocate memory\n", __func__);
		/*add for health monitor*/
	}

	return p;
}

static inline int tp_memcpy(void *dest, unsigned int dest_size,
			    void *src, unsigned int src_size,
			    unsigned int count)
{
	if (dest == NULL || src == NULL) {
		return -EINVAL;
	}

	if (count > dest_size || count > src_size) {
		TPD_INFO("%s: src_size = %d, dest_size = %d, count = %d\n",
			 __func__, src_size, dest_size, count);
		return -EINVAL;
	}

	memcpy((void *)dest, (void *)src, count);

	return 0;
}

static inline void tp_kfree(void **mem)
{
	if (*mem != NULL) {
		kfree(*mem);
		*mem = NULL;
	}
}

int tp_exception_report(void *tp_exception_data, tp_excep_type excep_tpye, void *summary, unsigned int summary_size)
{
	struct exception_data *exception_data = (struct exception_data *)tp_exception_data;

	if (!exception_data || !exception_data->exception_upload_support) {
		return 0;
	}
	exception_data->exception_upload_count++;
	switch (excep_tpye) {
	case EXCEP_BUS:
		/*bus error upload tow times*/
		exception_data->bus_error_upload_count++;
		if (exception_data->bus_error_count > MAX_BUS_ERROR_COUNT
				&& exception_data->bus_error_upload_count < 3) {
			exception_data->bus_error_count = 0;
		}
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(tp_exception_report);

