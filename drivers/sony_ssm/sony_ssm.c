/*
From 2f2b16bd129541f958f5771d32ba4e24fc5df839 Mon Sep 17 00:00:00 2001
From: Jalen He <jalen.he@sonymobile.com>
Date: Wed, 16 Jul 2014 15:40:23 +0800
Subject: [PATCH] msm: Added platform device for Super Stamina Mode

This device notifies user space when the platform is ready to,
or just entered, a specific power state.
The SUSPEND_PREPARE notification is sent when the platform is
preparing for suspend, i.e. upon PM_SUSPEND_PREPARE, which is
when tasks are frozen but before devices are suspended. It will
abort the suspend process to send this notification to user
space and then back off again and let the platform enter suspend
properly on the next attempt.
The LATE_RESUME notification is sent upon late_resume (through
subscription to early_suspend), i.e. when the display is being
switched on.
The device exposes sysfs attributes to let user space control
when it needs to be registered for these PM events from the
platform, and user space must also request for notifications
before they are actually sent. The notification itself is sent
in a kobject uevent.

Change-Id: Icb12e43feaaad2b681c09532d48572c80e4186cc
---
 drivers/Makefile            |   2 +
 drivers/sony_ssm/Makefile   |   1 +
 drivers/sony_ssm/sony_ssm.c | 371 ++++++++++++++++++++++++++++++++++++++++++++
 3 files changed, 374 insertions(+)
 create mode 100755 drivers/sony_ssm/Makefile
 create mode 100755 drivers/sony_ssm/sony_ssm.c

diff --git a/drivers/Makefile b/drivers/Makefile
index b94fcb1..3429843 100644
--- a/drivers/Makefile
+++ b/drivers/Makefile
@@ -153,3 +153,5 @@ obj-$(CONFIG_MTKPASR)	+= mtkpasr/
 obj-y   +=  debug_util/
 #20140603 tracy add for dtv
 obj-y	+=  nmi/nm32x/
+
+obj-y   += sony_xssm/
diff --git a/drivers/sony_ssm/Makefile b/drivers/sony_ssm/Makefile
new file mode 100755
index 0000000..c90127a
--- /dev/null
+++ b/drivers/sony_ssm/Makefile
@@ -0,0 +1 @@
+obj-y += sony_ssm.o
diff --git a/drivers/sony_ssm/sony_ssm.c b/drivers/sony_ssm/sony_ssm.c
new file mode 100755
index 0000000..63fba42
--- /dev/null
+++ b/drivers/sony_ssm/sony_ssm.c
@@ -0,0 +1,371 @@
*/
/* kernel/arch/arm/mach-ux500/sony_ssm.c
+ *
+ * Copyright (C) 2012 Sony Mobile Communications AB.
+ *
+ * Author: Mattias Larsson <mattias7.larsson@sonymobile.com>
+ *
+ * This program is free software; you can redistribute it and/or modify
+ * it under the terms of the GNU General Public License version 2, as
+ * published by the Free Software Foundation; either version 2
+ * of the License, or (at your option) any later version.
+ */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/suspend.h>
#include <linux/earlysuspend.h>

#define MODULE_NAME "sony_ssm"

enum ssm_state {
	SSM_SUSPEND_PREPARE = 0,
	SSM_LATE_RESUME = 1
};

struct ssm_data {
	struct device *dev;
	struct early_suspend early_suspend;
	struct notifier_block pm_notifier;
	//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
	//struct mutex lock;
	spinlock_t slock;
	//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^
	bool enabled;
	bool notify_next_suspend_prepare;
	bool notify_late_resume;
};

static void ssm_notify(struct ssm_data *sd, enum ssm_state state)
{  
	char event[8];
	char *envp[] = {event, NULL};
  printk(KERN_EMERG "sony_ssm ssm_notify  \n");
  
	snprintf(event, sizeof(event), "EVENT=%d", state);

	dev_dbg(sd->dev, "%s: Sending uevent EVENT=%d\n", __func__, state);

	kobject_uevent_env(&sd->dev->kobj, KOBJ_CHANGE, envp);
}

static void ssm_late_resume(struct early_suspend *h)
{  
	struct ssm_data *sd = container_of(h, struct ssm_data, early_suspend);
  printk(KERN_EMERG "sony_ssm ssm_late_resume  \n");
	dev_dbg(sd->dev, "%s\n", __func__);

	//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
	spin_lock(&sd->slock);
	//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^
	//<20150507-jimchen-kernel panic in sony_ssm ssm_enable
	//if (sd->notify_late_resume)
	//	ssm_notify(sd, SSM_LATE_RESUME);
	if (sd->notify_late_resume){
		spin_unlock(&sd->slock);
		ssm_notify(sd, SSM_LATE_RESUME);
		spin_lock(&sd->slock);
	}
	//>20150507-jimchen
	//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
	spin_unlock(&sd->slock);
	//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^
}

static int ssm_pm_notifier(struct notifier_block *nb, unsigned long event,
		void *ignored)
{  
	struct ssm_data *sd = container_of(nb, struct ssm_data, pm_notifier);
  printk(KERN_EMERG "sony_ssm ssm_pm_notifier  \n");
	dev_dbg(sd->dev, "%s: event=%lu\n", __func__, event);

	if (event == PM_SUSPEND_PREPARE) {
		//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
		spin_lock(&sd->slock);
		//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^
		if (sd->notify_next_suspend_prepare) {
			//<20150420-jimchen-kernel panic in sony_ssm ssm_enable
			spin_unlock(&sd->slock);
			ssm_notify(sd, SSM_SUSPEND_PREPARE);
			spin_lock(&sd->slock);
			//>20150420-jimchen
			sd->notify_next_suspend_prepare = false;
			//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
			spin_unlock(&sd->slock);
			//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^
			return NOTIFY_BAD;
		}
		//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
		spin_unlock(&sd->slock);
		//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^
	}

	return NOTIFY_DONE;
}

static int ssm_enable(struct ssm_data *sd)
{  
	int rc;
  printk(KERN_EMERG "sony_ssm ssm_enable  \n");
	dev_dbg(sd->dev, "%s\n", __func__);
	//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
	spin_lock(&sd->slock);
	//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^

	if (sd->enabled) {
		dev_err(sd->dev, "%s: Already enabled!\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	rc = register_pm_notifier(&sd->pm_notifier);
if (rc) {
		dev_err(sd->dev, "%s: Failed to register pm_notifier (%d)\n",
			__func__, rc);
		goto exit;
	}

        //<20150326-jimchen-kernel panic in sony_ssm ssm_enable
	spin_unlock(&sd->slock);
        //>20150326-jimchen

	register_early_suspend(&sd->early_suspend);

        //<20150326-jimchen-kernel panic in sony_ssm ssm_enable 
	spin_lock(&sd->slock);
        //>20150326-jimchen


	sd->notify_next_suspend_prepare = false;
	sd->notify_late_resume = false;
	sd->enabled = true;

exit:
	//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
	spin_unlock(&sd->slock);
	//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^
	return rc;
}

static void ssm_disable(struct ssm_data *sd)
{ 
	dev_dbg(sd->dev, "%s\n", __func__);
  printk(KERN_EMERG "sony_ssm ssm_disable  \n");
  //<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
	spin_lock(&sd->slock);
  //>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^
	if (sd->enabled) {
		unregister_early_suspend(&sd->early_suspend);
		unregister_pm_notifier(&sd->pm_notifier);
		sd->enabled = false;
	} else {
	dev_warn(sd->dev, "%s: Not enabled\n", __func__);
	}
	//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
	spin_unlock(&sd->slock);
	//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^
}

static ssize_t ssm_store_enable(struct device *pdev,
		struct device_attribute *attr, const char *pbuf, size_t count)
{  
	int rc;
	u8 val;
	struct ssm_data *sd = dev_get_drvdata(pdev);
  printk(KERN_EMERG "sony_ssm ssm_store_enable  \n");
	dev_dbg(sd->dev, "%s: %s\n", __func__, pbuf);

	rc = kstrtou8(pbuf, 2, &val);
	if (!rc) {
		if (!!val)
			rc = ssm_enable(sd);
		else
			ssm_disable(sd);
	}

	return rc == 0 ? count : rc;
}

static ssize_t ssm_store_request_next_suspend_prepare_notification(
		struct device *pdev, struct device_attribute *attr,
		const char *pbuf, size_t count)
{  
	int rc;
	u8 val;
	struct ssm_data *sd = dev_get_drvdata(pdev);
  printk(KERN_EMERG "sony_ssm ssm_store_request_next_suspend_prepare_notification  \n");
	dev_dbg(sd->dev, "%s: %s\n", __func__, pbuf);

	rc = kstrtou8(pbuf, 2, &val);
	if (!rc) {
		//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
		spin_lock(&sd->slock);
		//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^
		if (sd->enabled) {
			sd->notify_next_suspend_prepare = !!val;
		} else {
			rc = -EINVAL;
			dev_err(sd->dev, "%s: Notifications are not enabled\n",
				__func__);
		}
		//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
		spin_unlock(&sd->slock);
		//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^
	}

	return rc == 0 ? count : rc;
}

static ssize_t ssm_store_set_late_resume_notifications(struct device *pdev,
		struct device_attribute *attr, const char *pbuf, size_t count)
{  
	int rc;
	u8 val;
	struct ssm_data *sd = dev_get_drvdata(pdev);
  printk(KERN_EMERG "sony_ssm ssm_store_set_late_resume_notifications  \n");
	dev_dbg(sd->dev, "%s: %s\n", __func__, pbuf);

	rc = kstrtou8(pbuf, 2, &val);
	if (!rc) {
		//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
		spin_lock(&sd->slock);
		//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^
		if (sd->enabled) {
			sd->notify_late_resume = !!val;
		} else {
			rc = -EINVAL;
			dev_err(sd->dev, "%s: Notifications are not enabled\n",
				__func__);
		}
		//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
		spin_unlock(&sd->slock);
		//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^
	}

	return rc == 0 ? count : rc;
}

static struct device_attribute ssm_attrs[] = {
	__ATTR(enable, 0600, NULL,
			ssm_store_enable),
	__ATTR(set_request_next_suspend_prepare_notification, 0600, NULL,
			ssm_store_request_next_suspend_prepare_notification),
	__ATTR(set_late_resume_notifications, 0600, NULL,
			ssm_store_set_late_resume_notifications),
};

static int ssm_create_attrs(struct device *dev)
{  
	unsigned int i;
	printk(KERN_EMERG "sony_ssm ssm_create_attrs  \n");
	for (i = 0; i < ARRAY_SIZE(ssm_attrs); i++)
		if (device_create_file(dev, &ssm_attrs[i]))
			goto err;
	return 0;
err:
	while (i--)
		device_remove_file(dev, &ssm_attrs[i]);
	return -EIO;
}

static void ssm_remove_attrs(struct device *dev)
{  
	unsigned int i;
	printk(KERN_EMERG "sony_ssm ssm_remove_attrs  \n");
	for (i = 0; i < ARRAY_SIZE(ssm_attrs); i++)
		(void)device_remove_file(dev, &ssm_attrs[i]);
}

static int ssm_probe(struct platform_device *pdev)
{  
	int rc;
	struct ssm_data *sd;
  printk(KERN_EMERG "sony_ssm ssm_probe  \n");
	dev_dbg(&pdev->dev, "%s\n", __func__);

	sd = kzalloc(sizeof(struct ssm_data), GFP_KERNEL);
	if (!sd) {
		dev_err(&pdev->dev, "%s: OOM for ssm_data\n", __func__);
		return -ENOMEM;
	}

	sd->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	sd->early_suspend.resume = ssm_late_resume;

	sd->dev = &pdev->dev;

	//<20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system vvvvvvvvvvvvvvvv
	spin_lock_init(&sd->slock);
	//>20131007 genesis: GST report backlight can't turn on when voice call. Replace mutex_lock with spin_lock to fix deadlock in SMP system ^^^^^^^^^^^^^

	sd->enabled = false;
	sd->notify_next_suspend_prepare = false;
	sd->notify_late_resume = false;

	sd->pm_notifier.notifier_call = ssm_pm_notifier;
	sd->pm_notifier.priority = 0;

	rc = ssm_create_attrs(&pdev->dev);
	if (rc) {
		dev_err(sd->dev, "%s: Failed to create attrs (%d)\n",
			__func__, rc);
		goto err_create_attrs;
	}

	platform_set_drvdata(pdev, sd);

	return 0;

err_create_attrs:
	kfree(sd);
	return rc;
}

static int ssm_remove(struct platform_device *pdev)
{  
	struct ssm_data *sd = platform_get_drvdata(pdev);
  printk(KERN_EMERG "sony_ssm ssm_remove  \n");
	dev_dbg(sd->dev, "%s\n", __func__);

	if (sd->enabled) {
		unregister_early_suspend(&sd->early_suspend);
		unregister_pm_notifier(&sd->pm_notifier);
	}
	ssm_remove_attrs(sd->dev);
	kfree(sd);

	return 0;
}

static struct platform_device ssm_device = {
	.name = MODULE_NAME,
	.id = -1,
};

static struct platform_driver ssm_driver = {
	.probe = ssm_probe,
	.remove = ssm_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ssm_init(void)
{  
	int rc;
  printk(KERN_EMERG "sony_ssm ssm_init  \n");
	pr_debug("%s\n", __func__);

	rc = platform_driver_register(&ssm_driver);
	if (rc) {
		pr_err("%s: Failed to register driver (%d)\n", __func__, rc);
		return rc;
	}

	rc = platform_device_register(&ssm_device);
	if (rc) {
		platform_driver_unregister(&ssm_driver);
		pr_err("%s: Failed to register device (%d)\n", __func__, rc);
		return rc;
	}

	return 0;
}

static void __exit ssm_exit(void)
{
  printk(KERN_EMERG "sony_ssm ssm_exit  \n");
	pr_debug("%s\n", __func__);
	platform_device_unregister(&ssm_device);
	platform_driver_unregister(&ssm_driver);
}

late_initcall(ssm_init);
module_exit(ssm_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Power management notifications for Super Stamina Mode");
//-- 
//1.8.2.2

