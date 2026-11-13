/*
 *
 * File Name - sysfs_operations.c
 * Copyright (c) 2024-2025, Leopard CORPORATION.  All rights reserved.
 * Base on Copyright (c) 2014-2025, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <linux/module.h>


uint8_t sysfs_probe_dir;
struct kobject *kobj_ref_probe;
struct kobject *kobj_ref_probe_cam[8];
char probe_status_str[20];
uint8_t total_cam_probed;
extern uint8_t total_num_cam;


struct probe_status {
	char status[50];
	int percentage;
	char mcu_fw_status[50];
	int mcu_fw_status_percentage;
	char spi_fw_status[50];
	int spi_fw_status_percentage;
};

struct probe_status probe_status_array[8];







static ssize_t  sysfs_ecam_probe_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_num_cam_probed(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);
static ssize_t  sysfs_ecam_cam_0_probe_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_0_probe_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_0_mcu_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_0_mcu_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_0_spi_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_0_spi_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_1_probe_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_1_probe_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_1_mcu_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_1_mcu_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_1_spi_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_1_spi_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_2_probe_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_2_probe_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_2_mcu_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_2_mcu_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_2_spi_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_2_spi_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_3_probe_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_3_probe_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_3_mcu_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_3_mcu_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_3_spi_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_3_spi_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_4_probe_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_4_probe_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_4_mcu_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_4_mcu_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_4_spi_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_4_spi_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_5_probe_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_5_probe_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_5_mcu_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_5_mcu_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_5_spi_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_5_spi_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_6_probe_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_6_probe_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_6_mcu_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_6_mcu_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_6_spi_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_6_spi_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_7_probe_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_7_probe_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_7_mcu_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_7_mcu_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_7_spi_fw_status_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_7_spi_fw_status_percentage_check(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);


struct kobj_attribute probe_status_attr = __ATTR(status, 0440,
		sysfs_ecam_probe_status_check, NULL);

struct kobj_attribute num_cam_probed_attr = __ATTR(total_cam_probed, 0440,
		sysfs_ecam_num_cam_probed, NULL);
struct kobj_attribute cam_0_probe_status_attr = __ATTR(status, 0440,
		sysfs_ecam_cam_0_probe_status_check, NULL);

struct kobj_attribute cam_0_probe_status_percentage_attr = __ATTR(percentage, 0440,
		sysfs_ecam_cam_0_probe_status_percentage_check, NULL);

struct kobj_attribute cam_0_mcu_fw_status_attr = __ATTR(mcu_fw_status, 0440,
		sysfs_ecam_cam_0_mcu_fw_status_check, NULL);

struct kobj_attribute cam_0_mcu_fw_status_percentage_attr = __ATTR(mcu_fw_status_percentage, 0440,
		sysfs_ecam_cam_0_mcu_fw_status_percentage_check, NULL);

struct kobj_attribute cam_0_spi_fw_status_attr = __ATTR(spi_fw_status, 0440,
		sysfs_ecam_cam_0_spi_fw_status_check, NULL);

struct kobj_attribute cam_0_spi_fw_status_percentage_attr = __ATTR(spi_fw_status_percentage, 0440,
		sysfs_ecam_cam_0_spi_fw_status_percentage_check, NULL);

struct kobj_attribute cam_1_probe_status_attr = __ATTR(status, 0440,
		sysfs_ecam_cam_1_probe_status_check, NULL);

struct kobj_attribute cam_1_probe_status_percentage_attr = __ATTR(percentage, 0440,
		sysfs_ecam_cam_1_probe_status_percentage_check, NULL);

struct kobj_attribute cam_1_mcu_fw_status_attr = __ATTR(mcu_fw_status, 0440,
		sysfs_ecam_cam_1_mcu_fw_status_check, NULL);

struct kobj_attribute cam_1_mcu_fw_status_percentage_attr = __ATTR(mcu_fw_status_percentage, 0440,
		sysfs_ecam_cam_1_mcu_fw_status_percentage_check, NULL);

struct kobj_attribute cam_1_spi_fw_status_attr = __ATTR(spi_fw_status, 0440,
		sysfs_ecam_cam_1_spi_fw_status_check, NULL);

struct kobj_attribute cam_1_spi_fw_status_percentage_attr = __ATTR(spi_fw_status_percentage, 0440,
		sysfs_ecam_cam_1_spi_fw_status_percentage_check, NULL);

struct kobj_attribute cam_2_probe_status_attr = __ATTR(status, 0440,
		sysfs_ecam_cam_2_probe_status_check, NULL);

struct kobj_attribute cam_2_probe_status_percentage_attr = __ATTR(percentage, 0440,
		sysfs_ecam_cam_2_probe_status_percentage_check, NULL);

struct kobj_attribute cam_2_mcu_fw_status_attr = __ATTR(mcu_fw_status, 0440,
		sysfs_ecam_cam_2_mcu_fw_status_check, NULL);

struct kobj_attribute cam_2_mcu_fw_status_percentage_attr = __ATTR(mcu_fw_status_percentage, 0440,
		sysfs_ecam_cam_2_mcu_fw_status_percentage_check, NULL);

struct kobj_attribute cam_2_spi_fw_status_attr = __ATTR(spi_fw_status, 0440,
		sysfs_ecam_cam_2_spi_fw_status_check, NULL);

struct kobj_attribute cam_2_spi_fw_status_percentage_attr = __ATTR(spi_fw_status_percentage, 0440,
		sysfs_ecam_cam_2_spi_fw_status_percentage_check, NULL);

struct kobj_attribute cam_3_probe_status_attr = __ATTR(status, 0440,
		sysfs_ecam_cam_3_probe_status_check, NULL);

struct kobj_attribute cam_3_probe_status_percentage_attr = __ATTR(percentage, 0440,
		sysfs_ecam_cam_3_probe_status_percentage_check, NULL);

struct kobj_attribute cam_3_mcu_fw_status_attr = __ATTR(mcu_fw_status, 0440,
		sysfs_ecam_cam_3_mcu_fw_status_check, NULL);

struct kobj_attribute cam_3_mcu_fw_status_percentage_attr = __ATTR(mcu_fw_status_percentage, 0440,
		sysfs_ecam_cam_3_mcu_fw_status_percentage_check, NULL);

struct kobj_attribute cam_3_spi_fw_status_attr = __ATTR(spi_fw_status, 0440,
		sysfs_ecam_cam_3_spi_fw_status_check, NULL);

struct kobj_attribute cam_3_spi_fw_status_percentage_attr = __ATTR(spi_fw_status_percentage, 0440,
		sysfs_ecam_cam_3_spi_fw_status_percentage_check, NULL);

struct kobj_attribute cam_4_probe_status_attr = __ATTR(status, 0440,
		sysfs_ecam_cam_4_probe_status_check, NULL);

struct kobj_attribute cam_4_probe_status_percentage_attr = __ATTR(percentage, 0440,
		sysfs_ecam_cam_4_probe_status_percentage_check, NULL);

struct kobj_attribute cam_4_mcu_fw_status_attr = __ATTR(mcu_fw_status, 0440,
		sysfs_ecam_cam_4_mcu_fw_status_check, NULL);

struct kobj_attribute cam_4_mcu_fw_status_percentage_attr = __ATTR(mcu_fw_status_percentage, 0440,
		sysfs_ecam_cam_4_mcu_fw_status_percentage_check, NULL);

struct kobj_attribute cam_4_spi_fw_status_attr = __ATTR(spi_fw_status, 0440,
		sysfs_ecam_cam_4_spi_fw_status_check, NULL);

struct kobj_attribute cam_4_spi_fw_status_percentage_attr = __ATTR(spi_fw_status_percentage, 0440,
		sysfs_ecam_cam_4_spi_fw_status_percentage_check, NULL);

struct kobj_attribute cam_5_probe_status_attr = __ATTR(status, 0440,
		sysfs_ecam_cam_5_probe_status_check, NULL);

struct kobj_attribute cam_5_probe_status_percentage_attr = __ATTR(percentage, 0440,
		sysfs_ecam_cam_5_probe_status_percentage_check, NULL);

struct kobj_attribute cam_5_mcu_fw_status_attr = __ATTR(mcu_fw_status, 0440,
		sysfs_ecam_cam_5_mcu_fw_status_check, NULL);

struct kobj_attribute cam_5_mcu_fw_status_percentage_attr = __ATTR(mcu_fw_status_percentage, 0440,
		sysfs_ecam_cam_5_mcu_fw_status_percentage_check, NULL);

struct kobj_attribute cam_5_spi_fw_status_attr = __ATTR(spi_fw_status, 0440,
		sysfs_ecam_cam_5_spi_fw_status_check, NULL);

struct kobj_attribute cam_5_spi_fw_status_percentage_attr = __ATTR(spi_fw_status_percentage, 0440,
		sysfs_ecam_cam_5_spi_fw_status_percentage_check, NULL);

struct kobj_attribute cam_6_probe_status_attr = __ATTR(status, 0440,
		sysfs_ecam_cam_6_probe_status_check, NULL);

struct kobj_attribute cam_6_probe_status_percentage_attr = __ATTR(percentage, 0440,
		sysfs_ecam_cam_6_probe_status_percentage_check, NULL);

struct kobj_attribute cam_6_mcu_fw_status_attr = __ATTR(mcu_fw_status, 0440,
		sysfs_ecam_cam_6_mcu_fw_status_check, NULL);

struct kobj_attribute cam_6_mcu_fw_status_percentage_attr = __ATTR(mcu_fw_status_percentage, 0440,
		sysfs_ecam_cam_6_mcu_fw_status_percentage_check, NULL);

struct kobj_attribute cam_6_spi_fw_status_attr = __ATTR(spi_fw_status, 0440,
		sysfs_ecam_cam_6_spi_fw_status_check, NULL);

struct kobj_attribute cam_6_spi_fw_status_percentage_attr = __ATTR(spi_fw_status_percentage, 0440,
		sysfs_ecam_cam_6_spi_fw_status_percentage_check, NULL);

struct kobj_attribute cam_7_probe_status_attr = __ATTR(status, 0440,
		sysfs_ecam_cam_7_probe_status_check, NULL);

struct kobj_attribute cam_7_probe_status_percentage_attr = __ATTR(percentage, 0440,
		sysfs_ecam_cam_7_probe_status_percentage_check, NULL);

struct kobj_attribute cam_7_mcu_fw_status_attr = __ATTR(mcu_fw_status, 0440,
		sysfs_ecam_cam_7_mcu_fw_status_check, NULL);

struct kobj_attribute cam_7_mcu_fw_status_percentage_attr = __ATTR(mcu_fw_status_percentage, 0440,
		sysfs_ecam_cam_7_mcu_fw_status_percentage_check, NULL);

struct kobj_attribute cam_7_spi_fw_status_attr = __ATTR(spi_fw_status, 0440,
		sysfs_ecam_cam_7_spi_fw_status_check, NULL);

struct kobj_attribute cam_7_spi_fw_status_percentage_attr = __ATTR(spi_fw_status_percentage, 0440,
		sysfs_ecam_cam_7_spi_fw_status_percentage_check, NULL);



static ssize_t sysfs_ecam_probe_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_str);
}

static ssize_t sysfs_ecam_num_cam_probed(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", total_cam_probed);
}
static ssize_t sysfs_ecam_cam_0_probe_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[0].status);
}

static ssize_t sysfs_ecam_cam_0_probe_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[0].percentage);
}

static ssize_t sysfs_ecam_cam_0_mcu_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[0].mcu_fw_status);
}

static ssize_t sysfs_ecam_cam_0_mcu_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[0].mcu_fw_status_percentage);
}

static ssize_t sysfs_ecam_cam_0_spi_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[0].spi_fw_status);
}

static ssize_t sysfs_ecam_cam_0_spi_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[0].spi_fw_status_percentage);
}

static ssize_t sysfs_ecam_cam_1_probe_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[1].status);
}

static ssize_t sysfs_ecam_cam_1_probe_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[1].percentage);
}

static ssize_t sysfs_ecam_cam_1_mcu_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[1].mcu_fw_status);
}

static ssize_t sysfs_ecam_cam_1_mcu_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[1].mcu_fw_status_percentage);
}

static ssize_t sysfs_ecam_cam_1_spi_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[1].spi_fw_status);
}

static ssize_t sysfs_ecam_cam_1_spi_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[1].spi_fw_status_percentage);
}

static ssize_t sysfs_ecam_cam_2_probe_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[2].status);
}

static ssize_t sysfs_ecam_cam_2_probe_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[2].percentage);
}

static ssize_t sysfs_ecam_cam_2_mcu_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[2].mcu_fw_status);
}

static ssize_t sysfs_ecam_cam_2_mcu_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[2].mcu_fw_status_percentage);
}

static ssize_t sysfs_ecam_cam_2_spi_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[2].spi_fw_status);
}

static ssize_t sysfs_ecam_cam_2_spi_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[2].spi_fw_status_percentage);
}

static ssize_t sysfs_ecam_cam_3_probe_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[3].status);
}

static ssize_t sysfs_ecam_cam_3_probe_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[3].percentage);
}

static ssize_t sysfs_ecam_cam_3_mcu_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[3].mcu_fw_status);
}

static ssize_t sysfs_ecam_cam_3_mcu_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[3].mcu_fw_status_percentage);
}

static ssize_t sysfs_ecam_cam_3_spi_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[3].spi_fw_status);
}

static ssize_t sysfs_ecam_cam_3_spi_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[3].spi_fw_status_percentage);
}

static ssize_t sysfs_ecam_cam_4_probe_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[4].status);
}

static ssize_t sysfs_ecam_cam_4_probe_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[4].percentage);
}

static ssize_t sysfs_ecam_cam_4_mcu_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[4].mcu_fw_status);
}

static ssize_t sysfs_ecam_cam_4_mcu_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[4].mcu_fw_status_percentage);
}

static ssize_t sysfs_ecam_cam_4_spi_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[4].spi_fw_status);
}

static ssize_t sysfs_ecam_cam_4_spi_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[4].spi_fw_status_percentage);
}


static ssize_t sysfs_ecam_cam_5_probe_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[5].status);
}

static ssize_t sysfs_ecam_cam_5_probe_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[5].percentage);
}

static ssize_t sysfs_ecam_cam_5_mcu_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[5].mcu_fw_status);
}

static ssize_t sysfs_ecam_cam_5_mcu_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[5].mcu_fw_status_percentage);
}

static ssize_t sysfs_ecam_cam_5_spi_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[5].spi_fw_status);
}

static ssize_t sysfs_ecam_cam_5_spi_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[5].spi_fw_status_percentage);
}


static ssize_t sysfs_ecam_cam_6_probe_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[6].status);
}

static ssize_t sysfs_ecam_cam_6_probe_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[6].percentage);
}

static ssize_t sysfs_ecam_cam_6_mcu_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[6].mcu_fw_status);
}

static ssize_t sysfs_ecam_cam_6_mcu_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[6].mcu_fw_status_percentage);
}

static ssize_t sysfs_ecam_cam_6_spi_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[6].spi_fw_status);
}

static ssize_t sysfs_ecam_cam_6_spi_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[6].spi_fw_status_percentage);
}


static ssize_t sysfs_ecam_cam_7_probe_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[7].status);
}

static ssize_t sysfs_ecam_cam_7_probe_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[7].percentage);
}

static ssize_t sysfs_ecam_cam_7_mcu_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[7].mcu_fw_status);
}

static ssize_t sysfs_ecam_cam_7_mcu_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[7].mcu_fw_status_percentage);
}

static ssize_t sysfs_ecam_cam_7_spi_fw_status_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", probe_status_array[7].spi_fw_status);
}

static ssize_t sysfs_ecam_cam_7_spi_fw_status_percentage_check(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", probe_status_array[7].spi_fw_status_percentage);
}




/*
 * Creates sysfs dir to store the probing status of our Driver
 */
uint8_t create_probe_sysfs_entries(void)
{
	int err = 0;

	if (!sysfs_probe_dir) {
		pr_info("*********Creating Sysfs for probe status in /sys/kernel***********");
		kobj_ref_probe = kobject_create_and_add("ecam_probe_sysfs", kernel_kobj);

		/*Creating sysfs file for Probe Status*/
		if (sysfs_create_file(kobj_ref_probe, &probe_status_attr.attr))
			pr_err("Cannot create sysfs file......\n");

		sprintf(probe_status_str, "In Progress");

		if (sysfs_create_file(kobj_ref_probe, &num_cam_probed_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		sysfs_probe_dir = 1;
	}
	return err;
}

/*
 * Creates camera specific sysfs entries
 */
uint8_t create_cam_specific_sysfs_entries(uint8_t num_cam, uint8_t cam_id)
{
	uint8_t err = 0;
	char buf[10];

	sprintf(buf, "cam_%d", num_cam);
	kobj_ref_probe_cam[num_cam] = kobject_create_and_add(buf, kobj_ref_probe);

	if (num_cam == 0) {
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_0_probe_status_attr.attr))
			pr_err("Cannot create sysfs file......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_0_probe_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_0_mcu_fw_status_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_0_mcu_fw_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (cam_id == 1) {
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_0_spi_fw_status_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_0_spi_fw_status_percentage_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
		}
	} else if (num_cam == 1) {
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_1_probe_status_attr.attr))
			pr_err("Cannot create sysfs file......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_1_probe_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_1_mcu_fw_status_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_1_mcu_fw_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (cam_id == 1) {
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_1_spi_fw_status_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_1_spi_fw_status_percentage_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
		}
	} else if (num_cam == 2) {
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_2_probe_status_attr.attr))
			pr_err("Cannot create sysfs file......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_2_probe_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_2_mcu_fw_status_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_2_mcu_fw_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (cam_id == 1) {
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_2_spi_fw_status_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_2_spi_fw_status_percentage_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
		}
	} else if (num_cam == 3) {
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_3_probe_status_attr.attr))
			pr_err("Cannot create sysfs file......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_3_probe_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_3_mcu_fw_status_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_3_mcu_fw_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (cam_id == 1) {
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_3_spi_fw_status_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_3_spi_fw_status_percentage_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
		}
	} else if (num_cam == 4) {
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_4_probe_status_attr.attr))
			pr_err("Cannot create sysfs file......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_4_probe_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_4_mcu_fw_status_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_4_mcu_fw_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (cam_id == 1) {
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_4_spi_fw_status_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_4_spi_fw_status_percentage_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
		}
	} else if (num_cam == 5) {
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_5_probe_status_attr.attr))
			pr_err("Cannot create sysfs file......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_5_probe_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_5_mcu_fw_status_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_5_mcu_fw_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (cam_id == 1) {
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_5_spi_fw_status_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_5_spi_fw_status_percentage_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
		}
	} else if (num_cam == 6) {
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_6_probe_status_attr.attr))
			pr_err("Cannot create sysfs file......\n");

		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_6_probe_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");

		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_6_mcu_fw_status_attr.attr))
			pr_err("Cannot create sysfs file.......\n");

		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_6_mcu_fw_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (cam_id == 1) {
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_6_spi_fw_status_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_6_spi_fw_status_percentage_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
		}
	} else if (num_cam == 7) {
		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_7_probe_status_attr.attr))
			pr_err("Cannot create sysfs file......\n");

		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_7_probe_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");

		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_7_mcu_fw_status_attr.attr))
			pr_err("Cannot create sysfs file.......\n");

		if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_7_mcu_fw_status_percentage_attr.attr))
			pr_err("Cannot create sysfs file.......\n");
		if (cam_id == 1) {
			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_7_spi_fw_status_attr.attr))
				pr_err("Cannot create sysfs file.......\n");

			if (sysfs_create_file(kobj_ref_probe_cam[num_cam], &cam_7_spi_fw_status_percentage_attr.attr))
				pr_err("Cannot create sysfs file.......\n");
		}
	}

	sprintf(probe_status_array[num_cam].status, "Probing....");
	probe_status_array[num_cam].percentage = 5;
	sprintf(probe_status_array[num_cam].mcu_fw_status, "Checking for Update...");
	probe_status_array[num_cam].mcu_fw_status_percentage = 0;
	if (cam_id == 1) {
		sprintf(probe_status_array[num_cam].spi_fw_status, "Checking for Update");
		probe_status_array[num_cam].spi_fw_status_percentage = 0;
	}

	return err;
}
