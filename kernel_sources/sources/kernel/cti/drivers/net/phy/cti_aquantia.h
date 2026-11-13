/* SPDX-License-Identifier: GPL-2.0 */
/* HWMON driver for Aquantia PHY
 *
 * Author: Nikita Yushchenko <nikita.yoush@cogentembedded.com>
 * Author: Andrew Lunn <andrew@lunn.ch>
 * Author: Heiner Kallweit <hkallweit1@gmail.com>
 */

#include <linux/device.h>
#include <linux/phy.h>

#if IS_REACHABLE(CONFIG_HWMON)
int cti_aqr113_hwmon_probe(struct phy_device *phydev);
#else
static inline int cti_aqr113_hwmon_probe(struct phy_device *phydev) { return 0; }
#endif

int aquantia_upload_firmware(struct phy_device *phydev);


