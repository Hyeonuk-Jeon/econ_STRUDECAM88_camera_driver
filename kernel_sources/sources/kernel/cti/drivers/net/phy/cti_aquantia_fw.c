#define DEBUG
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mdio.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/of.h>

#include "cti_aquantia.h"

#define AQR113_FW_FILENAME "AQR113-G4_v5.5.6.cld"

/* addresses of memory segments in the phy */
#define DRAM_BASE_ADDR 0x3FFE0000
#define IRAM_BASE_ADDR 0x40000000

/* firmware image format constants */
#define VERSION_STRING_SIZE     0x40
#define VERSION_STRING_OFFSET   0x0200
#define HEADER_OFFSET           0x300

#define UP_CONTROL 0xc001
#define UP_RESET BIT(15)
#define UP_RUN_STALL_OVERRIDE BIT(6)
#define UP_RUN_STALL BIT(0)

#define MAILBOX_CONTROL 0x0200
#define MAILBOX_EXECUTE BIT(15)
#define MAILBOX_WRITE BIT(14)
#define MAILBOX_RESET_CRC BIT(12)
#define MAILBOX_BUSY BIT(8)

#define MAILBOX_CRC 0x0201

#define MAILBOX_ADDR_MSW 0x0202
#define MAILBOX_ADDR_LSW 0x0203

#define MAILBOX_DATA_MSW 0x0204
#define MAILBOX_DATA_LSW 0x0205

#define NVR_CONTROL 0x0100
#define NVR_EXECUTE BIT(15)
#define NVR_WRITE BIT(14)
#define NVR_RESET_CRC BIT(12)
#define NVR_BUSY BIT(8)

#define NVR_OP_CHIP_ERASE   0x60
#define NVR_OP_WRITE_EN     0x06

#define NVR_CRC 0x0101

#define NVR_ADDR_MSW 0x0102
#define NVR_ADDR_LSW 0x0103

#define NVR_DATA_MSW 0x0104
#define NVR_DATA_LSW 0x0105

#define GLOBAL_FIRMWARE_ID 0x20
#define GLOBAL_FAULT 0xc850
#define GLOBAL_RSTATUS_1 0xc885

#define GLOBAL_ALARM_1 0xcc00
#define SYSTEM_READY_BIT 0x40

#define GLOBAL_STANDARD_CONTROL 0x0
#define SOFT_RESET BIT(15)
#define LOW_POWER BIT(11)

#pragma pack(1)
struct fw_header {
	uint8_t padding[4];
	uint8_t iram_offset[3];
	uint8_t iram_size[3];
	uint8_t dram_offset[3];
	uint8_t dram_size[3];
};
#pragma pack()

struct fw_info
{
    uint32_t primary_offset;
    uint32_t iram_offset;
    uint32_t iram_size;
    uint32_t dram_offset;
    uint32_t dram_size;
    char version[VERSION_STRING_SIZE];
};

/* load data into the phy's memory */
static int aquantia_load_memory(struct phy_device *phydev, uint32_t addr,
				const uint8_t *data, size_t len)
{
	size_t pos;
//	uint16_t crc = 0, up_crc;

	phy_write_mmd(phydev, MDIO_MMD_VEND1, MAILBOX_CONTROL, MAILBOX_RESET_CRC);
	phy_write_mmd(phydev, MDIO_MMD_VEND1, MAILBOX_ADDR_MSW, addr >> 16);
	phy_write_mmd(phydev, MDIO_MMD_VEND1, MAILBOX_ADDR_LSW, addr & 0xfffc);

	for (pos = 0; pos < len; pos += min(sizeof(uint32_t), len - pos)) {
		uint32_t word = 0;

		memcpy(&word, &data[pos], min(sizeof(uint32_t), len - pos));

		phy_write_mmd(phydev, MDIO_MMD_VEND1, MAILBOX_DATA_MSW,
			  (word >> 16));
		phy_write_mmd(phydev, MDIO_MMD_VEND1, MAILBOX_DATA_LSW,
			  word & 0xffff);

		phy_write_mmd(phydev, MDIO_MMD_VEND1, MAILBOX_CONTROL,
			          MAILBOX_EXECUTE | MAILBOX_WRITE);

		/* keep a big endian CRC to match the phy processor */
		word = cpu_to_be32(word);
		//crc = crc16_ccitt(crc, (uint8_t *)&word, sizeof(word));
	}
/*
	up_crc = phy_read_mmd(phydev, MDIO_MMD_VEND1, MAILBOX_CRC);
	if (crc != up_crc) {
		phydev_err(phydev, "%s crc mismatch: calculated 0x%04hx phy 0x%04hx\n",
		       phydev->dev->name, crc, up_crc);
		return -EINVAL;
	}
*/
	return 0;
}


/* load data into the phy's memory */
static int aquantia_write_flash(struct phy_device *phydev,
				                const uint8_t *data, size_t len)
{
	//size_t pos;
//	uint16_t crc = 0, up_crc;

	phy_write_mmd(phydev, MDIO_MMD_VEND1, NVR_CONTROL, NVR_RESET_CRC);
	phy_write_mmd(phydev, MDIO_MMD_VEND1, NVR_ADDR_MSW, 0);
	phy_write_mmd(phydev, MDIO_MMD_VEND1, NVR_ADDR_LSW, 0);
	phy_write_mmd(phydev, MDIO_MMD_VEND1, NVR_DATA_MSW, 0);
    phy_write_mmd(phydev, MDIO_MMD_VEND1, NVR_DATA_LSW, 0);


    phydev_dbg(phydev, "Enabling Write\n");
    phy_write_mmd(phydev, MDIO_MMD_VEND1, NVR_CONTROL,
                  NVR_EXECUTE | NVR_WRITE | NVR_OP_WRITE_EN);

    phydev_dbg(phydev, "Erasing Flash\n");
    phy_write_mmd(phydev, MDIO_MMD_VEND1, NVR_CONTROL,
                  NVR_EXECUTE | NVR_WRITE | NVR_OP_CHIP_ERASE);

/*
	for (pos = 0; pos < len; pos += min(sizeof(uint32_t), len - pos)) {
		uint32_t word = 0;

		memcpy(&word, &data[pos], min(sizeof(uint32_t), len - pos));

		phy_write_mmd(phydev, MDIO_MMD_VEND1, NVR_DATA_MSW,
			  (word >> 16));
		phy_write_mmd(phydev, MDIO_MMD_VEND1, NVR_DATA_LSW,
			  word & 0xffff);

		phy_write_mmd(phydev, MDIO_MMD_VEND1, NVR_CONTROL,
			          NVR_EXECUTE | NVR_WRITE);

		// keep a big endian CRC to match the phy processor 
		word = cpu_to_be32(word);
		//crc = crc16_ccitt(crc, (uint8_t *)&word, sizeof(word));
	}
*/
/*
	up_crc = phy_read_mmd(phydev, MDIO_MMD_VEND1, MAILBOX_CRC);
	if (crc != up_crc) {
		phydev_err(phydev, "%s crc mismatch: calculated 0x%04hx phy 0x%04hx\n",
		       phydev->dev->name, crc, up_crc);
		return -EINVAL;
	}
*/
	return 0;
}



static uint32_t unpack_u24(const uint8_t *data)
{
	return (data[2] << 16) + (data[1] << 8) + data[0];
}

static int aquantia_get_fw_info(struct phy_device *phydev, 
                                struct fw_info* info, const struct firmware *fw)
{
    const struct fw_header *header;

    if(!info || !fw || fw->size == 0){
        return -EINVAL;
    }

    /* Find the DRAM and IRAM sections within the firmware file. */
    info->primary_offset = ((fw->data[9] & 0xf) << 8 | fw->data[8]) << 12;

    header = (struct fw_header *)&fw->data[info->primary_offset + HEADER_OFFSET];

    info->iram_offset = info->primary_offset + unpack_u24(header->iram_offset);
    info->iram_size = unpack_u24(header->iram_size);

    info->dram_offset = info->primary_offset + unpack_u24(header->dram_offset);
    info->dram_size = unpack_u24(header->dram_size);

    phydev_dbg(phydev, "primary %d iram offset=%d size=%d dram offset=%d size=%d\n",
               info->primary_offset, info->iram_offset, info->iram_size,
               info->dram_offset, info->dram_size);

    strlcpy(info->version,
            (char *)&fw->data[info->dram_offset + VERSION_STRING_OFFSET],
            VERSION_STRING_SIZE);

    phydev_info(phydev, "loading firmare version '%s'\n", info->version);

    return 0;
}

//static void aquantia_write_firmware(const struct firmware *fw, void* context)
static int aquantia_write_firmware(struct phy_device *phydev, const struct firmware *fw)
{
	int ret;
    struct fw_info info;

    if(!phydev || !fw){
        return -EINVAL;
    }

    ret = aquantia_get_fw_info(phydev, &info, fw);
    if (ret != 0) {
        phydev_err(phydev, "failed to get FW info\n");
        return ret;
    }
    
	/* stall the microcprocessor on the phy */
	phy_write_mmd(phydev, MDIO_MMD_VEND1, UP_CONTROL,
		          UP_RUN_STALL | UP_RUN_STALL_OVERRIDE);

	phydev_dbg(phydev, "loading dram 0x%08x from offset=%d size=%d\n",
	           DRAM_BASE_ADDR, info.dram_offset, info.dram_size);

	ret = aquantia_load_memory(phydev, DRAM_BASE_ADDR, 
                               &fw->data[info.dram_offset], info.dram_size);
	if (ret != 0){
        phydev_err(phydev, "failed to load DRAM\n");
        return ret;
    }

	phydev_dbg(phydev, "loading iram 0x%08x from offset=%d size=%d\n",
	           IRAM_BASE_ADDR, info.iram_offset, info.iram_size);

	ret = aquantia_load_memory(phydev, IRAM_BASE_ADDR, 
                               &fw->data[info.iram_offset], info.iram_size);
	if (ret != 0){
        phydev_err(phydev, "failed to load IRAM\n");
        return ret;
    }

	/* make sure soft reset and low power mode are clear */
	phy_write_mmd(phydev, MDIO_MMD_VEND1, GLOBAL_STANDARD_CONTROL, 0);

	/* Release the microprocessor. UP_RESET must be held for 100 usec. */
	phy_write_mmd(phydev, MDIO_MMD_VEND1, UP_CONTROL,
		          UP_RUN_STALL | UP_RUN_STALL_OVERRIDE | UP_RESET);

	udelay(100);

	phy_write_mmd(phydev, MDIO_MMD_VEND1, UP_CONTROL, UP_RUN_STALL_OVERRIDE);

	phydev_info(phydev, "firmare loading done.\n");

    //msleep(1000);

    return ret;
}


int aquantia_flash_firmware(struct phy_device *phydev)
{

    int ret;
    const struct firmware *fw;

    phydev_dbg(phydev, "Requesting FW\n");
/*
    ret = request_firmware_nowait(THIS_MODULE, 1, AQR113_FW_FILENAME, 
                                  &phydev->mdio.dev, GFP_KERNEL, 
                                  (void*)phydev, aquantia_write_firmware);
*/
    ret = request_firmware(&fw, AQR113_FW_FILENAME, &phydev->mdio.dev);

    if (ret != 0) {
        phydev_err(phydev, "failed to load firmware %s, ret: %d\n",
                   AQR113_FW_FILENAME, ret);
        return ret;
    }


	/* stall the microcprocessor on the phy */
	phy_write_mmd(phydev, MDIO_MMD_VEND1, UP_CONTROL,
		          UP_RUN_STALL | UP_RUN_STALL_OVERRIDE);


    ret = aquantia_write_flash(phydev, fw->data, fw->size);

    release_firmware(fw);

	/* make sure soft reset and low power mode are clear */
	phy_write_mmd(phydev, MDIO_MMD_VEND1, GLOBAL_STANDARD_CONTROL, 0);

	/* Release the microprocessor. UP_RESET must be held for 100 usec. */
	phy_write_mmd(phydev, MDIO_MMD_VEND1, UP_CONTROL,
		          UP_RUN_STALL | UP_RUN_STALL_OVERRIDE | UP_RESET);

	udelay(100);

	phy_write_mmd(phydev, MDIO_MMD_VEND1, UP_CONTROL, UP_RUN_STALL_OVERRIDE);

	phydev_info(phydev, "firmare flashing done.\n");

    return ret;

}

int aquantia_upload_firmware(struct phy_device *phydev)
{

    int ret;
    const struct firmware *fw;

    phydev_dbg(phydev, "Requesting FW\n");
/*
    ret = request_firmware_nowait(THIS_MODULE, 1, AQR113_FW_FILENAME, 
                                  &phydev->mdio.dev, GFP_KERNEL, 
                                  (void*)phydev, aquantia_write_firmware);
*/
    ret = request_firmware(&fw, AQR113_FW_FILENAME, &phydev->mdio.dev);

    if (ret != 0) {
        phydev_err(phydev, "failed to load firmware %s, ret: %d\n",
                   AQR113_FW_FILENAME, ret);
        return ret;
    }

    ret = aquantia_write_firmware(phydev, fw);

    release_firmware(fw);
    return ret;
}
