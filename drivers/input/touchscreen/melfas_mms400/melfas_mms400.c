/***********uniscope-driver-modify-file-on-qualcomm-platform*****************/
 * MELFAS MMS400 Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * This module is tested on the Google AOSP (Nexus) platforms.
 *
 * Board Type : Maguro (Google Galaxy Nexus) - Android 4.3 with Kernel 3.0
 * DeviceTree Type : Hammerhead (Google Nexus 5) - Android 5.0 with Kernel 3.4
 *
 */

#include "melfas_mms400.h"
#include <linux/notifier.h>
#include <linux/fb.h>

#define PINCTRL_STATE_ACTIVE	"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_ts_release"

#if MMS_USE_NAP_MODE
struct wake_lock mms_wake_lock;
#endif

static unsigned int mms_wakeup_control_flag = 0;  

/**
* Reboot chip
*
* Caution : IRQ must be disabled before mms_reboot and enabled after mms_reboot.
*/
void mms_reboot(struct mms_ts_info *info)
{
	struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);
		
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);
	
	i2c_lock_adapter(adapter);
	
	mms_power_off(info);
	mms_power_on(info);

	i2c_unlock_adapter(adapter);

	msleep(10);

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}

/**
* I2C Read
*/
int mms_i2c_read(struct mms_ts_info *info, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len)
{
	int retry = I2C_RETRY_COUNT;
	int res;

	struct i2c_msg msg[] = {
		{
			.addr = info->client->addr,
			.flags = 0,
			.buf = write_buf,
			.len = write_len,
		}, {
			.addr = info->client->addr,
			.flags = I2C_M_RD,
			.buf = read_buf,
			.len = read_len,
		},
	};

	while(retry--){	
		res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));

		if(res == ARRAY_SIZE(msg)){
			goto DONE;
		}
		else if(res < 0){
			dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n", __func__, res);
		}
		else if(res != ARRAY_SIZE(msg)){
			dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, ARRAY_SIZE(msg), res);
		}			
		else{
			dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n", __func__, res);
		}
	}

	goto ERROR_REBOOT;
	
ERROR_REBOOT:
	mms_reboot(info);
	return 1;
	
DONE:
	return 0;
}


/**
* I2C Read (Continue)
*/
int mms_i2c_read_next(struct mms_ts_info *info, char *read_buf, int start_idx, unsigned int read_len)
{
	int retry = I2C_RETRY_COUNT;
	int res;
	u8 rbuf[read_len];

	while(retry--){
		res = i2c_master_recv(info->client, rbuf, read_len);
		
		if(res == read_len){
			goto DONE;
		}
		else if(res < 0){
			dev_err(&info->client->dev, "%s [ERROR] i2c_master_recv - errno [%d]\n", __func__, res);
		}
		else if(res != read_len){
			dev_err(&info->client->dev, "%s [ERROR] length mismatch - read[%d] result[%d]\n", __func__, read_len, res);
		}			
		else{
			dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n", __func__, res);
		}
	}

	goto ERROR_REBOOT;
	
ERROR_REBOOT:
	mms_reboot(info);
	return 1;

DONE:
	memcpy(&read_buf[start_idx], rbuf, read_len);
	
	return 0;
}

/**
* I2C Write
*/
int mms_i2c_write(struct mms_ts_info *info, char *write_buf, unsigned int write_len)
{
	int retry = I2C_RETRY_COUNT;
	int res;

	while(retry--){
		res = i2c_master_send(info->client, write_buf, write_len);

		if(res == write_len){
			goto DONE;
		}
		else if(res < 0){
			dev_err(&info->client->dev, "%s [ERROR] i2c_master_send - errno [%d]\n", __func__, res);
		}
		else if(res != write_len){
			dev_err(&info->client->dev, "%s [ERROR] length mismatch - write[%d] result[%d]\n", __func__, write_len, res);
		}			
		else{
			dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n", __func__, res);
		}
	}
	
	goto ERROR_REBOOT;
	
ERROR_REBOOT:
	mms_reboot(info);
	return 1;
	
DONE:
	return 0;
}

/**
* Enable device
*/
int mms_enable(struct mms_ts_info *info)
{
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);
	
	if (info->enabled){
		dev_err(&info->client->dev, "%s [ERROR] device already enabled\n", __func__);
		goto EXIT;
	}

	mutex_lock(&info->lock);
	regulator_power_on(info);
	mms_power_on(info);
	enable_irq(info->client->irq);
	info->enabled = true;
	
	mutex_unlock(&info->lock);

	//Post-enable process
	if(info->disable_esd == true){
		//Disable ESD alert
		mms_disable_esd_alert(info);
	}	
	
EXIT:
	dev_info(&info->client->dev, MMS_DEVICE_NAME" Enabled\n");
	
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

/**
* Disable device
*/
int mms_disable(struct mms_ts_info *info)
{
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);
	
	if (!info->enabled){
		dev_err(&info->client->dev, "%s [ERROR] device already disabled\n", __func__);
		goto EXIT;
	}
	
	mutex_lock(&info->lock);

	info->enabled = false;
	disable_irq(info->client->irq);
	pr_err("tptptp  mms_disable\n");
	regulator_power_off(info);
	//mms_power_off(info);

	mutex_unlock(&info->lock);
	
EXIT:	
	dev_info(&info->client->dev, MMS_DEVICE_NAME" Disabled\n");
	
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

#if MMS_USE_INPUT_OPEN_CLOSE
/**
* Open input device
*/
static int mms_input_open(struct input_dev *dev) 
{
	struct mms_ts_info *info = input_get_drvdata(dev);
	
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	if(info->init == true){
		info->init = false;
	} 
	else{
		mms_enable(info);
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	
	return 0;
}

/**
* Close input device
*/
static void mms_input_close(struct input_dev *dev) 
{
	struct mms_ts_info *info = input_get_drvdata(dev);

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	mms_disable(info);

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return;
}
#endif

/**
* Get ready status
*/
int mms_get_ready_status(struct mms_ts_info *info)
{
	u8 wbuf[16];
	u8 rbuf[16];
	int ret = 0;
	
	//dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_READY_STATUS;
	if(mms_i2c_read(info, wbuf, 2, rbuf, 1)){
		dev_err(&info->client->dev, "%s [ERROR] mms_i2c_read\n", __func__);
		goto ERROR;
	}
	ret = rbuf[0];

	//check status
	if((ret == MIP_CTRL_STATUS_NONE) || (ret == MIP_CTRL_STATUS_LOG) || (ret == MIP_CTRL_STATUS_READY)){
		//dev_dbg(&info->client->dev, "%s - status [0x%02X]\n", __func__, ret);
	}
	else{
		dev_err(&info->client->dev, "%s [ERROR] Unknown status [0x%02X]\n", __func__, ret);
		goto ERROR;
	}

	if(ret == MIP_CTRL_STATUS_LOG){
		//skip log event
		wbuf[0] = MIP_R0_LOG;
		wbuf[1] = MIP_R1_LOG_TRIGGER;
		wbuf[2] = 0;
		if(mms_i2c_write(info, wbuf, 3)){
			dev_err(&info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
		}
	}
	
	//dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return ret;
	
ERROR:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return -1;
}

/**
* Read chip firmware version
*/
int mms_get_fw_version(struct mms_ts_info *info, u8 *ver_buf)
{
	u8 rbuf[8];
	u8 wbuf[2];
	int i;
	
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_VERSION_BOOT;
	if(mms_i2c_read(info, wbuf, 2, rbuf, 8)){
		goto ERROR;
	};

	for(i = 0; i < MMS_FW_MAX_SECT_NUM; i++){
		ver_buf[0 + i * 2] = rbuf[1 + i * 2];
		ver_buf[1 + i * 2] = rbuf[0 + i * 2];
	}	
	
	return 0;

ERROR:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;	
}

/**
* Read chip firmware version for u16
*/
int mms_get_fw_version_u16(struct mms_ts_info *info, u16 *ver_buf_u16)
{
	u8 rbuf[8];
	int i;
	
	if(mms_get_fw_version(info, rbuf)){
		goto ERROR;
	}

	for(i = 0; i < MMS_FW_MAX_SECT_NUM; i++){
		ver_buf_u16[i] = (rbuf[0 + i * 2] << 8) | rbuf[1 + i * 2];
	}	
	
	return 0;

ERROR:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;	
}

/**
* Disable ESD alert
*/
int mms_disable_esd_alert(struct mms_ts_info *info)
{
	u8 wbuf[4];
	u8 rbuf[4];
	
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);
	
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_DISABLE_ESD_ALERT;
	wbuf[2] = 1;
	if(mms_i2c_write(info, wbuf, 3)){
		dev_err(&info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
		goto ERROR;
	}	

	if(mms_i2c_read(info, wbuf, 2, rbuf, 1)){
		dev_err(&info->client->dev, "%s [ERROR] mms_i2c_read\n", __func__);
		goto ERROR;
	}	

	if(rbuf[0] != 1){
		dev_dbg(&info->client->dev, "%s [ERROR] failed\n", __func__);
		goto ERROR;
	}
	
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
	
ERROR:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/**
* Alert event handler - ESD
*/
static int mms_alert_handler_esd(struct mms_ts_info *info, u8 *rbuf)
{
	u8 frame_cnt = rbuf[2];
	
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	dev_dbg(&info->client->dev, "%s - frame_cnt[%d]\n", __func__, frame_cnt);

	if(frame_cnt == 0){
		//sensor crack, not ESD
		info->esd_cnt++;
		dev_dbg(&info->client->dev, "%s - esd_cnt[%d]\n", __func__, info->esd_cnt);

		if(info->disable_esd == true){
			mms_disable_esd_alert(info);
		}
		else if(info->esd_cnt > ESD_COUNT_FOR_DISABLE){
			//Disable ESD alert
			if(mms_disable_esd_alert(info)){
			}
			else{
				info->disable_esd = true;
			}
		}
		else{
			//Reset chip
			mms_reboot(info);
		}
	}
	else{
		//ESD detected
		//Reset chip
		mms_reboot(info);
		info->esd_cnt = 0;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

//ERROR:	
	//dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	//return 1;
}

/**
* Alert event handler - Wake-up
*/
static int mms_alert_handler_wakeup(struct mms_ts_info *info, u8 *rbuf)
{
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	if(mms_wakeup_event_handler(info, rbuf)){
		goto ERROR;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
	
ERROR:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/**
* Interrupt handler
*/
static irqreturn_t mms_interrupt(int irq, void *dev_id)
{
	struct mms_ts_info *info = dev_id;
	struct i2c_client *client = info->client;
	u8 wbuf[8];
	u8 rbuf[256];
	unsigned int size = 0;
	int event_size = info->event_size;
	u8 category = 0;
	u8 alert_type = 0;
	
	dev_dbg(&client->dev, "%s [START]\n", __func__);

	pr_err("mms_interrupt_mms_interrupt\n");
 	//Read first packet
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
	if(mms_i2c_read(info, wbuf, 2, rbuf, (1 + event_size))){
		dev_err(&client->dev, "%s [ERROR] Read packet info\n", __func__);
		goto ERROR;
	}

	dev_dbg(&client->dev, "%s - info [0x%02X]\n", __func__, rbuf[0]);	
	
	//Check event
	size = (rbuf[0] & 0x7F);
	
	dev_dbg(&client->dev, "%s - packet size [%d]\n", __func__, size);	

	category = ((rbuf[0] >> 7) & 0x1);
	if(category == 0){
		//Touch event
		if(size > event_size){
			//Read next packet
			if(mms_i2c_read_next(info, rbuf, (1 + event_size), (size - event_size))){
				dev_err(&client->dev, "%s [ERROR] Read next packet\n", __func__);
				goto ERROR;
			}
		}
		
		info->esd_cnt = 0;
		
		mms_input_event_handler(info, size, rbuf);
	}
	else{
		//Alert event
		alert_type = rbuf[1];
		
		dev_dbg(&client->dev, "%s - alert type [%d]\n", __func__, alert_type);
				
		if(alert_type == MIP_ALERT_ESD){
			//ESD detection
			if(mms_alert_handler_esd(info, rbuf)){
				goto ERROR;
			}
		}
		else if(alert_type == MIP_ALERT_WAKEUP){
			//Wake-up gesture
			if(mms_alert_handler_wakeup(info, rbuf)){
				goto ERROR;
			}
		}
		else{
			dev_err(&client->dev, "%s [ERROR] Unknown alert type [%d]\n", __func__, alert_type);
			goto ERROR;
		}		
	}

	dev_dbg(&client->dev, "%s [DONE]\n", __func__);
	return IRQ_HANDLED;
	
ERROR:
	if(RESET_ON_EVENT_ERROR){	
		dev_info(&client->dev, "%s - Reset on error\n", __func__);
		
		mms_disable(info);
		mms_clear_input(info);
		mms_enable(info);
	}

	dev_err(&client->dev, "%s [ERROR]\n", __func__);
	return IRQ_HANDLED;
}

/**
* Update firmware from kernel built-in binary
*/
int mms_fw_update_from_kernel(struct mms_ts_info *info)
{
	const char *fw_name = INTERNAL_FW_PATH;
	const struct firmware *fw;
	int retires = 3;
	int ret;
	
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	request_firmware(&fw, fw_name, &info->client->dev);
	
	if (!fw) {
		dev_err(&info->client->dev, "%s [ERROR] request_firmware\n", __func__);
		goto ERROR;
	}

	do {
		ret = mms_flash_fw(info, fw->data, fw->size, false, true);
		if(ret >= fw_err_none){
			break;
		}
	} while (--retires);

	if (!retires) {
		dev_err(&info->client->dev, "%s [ERROR] mms_flash_fw failed\n", __func__);
		goto ERROR;
	}
	
	release_firmware(fw);

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

ERROR:
	release_firmware(fw);
	
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return -1;
}

/**
* Update firmware from external storage
*/
int mms_fw_update_from_storage(struct mms_ts_info *info, bool force)
{
	struct file *fp; 
	mm_segment_t old_fs;
	size_t fw_size, nread;
	int ret = 0;
	
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	mutex_lock(&info->lock);	
 	disable_irq(info->client->irq);
	
	old_fs = get_fs();
	set_fs(KERNEL_DS);  
	fp = filp_open(EXTERNAL_FW_PATH, O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		dev_err(&info->client->dev, "%s [ERROR] file_open - path[%s]\n", __func__, EXTERNAL_FW_PATH);
		ret = fw_err_file_open;
		goto ERROR;
	}
	
 	fw_size = fp->f_path.dentry->d_inode->i_size;
	if (0 < fw_size) {
		unsigned char *fw_data;
		fw_data = kzalloc(fw_size, GFP_KERNEL);
		nread = vfs_read(fp, (char __user *)fw_data, fw_size, &fp->f_pos);
		dev_dbg(&info->client->dev, "%s - path [%s] size [%u]\n", __func__,EXTERNAL_FW_PATH, fw_size);
		
		if (nread != fw_size) {
			dev_err(&info->client->dev, "%s [ERROR] vfs_read - size[%d] read[%d]\n", __func__, fw_size, nread);
			ret = fw_err_file_read;
		}
		else{
			ret = mms_flash_fw(info, fw_data, fw_size, force, true);
		}
		
		kfree(fw_data);
	}
	else{
		dev_err(&info->client->dev, "%s [ERROR] fw_size [%d]\n", __func__, fw_size);
		ret = fw_err_file_read;
	}
	
 	filp_close(fp, current->files);

ERROR:
	set_fs(old_fs);	
	enable_irq(info->client->irq);	
	mutex_unlock(&info->lock);

	if(ret == 0){
		dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	}
	else{
		dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	}
	
	return ret;
}

static ssize_t mms_sys_fw_update(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	int result = 0;
	u8 data[255];
	int ret = 0;
	
	memset(info->print_buf, 0, PAGE_SIZE);

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	ret = mms_fw_update_from_storage(info, true);
	
	switch(ret){
		case fw_err_none:
			sprintf(data, "F/W update success.\n");
			break;
		case fw_err_uptodate:
			sprintf(data, "F/W is already up-to-date.\n");
			break;
		case fw_err_download:
			sprintf(data, "F/W update failed : Download error\n");
			break;
		case fw_err_file_type:
			sprintf(data, "F/W update failed : File type error\n");
			break;
		case fw_err_file_open:			
			sprintf(data, "F/W update failed : File open error [%s]\n", EXTERNAL_FW_PATH);
			break;
		case fw_err_file_read:
			sprintf(data, "F/W update failed : File read error\n");
			break;
		default:
			sprintf(data, "F/W update failed.\n");
			break;
	}
	
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	
	strcat(info->print_buf, data);
	result = snprintf(buf, PAGE_SIZE, "%s\n", info->print_buf);
	return result;
}
static DEVICE_ATTR(fw_update, 0666, mms_sys_fw_update, NULL);

/**
* Sysfs attr info
*/
static struct attribute *mms_attrs[] = {
	&dev_attr_fw_update.attr,
	NULL,
};

/**
* Sysfs attr group info
*/
static const struct attribute_group mms_attr_group = {
	.attrs = mms_attrs,
};

/**
* Initial config
*/
static int mms_init_config(struct mms_ts_info *info)
{
	u8 wbuf[8];
	u8 rbuf[64];
	
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	pr_err("ppppppp-ppppppppp\n");
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_PRODUCT_NAME;
	mms_i2c_read(info, wbuf, 2, rbuf, 16);
	memcpy(info->product_name, rbuf, 16);
	dev_dbg(&info->client->dev, "%s - product_name[%s]\n", __func__, info->product_name);
	
	mms_get_fw_version(info, rbuf);
	memcpy(info->fw_version, rbuf, 8);
	dev_info(&info->client->dev, "%s - F/W Version : %02X.%02X %02X.%02X %02X.%02X %02X.%02X\n", __func__, info->fw_version[0], info->fw_version[1], info->fw_version[2], info->fw_version[3], info->fw_version[4], info->fw_version[5], info->fw_version[6], info->fw_version[7]);	
	
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_RESOLUTION_X;
	mms_i2c_read(info, wbuf, 2, rbuf, 7);

#if 1
	//Set resolution using chip info
	info->max_x = (rbuf[0]) | (rbuf[1] << 8);
	info->max_y = (rbuf[2]) | (rbuf[3] << 8);
#else
	//Set resolution using platform data
	info->max_x = info->pdata->max_x;
	info->max_y = info->pdata->max_y;
#endif
	dev_dbg(&info->client->dev, "%s - max_x[%d] max_y[%d]\n", __func__, info->max_x, info->max_y);

	info->node_x = rbuf[4];
	info->node_y = rbuf[5];
	info->node_key = rbuf[6];
	dev_dbg(&info->client->dev, "%s - node_x[%d] node_y[%d] node_key[%d]\n", __func__, info->node_x, info->node_y, info->node_key);

	if(info->node_key > 0){
		//Enable touchkey
		info->tkey_enable = true;
	}

	/*
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_SUPPORTED_FUNC;
	mms_i2c_read(info, wbuf, 2, rbuf, 7);
	info->event_size = rbuf[6];
	dev_dbg(&info->client->dev, "%s - event_size[%d] \n", __func__, info->event_size);
	*/
	info->event_size = 6;
		pr_err("ppppppp---------------------\n");

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
	
//ERROR:
	//dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	//return 1;
}
/*
static int mms_ts_pinctrl_init(struct mms_ts_info *mms_data)
{
	int retval;

	//Get pinctrl if target uses pinctrl 
	mms_data->ts_pinctrl = devm_pinctrl_get(&(mms_data->client->dev));
	if (IS_ERR_OR_NULL(mms_data->ts_pinctrl)) {
		retval = PTR_ERR(mms_data->ts_pinctrl);
		dev_dbg(&mms_data->client->dev,
			"Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}
	pr_err("probe------------------2.6\n");

	mms_data->pinctrl_state_active
		= pinctrl_lookup_state(mms_data->ts_pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(mms_data->pinctrl_state_active)) {
		retval = PTR_ERR(mms_data->pinctrl_state_active);
		dev_err(&mms_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}
	pr_err("probe------------------2.7\n");

	mms_data->pinctrl_state_suspend
		= pinctrl_lookup_state(mms_data->ts_pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(mms_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(mms_data->pinctrl_state_suspend);
		dev_err(&mms_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	mms_data->pinctrl_state_release
		= pinctrl_lookup_state(mms_data->ts_pinctrl,
			PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(mms_data->pinctrl_state_release)) {
		retval = PTR_ERR(mms_data->pinctrl_state_release);
		dev_dbg(&mms_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(mms_data->ts_pinctrl);
err_pinctrl_get:
	mms_data->ts_pinctrl = NULL;
	return retval;
}
*/
/**
* Initialize driver
*/
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct mms_ts_info *info =
		container_of(self, struct mms_ts_info, fb_notif);

	pr_err("tptptp  fb_notifier_callback\n");
	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			info && info->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			mms_resume(&info->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			mms_suspend(&info->client->dev);
	}

	return 0;
}

static int mms_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mms_ts_info *info;
	struct input_dev *input_dev;
	int ret = 0;	
	
	dev_dbg(&client->dev, "%s [START]\n", __func__);

	pr_err("mms_probe  tptptptp--------\n");
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)){
		dev_err(&client->dev, "%s [ERROR] i2c_check_functionality\n", __func__);		
		ret = -EIO;
		goto ERROR;
	}

	//dev_dbg(&client->dev, "%s - i2c_check_functionality\n", __func__);
	
	info = kzalloc(sizeof(struct mms_ts_info), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!info || !input_dev) {
		dev_err(&client->dev, "%s [ERROR]\n", __func__);
		ret = -ENOMEM;
		goto ERROR;
	}

	info->client = client;
	info->input_dev = input_dev;
	info->irq = -1;
	info->init = true;
	mutex_init(&info->lock);

	//dev_dbg(&client->dev, "%s - input_allocate_device\n", __func__);

	//Get platform data
	if (client->dev.of_node) {
		info->pdata  = devm_kzalloc(&client->dev, sizeof(struct mms_platform_data), GFP_KERNEL);
		if (!info->pdata) {
			dev_err(&client->dev, "%s [ERROR] pdata devm_kzalloc\n", __func__);
			//goto error_platform_data;
		}

		ret = mms_parse_devicetree(&client->dev, info);
		if (ret){
			dev_err(&client->dev, "%s [ERROR] mms_parse_dt\n", __func__);
			goto ERROR;
		}
	} else
	{
		info->pdata = client->dev.platform_data;
		if (info->pdata == NULL) {
			dev_err(&client->dev, "%s [ERROR] pdata is null\n", __func__);
			ret = -EINVAL;
			goto ERROR;
		}
	}
	//pr_err("probe -----------------1\n");
	//snprintf(info->phys, sizeof(info->phys), "%s/input0", dev_name(&client->dev));
	
	//input_dev->name = "MELFAS_" CHIP_NAME "_Touchscreen";
	input_dev->name = "melfas_mms438_ts";
	//input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
#if MMS_USE_INPUT_OPEN_CLOSE	
	input_dev->open = mms_input_open;
	input_dev->close = mms_input_close;
#endif

	//Create device
	input_set_drvdata(input_dev, info);
	i2c_set_clientdata(client, info);
//	pr_err("probe------------------2\n");
	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "%s [ERROR] input_register_device\n", __func__);
		ret = -EIO;
		goto ERROR;
	}

	//dev_dbg(&client->dev, "%s - input_register_device\n", __func__);
	pr_err("probe------------------2.5\n");
/*
	ret = mms_ts_pinctrl_init(info);
	if (!ret && info->ts_pinctrl) {
		ret = pinctrl_select_state(info->ts_pinctrl,
					info->pinctrl_state_active);
		if (ret < 0) {
			dev_err(&client->dev,
				"failed to select pin to active state");
			goto pinctrl_deinit;
		}
	} else {
		goto ERROR;
	}
	pr_err("probe------------------3\n");*/

	//Power on
	info->power_on=false;
	regulator_power_init(info);
	regulator_power_on(info);
	mms_power_on(info);
	info->fb_notif.notifier_call = fb_notifier_callback;

	ret = fb_register_client(&info->fb_notif);

	if (ret)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			ret);
	//Firmware update
#if MMS_USE_AUTO_FW_UPDATE
	/*	
	info->fw_name = kstrdup(INTERNAL_FW_PATH, GFP_KERNEL);
	ret = request_firmware_nowait(THIS_MODULE, true, fw_name, &client->dev, GFP_KERNEL, info, mms_fw_update_boot);
	if (ret) {
		dev_err(&client->dev, "%s [ERROR] request_firmware_nowait\n", __func__);
		ret = -EIO;
		//goto ERROR;	
	}
	*/
	ret = mms_fw_update_from_kernel(info);
	if(ret){
		dev_err(&client->dev, "%s [ERROR] mms_fw_update_from_kernel\n", __func__);
	}
#endif

	//Initial config
	mms_init_config(info);
       	pr_err("probe------------------3\n");

	//Config input interface	
	mms_config_input(info);

#if MMS_USE_CALLBACK
	//Config callback functions
	mms_config_callback(info);
#endif

	//Set interrupt handler	
	ret = request_threaded_irq(client->irq, NULL, mms_interrupt, IRQF_TRIGGER_LOW | IRQF_ONESHOT, MMS_DEVICE_NAME, info);
	//ret = request_threaded_irq(client->irq, NULL, mms_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, MMS_DEVICE_NAME, info);
	if (ret) {
		dev_err(&client->dev, "%s [ERROR] request_threaded_irq\n", __func__);
		goto ERROR;
	}

	disable_irq(client->irq);
	info->irq = client->irq;

#if MMS_USE_NAP_MODE
	//Wake lock for nap mode
	wake_lock_init(&mms_wake_lock, WAKE_LOCK_SUSPEND, "mms_wake_lock");
#endif
		pr_err("probe------------------4\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	//Config early suspend
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +1;
	info->early_suspend.suspend = mms_early_suspend;
	info->early_suspend.resume = mms_late_resume;
	
	register_early_suspend(&info->early_suspend);
	
	dev_dbg(&client->dev, "%s - register_early_suspend\n", __func__);
#endif

	//Enable device
	mms_enable(info);

#if MMS_USE_DEV_MODE
	//Create dev node (optional)
	if(mms_dev_create(info)){
		dev_err(&client->dev, "%s [ERROR] mms_dev_create\n", __func__);
		ret = -EAGAIN;
		goto ERROR;
	}	

	//Create dev
	info->class = class_create(THIS_MODULE, MMS_DEVICE_NAME);
	device_create(info->class, NULL, info->mms_dev, NULL, MMS_DEVICE_NAME);
#endif
		pr_err("probe------------------5\n");

#if MMS_USE_TEST_MODE
	//Create sysfs for test mode (optional)
	if (mms_sysfs_create(info)){
		dev_err(&client->dev, "%s [ERROR] mms_sysfs_create\n", __func__);
		ret = -EAGAIN;
		goto ERROR;
	}
#endif

#if MMS_USE_CMD_MODE
	//Create sysfs for command mode (optional)
	if (mms_sysfs_cmd_create(info)){
		dev_err(&client->dev, "%s [ERROR] mms_sysfs_cmd_create\n", __func__);
		ret = -EAGAIN;
		goto ERROR;
	}

	//Create link for cmd
	/*
	if (sysfs_create_link(NULL, &client->dev.kobj, "tsp")) {
		dev_err(&client->dev, "%s [ERROR] sysfs_create_link\n", __func__);
		ret = -EAGAIN;
		goto ERROR;
	}
	*/
#endif
	
	//Create sysfs
	if (sysfs_create_group(&client->dev.kobj, &mms_attr_group)) {
		dev_err(&client->dev, "%s [ERROR] sysfs_create_group\n", __func__);
		ret = -EAGAIN;
		goto ERROR;
	}

	if (sysfs_create_link(NULL, &client->dev.kobj, MMS_DEVICE_NAME)) {
		dev_err(&client->dev, "%s [ERROR] sysfs_create_link\n", __func__);
		ret = -EAGAIN;
		goto ERROR;
	}
	pr_err("tptptptp -------------- \n");
	dev_dbg(&client->dev, "%s [DONE]\n", __func__);	
	dev_info(&client->dev, "MELFAS " CHIP_NAME " Touchscreen is initialized successfully.\n");	
	return 0;
/*
pinctrl_deinit:
	if (info->ts_pinctrl) {
		if (IS_ERR_OR_NULL(info->pinctrl_state_release)) {
			devm_pinctrl_put(info->ts_pinctrl);
			info->ts_pinctrl = NULL;
		} else {
			ret = pinctrl_select_state(info->ts_pinctrl,
					info->pinctrl_state_release);
			if (ret)
				pr_err("failed to select relase pinctrl state\n");
		}
	}*/
ERROR:
	dev_dbg(&client->dev, "%s [ERROR]\n", __func__);
	dev_err(&client->dev, "MELFAS " CHIP_NAME " Touchscreen initialization failed.\n");	
	return ret;
}

/**
* Remove driver
*/
static int mms_remove(struct i2c_client *client)
{
	struct mms_ts_info *info = i2c_get_clientdata(client);
	int retval;

	if (info->irq >= 0){
		free_irq(info->irq, info);
	}

#if MMS_USE_CMD_MODE
	mms_sysfs_cmd_remove(info);
#endif

#if MMS_USE_TEST_MODE
	mms_sysfs_remove(info);
#endif

	sysfs_remove_group(&info->client->dev.kobj, &mms_attr_group);
	sysfs_remove_link(NULL, MMS_DEVICE_NAME);
	kfree(info->print_buf);

#if MMS_USE_DEV_MODE
	device_destroy(info->class, info->mms_dev);
	class_destroy(info->class);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&info->early_suspend);
#endif

	if (info->ts_pinctrl) {
		if (IS_ERR_OR_NULL(info->pinctrl_state_release)) {
			devm_pinctrl_put(info->ts_pinctrl);
			info->ts_pinctrl = NULL;
		} else {
			retval = pinctrl_select_state(info->ts_pinctrl,
					info->pinctrl_state_release);
			if (retval < 0)
				pr_err("failed to select release pinctrl state\n");
		}
	}

	input_unregister_device(info->input_dev);

	kfree(info->fw_name);
	kfree(info);

	return 0;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
/**
* Device suspend event handler
*/
int mms_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	
	dev_dbg(&client->dev, "%s [START]\n", __func__);
	pr_err("mms_suspend\n");
#if MMS_USE_GESTURE_WAKEUP_MODE
		//mutex_lock(&info->lock);

	if(mms_wakeup_control_flag==1)
	{

    	      msleep(100);
             mms_reset(info);
     	      msleep(100);
		ret  =mms_enable_gesture_wakeup_mode(info);
		if(ret ==0){
			//dev_err(&client->dev," ahe gesture close OK!!\n");
			printk("[HST] gesture open OK!!\n");
			}else{
			//dev_err(&client->dev," ahe gesture close fail!!\n");
			printk(" [HST] gesture open fail!!\n");
			}
	}	
#endif
	
#if MMS_USE_NAP_MODE
	//write cmd to enter nap mode
	//
	
	info->nap_mode = true;
	dev_dbg(&client->dev, "%s - nap mode : on\n", __func__);

	if(!wake_lock_active(&mms_wake_lock)) {
		wake_lock(&mms_wake_lock);
		dev_dbg(&client->dev, "%s - wake_lock\n", __func__);
	}
#else
if(mms_wakeup_control_flag==0){
	mms_disable(info);
}	
#endif

	mms_clear_input(info);

	dev_dbg(&client->dev, "%s [DONE]\n", __func__);

	return 0;

}

/**
* Device resume event handler
*/
int mms_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	int ret = 0;

	dev_dbg(&client->dev, "%s [START]\n", __func__);

	if(info->enabled == false){
		ret = mms_enable(info);
	}

#if MMS_USE_NAP_MODE
	if (wake_lock_active(&mms_wake_lock)){
		wake_unlock(&mms_wake_lock);
		dev_dbg(&client->dev, "%s - wake_unlock\n", __func__);
	}
	
	info->nap_mode = false;
	dev_dbg(&client->dev, "%s - nap mode : off\n", __func__);	
#endif

	dev_dbg(&client->dev, "%s [DONE]\n", __func__);

	return ret;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
/**
* Early suspend handler
*/
void mms_early_suspend(struct early_suspend *h)
{
	struct mms_ts_info *info = container_of(h, struct mms_ts_info, early_suspend);
	
	mms_suspend(&info->client->dev);
}

/**
* Late resume handler
*/
void mms_late_resume(struct early_suspend *h)
{
	struct mms_ts_info *info = container_of(h, struct mms_ts_info, early_suspend);

	mms_resume(&info->client->dev);
}
#endif

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
/**
* PM info
*/
const struct dev_pm_ops mms_pm_ops = {
#if 0
	SET_SYSTEM_SLEEP_PM_OPS(mms_suspend, mms_resume)
#else
	.suspend	= mms_suspend,
	.resume = mms_resume,
#endif
};
#endif

#if MMS_USE_DEVICETREE
/**
* Device tree match table
*/
static const struct of_device_id mms_match_table[] = {
	{ .compatible = "melfas,"MMS_DEVICE_NAME,},
	{},
};
MODULE_DEVICE_TABLE(of, mms_match_table);
#endif

/**
* I2C Device ID
*/
static const struct i2c_device_id mms_id[] = {
	{MMS_DEVICE_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, mms_id);

/**
* I2C driver info
*/
static struct i2c_driver mms_driver = {
	.id_table	= mms_id,
	.probe = mms_probe,
	.remove = mms_remove,
	.driver = {
		.name = MMS_DEVICE_NAME,
		.owner = THIS_MODULE,
#if MMS_USE_DEVICETREE
		.of_match_table = mms_match_table,
#endif
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
		//.pm 	= &mms_pm_ops,
#endif
	},
};

/**
* Init driver
*/
static int __init mms_init(void)
{
	return i2c_add_driver(&mms_driver);
}

/**
* Exit driver
*/
static void __exit mms_exit(void)
{
	i2c_del_driver(&mms_driver);
}

module_init(mms_init);
module_exit(mms_exit);

MODULE_DESCRIPTION("MELFAS MMS400 Touchscreen");
MODULE_VERSION("2014.12.02");
MODULE_AUTHOR("Jee SangWon <jeesw@melfas.com>");
MODULE_LICENSE("GPL"); 

