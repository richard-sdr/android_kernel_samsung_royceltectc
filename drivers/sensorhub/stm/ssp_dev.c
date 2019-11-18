/*
 *  Copyright (C) 2015, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include "ssp_dev.h"
#include "ssp_motor.h"
#ifdef CONFIG_SSP_DUAL_LCD
#include <linux/hall.h>
#endif
#ifdef CONFIG_SSP_MOTOR
#include "../../motor/max77843_haptic.h"
#endif
/*
extern int poweroff_charging;
extern int boot_mode_recovery;
*/
#ifdef CONFIG_SSP_DUAL_LCD
//int flip_status;
static struct ssp_data *ssp_data_check = NULL;

void magnetic_set_ssp_info(struct ssp_data *data)
{
	if (data != NULL)
		ssp_data_check = data;
	else
		pr_info("%s : ssp info is null\n", __func__);
}

static struct ssp_data *magnetic_get_ssp_info(void)
{
	return ssp_data_check;
}

void samsung_switching_ssp(int flip)
{
	struct ssp_data *data;

	data = magnetic_get_ssp_info();

	//flip_status = flip;
	// sangmin
	data->flip_status = flip;

	if ((atomic_read(&data->aSensorEnable) & (1 << GEOMAGNETIC_SENSOR))
		|| (atomic_read(&data->aSensorEnable) & (1 << ORIENTATION_SENSOR))
		|| (atomic_read(&data->aSensorEnable) & (1 << ROTATION_VECTOR))){
	/*flip_status 0:open, 1:close*/
		set_pdc_matrix(data);
		// sangmin
		set_magnetic_cal_with_folder_state(data, data->flip_status);
		//set_magnetic_cal_with_folder_state(data, flip_status);
	}
	
	pr_info("[SSP] %s : Folder is %sed now.\n", __func__,
			data->flip_status ? "clos":"open");

	//pr_info("[SSP] %s : Folder is %sed now.\n", __func__,
	//		flip_status ? "clos":"open");
}
EXPORT_SYMBOL(samsung_switching_ssp);

#if 0
int ssp_get_flip_status(void)
{
	return flip_status;
}
#endif

static int ssp_hall_ic_notify(struct notifier_block *nb,
				unsigned long action, void *v)
{
	samsung_switching_ssp((int)action);

	return 0;
}

#endif

#ifdef CONFIG_SSP_MOTOR
int ssp_motor_callback(int state)
{
	struct ssp_data *data;
	data = magnetic_get_ssp_info();

	pr_info("[SSP] %s : Motor state %d\n",__func__, state);
	set_magnetic_cal_with_motor_state(data,(bool)state);

	return 0;
}

int (*getMotorCallback(void))(int)
{
	return ssp_motor_callback;
}

#endif

void ssp_enable(struct ssp_data *data, bool enable)
{
	ssp_infof("enable = %d, old enable = %d",
		enable, data->bSspShutdown);

	if (enable && data->bSspShutdown) {
		data->bSspShutdown = false;
		enable_irq(data->iIrq);
		enable_irq_wake(data->iIrq);
	} else if (!enable && !data->bSspShutdown) {
		data->bSspShutdown = true;
		disable_irq(data->iIrq);
		disable_irq_wake(data->iIrq);
	} else
		ssp_errf("enable error");
}
/************************************************************************/
/* interrupt happened due to transition/change of SSP MCU		*/
/************************************************************************/

static irqreturn_t sensordata_irq_thread_fn(int iIrq, void *dev_id)
{
	struct ssp_data *data = dev_id;
	struct timespec ts;

	ts = ktime_to_timespec(ktime_get_boottime());
	data->timestamp = ts.tv_sec * 1000000000ULL + ts.tv_nsec;

	if (gpio_get_value(data->mcu_int1)) {
		ssp_info("MCU int HIGH");
		return IRQ_HANDLED;
	}
	select_irq_msg(data);
	data->uIrqCnt++;

	return IRQ_HANDLED;
}

/*************************************************************************/
/* initialize sensor hub						 */
/*************************************************************************/

static void initialize_variable(struct ssp_data *data)
{
	int iSensorIndex;

	int data_len[SENSOR_MAX] = SENSOR_DATA_LEN;
	int report_mode[SENSOR_MAX] = SENSOR_REPORT_MODE;
	memcpy(&data->data_len, data_len, sizeof(data->data_len));
	memcpy(&data->report_mode, report_mode, sizeof(data->report_mode));

	data->cameraGyroSyncMode = false;

	for (iSensorIndex = 0; iSensorIndex < SENSOR_MAX; iSensorIndex++) {
		data->adDelayBuf[iSensorIndex] = DEFUALT_POLLING_DELAY;
		data->aiCheckStatus[iSensorIndex] = INITIALIZATION_STATE;
	}

	data->uSensorState = NORMAL_SENSOR_STATE_K;
	data->uMagCntlRegData = 1;

	data->bSspShutdown = true;
	data->bTimeSyncing = true;

	data->buf[GYROSCOPE_SENSOR].gyro_dps = GYROSCOPE_DPS2000;
	data->uIr_Current = DEFUALT_IR_CURRENT;

#if 1
    if(sec_debug_get_debug_level() > 0)
    {
        data->bMcuDumpMode = true;
        ssp_info("Mcu Dump Enabled");
    }

#else
#if CONFIG_SEC_DEBUG
	data->bMcuDumpMode = sec_debug_is_enabled();
#endif
#endif
	INIT_LIST_HEAD(&data->pending_list);

	initialize_function_pointer(data);
}

static int initialize_6axis(struct ssp_data *data)
{
	int new_acc_type = get_6axis_type(data);

	if (new_acc_type < 0)
		ssp_errf("get_6axis_type failed");
	else
		data->acc_type = new_acc_type;

	ssp_infof("6axis type = %d", data->acc_type);

	return SUCCESS;
}

int initialize_mcu(struct ssp_data *data)
{
	int iRet = 0;

	clean_pending_list(data);

	iRet = get_chipid(data);
	ssp_info("MCU device ID = %d, reading ID = %d", DEVICE_ID, iRet);
	if (iRet != DEVICE_ID) {
		if (iRet < 0) {
			ssp_errf("MCU is not working : 0x%x", iRet);
		} else {
			ssp_errf("MCU identification failed");
			iRet = -ENODEV;
		}
		goto out;
	}

	iRet = set_sensor_position(data);
	if (iRet < 0) {
		ssp_errf("set_sensor_position failed");
		goto out;
	}

	if (data->accel_dot >= 0) {
		iRet = set_6axis_dot(data);
		if (iRet < 0) {
			ssp_errf("set_6axis_dot failed");
			goto out;
		}
	}

	iRet = initialize_6axis(data);
	if (iRet < 0)
		ssp_errf("initialize_6axis err(%d)", iRet);

#ifdef CONFIG_SENSORS_MULTIPLE_GLASS_TYPE
    	iRet = set_glass_type(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - set_sensor_position failed\n", __func__);
		goto out;
	}
#endif

	/* Hall IC threshold */
	if (data->hall_threshold[0]) {
		iRet = set_hall_threshold(data);
		if (iRet < 0) {
			ssp_errf("set_hall_threshold failed");
			goto out;
		}
	}

	data->uSensorState = get_sensor_scanning_info(data);
	if (data->uSensorState == 0) {
		ssp_errf("get_sensor_scanning_info failed");
		iRet = ERROR;
		goto out;
	}

	iRet = initialize_magnetic_sensor(data);
	if (iRet < 0)
		ssp_errf("initialize magnetic sensor failed");

	data->uCurFirmRev = get_firmware_rev(data);
	ssp_info("MCU Firm Rev : New = %8u", data->uCurFirmRev);

out:
	return iRet;
}

static int initialize_irq(struct ssp_data *data)
{
	int iRet, iIrq;
	iIrq = gpio_to_irq(data->mcu_int1);

	ssp_info("requesting IRQ %d", iIrq);
	iRet = request_threaded_irq(iIrq, NULL, sensordata_irq_thread_fn,
			    IRQF_TRIGGER_FALLING|IRQF_ONESHOT, "SSP_Int", data);
	if (iRet < 0) {
		ssp_errf("request_irq(%d) failed for gpio %d (%d)",
		       iIrq, iIrq, iRet);
		goto err_request_irq;
	}

	/* start with interrupts disabled */
	data->iIrq = iIrq;
	disable_irq(data->iIrq);
	return 0;

err_request_irq:
	gpio_free(data->mcu_int1);
	return iRet;
}

#ifdef CONFIG_SENSORS_SSP_ROYCELTE
int proximity_regulator_onoff(struct ssp_data *data, bool onoff)
{
	int rc = 0;
	struct regulator *regulator_led_3p3;
	struct regulator *regulator_vdd_1p8;

	pr_info("[SSP] %s - %d\n", __func__, onoff);

	regulator_vdd_1p8 = regulator_get(NULL, data->prox_vdd);
	if (IS_ERR(regulator_vdd_1p8)) {
	pr_err("[SSP] %s - vdd_1p8 regulator_get fail\n", __func__);
		return -ENODEV;
	}

	regulator_led_3p3 = regulator_get(NULL, data->prox_led);
	if (IS_ERR(regulator_led_3p3)) {
		pr_err("[SSP] %s - vdd_3p3 regulator_get fail\n", __func__);
		regulator_put(regulator_vdd_1p8);
		return -ENODEV;
	}

	if (onoff) {
		rc = regulator_enable(regulator_vdd_1p8);
		if (rc) {
			pr_err("[SSP] %s - enable vdd_1p8 failed, rc=%d\n",
			__func__, rc);
			goto done;
		}
		rc = regulator_enable(regulator_led_3p3);
		if (rc) {
			pr_err("[SSP] %s - enable led_3p3 failed, rc=%d\n",
				__func__, rc);
			goto done;
		}
	}else {
		rc = regulator_disable(regulator_vdd_1p8);
		if (rc) {
			pr_err("[SSP] %s - disable vdd_1p8 failed, rc=%d\n",
				__func__, rc);
			goto done;
		}
		rc = regulator_disable(regulator_led_3p3);
		if (rc) {
			pr_err("[SSP] %s - disable led_3p3 failed, rc=%d\n",
				__func__, rc);
			goto done;
		}
	}
done:
	regulator_put(regulator_led_3p3);
	regulator_put(regulator_vdd_1p8);
	return rc;
}
#endif

static void work_function_firmware_update(struct work_struct *work)
{
	struct ssp_data *data = container_of((struct delayed_work *)work,
				struct ssp_data, work_firmware);
	int iRet;

	ssp_infof();

	iRet = forced_to_download_binary(data, KERNEL_BINARY);
	if (iRet < 0) {
		ssp_infof("forced_to_download_binary failed!");
		data->uSensorState = 0;
		return;
	}

	queue_refresh_task(data, SSP_SW_RESET_TIME);
}

static int ssp_parse_dt(struct device *dev,
	struct ssp_data *data)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int errorno = 0;
#if defined(CONFIG_SSP_DUAL_LCD)
	u32 len, temp;
	int i;
#endif


	/* gpio pins */
        ssp_errf("ssp_parse_dt");
	data->mcu_int1 = of_get_named_gpio_flags(np, "ssp,mcu_int1-gpio",
		0, &flags);
	if (data->mcu_int1 < 0) {
		errorno = data->mcu_int1;
                ssp_errf("mcu_int1<0");
		goto dt_exit;
	}

	data->mcu_int2 = of_get_named_gpio_flags(np, "ssp,mcu_int2-gpio",
		0, &flags);
	if (data->mcu_int2 < 0) {
		errorno = data->mcu_int2;
                ssp_errf("mcu_int2<0");
		goto dt_exit;
	}

	data->ap_int = of_get_named_gpio_flags(np, "ssp,ap_int-gpio",
		0, &flags);
	if (data->ap_int < 0) {
		errorno = data->ap_int;
                ssp_errf("ap_int<0");
		goto dt_exit;
	}

	data->rst = of_get_named_gpio_flags(np, "ssp,rst-gpio",
		0, &flags);
	if (data->rst < 0) {
		errorno = data->rst;
                ssp_errf("rst<0");
		goto dt_exit;
	}
#ifdef CONFIG_SSP_DUAL_LCD
	data->folder_state = of_get_named_gpio_flags(np, "ssp,hall_flip-gpio",
		0, &flags);
	if (data->folder_state < 0) {
		errorno = data->folder_state;
                ssp_errf("folder_state<0");
		goto dt_exit;
	}
#endif

	/* sensor positions */
	if (of_property_read_u32(np, "ssp,acc-position", &data->accel_position))
		data->accel_position = 0;

	if (of_property_read_u32(np, "ssp,acc-dot", &data->accel_dot))
		data->accel_dot = -1;

	if (of_property_read_u32(np, "ssp,mag-position", &data->mag_position))
		data->mag_position = 0;

	ssp_info("acc-posi[%d] acc-dot[%d] mag-posi[%d]",
			data->accel_position, data->accel_dot, data->mag_position);

	/* prox thresh */
	if (of_property_read_u32(np, "ssp,prox-hi_thresh",
			&data->uProxHiThresh_default))
		data->uProxHiThresh_default = DEFUALT_HIGH_THRESHOLD;

	if (of_property_read_u32(np, "ssp,prox-low_thresh",
			&data->uProxLoThresh_default))
		data->uProxLoThresh_default = DEFUALT_LOW_THRESHOLD;

	ssp_info("hi-thresh[%u] low-thresh[%u]",
		data->uProxHiThresh_default, data->uProxLoThresh_default);
	/* prox vdd*/
#ifdef CONFIG_SENSORS_SSP_ROYCELTE
	if (of_property_read_string(np, "ssp,prox_vdd_1p8",
		&data->prox_vdd) < 0)
		pr_err("%s - get prox_vdd error\n", __func__);

	if (of_property_read_string(np, "ssp,prox_led_3p3",
		&data->prox_led) < 0)
		pr_err("%s - get prox_led error\n", __func__);
 
	pr_err("%s - get prox_vcc pass %s %s\n", __func__,data->prox_vdd,data->prox_led);
#endif

#ifdef CONFIG_SENSORS_MULTIPLE_GLASS_TYPE
	if (of_property_read_u32(np, "ssp-glass-type", &data->glass_type))
		    data->glass_type = 0;
#endif

	/* acc type */
    	if (of_property_read_u32(np, "ssp-acc-type", &data->acc_type))
		    data->acc_type = 0;

	ssp_info("acc-type = %d", data->acc_type);

	/* pressure type */
    	if (of_property_read_u32(np, "ssp-pressure-type", &data->pressure_type))
		    data->pressure_type = 0;

	ssp_info("pressure-type = %d", data->pressure_type);

	/* mag matrix */
#if defined(CONFIG_SSP_DUAL_LCD)
	if (!of_get_property(np, "ssp,mag-array0", &len)) {
		pr_info("[SSP] No static matrix at DT for mag!(%p)\n", data->pdc_matrix);
                ssp_errf("[SSP] No static matrix at DT for mag!");
		goto dt_exit;
	}
	if (len/4 != PDC_SIZE) {
		pr_err("[SSP] Length/4:%d should be 27(AK09911 matrix) for mag!\n", len/4);
                ssp_errf("[SSP] Length/4:should be 27(AK09911 matrix) for mag");
		goto dt_exit;
	}

	for (i = 0; i < PDC_SIZE; i++) {
		if (of_property_read_u32_index(np, "ssp,mag-array0", i, &temp)) {
			pr_err("[SSP] %s cannot get u32 of array[%d]!\n", __func__, i);
                        ssp_errf("[SSP] cannot get u32 of array0");
			goto dt_exit;
		}
		data->pdc_matrix[FOLDER_OPEN][i] = (unsigned char)temp;
		if (of_property_read_u32_index(np, "ssp,mag-array1", i, &temp)) {
			pr_err("[SSP] %s cannot get u32 of array[%d]!\n", __func__, i);
                        ssp_errf("[SSP] cannot get u32 of array1");
			goto dt_exit;
		}
		data->pdc_matrix[FOLDER_CLOSE][i] = (unsigned char)temp;
	}
#else
	if (of_property_read_u8_array(np, "ssp,mag-array",
		data->pdc_matrix, sizeof(data->pdc_matrix))) {
		ssp_err("no mag-array, set as 0");
	}
#endif


	/* Hall IC threshold */
	if (of_property_read_u16_array(np, "ssp-hall-threshold",
		data->hall_threshold, ARRAY_SIZE(data->hall_threshold))) {
		ssp_err("no hall-threshold, set as 0");
	} else {
		ssp_info("hall thr: %d %d %d %d %d",
			data->hall_threshold[0], data->hall_threshold[1],
			data->hall_threshold[2], data->hall_threshold[3],
			data->hall_threshold[4]);
	}

	/* set off gpio pins */
	errorno = gpio_request(data->mcu_int1, "mcu_ap_int1");
	if (errorno) {
		ssp_err("failed to request MCU_INT1 for SSP");
		goto dt_exit;
	}
	errorno = gpio_direction_input(data->mcu_int1);
	if (errorno) {
		ssp_err("failed to set mcu_int1 as input");
		goto dt_exit;
	}
	errorno = gpio_request(data->mcu_int2, "MCU_INT2");
	if (errorno) {
		ssp_err("failed to request MCU_INT2 for SSP");
		goto dt_exit;
	}
	gpio_direction_input(data->mcu_int2);

	errorno = gpio_request(data->ap_int, "AP_MCU_INT");
	if (errorno) {
		ssp_err("failed to request AP_INT for SSP");
		goto dt_exit;
	}
	gpio_direction_output(data->ap_int, 1);

	errorno = gpio_request(data->rst, "MCU_RST");
	if (errorno) {
		ssp_err("failed to request MCU_RST for SSP");
		goto dt_exit;
	}
	gpio_direction_output(data->rst, 1);

dt_exit:
	return errorno;
}

static int ssp_suspend(struct device *dev)
{
	struct spi_device *spi_dev = to_spi_device(dev);
	struct ssp_data *data = spi_get_drvdata(spi_dev);

	ssp_infof();
	data->uLastResumeState = MSG2SSP_AP_STATUS_SUSPEND;
	disable_debug_timer(data);

	if (SUCCESS != ssp_send_cmd(data, MSG2SSP_AP_STATUS_SUSPEND, 0))
		ssp_errf("MSG2SSP_AP_STATUS_SUSPEND failed");

	data->bTimeSyncing = false;
	disable_irq_nosync(data->iIrq);
	return 0;
}

static int ssp_resume(struct device *dev)
{
	struct spi_device *spi_dev = to_spi_device(dev);
	struct ssp_data *data = spi_get_drvdata(spi_dev);

	enable_irq(data->iIrq);
	ssp_infof();
	enable_debug_timer(data);

	if (SUCCESS != ssp_send_cmd(data, MSG2SSP_AP_STATUS_RESUME, 0))
		ssp_errf("MSG2SSP_AP_STATUS_RESUME failed");

	data->uLastResumeState = MSG2SSP_AP_STATUS_RESUME;

	return 0;
}

static const struct dev_pm_ops ssp_pm_ops = {
	.suspend = ssp_suspend,
	.resume = ssp_resume
};


static int ssp_probe(struct spi_device *spi)
{
	int iRet = 0;
	struct ssp_data *data;

	ssp_infof();
/*
	if (poweroff_charging == 1 || boot_mode_recovery == 1) {
		ssp_err("probe exit : lpm %d, recovery %d",
			poweroff_charging, boot_mode_recovery);
		return -ENODEV;
	}
*/
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ssp_errf("failed to allocate memory for data");
		iRet = -ENOMEM;
		goto exit;
	}

	if (spi->dev.of_node) {
		iRet = ssp_parse_dt(&spi->dev, data);
		if (iRet) {
			ssp_errf("Failed to parse DT");
			goto err_setup;
		}
	} else {
		ssp_errf("failed to get device node");
		iRet = -ENODEV;
		goto err_setup;
	}

	spi->mode = SPI_MODE_1;
	if (spi_setup(spi)) {
		ssp_errf("failed to setup spi");
		iRet = -ENODEV;
		goto err_setup;
	}
#ifdef CONFIG_SENSORS_SSP_ROYCELTE
	if (proximity_regulator_onoff(data,true) < 0) {
		pr_err("[SSP] %s - prox regulator enable fail\n",	__func__);
	}
    mdelay(10);
#endif
	data->fw_dl_state = FW_DL_STATE_NONE;
	data->spi = spi;
	spi_set_drvdata(spi, data);

	mutex_init(&data->comm_mutex);
	mutex_init(&data->pending_mutex);

	pr_info("\n#####################################################\n");

	INIT_DELAYED_WORK(&data->work_firmware, work_function_firmware_update);

	wake_lock_init(&data->ssp_wake_lock,
		WAKE_LOCK_SUSPEND, "ssp_wake_lock");

	iRet = initialize_indio_dev(data);
	if (iRet < 0) {
		ssp_errf("could not create input device");
		goto err_input_register_device;
	}

	iRet = initialize_debug_timer(data);
	if (iRet < 0) {
		ssp_errf("could not create workqueue");
		goto err_create_workqueue;
	}

	iRet = initialize_irq(data);
	if (iRet < 0) {
		ssp_errf("could not create irq");
		goto err_setup_irq;
	}

	iRet = initialize_sysfs(data);
	if (iRet < 0) {
		ssp_errf("could not create sysfs");
		goto err_sysfs_create;
	}

	initialize_variable(data);

	/* init sensorhub device */
	iRet = ssp_sensorhub_initialize(data);
	if (iRet < 0) {
		ssp_errf("ssp_sensorhub_initialize err(%d)", iRet);
		ssp_sensorhub_remove(data);
	}

	ssp_enable(data, true);
	/* check boot loader binary */
	data->fw_dl_state = check_fwbl(data);

	if (data->fw_dl_state == FW_DL_STATE_NONE) {
		iRet = initialize_mcu(data);
		if (iRet == ERROR) {
			toggle_mcu_reset(data);
		} else if (iRet < ERROR) {
			ssp_errf("initialize_mcu failed");
			goto err_read_reg;
		}
	}
	
#ifdef CONFIG_SSP_DUAL_LCD
	/* if read gpio , 0: close, 1:open so flip_status 0:open, 1:close*/
	//sangmin
	data->flip_status = data->folder_state; //!(gpio_get_value(HALL_SENSOR_INT_GPIO));
	pr_info("[SSP] %s : Folder is %sed now.\n", __func__,data->flip_status ? "clos":"open");

	//flip_status = data->folder_state; //!(gpio_get_value(HALL_SENSOR_INT_GPIO));
	//pr_info("[SSP] %s : Folder is %sed now.\n", __func__, flip_status ? "clos":"open");
#endif

	ssp_infof("probe success!");

	enable_debug_timer(data);

	if (data->fw_dl_state == FW_DL_STATE_NEED_TO_SCHEDULE) {
		ssp_info("Firmware update is scheduled");
		schedule_delayed_work(&data->work_firmware,
				msecs_to_jiffies(1000));
		data->fw_dl_state = FW_DL_STATE_SCHEDULED;
	} else if (data->fw_dl_state == FW_DL_STATE_FAIL) {
		data->bSspShutdown = true;
	}
	data->bProbeIsDone = true;
	iRet = 0;
#ifdef CONFIG_SSP_DUAL_LCD
	magnetic_set_ssp_info(data);
	data->hall_ic_nb.notifier_call = ssp_hall_ic_notify;
	hall_ic_register_notify(&data->hall_ic_nb);
#endif

#ifdef CONFIG_SSP_MOTOR
	ssp_infof("motor callback set!");

	//register motor
	setMotorCallback(ssp_motor_callback);
#endif
	goto exit;

err_read_reg:
	remove_sysfs(data);
err_sysfs_create:
	free_irq(data->iIrq, data);
	gpio_free(data->mcu_int1);
err_setup_irq:
	destroy_workqueue(data->debug_wq);
err_create_workqueue:
	remove_indio_dev(data);
err_input_register_device:
	wake_lock_destroy(&data->ssp_wake_lock);
	mutex_destroy(&data->comm_mutex);
	mutex_destroy(&data->pending_mutex);

err_setup:
	kfree(data);
	ssp_errf("probe failed!");
exit:
	pr_info("#####################################################\n\n");
	return iRet;
}

static void ssp_shutdown(struct spi_device *spi_dev)
{
	struct ssp_data *data = spi_get_drvdata(spi_dev);

	ssp_infof();

#ifdef CONFIG_SSP_MOTOR
	// motor unregister
#endif
#ifdef CONFIG_SSP_DUAL_LCD
	// hall_ic unregister
	hall_ic_unregister_notify(&data->hall_ic_nb);
#endif

	if (data->bProbeIsDone == false)
		goto exit;

	disable_debug_timer(data);

	if (data->fw_dl_state >= FW_DL_STATE_SCHEDULED &&
		data->fw_dl_state < FW_DL_STATE_DONE) {
		ssp_errf("cancel_delayed_work_sync state = %d",
			data->fw_dl_state);
		cancel_delayed_work_sync(&data->work_firmware);
	}

	if (SUCCESS != ssp_send_cmd(data, MSG2SSP_AP_STATUS_SHUTDOWN, 0))
		ssp_errf("MSG2SSP_AP_STATUS_SHUTDOWN failed");

	data->bSspShutdown = true;
	disable_irq_nosync(data->iIrq);
	disable_irq_wake(data->iIrq);

	clean_pending_list(data);

	free_irq(data->iIrq, data);
	gpio_free(data->mcu_int1);

	remove_sysfs(data);

	ssp_sensorhub_remove(data);

	del_timer_sync(&data->debug_timer);
	cancel_work_sync(&data->work_debug);
	cancel_delayed_work_sync(&data->work_refresh);
	destroy_workqueue(data->debug_wq);
	wake_lock_destroy(&data->ssp_wake_lock);
	mutex_destroy(&data->comm_mutex);
	mutex_destroy(&data->pending_mutex);
	toggle_mcu_reset(data);
	ssp_infof("done");
exit:
	kfree(data);
}

static const struct spi_device_id ssp_id[] = {
	{"ssp", 0},
	{}
};

MODULE_DEVICE_TABLE(spi, ssp_id);

#ifdef CONFIG_OF
static struct of_device_id ssp_match_table[] = {
	{ .compatible = "ssp,STM32F",},
	{},
};
#endif

static struct spi_driver ssp_driver = {
	.probe = ssp_probe,
	.shutdown = ssp_shutdown,
	.id_table = ssp_id,
	.driver = {
		.pm = &ssp_pm_ops,
		.owner = THIS_MODULE,
		.name = "ssp",
#ifdef CONFIG_OF
		.of_match_table = ssp_match_table
#endif
	},
};

static int __init ssp_init(void)
{
	return spi_register_driver(&ssp_driver);
}

static void __exit ssp_exit(void)
{
	spi_unregister_driver(&ssp_driver);
}

late_initcall(ssp_init);
module_exit(ssp_exit);
MODULE_DESCRIPTION("Seamless Sensor Platform(SSP) dev driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
