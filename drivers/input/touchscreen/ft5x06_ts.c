/*
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR        Note
 *    1.0		  2010-01-05			WenFS    only support mulititouch	Wenfs 2010-10-01
 *    2.0          2011-09-05                   Duxx      Add touch key, and project setting update, auto CLB command
 *    3.0		  2011-09-09			Luowj
 *
 */

#define DEBUG
#define AW_DEBUG
//#define PRINT_INT_INFO
//#define PRINT_POINT_INFO
#include <linux/i2c.h>
#include <linux/input.h>
#include "ft5x06_ts.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
    #include <linux/pm.h>
    #include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <linux/semaphore.h>
//#include <linux/mutex.h>

#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <mach/sys_config.h>


//#include <asm/jzsoc.h>

#include "ctp_platform_ops.h"

struct i2c_dev{
struct list_head list;
struct i2c_adapter *adap;
struct device *dev;
};

static struct class *i2c_dev_class;
static LIST_HEAD (i2c_dev_list);
static DEFINE_SPINLOCK(i2c_dev_list_lock);

static struct i2c_client *this_client;

#define CONFIG_FT5X0X_MULTITOUCH 1

#ifdef PRINT_POINT_INFO
#define print_point_info(fmt, args...)   \
        do{                              \
                pr_info(fmt, ##args);     \
        }while(0)
#else
#define print_point_info(fmt, args...)   //
#endif

#ifdef PRINT_INT_INFO
#define print_int_info(fmt, args...)     \
        do{                              \
                pr_info(fmt, ##args);     \
        }while(0)
#else
#define print_int_info(fmt, args...)   //
#endif
///////////////////////////////////////////////
//specific tp related macro: need be configured for specific tp
#define CTP_IRQ_NO			(gpio_int_info[0].port_num)

#define CTP_IRQ_MODE			(NEGATIVE_EDGE)
#define CTP_NAME			FT5X0X_NAME
#define TS_RESET_LOW_PERIOD		(1)
#define TS_INITIAL_HIGH_PERIOD		(30)
#define TS_WAKEUP_LOW_PERIOD	(20)
#define TS_WAKEUP_HIGH_PERIOD	(20)
#define TS_POLL_DELAY			(10)	/* ms delay between samples */
#define TS_POLL_PERIOD			(10)	/* ms delay between samples */
#define SCREEN_MAX_X			(screen_max_x)
#define SCREEN_MAX_Y			(screen_max_y)
#define PRESS_MAX			(255)


static void* __iomem gpio_addr = NULL;
static int gpio_int_hdle = 0;
static int gpio_wakeup_hdle = 0;
static int gpio_reset_hdle = 0;
static int gpio_wakeup_enable = 1;
static int gpio_reset_enable = 1;
static user_gpio_set_t  gpio_int_info[1];

static int screen_max_x = 800;
static int screen_max_y = 480;
static int revert_x_flag = 0;
static int revert_y_flag = 0;
static int exchange_x_y_flag = 0;
static int	int_cfg_addr[]={PIO_INT_CFG0_OFFSET,PIO_INT_CFG1_OFFSET,
			PIO_INT_CFG2_OFFSET, PIO_INT_CFG3_OFFSET};
/* Addresses to scan */
static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};
static __u32 twi_id = 0;

/*
 * ctp_get_pendown_state  : get the int_line data state,
 *
 * return value:
 *             return PRESS_DOWN: if down
 *             return FREE_UP: if up,
 *             return 0: do not need process, equal free up.
 */
static int ctp_get_pendown_state(void)
{
	unsigned int reg_val;
	static int state = FREE_UP;

	//get the input port state
	reg_val = readl(gpio_addr + PIOH_DATA);
	//pr_info("reg_val = %x\n",reg_val);
	if(!(reg_val & (1<<CTP_IRQ_NO))){
		state = PRESS_DOWN;
		print_int_info("pen down. \n");
	}else{ //touch panel is free up
		state = FREE_UP;
		print_int_info("free up. \n");
	}
	return state;
}

/**
 * ctp_clear_penirq - clear int pending
 *
 */
static void ctp_clear_penirq(void)
{
	int reg_val;

	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);

	if((reg_val = (reg_val&(1<<(CTP_IRQ_NO))))){
		print_int_info("==CTP_IRQ_NO:%d=\n",CTP_IRQ_NO);
		writel(reg_val,gpio_addr + PIO_INT_STAT_OFFSET);
	}
	return;
}

/**
 * ctp_set_irq_mode - according sysconfig's subkey "ctp_int_port" to config int port.
 *
 * return value:
 *              0:      success;
 *              others: fail;
 */
static int ctp_set_irq_mode(char *major_key , char *subkey, ext_int_mode int_mode)
{
	int ret = 0;
	__u32 reg_num = 0;
	__u32 reg_addr = 0;
	__u32 reg_val = 0;
	//config gpio to int mode
	pr_info("%s: config gpio to int mode. \n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex(major_key, subkey);
	if(!gpio_int_hdle){
		pr_info("request tp_int_port failed. \n");
		ret = -1;
		goto request_tp_int_port_failed;
	}
	gpio_get_one_pin_status(gpio_int_hdle, gpio_int_info, subkey, 1);
	pr_info("%s, %d: gpio_int_info, port = %d, port_num = %d. \n", __func__, __LINE__, \
		gpio_int_info[0].port, gpio_int_info[0].port_num);
#endif

#ifdef AW_GPIO_INT_API_ENABLE
#else
	pr_info(" INTERRUPT CONFIG\n");
	reg_num = (gpio_int_info[0].port_num)%8;
	reg_addr = (gpio_int_info[0].port_num)/8;
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= (~(7 << (reg_num * 4)));
	reg_val |= (int_mode << (reg_num * 4));
	writel(reg_val,gpio_addr+int_cfg_addr[reg_addr]);

	ctp_clear_penirq();

	reg_val = readl(gpio_addr+PIO_INT_CTRL_OFFSET);
	reg_val |= (1 << (gpio_int_info[0].port_num));
	writel(reg_val,gpio_addr+PIO_INT_CTRL_OFFSET);

	udelay(1);
#endif

request_tp_int_port_failed:
	return ret;
}

/**
 * ctp_set_gpio_mode - according sysconfig's subkey "ctp_io_port" to config io port.
 *
 * return value:
 *              0:      success;
 *              others: fail;
 */
static int ctp_set_gpio_mode(void)
{
	//int reg_val;
	int ret = 0;
	//config gpio to io mode
	pr_info("%s: config gpio to io mode. \n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex("ctp_para", "ctp_io_port");
	if(!gpio_int_hdle){
		pr_info("request ctp_io_port failed. \n");
		ret = -1;
		goto request_tp_io_port_failed;
	}
#endif
	return ret;

request_tp_io_port_failed:
	return ret;
}

/**
 * ctp_judge_int_occur - whether interrupt occur.
 *
 * return value:
 *              0:      int occur;
 *              others: no int occur;
 */
static int ctp_judge_int_occur(void)
{
	//int reg_val[3];
	int reg_val;
	int ret = -1;

	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if(reg_val&(1<<(CTP_IRQ_NO))){
		ret = 0;
	}
	return ret;
}

/**
 * ctp_free_platform_resource - corresponding with ctp_init_platform_resource
 *
 */
static void ctp_free_platform_resource(void)
{
	if(gpio_addr){
		iounmap(gpio_addr);
	}

	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}

	if(gpio_wakeup_hdle){
		gpio_release(gpio_wakeup_hdle, 2);
	}

	if(gpio_reset_hdle){
		gpio_release(gpio_reset_hdle, 2);
	}

	return;
}


/**
 * ctp_init_platform_resource - initialize platform related resource
 * return value: 0 : success
 *               -EIO :  i/o err.
 *
 */
static int ctp_init_platform_resource(void)
{
	int ret = 0;

	gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);
	//pr_info("%s, gpio_addr = 0x%x. \n", __func__, gpio_addr);
	if(!gpio_addr) {
		ret = -EIO;
		goto exit_ioremap_failed;
	}
	//    gpio_wakeup_enable = 1;
	gpio_wakeup_hdle = gpio_request_ex("ctp_para", "ctp_wakeup");
	if(!gpio_wakeup_hdle) {
		pr_warning("%s: tp_wakeup request gpio fail!\n", __func__);
		gpio_wakeup_enable = 0;
	}

	gpio_reset_hdle = gpio_request_ex("ctp_para", "ctp_reset");
	if(!gpio_reset_hdle) {
		pr_warning("%s: tp_reset request gpio fail!\n", __func__);
		gpio_reset_enable = 0;
	}

	return ret;

exit_ioremap_failed:
	ctp_free_platform_resource();
	return ret;
}


/**
 * ctp_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:
 *                    = 0; success;
 *                    < 0; err
 */
static int ctp_fetch_sysconfig_para(void)
{
	int ret = -1;
	int ctp_used = -1;
	char name[I2C_NAME_SIZE];
	__u32 twi_addr = 0;
	//__u32 twi_id = 0;
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;

	pr_info("%s. \n", __func__);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_used", &ctp_used, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	if(1 != ctp_used){
		pr_err("%s: ctp_unused. \n",  __func__);
		//ret = 1;
		return ret;
	}

	if(SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_name", (int *)(&name), &type, sizeof(name)/sizeof(int))){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	if(strcmp(CTP_NAME, name)){
		pr_err("%s: name %s does not match CTP_NAME. \n", __func__, name);
		pr_err(CTP_NAME);
		//ret = 1;
		return ret;
	}

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}
	//big-endian or small-endian?
	//pr_info("%s: before: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	u_i2c_addr.dirty_addr_buf[0] = twi_addr;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
	pr_info("%s: after: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	//pr_info("%s: after: ctp_twi_addr is 0x%x, u32_dirty_addr_buf: 0x%hx. u32_dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u32_dirty_addr_buf[0],u32_dirty_addr_buf[1]);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}
	pr_info("%s: ctp_twi_id is %d. \n", __func__, twi_id);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_x", &screen_max_x, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: screen_max_x = %d. \n", __func__, screen_max_x);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_y", &screen_max_y, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: screen_max_y = %d. \n", __func__, screen_max_y);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_x_flag", &revert_x_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: revert_x_flag = %d. \n", __func__, revert_x_flag);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_y_flag", &revert_y_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: revert_y_flag = %d. \n", __func__, revert_y_flag);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_exchange_x_y_flag", &exchange_x_y_flag, 1)){
		pr_err("ft5x0x_ts: script_parser_fetch err. \n");
		goto script_parser_fetch_err;
	}
	pr_info("%s: exchange_x_y_flag = %d. \n", __func__, exchange_x_y_flag);

	return 0;

script_parser_fetch_err:
	pr_notice("=========script_parser_fetch_err============\n");
	return ret;
}

/**
 * ctp_reset - function
 *
 */
static void ctp_reset(void)
{
	if(gpio_reset_enable){
		pr_info("%s. \n", __func__);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_reset_hdle, 0, "ctp_reset")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_RESET_LOW_PERIOD);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_reset_hdle, 1, "ctp_reset")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_INITIAL_HIGH_PERIOD);
	}
}

/**
 * ctp_wakeup - function
 *
 */
static void ctp_wakeup(void)
{
	if(1 == gpio_wakeup_enable){
		pr_info("%s. \n", __func__);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 0, "ctp_wakeup")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_WAKEUP_LOW_PERIOD);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 1, "ctp_wakeup")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_WAKEUP_HIGH_PERIOD);

	}
	return;
}
/**
 * ctp_detect - Device detection callback for automatic device creation
 * return value:
 *                    = 0; success;
 *                    < 0; err
 */
int ctp_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if(twi_id == adapter->nr)
	{
		pr_info("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, CTP_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, CTP_NAME, I2C_NAME_SIZE);
		return 0;
	}else{
		pr_info("%s: NOT Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, CTP_NAME, i2c_adapter_id(adapter), client->addr);
		return -ENODEV;
	}
}
////////////////////////////////////////////////////////////////

static struct ctp_platform_ops ctp_ops = {
	.get_pendown_state = ctp_get_pendown_state,
	.clear_penirq	   = ctp_clear_penirq,
	.set_irq_mode      = ctp_set_irq_mode,
	.set_gpio_mode     = ctp_set_gpio_mode,
	.judge_int_occur   = ctp_judge_int_occur,
	.init_platform_resource = ctp_init_platform_resource,
	.free_platform_resource = ctp_free_platform_resource,
	.fetch_sysconfig_para = ctp_fetch_sysconfig_para,
	.ts_reset =          ctp_reset,
	.ts_wakeup =         ctp_wakeup,
	.ts_detect = ctp_detect,
};

int fts_ctpm_fw_upgrade_with_i_file(void);

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	spin_lock(&i2c_dev_list_lock);

	list_for_each_entry(i2c_dev,&i2c_dev_list,list){
		pr_info("--line = %d ,i2c_dev->adapt->nr = %d,index = %d.\n",__LINE__,i2c_dev->adap->nr,index);
		if(i2c_dev->adap->nr == index){
		     goto found;
		}
	}
	i2c_dev = NULL;

found:
	spin_unlock(&i2c_dev_list_lock);

	return i2c_dev ;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS){
		pr_info("i2c-dev:out of device minors (%d) \n",adap->nr);
		return ERR_PTR (-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev){
		return ERR_PTR(-ENOMEM);
	}
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);

	return i2c_dev;
}




struct ts_event {
        u16 au16_x[CFG_MAX_TOUCH_POINTS];              //x coordinate
        u16 au16_y[CFG_MAX_TOUCH_POINTS];              //y coordinate
        u8  au8_touch_event[CFG_MAX_TOUCH_POINTS];     //touch event:  0 -- down; 1-- contact; 2 -- contact
        u8  au8_finger_id[CFG_MAX_TOUCH_POINTS];       //touch ID
	u16	pressure;
        u8  touch_point;
};


struct ft5x0x_ts_data {
        struct i2c_client     *client;
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend	early_suspend;
//	struct mutex device_mode_mutex;   /* Ensures that only one function can specify the Device Mode at a time. */
};

//register address
#define FT5x06_REG_FW_VER 0xA6

#define FT5x0x_TX_NUM	21
#define FT5x0x_RX_NUM   12
//u16 g_RawData[FT5x0x_TX_NUM][FT5x0x_RX_NUM];
static u8 ft5x0x_enter_factory(struct ft5x0x_ts_data *ft5x0x_ts);
static u8 ft5x0x_enter_work(struct ft5x0x_ts_data *ft5x0x_ts);

#if CFG_SUPPORT_TOUCH_KEY
int tsp_keycodes[CFG_NUMOFKEYS] ={

        KEY_MENU,
        KEY_HOME,
        KEY_BACK,
        KEY_SEARCH
};

char *tsp_keyname[CFG_NUMOFKEYS] ={

        "Menu",
        "Home",
        "Back",
        "Search"
};

static bool tsp_keystatus[CFG_NUMOFKEYS];
#endif
/***********************************************************************************************
Name	:	ft5x0x_i2c_rxdata

Input	:	*rxdata
*length

Output	:	ret

function	:

***********************************************************************************************/
static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

        //msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	:	 ft5x0x_write_reg

Input	:	addr -- address
                     para -- parameter

Output	:

function	:	write register of ft5x0x

***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
        u8 buf[3];
        int ret = -1;

        buf[0] = addr;
        buf[1] = para;
        ret = ft5x0x_i2c_txdata(buf, 2);
        if (ret < 0) {
                pr_err("write reg failed! %#x ret: %d", buf[0], ret);
                return -1;
        }

        return 0;
}


/***********************************************************************************************
Name	:	ft5x0x_read_reg

Input	:	addr
                     pdata

Output	:

function	:	read register of ft5x0x

***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msgs[2];

	buf[0] = addr;    //register address
	msgs[0].addr = this_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = buf;
	msgs[1].addr = this_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;

}


/***********************************************************************************************
Name	:	 ft5x0x_read_fw_ver

Input	:	 void


Output	:	 firmware version

function	:	 read TP firmware version

***********************************************************************************************/
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver = 0;
	ft5x0x_read_reg(FT5X0X_REG_FIRMID, &ver);
	return(ver);
}

#if 1  //upgrade related
typedef enum
{
        ERR_OK,
        ERR_MODE,
        ERR_READID,
        ERR_ERASE,
        ERR_STATUS,
        ERR_ECC,
        ERR_DL_ERASE_FAIL,
        ERR_DL_PROGRAM_FAIL,
        ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

typedef struct _FTS_CTP_PROJECT_SETTING_T
{
        unsigned char uc_i2C_addr;             //I2C slave address (8 bit address)
        unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;	1----1.8v
        unsigned char uc_panel_factory_id;     //TP panel factory ID
}FTS_CTP_PROJECT_SETTING_T;

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE              0x0

#define I2C_CTPM_ADDRESS       0x70


void delay_qt_ms(unsigned long  w_ms)
{
        unsigned long i;
        unsigned long j;

        for (i = 0; i < w_ms; i++)
        {
                for (j = 0; j < 1000; j++)
                {
                        udelay(1);
                }
        }
}


/*
  [function]:
  callback: read data from ctpm by i2c interface,implemented by special user;
  [parameters]:
  bt_ctpm_addr[in]    :the address of the ctpm;
  pbt_buf[out]        :data buffer;
  dw_lenth[in]        :the length of the data buffer;
  [return]:
  FTS_TRUE     :success;
  FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
        int ret;

        ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

        if(ret<=0)
        {
                printk("[FTS]i2c_read_interface error\n");
                return FTS_FALSE;
        }

        return FTS_TRUE;
}

/*
  [function]:
  callback: write data to ctpm by i2c interface,implemented by special user;
  [parameters]:
  bt_ctpm_addr[in]    :the address of the ctpm;
  pbt_buf[in]        :data buffer;
  dw_lenth[in]        :the length of the data buffer;
  [return]:
  FTS_TRUE     :success;
  FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
        int ret;
        ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
        if(ret<=0)
        {
                printk("[FTS]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
                return FTS_FALSE;
        }

        return FTS_TRUE;
}

/*
  [function]:
  send a command to ctpm.
  [parameters]:
  btcmd[in]        :command code;
  btPara1[in]    :parameter 1;
  btPara2[in]    :parameter 2;
  btPara3[in]    :parameter 3;
  num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;
  [return]:
  FTS_TRUE    :success;
  FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
        FTS_BYTE write_cmd[4] = {0};

        write_cmd[0] = btcmd;
        write_cmd[1] = btPara1;
        write_cmd[2] = btPara2;
        write_cmd[3] = btPara3;
        return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
  [function]:
  write data to ctpm , the destination address is 0.
  [parameters]:
  pbt_buf[in]    :point to data buffer;
  bt_len[in]        :the data numbers;
  [return]:
  FTS_TRUE    :success;
  FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{

        return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
  [function]:
  read out data from ctpm,the destination address is 0.
  [parameters]:
  pbt_buf[out]    :point to data buffer;
  bt_len[in]        :the data numbers;
  [return]:
  FTS_TRUE    :success;
  FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
        return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}


/*
  [function]:
  burn the FW to ctpm.
  [parameters]:(ref. SPEC)
  pbt_buf[in]    :point to Head+FW ;
  dw_lenth[in]:the length of the FW + 6(the Head length);
  bt_ecc[in]    :the ECC of the FW
  [return]:
  ERR_OK        :no error;
  ERR_MODE    :fail to switch to UPDATE mode;
  ERR_READID    :read id fail;
  ERR_ERASE    :erase chip fail;
  ERR_STATUS    :status error;
  ERR_ECC        :ecc error.
*/


#define    FTS_PACKET_LENGTH        128

static unsigned char CTPM_FW[]=
{
#if CFG_SUPPORT_AUTO_UPG
#include "ft_app.i"
#endif
};

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
        FTS_BYTE reg_val[2] = {0};
        FTS_DWRD i = 0;

        FTS_DWRD  packet_number;
        FTS_DWRD  j;
        FTS_DWRD  temp;
        FTS_DWRD  lenght;
        FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
        FTS_BYTE  auc_i2c_write_buf[10];
        FTS_BYTE bt_ecc;
        int      i_ret;

        if(dw_lenth == 0)
                return 0;

        /*********Step 1:Reset  CTPM *****/
        /*write 0xaa to register 0xfc*/
        ft5x0x_write_reg(0xfc,0xaa);
        delay_qt_ms(50);
        /*write 0x55 to register 0xfc*/
        ft5x0x_write_reg(0xfc,0x55);
        printk("[FTS] Step 1: Reset CTPM test\n");

        delay_qt_ms(30);


        /*********Step 2:Enter upgrade mode *****/
        auc_i2c_write_buf[0] = 0x55;
        auc_i2c_write_buf[1] = 0xaa;
        do
        {
                i ++;
                i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
                delay_qt_ms(5);
        }while(i_ret <= 0 && i < 5 );

        /*********Step 3:check READ-ID***********************/
        cmd_write(0x90,0x00,0x00,0x00,4);
        byte_read(reg_val,2);
        if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
        {
                printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
        }
        else
        {
                return ERR_READID;
                //i_is_new_protocol = 1;
        }

        cmd_write(0xcd,0x0,0x00,0x00,1);
        byte_read(reg_val,1);
        printk("[FTS] bootloader version = 0x%x\n", reg_val[0]);

        /*********Step 4:erase app and panel paramenter area ********************/
        cmd_write(0x61,0x00,0x00,0x00,1);  //erase app area
        delay_qt_ms(1500);
        cmd_write(0x63,0x00,0x00,0x00,1);  //erase panel parameter area
        delay_qt_ms(100);
        printk("[FTS] Step 4: erase. \n");

        /*********Step 5:write firmware(FW) to ctpm flash*********/
        bt_ecc = 0;
        printk("[FTS] Step 5: start upgrade. \n");
        dw_lenth = dw_lenth - 8;
        packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
        packet_buf[0] = 0xbf;
        packet_buf[1] = 0x00;
        for (j=0;j<packet_number;j++)
        {
                temp = j * FTS_PACKET_LENGTH;
                packet_buf[2] = (FTS_BYTE)(temp>>8);
                packet_buf[3] = (FTS_BYTE)temp;
                lenght = FTS_PACKET_LENGTH;
                packet_buf[4] = (FTS_BYTE)(lenght>>8);
                packet_buf[5] = (FTS_BYTE)lenght;

                for (i=0;i<FTS_PACKET_LENGTH;i++)
                {
                        packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
                        bt_ecc ^= packet_buf[6+i];
                }

                byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
                delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
                if ((j * FTS_PACKET_LENGTH % 1024) == 0)
                {
                        printk("[FTS] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
                }
        }

        if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
        {
                temp = packet_number * FTS_PACKET_LENGTH;
                packet_buf[2] = (FTS_BYTE)(temp>>8);
                packet_buf[3] = (FTS_BYTE)temp;

                temp = (dw_lenth) % FTS_PACKET_LENGTH;
                packet_buf[4] = (FTS_BYTE)(temp>>8);
                packet_buf[5] = (FTS_BYTE)temp;

                for (i=0;i<temp;i++)
                {
                        packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
                        bt_ecc ^= packet_buf[6+i];
                }

                byte_write(&packet_buf[0],temp+6);
                delay_qt_ms(20);
        }

        //send the last six byte
        for (i = 0; i<6; i++)
        {
                temp = 0x6ffa + i;
                packet_buf[2] = (FTS_BYTE)(temp>>8);
                packet_buf[3] = (FTS_BYTE)temp;
                temp =1;
                packet_buf[4] = (FTS_BYTE)(temp>>8);
                packet_buf[5] = (FTS_BYTE)temp;
                packet_buf[6] = pbt_buf[ dw_lenth + i];
                bt_ecc ^= packet_buf[6];

                byte_write(&packet_buf[0],7);
                delay_qt_ms(20);
        }

        /*********Step 6: read out checksum***********************/
        /*send the opration head*/
        cmd_write(0xcc,0x00,0x00,0x00,1);
        byte_read(reg_val,1);
        printk("[FTS] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
        if(reg_val[0] != bt_ecc)
        {
                return ERR_ECC;
        }

        /*********Step 7: reset the new FW***********************/
        cmd_write(0x07,0x00,0x00,0x00,1);

        msleep(300);  //make sure CTP startup normally

        return ERR_OK;
}

static int fts_Get_RawData(u16 RawData[][FT5x0x_RX_NUM])
{
	int retval  = 0;
	int i       = 0;
	u8  devmode = 0x00;
	u8  rownum  = 0x00;
        struct i2c_msg msgs[1];
	u8 read_buffer[FT5x0x_RX_NUM * 2];
	u8 read_len = 0;
	//u8 write_buffer[2];
	struct ft5x0x_ts_data * ft5x0x_ts =  i2c_get_clientdata(this_client);
	if(ft5x0x_enter_factory(ft5x0x_ts)<0)
	{
		pr_err("%s ERROR: could not enter factory mode", __FUNCTION__);
		retval = -1;
		goto error_return;
	}
	//scan
	if(ft5x0x_read_reg(0x00, &devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	devmode |= 0x80;
	if(ft5x0x_write_reg(0x00, devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	msleep(20);
	if(ft5x0x_read_reg(0x00, &devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	if(0x00 != (devmode&0x80))
	{
		pr_err("%s %d ERROR: could not scan", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}

	pr_info("Read rawdata .......\n");
	for(rownum=0; rownum<FT5x0x_TX_NUM; rownum++)
	{
		memset(read_buffer, 0, (FT5x0x_RX_NUM * 2));

		if(ft5x0x_write_reg(0x01, rownum)<0)
		{
			pr_err("%s ERROR:could not write rownum", __FUNCTION__);
			retval = -1;
			goto error_return;
		}
		msleep(1);
		read_len = FT5x0x_RX_NUM * 2;
		if(ft5x0x_write_reg(0x10, read_len)<0)
		{
			pr_err("%s ERROR:could not write rownum", __FUNCTION__);
			retval = -1;
			goto error_return;
		}

                msgs[0].addr	= this_client->addr;
                msgs[0].flags	= 1;
                msgs[0].len	= FT5x0x_RX_NUM * 2;
                msgs[0].buf	= read_buffer;
		retval = i2c_transfer(this_client->adapter, msgs, 1);
		if (retval < 0)
		{
			pr_err("%s ERROR:Could not read row %u raw data", __FUNCTION__, rownum);
			retval = -1;
			goto error_return;
		}
		for(i=0; i<FT5x0x_RX_NUM; i++)
		{
			RawData[rownum][i] = (read_buffer[i<<1]<<8) + read_buffer[(i<<1)+1];
		}
	}
error_return:
	if(ft5x0x_enter_work(ft5x0x_ts)<0)
	{
		pr_err("%s ERROR:could not enter work mode ", __FUNCTION__);
		retval = -1;
	}
	return retval;
}

int fts_ctpm_auto_clb(void)
{
        unsigned char uc_temp;
        unsigned char i ;

        printk("[FTS] start auto CLB.\n");
        msleep(200);
        ft5x0x_write_reg(0, 0x40);
        delay_qt_ms(100);   //make sure already enter factory mode
        ft5x0x_write_reg(2, 0x4);  //write command to start calibration
        delay_qt_ms(300);
        for(i=0;i<100;i++)
        {
                ft5x0x_read_reg(0,&uc_temp);
                if ( ((uc_temp&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
                {
                        break;
                }
                delay_qt_ms(200);
                printk("[FTS] waiting calibration %d\n",i);

        }
        printk("[FTS] calibration OK.\n");

        msleep(300);
        ft5x0x_write_reg(0, 0x40);  //goto factory mode
        delay_qt_ms(100);   //make sure already enter factory mode
        ft5x0x_write_reg(2, 0x5);  //store CLB result
        delay_qt_ms(300);
        ft5x0x_write_reg(0, 0x0); //return to normal mode
        msleep(300);
        printk("[FTS] store CLB result OK.\n");
        return 0;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
        FTS_BYTE*     pbt_buf = FTS_NULL;
        int i_ret;

        //=========FW upgrade========================*/
        pbt_buf = CTPM_FW;
        /*call the upgrade function*/
        i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
        if (i_ret != 0)
        {
                printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
                //error handling ...
                //TBD
        }
        else
        {
                printk("[FTS] upgrade successfully.\n");
                fts_ctpm_auto_clb();  //start auto CLB
        }

        return i_ret;
}

unsigned char fts_ctpm_get_i_file_ver(void)
{
        unsigned int ui_sz;
        ui_sz = sizeof(CTPM_FW);
        if (ui_sz > 2)
        {
                return CTPM_FW[ui_sz - 2];
        }
        else
        {
                //TBD, error handling?
                return 0xff; //default value
        }
}

#define    FTS_SETTING_BUF_LEN        128

//update project setting
//only update these settings for COB project, or for some special case
int fts_ctpm_update_project_setting(void)
{
        unsigned char uc_i2c_addr;             //I2C slave address (8 bit address)
        unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;	1----1.8v
        unsigned char uc_panel_factory_id;     //TP panel factory ID

        unsigned char buf[FTS_SETTING_BUF_LEN];
        FTS_BYTE reg_val[2] = {0};
        FTS_BYTE  auc_i2c_write_buf[10];
        FTS_BYTE  packet_buf[FTS_SETTING_BUF_LEN + 6];
        FTS_DWRD i = 0;
        int      i_ret;

        uc_i2c_addr = 0x70;
        uc_io_voltage = 0x0;
        uc_panel_factory_id = 0x5a;

        /*********Step 1:Reset  CTPM *****/
        /*write 0xaa to register 0xfc*/
        ft5x0x_write_reg(0xfc,0xaa);
        delay_qt_ms(50);
        /*write 0x55 to register 0xfc*/
        ft5x0x_write_reg(0xfc,0x55);
        printk("[FTS] Step 1: Reset CTPM test\n");

        delay_qt_ms(30);

        /*********Step 2:Enter upgrade mode *****/
        auc_i2c_write_buf[0] = 0x55;
        auc_i2c_write_buf[1] = 0xaa;
        do
        {
                i ++;
                i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
                delay_qt_ms(5);
        }while(i_ret <= 0 && i < 5 );

        /*********Step 3:check READ-ID***********************/
        cmd_write(0x90,0x00,0x00,0x00,4);
        byte_read(reg_val,2);
        if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
        {
                printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
        }
        else
        {
                return ERR_READID;
        }

        cmd_write(0xcd,0x0,0x00,0x00,1);
        byte_read(reg_val,1);
        printk("bootloader version = 0x%x\n", reg_val[0]);


        /* --------- read current project setting  ---------- */
        //set read start address
        buf[0] = 0x3;
        buf[1] = 0x0;
        buf[2] = 0x78;
        buf[3] = 0x0;
        byte_write(buf, 4);
        byte_read(buf, FTS_SETTING_BUF_LEN);

        printk("[FTS] old setting: uc_i2c_addr = 0x%x, uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
               buf[0],  buf[2], buf[4]);
        for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
        {
                if (i % 16 == 0)     printk("\n");
                printk("0x%x, ", buf[i]);

        }
        printk("\n");

        /*--------- Step 4:erase project setting --------------*/
        cmd_write(0x62,0x00,0x00,0x00,1);
        delay_qt_ms(100);

        /*----------  Set new settings ---------------*/
        buf[0] = uc_i2c_addr;
        buf[1] = ~uc_i2c_addr;
        buf[2] = uc_io_voltage;
        buf[3] = ~uc_io_voltage;
        buf[4] = uc_panel_factory_id;
        buf[5] = ~uc_panel_factory_id;
        packet_buf[0] = 0xbf;
        packet_buf[1] = 0x00;
        packet_buf[2] = 0x78;
        packet_buf[3] = 0x0;
        packet_buf[4] = 0;
        packet_buf[5] = FTS_SETTING_BUF_LEN;
        for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
        {
                packet_buf[6 + i] = buf[i];
                if (i % 16 == 0)     printk("\n");
                printk("0x%x, ", buf[i]);
        }
        printk("\n");
        byte_write(&packet_buf[0],FTS_SETTING_BUF_LEN + 6);
        delay_qt_ms(100);

        /********* reset the new FW***********************/
        cmd_write(0x07,0x00,0x00,0x00,1);

        msleep(200);

        return 0;

}



#if CFG_SUPPORT_AUTO_UPG

int fts_ctpm_auto_upg(void)
{
        unsigned char uc_host_fm_ver;
        unsigned char uc_tp_fm_ver;
        int           i_ret;

        uc_tp_fm_ver = ft5x0x_read_fw_ver();
        uc_host_fm_ver = fts_ctpm_get_i_file_ver();
        if ( uc_tp_fm_ver == 0xa6  ||   //the firmware in touch panel maybe corrupted
             uc_tp_fm_ver < uc_host_fm_ver //the firmware in host flash is new, need upgrade
                )
        {
                msleep(100);
                printk("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
                       uc_tp_fm_ver, uc_host_fm_ver);
                i_ret = fts_ctpm_fw_upgrade_with_i_file();
                if (i_ret == 0)
                {
                        msleep(300);
                        uc_host_fm_ver = fts_ctpm_get_i_file_ver();
                        printk("[FTS] upgrade to new version 0x%x\n", uc_host_fm_ver);
                }
                else
                {
                        printk("[FTS] upgrade failed ret=%d.\n", i_ret);
                }
        }

        return 0;
}

#endif

#endif


/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_release(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_sync(data->input_dev);
}


//read touch point information
static int ft5x0x_read_data(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[CFG_POINT_READ_BUF] = {0};
	int ret = -1;
	int i;

	ret = ft5x0x_i2c_rxdata(buf, CFG_POINT_READ_BUF);
        if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x0F;
	print_point_info("touch point = %d\n",event->touch_point);

        if (event->touch_point > CFG_MAX_TOUCH_POINTS)
        {
                event->touch_point = CFG_MAX_TOUCH_POINTS;
        }

	u16 swp;
        for (i = 0; i < event->touch_point; i++)
        {
                event->au16_x[i] = (s16)(buf[3 + 6*i] & 0x0F)<<8 | (s16)buf[4 + 6*i];
                event->au16_y[i] = (s16)(buf[5 + 6*i] & 0x0F)<<8 | (s16)buf[6 + 6*i];
                event->au8_touch_event[i] = buf[0x3 + 6*i] >> 6;
                event->au8_finger_id[i] = (buf[5 + 6*i])>>4;
                printk("[%d, %d]", event->au16_x[i], event->au16_y[i]);
		if(1 == exchange_x_y_flag){
			swp = event->au16_x[i];
			event->au16_x[i] = event->au16_y[i];
			event->au16_y[i] = swp;
		}
		if(1 == revert_x_flag){
			event->au16_x[i] = SCREEN_MAX_X - event->au16_x[i];
		}
		if(1 == revert_y_flag){
			event->au16_y[i] = SCREEN_MAX_Y - event->au16_y[i];
		}
                printk("(%d, %d)@%d  ", event->au16_x[i], event->au16_y[i], (i+1));
        }

        printk("\n");

        event->pressure = 200;

        return 0;
}

/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/


#if CFG_SUPPORT_TOUCH_KEY
int ft5x0x_touch_key_process(struct input_dev *dev, int x, int y, int touch_event)
{
        int i;
        int key_id;

        if ( y < 517&&y > 497)
        {
                key_id = 1;
        }
        else if ( y < 367&&y > 347)
        {
                key_id = 0;
        }

        else if ( y < 217&&y > 197)
        {
                key_id = 2;
        }
        else if (y < 67&&y > 47)
        {
                key_id = 3;
        }
        else
        {
                key_id = 0xf;
        }

        for(i = 0; i <CFG_NUMOFKEYS; i++ )
        {
                if(tsp_keystatus[i])
                {
                        input_report_key(dev, tsp_keycodes[i], 0);

                        printk("[FTS] %s key is release. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);

                        tsp_keystatus[i] = KEY_RELEASE;
                }
                else if( key_id == i )
                {
                        if( touch_event == 0)                                  // detect
                        {
                                input_report_key(dev, tsp_keycodes[i], 1);
                                printk( "[FTS] %s key is pressed. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
                                tsp_keystatus[i] = KEY_PRESS;
                        }
                }
        }
        return 0;

}
#endif

static void ft5x0x_report_value(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	int i;

	for (i  = 0; i < event->touch_point; i++)
	{
                if (event->au16_x[i] < SCREEN_MAX_X && event->au16_y[i] < SCREEN_MAX_Y)
                        // LCD view area
                {
                        if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
                        {
                                input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
                        }
                        else
                        {
                                input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
                        }
                        input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
                        input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
                        input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
                        input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->au8_finger_id[i]);
                        //input_mt_sync(data->input_dev);
                }
                else //maybe the touch key area
                {
#if CFG_SUPPORT_TOUCH_KEY
                        if (event->au16_x[i] >= SCREEN_MAX_X)
                        {
                                ft5x0x_touch_key_process(data->input_dev, event->au16_x[i], event->au16_y[i],
                                                         event->au8_touch_event[i]);
                        }
#endif
                }

		input_mt_sync(data->input_dev);
	}
        input_report_key(data->input_dev, BTN_TOUCH, (event->touch_point > 0));
	input_sync(data->input_dev);

        if (event->touch_point == 0) {
                ft5x0x_ts_release();
                return ;
        }


}	/*end ft5x0x_report_value*/


/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
        struct ft5x0x_ts_data *ts = container_of(work, struct ft5x0x_ts_data, pen_event_work);

	ret = ft5x0x_read_data();
	if (ret == 0) {
		ft5x0x_report_value();
	}

//	enable_irq(ts->client->irq);
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;

	print_int_info("==========------ft5x0x_ts TS Interrupt-----============\n");
	if(!ctp_ops.judge_int_occur()){
		print_int_info("==IRQ_EINT%d=\n",CTP_IRQ_NO);
		ctp_ops.clear_penirq();
		if (!work_pending(&ft5x0x_ts->pen_event_work))
		{
			print_int_info("Enter work\n");
			queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
		}
	}else{
		print_int_info("Other Interrupt\n");
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
//	struct ft5x0x_ts_data *ts;
//	ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);

	printk("==ft5x0x_ts_suspend=\n");
//	disable_irq(this_client->irq);
//	disable_irq(IRQ_EINT(6));
//	cancel_work_sync(&ts->pen_event_work);
//	flush_workqueue(ts->ts_workqueue);
	// ==set mode ==,
	ft5x0x_write_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	printk("==ft5x0x_ts_resume=\n");
	// wake the mode
//	__gpio_as_output(GPIO_FT5X0X_WAKE);
//	__gpio_clear_pin(GPIO_FT5X0X_WAKE);		//set wake = 0,base on system
//	 msleep(100);
//	__gpio_set_pin(GPIO_FT5X0X_WAKE);			//set wake = 1,base on system
//	msleep(100);
//	enable_irq(this_client->irq);
//	enable_irq(IRQ_EINT(6));
}
#endif  //CONFIG_HAS_EARLYSUSPEND

/* sysfs */

static u8 ft5x0x_enter_factory(struct ft5x0x_ts_data *ft5x0x_ts)
{
	u8 regval;
	flush_workqueue(ft5x0x_ts->ts_workqueue);
	disable_irq_nosync(ft5x0x_ts->client->irq);
	ft5x0x_write_reg(0x00, 0x40);  //goto factory mode
	delay_qt_ms(100);   //make sure already enter factory mode
	if(ft5x0x_read_reg(0x00, &regval)<0)
		pr_err("%s ERROR: could not read register\n", __FUNCTION__);
	else
	{
		if((regval & 0x70) != 0x40)
		{
			pr_err("%s() - ERROR: The Touch Panel was not put in Factory Mode. The Device Mode register contains 0x%02X\n", __FUNCTION__, regval);
			return -1;
		}
	}
	return 0;
}
static u8 ft5x0x_enter_work(struct ft5x0x_ts_data *ft5x0x_ts)
{
	u8 regval;
        ft5x0x_write_reg(0x00, 0x00); //return to normal mode
        msleep(100);

	if(ft5x0x_read_reg(0x00, &regval)<0)
		pr_err("%s ERROR: could not read register\n", __FUNCTION__);
	else
	{
		if((regval & 0x70) != 0x00)
		{
			pr_err("%s() - ERROR: The Touch Panel was not put in Work Mode. The Device Mode register contains 0x%02X\n", __FUNCTION__, regval);
//			enable_irq(ft5x0x_ts->client->irq);
			return -1;
		}
	}
//	enable_irq(ft5x0x_ts->client->irq);
	return 0;
}

static int ft5x0x_GetFirmwareSize(char * firmware_name)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "/sdcard/%s", firmware_name);
	pr_info("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
        }

	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		return -1;
        }
	inode=pfile->f_dentry->d_inode;
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}
static int ft5x0x_ReadFirmware(char * firmware_name, unsigned char * firmware_buf)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

        memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "/sdcard/%s", firmware_name);
	pr_info("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
        }

	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		return -1;
        }
	inode=pfile->f_dentry->d_inode;
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size;
	//char * buf;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	vfs_read(pfile, firmware_buf, fsize, &pos);

	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

int fts_ctpm_fw_upgrade_with_app_file(char * firmware_name)
{
	FTS_BYTE*     pbt_buf = FTS_NULL;
	int i_ret; u8 fwver;
	int fwsize = ft5x0x_GetFirmwareSize(firmware_name);
	if(fwsize <= 0)
	{
		pr_err("%s ERROR:Get firmware size failed\n", __FUNCTION__);
		return -1;
	}
        //=========FW upgrade========================*/
        pbt_buf = (unsigned char *) kmalloc(fwsize+1,GFP_ATOMIC);
	if(ft5x0x_ReadFirmware(firmware_name, pbt_buf))
	{
                pr_err("%s() - ERROR: request_firmware failed\n", __FUNCTION__);
		kfree(pbt_buf);
		return -1;
	}
	/*call the upgrade function*/
	i_ret =  fts_ctpm_fw_upgrade(pbt_buf, fwsize);
	if (i_ret != 0)
	{
                pr_err("%s() - ERROR:[FTS] upgrade failed i_ret = %d.\n",__FUNCTION__,  i_ret);
                //error handling ...
                //TBD
	}
        else
	{
                pr_info("[FTS] upgrade successfully.\n");
		if(ft5x0x_read_reg(FT5x06_REG_FW_VER, &fwver)>=0)
			pr_info("the new fw ver is 0x%02x\n", fwver);
		fts_ctpm_auto_clb();  //start auto CLB
	}
	kfree(pbt_buf);
	return i_ret;
}

static ssize_t ft5x0x_tpfwver_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
	struct ft5x0x_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	u8	   fwver = 0;

	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );

//	mutex_lock(&data->device_mode_mutex);
	if(ft5x0x_read_reg(FT5x06_REG_FW_VER, &fwver) < 0)
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", fwver);

//	mutex_unlock(&data->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x0x_tpfwver_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5x0x_tprwreg_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5x0x_tprwreg_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
	struct ft5x0x_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	unsigned int num_read_chars = 0;
	int retval;
	unsigned long wmreg=0;
        u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5];

        memset(valbuf, 0, sizeof(valbuf));
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
//	mutex_lock(&data->device_mode_mutex);
	num_read_chars = count - 1;

	if(num_read_chars!=2)
	{
		if(num_read_chars!=4)
		{
			pr_info("please input 2 or 4 character\n");
			goto error_return;
		}
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);

	if (0 != retval)
	{
		pr_err("%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n",
                       __FUNCTION__, buf);
		goto error_return;
	}

	if(2 == num_read_chars)
	{
		//read register
		regaddr = (unsigned short)wmreg;
		if(ft5x0x_read_reg(regaddr, &regvalue) < 0)
			pr_err("Could not read the register(0x%02x)\n", regaddr);
		else
			pr_info("the register(0x%02x) is 0x%02x\n", regaddr, regvalue);
	}
	else
	{
		regaddr = (wmreg>>8) & 0xFF;
		regvalue = wmreg & 0xFF;
		if(ft5x0x_write_reg(regaddr, regvalue)<0)
			pr_err("Could not write the register(0x%02x)\n", regaddr);
		else
			pr_err("Write 0x%02x into register(0x%02x) successful\n", regvalue, regaddr);
	}
error_return:
//	mutex_unlock(&data->device_mode_mutex);

	return count;
}


static ssize_t ft5x0x_fwupdate_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
        return -EPERM;
}
//upgrade from *.i
static ssize_t ft5x0x_fwupdate_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	struct ft5x0x_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	u8 uc_host_fm_ver;
        int i_ret;

	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
//	mutex_lock(&data->device_mode_mutex);

	disable_irq(data->client->irq);
	i_ret = fts_ctpm_fw_upgrade_with_i_file();
        if (i_ret == 0) {
                msleep(300);
                uc_host_fm_ver = fts_ctpm_get_i_file_ver();
                pr_info("%s [FTS] upgrade to new version 0x%x\n", __FUNCTION__, uc_host_fm_ver);
        } else {
                pr_err("%s ERROR:[FTS] upgrade failed ret=%d.\n", __FUNCTION__, i_ret);
        }
//	enable_irq(data->client->irq);

//	mutex_unlock(&data->device_mode_mutex);

	return count;
}

static ssize_t ft5x0x_fwupgradeapp_show(struct device *dev,
                                        struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
        return -EPERM;
}
//upgrade from app.bin
static ssize_t ft5x0x_fwupgradeapp_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count)
{
	struct ft5x0x_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	char fwname[128];

	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
        memset(fwname, 0, sizeof(fwname));

	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

//	mutex_lock(&data->device_mode_mutex);
	disable_irq(data->client->irq);

	fts_ctpm_fw_upgrade_with_app_file(fwname);

//	enable_irq(data->client->irq);

//	mutex_unlock(&data->device_mode_mutex);

	return count;
}

static ssize_t ft5x0x_rawdata_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
	struct ft5x0x_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	unsigned int num_read_chars = 0;
	int i=0, j=0;
	u16 RawData[FT5x0x_TX_NUM][FT5x0x_RX_NUM];

	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
//	mutex_lock(&data->device_mode_mutex);

	if(fts_Get_RawData(RawData)<0)
		sprintf(buf, "%s", "could not get rawdata\n");
	else {
		for(i=0; i<FT5x0x_TX_NUM; i++)
		{
			for(j=0; j<FT5x0x_RX_NUM; j++)
			{
				num_read_chars += sprintf(&(buf[num_read_chars]), "%u ", RawData[i][j]);
			}
			buf[num_read_chars-1] = '\n';
		}
	}
//	mutex_unlock(&data->device_mode_mutex);

	return (ssize_t)num_read_chars;
}
//upgrade from app.bin
static ssize_t ft5x0x_rawdata_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{

	return -EPERM;
}


/* sysfs */
//static DEVICE_ATTR(rawbase, S_IRUGO|S_IWUSR, ft5x0x_rawbase_show, ft5x0x_rawbase_store);
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, ft5x0x_tpfwver_show, ft5x0x_tpfwver_store);
//upgrade from *.i
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, ft5x0x_fwupdate_show, ft5x0x_fwupdate_store);
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, ft5x0x_tprwreg_show, ft5x0x_tprwreg_store);
//upgrade from app.bin
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, ft5x0x_fwupgradeapp_show, ft5x0x_fwupgradeapp_store);
static DEVICE_ATTR(ftsrawdatashow, S_IRUGO|S_IWUSR, ft5x0x_rawdata_show, ft5x0x_rawdata_store);


static struct attribute *ft5x0x_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsrawdatashow.attr,
	NULL
};

static struct attribute_group ft5x0x_attribute_group = {
	.attrs = ft5x0x_attributes
};


/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static int
ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	struct device *dev;
	struct i2c_dev *i2c_dev;
	int err = 0;
	unsigned char uc_reg_value;
#if CFG_SUPPORT_TOUCH_KEY
        int i;
#endif

	printk("[FTS] ft5x0x_ts_probe, driver version is %s.\n", CFG_FTS_CTP_DRIVER_VERSION);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(struct ft5x0x_ts_data), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;

	//pr_info("ft5x0x_ts_probe : client->addr = %d. \n", client->addr);
	this_client->addr = client->addr;
	//pr_info("ft5x0x_ts_probe : client->addr = %d. \n", client->addr);
	i2c_set_clientdata(client, ft5x0x_ts);

//	mutex_init(&ft5x0x_ts->device_mode_mutex);
//	pr_info("==INIT_WORK=\n");
	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);
	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
#if 0
        ft5x0x_ts->client = client;

	err = request_irq(ft5x0x_ts->client->irq, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING, "ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq(ft5x0x_ts->client->irq);
#endif

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x0x_ts->input_dev = input_dev;

	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

        /* For single touch */
        input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
        input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);

        /* For multi touch */
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
        input_set_abs_params(input_dev,
			     ABS_MT_TRACKING_ID, 0, 5, 0, 0);

        set_bit(EV_KEY, input_dev->evbit);
        set_bit(EV_ABS, input_dev->evbit);
        //set_bit(EV_SYN, input_dev->evbit);
        set_bit(BTN_TOUCH, input_dev->keybit);

#if CFG_SUPPORT_TOUCH_KEY
        //setup key code area
        set_bit(EV_SYN, input_dev->evbit);
        set_bit(BTN_TOUCH, input_dev->keybit);
        input_dev->keycode = tsp_keycodes;
        for(i = 0; i < CFG_NUMOFKEYS; i++)
        {
                input_set_capability(input_dev, EV_KEY, ((int*)input_dev->keycode)[i]);
                tsp_keystatus[i] = KEY_RELEASE;
        }
#endif

	input_dev->name	= FT5X0X_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
                        "ft5x0x_ts_probe: failed to register input device: %s\n",
                        dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("==register_early_suspend =\n");
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

        msleep(150);  //make sure CTP already finish startup process

        //get some register information
        uc_reg_value = ft5x0x_read_fw_ver();
        printk("[FTS] Firmware version = 0x%x\n", uc_reg_value);
        ft5x0x_read_reg(FT5X0X_REG_PERIODACTIVE, &uc_reg_value);
        printk("[FTS] report rate is %dHz.\n", uc_reg_value * 10);
        ft5x0x_read_reg(FT5X0X_REG_THGROUP, &uc_reg_value);
        printk("[FTS] touch threshold is %d.\n", uc_reg_value * 4);

#if CFG_SUPPORT_AUTO_UPG
        fts_ctpm_auto_upg();
#endif

#if CFG_SUPPORT_UPDATE_PROJECT_SETTING
        fts_ctpm_update_project_setting();
#endif

#if 0
//        enable_irq(ft5x0x_ts->client->irq);
        //create sysfs
        err = sysfs_create_group(&client->dev.kobj, &ft5x0x_attribute_group);
        if (0 != err) {
                dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed: %d\n", __FUNCTION__, err);
                sysfs_remove_group(&client->dev.kobj, &ft5x0x_attribute_group);
        } else {
                printk("ft5x0x:%s() - sysfs_create_group() succeeded.\n", __FUNCTION__);
        }
#endif

#if 1
	err = ctp_ops.set_irq_mode("ctp_para", "ctp_int_port", CTP_IRQ_MODE);
	if(0 != err){
		pr_info("%s:ctp_ops.set_irq_mode err. \n", __func__);
		goto exit_set_irq_mode;
	}
	err = request_irq(SW_INT_IRQNO_PIO, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING | IRQF_SHARED, "ft5x0x_ts", ft5x0x_ts);

	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_ts_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)){
		err = PTR_ERR(i2c_dev);
		return err;
	}
	dev = device_create(i2c_dev_class, &client->adapter->dev, MKDEV(I2C_MAJOR,client->adapter->nr), NULL, "aw_i2c_ts%d", client->adapter->nr);
	if (IS_ERR(dev))	{
			err = PTR_ERR(dev);
			return err;
	}
#else
        ft5x0x_ts->client = client;

	err = request_irq(ft5x0x_ts->client->irq, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING, "ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq(ft5x0x_ts->client->irq);
#endif
	printk("[FTS] ==probe over =\n");
        return 0;

exit_irq_request_failed:
exit_set_irq_mode:
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	enable_irq(SW_INT_IRQNO_PIO);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(SW_INT_IRQNO_PIO, ft5x0x_ts);
exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	pr_info("==ft5x0x_ts_remove=\n");
	ft5x0x_ts = i2c_get_clientdata(client);
	ft5x0x_write_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	free_irq(SW_INT_IRQNO_PIO, ft5x0x_ts);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
#endif
//	free_irq(client->irq, ft5x0x_ts);
//	mutex_destroy(&ft5x0x_ts->device_mode_mutex);
//	free_irq(ft5x0x_ts->client->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	input_free_device(ft5x0x_ts->input_dev);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	kfree(ft5x0x_ts);
	i2c_set_clientdata(client, NULL);
	ctp_ops.free_platform_resource();
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ CTP_NAME, 0 },
	{}
};


MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
	},
	.address_list	= u_i2c_addr.normal_i2c,
};

static int aw_open(struct inode *inode, struct file *file)
{
	int subminor;
	int ret = 0;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	struct i2c_dev *i2c_dev;

	pr_info("====%s======.\n", __func__);

	#ifdef AW_DEBUG
	        pr_info("enter aw_open function\n");
	#endif

	subminor = iminor(inode);
	#ifdef AW_DEBUG
	      pr_info("subminor=%d\n",subminor);
	#endif

	//lock_kernel();
	i2c_dev = i2c_dev_get_by_minor(2);
	if (!i2c_dev)	{
		pr_info("error i2c_dev\n");
		return -ENODEV;
	}

	adapter = i2c_get_adapter(i2c_dev->adap->nr);
	if (!adapter)	{
		return -ENODEV;
	}

	client = kzalloc(sizeof(*client), GFP_KERNEL);

	if (!client)	{
		i2c_put_adapter(adapter);
		ret = -ENOMEM;
	}
	snprintf(client->name, I2C_NAME_SIZE, "pctp_i2c_ts%d", adapter->nr);
	client->driver = &ft5x0x_ts_driver;
	client->adapter = adapter;
	file->private_data = client;

	return 0;
}

static long aw_ioctl(struct file *file, unsigned int cmd,unsigned long arg )
{
	//struct i2c_client *client = (struct i2c_client *) file->private_data;

	pr_info("====%s====.\n",__func__);

	#ifdef AW_DEBUG
	       pr_info("line :%d,cmd = %d,arg = %d.\n",__LINE__,cmd,arg);
	#endif

	switch (cmd) {
		case UPGRADE:
		pr_info("==UPGRADE_WORK=\n");
		fts_ctpm_fw_upgrade_with_i_file();
		// calibrate();

		break;

		default:
		break;

	}

	return 0;
}

static int aw_release (struct inode *inode, struct file *file)
{
	struct i2c_client *client = file->private_data;
	#ifdef AW_DEBUG
	    pr_info("enter aw_release function.\n");
	#endif

	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;
	return 0;
}

static const struct file_operations aw_i2c_ts_fops ={
	.owner = THIS_MODULE,
	//.read = aw_read,
	//.write = aw_write,
	.open = aw_open,
	.unlocked_ioctl = aw_ioctl,
	.release = aw_release,
};

/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static int __init ft5x0x_ts_init(void)
{
	int ret = -1;
	int err = -1;
	printk("==ft5x0x_ts_init==\n");

	if (ctp_ops.fetch_sysconfig_para)
	{
		if(ctp_ops.fetch_sysconfig_para()){
			pr_info("%s: err.\n", __func__);
			return -1;
		}
	}
	pr_info("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	err = ctp_ops.init_platform_resource();
	if(0 != err){
	    pr_info("%s:ctp_ops.init_platform_resource err. \n", __func__);
	}

	//reset
	ctp_ops.ts_reset();
	//wakeup
	ctp_ops.ts_wakeup();

	ft5x0x_ts_driver.detect = ctp_ops.ts_detect;

	ret= register_chrdev(I2C_MAJOR,"aw_i2c_ts",&aw_i2c_ts_fops );
	if(ret) {
		pr_info(KERN_ERR "%s:register chrdev failed\n",__FILE__);
		return ret;
	}
	else
		pr_info(KERN_ERR "%s:register chrdev success\n",__FILE__);

	i2c_dev_class = class_create(THIS_MODULE,"aw_i2c_dev");
	if (IS_ERR(i2c_dev_class)) {
		ret = PTR_ERR(i2c_dev_class);
		class_destroy(i2c_dev_class);
	}

	ret = i2c_add_driver(&ft5x0x_ts_driver);
	printk("i2c_add_driver ret=%d\n",ret);
	return ret;
//	return i2c_add_driver(&ft5x0x_ts_driver);
}

/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void __exit ft5x0x_ts_exit(void)
{
	pr_info("==ft5x0x_ts_exit==\n");
	i2c_del_driver(&ft5x0x_ts_driver);
}

late_initcall(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
