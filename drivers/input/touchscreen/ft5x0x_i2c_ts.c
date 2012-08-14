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
 *
 *    note: only support mulititouch    Wenfs 2010-10-01
 */

//#define CONFIG_FTS_CUSTOME_ENV
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/ft5x0x_i2c_ts.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input/mt.h>

/* -------------- global variable definition -----------*/
static struct i2c_client *this_client;
static REPORT_FINGER_INFO_T _st_finger_infos[CFG_MAX_POINT_NUM];
static unsigned int _sui_irq_num=6;/* IRQ_EINT(6);*/
static int _si_touch_num = 0; 
static int global_i = 0;
static struct ft5x0x_platform_data *global_pdata = NULL;

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#include <linux/regulator/consumer.h>
void omap4_commix_vaux2_power(int action);
static void fts_ts_early_suspend(struct early_suspend *h);
static void fts_ts_late_resume(struct early_suspend *h);
#endif

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

/***********************************************************************
    [function]: 
		           callback:              read data from ctpm by i2c interface;
    [parameters]:
			    buffer[in]:            data buffer;
			    length[in]:           the length of the data buffer;
    [return]:
			    FTS_TRUE:            success;
			    FTS_FALSE:           fail;
************************************************************************/
static bool i2c_read_interface(u8* pbt_buf, int dw_lenth)
{
    int ret;
    
    ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[TSP]i2c_read_interface error\n");
        return FTS_FALSE;
    }
  
    return FTS_TRUE;
}



/***********************************************************************
    [function]: 
		           callback:               write data to ctpm by i2c interface;
    [parameters]:
			    buffer[in]:             data buffer;
			    length[in]:            the length of the data buffer;
    [return]:
			    FTS_TRUE:            success;
			    FTS_FALSE:           fail;
************************************************************************/
static bool  i2c_write_interface(u8* pbt_buf, int dw_lenth)
{
	int ret;

	
    ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
    if(ret< 0)
    {
        printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}



/***********************************************************************
    [function]: 
		           callback:                 read register value ftom ctpm by i2c interface;
    [parameters]:
                        reg_name[in]:         the register which you want to read;
			    rx_buf[in]:              data buffer which is used to store register value;
			    rx_length[in]:          the length of the data buffer;
    [return]:
			    FTS_TRUE:              success;
			    FTS_FALSE:             fail;
************************************************************************/
static bool fts_register_read(u8 reg_name, u8* rx_buf, int rx_length)
{
	u8 read_cmd[2]= {0};
	u8 cmd_len 	= 0;


	read_cmd[0] = reg_name;
	cmd_len = 1;

	if(!i2c_write_interface(&read_cmd[0], cmd_len)) {  //send register addr
		return FTS_FALSE;
	}

	if(!i2c_read_interface(rx_buf, rx_length)) {  //call the read callback function to get the register value
		return FTS_FALSE;
	}
	return FTS_TRUE;
}


/***********************************************************************
    [function]: 
		           callback:                read register value ftom ctpm by i2c interface;
    [parameters]:
                        reg_name[in]:         the register which you want to write;
			    tx_buf[in]:              buffer which is contained of the writing value;
    [return]:
			    FTS_TRUE:              success;
			    FTS_FALSE:             fail;
************************************************************************/
static bool fts_register_write(u8 reg_name, u8* tx_buf)
{
	u8 write_cmd[2] = {0};

	write_cmd[0] = reg_name;
	write_cmd[1] = *tx_buf;

	/*call the write callback function*/
	return i2c_write_interface(write_cmd, 2);
}



/***********************************************************************
    [function]: 
		           callback:        report to the input system that the finger is put up;
    [parameters]:
                         null;
    [return]:
                         null;
************************************************************************/
static void fts_ts_release(void)
{
    struct FTS_TS_DATA_T *data = i2c_get_clientdata(this_client);
    int i;
    int i_need_sync = 0;
    for ( i= 0; i<CFG_MAX_POINT_NUM; ++i )
    {
        if ( _st_finger_infos[i].u2_pressure == -1 )
            continue;

        _st_finger_infos[i].u2_pressure = 0;

#if defined(CONFIG_SMARTQ_X7)
        input_report_abs(data->input_dev, ABS_MT_POSITION_X, _st_finger_infos[i].i2_y);
        input_report_abs(data->input_dev, ABS_MT_POSITION_Y, 1280 - _st_finger_infos[i].i2_x);
#else
        input_report_abs(data->input_dev, ABS_MT_POSITION_X, _st_finger_infos[i].i2_x);
        input_report_abs(data->input_dev, ABS_MT_POSITION_Y, _st_finger_infos[i].i2_y);
#endif
//        input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, _st_finger_infos[i].u2_pressure);
//        input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, _st_finger_infos[i].ui2_id);
//		input_report_key(data->input_dev, BTN_TOUCH, 0);
//        input_mt_sync(data->input_dev);

        i_need_sync = 1;

        if ( _st_finger_infos[i].u2_pressure == 0 )
            _st_finger_infos[i].u2_pressure= -1;
    }

     if (i_need_sync)
     {
        input_sync(data->input_dev);
      }
	 
    _si_touch_num = 0;
}






/***********************************************************************
    [function]: 
		           callback:                 read touch  data ftom ctpm by i2c interface;
    [parameters]:
			    rxdata[in]:              data buffer which is used to store touch data;
			    length[in]:              the length of the data buffer;
    [return]:
			    FTS_TRUE:              success;
			    FTS_FALSE:             fail;
************************************************************************/
static int fts_i2c_rxdata(u8 *rxdata, int length)
{
      int ret;
      struct i2c_msg msg;

//	  printk("this_client->addr = 0x%x fun = %s line = %d+++++++++++++++++++++++++++++++++++++++\n", this_client->addr,__func__, __LINE__ );
	
      msg.addr = this_client->addr;
      msg.flags = 0;
      msg.len = 1;
      msg.buf = rxdata;
      ret = i2c_transfer(this_client->adapter, &msg, 1);
	  
//	if (ret < 0)
//		pr_err("msg %s i2c write error,line =%d : %d\n", __func__, __LINE__, ret);
		
      msg.addr = this_client->addr;
    
	  msg.flags = I2C_M_RD;
      msg.len = length;
      msg.buf = rxdata;
      ret = i2c_transfer(this_client->adapter, &msg, 1);
//	if (ret < 0)
//		pr_err("msg %s i2c write error,line = %d: %d\n", __func__, __LINE__, ret);
//printk("ret=%d\n",ret);		
	return ret;
}





/***********************************************************************
    [function]: 
		           callback:                send data to ctpm by i2c interface;
    [parameters]:
			    txdata[in]:              data buffer which is used to send data;
			    length[in]:              the length of the data buffer;
    [return]:
			    FTS_TRUE:              success;
			    FTS_FALSE:             fail;
************************************************************************/
static int fts_i2c_txdata(u8 *txdata, int length)
{
	int ret;

	struct i2c_msg msg;

      msg.addr = this_client->addr;
      msg.flags = 0;
      msg.len = length;
      msg.buf = txdata;
	ret = i2c_transfer(this_client->adapter, &msg, 1);
//	if (ret < 0)
//		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}




/***********************************************************************
    [function]: 
		           callback:            gather the finger information and calculate the X,Y
		                                   coordinate then report them to the input system;
    [parameters]:
                         null;
    [return]:
                         null;
************************************************************************/
//#define DEBUG
#if 0
int fts_read_data(void)
{
//printk("fun = %s line = %d +++++++++++++++++++++++++++++++++++++++++++++++++++++++\n", __func__, __LINE__);
    struct FTS_TS_DATA_T *data = i2c_get_clientdata(this_client);
    struct FTS_TS_EVENT_T *event = &data->event;
    u8 buf[32] = {0};
    static int key_id=0x80;
	
    int i,j,id,temp,i_count,ret = -1;
    int touch_point_num = 0, touch_event, x, y, pressure, size;
    REPORT_FINGER_INFO_T touch_info[CFG_MAX_POINT_NUM];

   
    i_count = 0;

    do 
    {
        buf[0] = 3;
		
        id = 0xe;  
		
        ret=fts_i2c_rxdata(buf, 30);
#ifdef DEBUG
		printk("1  dump %02x %02x %02x %02x %02x %02x\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
		printk("2  dump %02x %02x %02x %02x %02x %02x\n",buf[6],buf[7],buf[8],buf[9],buf[10],buf[11]);
		printk("3  dump %02x %02x %02x %02x %02x %02x\n",buf[12],buf[13],buf[14],buf[15],buf[16],buf[17]);
		printk("4  dump %02x %02x %02x %02x %02x %02x\n",buf[18],buf[19],buf[20],buf[21],buf[22],buf[23]);
		printk("5  dump %02x %02x %02x %02x %02x %02x\n",buf[24],buf[25],buf[26],buf[27],buf[28],buf[29]);
#endif
        if (ret > 0)  
        {
            for(j=0;j<5;j++)
			{			
            id = buf[j*6+2]>>4;
		
#ifdef DEBUG
printk("debug : +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ id = %d\n", id);
#endif
	      //printk("\n--the id number is %d---\n",id);
            touch_event = buf[j*6+0]>>6;     
            if (id >= 0 && id< CFG_MAX_POINT_NUM)  
            {
            
                temp = buf[j*6+0]& 0x0f;
                temp = temp<<8;
                temp = temp | buf[j*6+1];
                x = temp; 
                if(x >= 300 && x < 800) {
                    x -= 10;
                } else if(x >= 800 && x <= 1045) {
                    x -= 20;
                }

                temp = (buf[j*6+2])& 0x0f;
                temp = temp<<8;
                temp = temp | buf[j*6+3];
                y=temp;

                pressure = buf[j*6+4] & 0x3f; 
                size = buf[j*6+5]&0xf0;
                size = (id<<8)|size;
                touch_event = buf[j*6+0]>>6; 
#ifdef DEBUG
printk("x=%d,y=%d,touch=%d,key_id= 0x%x\n",x,y,touch_event,key_id);
#endif
				
                if (touch_event == 0)  //press down
                {
		       if(y>=0 && y<850)
		      {

				
                    _st_finger_infos[id].u2_pressure= pressure;
                    _st_finger_infos[id].i2_x= (int16_t)x;
                    _st_finger_infos[id].i2_y= (int16_t)y;
                    _st_finger_infos[id].ui2_id  = size;
                    _si_touch_num ++;
#ifdef DEBUG
		      printk("\n--report x position  is  %d----\n",_st_finger_infos[id].i2_x);
		      printk("\n--report y position  is  %d----\n",_st_finger_infos[id].i2_y);
#endif
		      }  
				
               else if(y>=850 && y<=860)
               {
	                if (x>=75 && x<=90)
	                {
	                    key_id = 0;
#ifdef DEBUG
				printk("\n---virtual key 1 press---");		
#endif
	                }
	                else if ( x>=185 && x<=200)
	                {
	                    key_id = 1;
#ifdef DEBUG
				printk("\n---virtual key 2 press---");		
#endif
	                }
	                else if (x>=290 && x<=305)
	                {
	                    key_id = 2;
#ifdef DEBUG
				printk("\n---virtual key 3 press---");		
#endif
	                }
	                else if ( x>=405 && x<=420)
	                {
	                    key_id = 3;
#ifdef DEBUG
				printk("\n---virtual key 4 press---");		
#endif
	                }


	                  input_report_key(data->input_dev, tsp_keycodes[key_id], 1);
	                  tsp_keystatus[key_id] = KEY_PRESS;
			 
                   }
                }   
				
                else if (touch_event == 1) //up event
                {
    
                          _st_finger_infos[id].u2_pressure= 0;
		       	
			 if(key_id !=0x80)  	
			{    
			         i=key_id;
                             printk("\n");
                             printk("\n---virtual key %d release---\n",++i);
			         for(i=0;i<8;i++)
	                          input_report_key(data->input_dev, tsp_keycodes[key_id], 0);
					 
                             key_id=0x80;
			  }
                }
				
                else if (touch_event == 2) //move
                {
                    _st_finger_infos[id].u2_pressure= pressure;
                    _st_finger_infos[id].i2_x= (int16_t)x;
                    _st_finger_infos[id].i2_y= (int16_t)y;
                    _st_finger_infos[id].ui2_id  = size;
                    _si_touch_num ++;
                }
                else        
                    /*bad event, ignore*/
                     continue;  
                


                //if ( (touch_event==0) || (touch_event==2) )
                if ( (touch_event==0) || (touch_event==2) )
				{
#if 1
					for( i= 0; i<CFG_MAX_POINT_NUM; ++i )
					{
#ifdef DEBUG
						printk("debug : _st_finger_infos[%d].ui2_id = %d\n", i, _st_finger_infos[i].ui2_id);
						printk("debug : _st_finger_infos[%d].u2_pressure = %d\n", i, _st_finger_infos[i].u2_pressure);
						printk("debug : _st_finger_infos[%d].i2_x = %d\n", i, _st_finger_infos[i].i2_x);
						printk("debug : _st_finger_infos[%d].i2_y = %d\n", i, _st_finger_infos[i].i2_y);
#endif

						if ( _st_finger_infos[i].u2_pressure == -1 )
							continue;
						_st_finger_infos[i].u2_pressure = 0;
						global_i |= (1<<i);
						input_mt_slot(data->input_dev, i);
						input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,true);
						//input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, _st_finger_infos[i].ui2_id);
						//input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, _st_finger_infos[i].u2_pressure);
						input_report_abs(data->input_dev, ABS_MT_POSITION_X,  _st_finger_infos[i].i2_x);
						input_report_abs(data->input_dev, ABS_MT_POSITION_Y,  _st_finger_infos[i].i2_y);
						//input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, _st_finger_infos[i].ui2_id);
						//input_report_key(data->input_dev, BTN_TOUCH, 1);
						//input_mt_sync(data->input_dev);

						if(_st_finger_infos[i].u2_pressure == 0 )
						{
							_st_finger_infos[i].u2_pressure= -1;
						}

					}
#endif
					//input_report_key(data->input_dev, BTN_TOUCH, 1);
					//input_mt_sync(data->input_dev);
				}
                if ( (touch_event==1) )
                {
#ifdef DEBUG
					printk("[TSP]id=%d up line= %d\n",  id, __LINE__);
					printk("debug : ++++++++++++ tp is released.\n");
#endif
					i=id;					
					global_i &= ~(1<<i);
					input_mt_slot(data->input_dev, i);
					input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,false);
					//input_report_key(data->input_dev, BTN_TOUCH, 0);
						//input_mt_sync(data->input_dev);
                }



               /*     input_sync(data->input_dev);*/

                    if (_si_touch_num == 0 )
                    {
                        fts_ts_release();
                    }
                    _si_touch_num = 0;
		     }    
			}			  		
	           }	
    
		i_count ++;
    }while( id != 0xf && i_count < 12);
 
	input_sync(data->input_dev);

	event->touch_point = touch_point_num;        
	if (event->touch_point == 0) 
	        return 1; 

	switch (event->touch_point) {
	    case 5:
	        event->x5           = touch_info[4].i2_x;
	        event->y5           = touch_info[4].i2_y;
	        event->pressure5 = touch_info[4].u2_pressure;
	    case 4:
	        event->x4          = touch_info[3].i2_x;
	        event->y4          = touch_info[3].i2_y;
	        event->pressure4= touch_info[3].u2_pressure;
	    case 3:
	        event->x3          = touch_info[2].i2_x;
	        event->y3          = touch_info[2].i2_y;
	        event->pressure3= touch_info[2].u2_pressure;
	    case 2:
	        event->x2          = touch_info[1].i2_x;
	        event->y2          = touch_info[1].i2_y;
	        event->pressure2= touch_info[1].u2_pressure;
	    case 1:
	        event->x1          = touch_info[0].i2_x;
	        event->y1          = touch_info[0].i2_y;
	        event->pressure1= touch_info[0].u2_pressure;
	        break;
	    default:
	        return -1;
	}
    
    return 0;
}
#else
int fts_read_data(void)
{
	struct FTS_TS_DATA_T *data = i2c_get_clientdata(this_client);
	u8 buf[32] = {0};
	int i,j,id,temp,i_count = 0,ret = -1;
	int touch_event, x, y, pressure, size;

	do {
		buf[0] = 3;
		id = 0xe;

		ret = fts_i2c_rxdata(buf, 30);

		if (ret > 0) {
			for(j=0;j<5;j++) {
				id = buf[j*6+2]>>4;
				touch_event = buf[j*6+0]>>6;

				if (id >= 0 && id < CFG_MAX_POINT_NUM) {
					temp = buf[j*6+0]& 0x0f;
					temp = temp<<8;
					temp = temp | buf[j*6+1];
					x = temp;
#if !defined(CONFIG_SMARTQ_X7)
					if(x >= 300 && x < 800) {
						x -= 10;
					} else if(x >= 800 && x <= 1045) {
						x -= 20;
					}
#endif

					temp = (buf[j*6+2])& 0x0f;
					temp = temp<<8;
					temp = temp | buf[j*6+3];
					y = temp;

					pressure = buf[j*6+4] & 0x3f;
					size = buf[j*6+5]&0xf0;
					size = (id<<8)|size;
					touch_event = buf[j*6+0]>>6;

					if ((touch_event == 0) || (touch_event == 2)) { //press down or move
						_st_finger_infos[id].u2_pressure = pressure;
						_st_finger_infos[id].i2_x = (int16_t)x;
						_st_finger_infos[id].i2_y = (int16_t)y;
						_st_finger_infos[id].ui2_id  = size;
						_si_touch_num ++;

						for(i= 0; i<CFG_MAX_POINT_NUM; ++i) {
							if ( _st_finger_infos[i].u2_pressure == -1 )
								continue;
							_st_finger_infos[i].u2_pressure = 0;
							global_i |= (1<<i);
							input_mt_slot(data->input_dev, i);
							input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,true);
#if defined(CONFIG_SMARTQ_X7)
							input_report_abs(data->input_dev, ABS_MT_POSITION_X, _st_finger_infos[i].i2_y);
							input_report_abs(data->input_dev, ABS_MT_POSITION_Y, 1280 -_st_finger_infos[i].i2_x);
#else
							input_report_abs(data->input_dev, ABS_MT_POSITION_X, _st_finger_infos[i].i2_x);
							input_report_abs(data->input_dev, ABS_MT_POSITION_Y, _st_finger_infos[i].i2_y);
#endif

							if(_st_finger_infos[i].u2_pressure == 0 )
							{
								_st_finger_infos[i].u2_pressure= -1;
							}
						}
					} else if (touch_event == 1) { //press up
						_st_finger_infos[id].u2_pressure= 0;
						global_i &= ~(1<<i);

						input_mt_slot(data->input_dev, id);
						input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,false);
					} else {
						/*bad event, ignore*/
						continue;
					}

					if (_si_touch_num == 0) {
						fts_ts_release();
					}
					_si_touch_num = 0;
				}
			}
		}
		i_count ++;
	} while( id != 0xf && i_count < 12);

	input_sync(data->input_dev);

	return 1;
}
#endif


static void fts_work_func(struct work_struct *work)
{
    fts_read_data();    
}




static irqreturn_t fts_ts_irq(int irq, void *dev_id)
{
    struct FTS_TS_DATA_T *ft5x0x_ts = dev_id;
    if (!work_pending(&ft5x0x_ts->pen_event_work)) {
        queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
    }

    return IRQ_HANDLED;
}



//#define upgrade_tp_firmware
#ifdef upgrade_tp_firmware
/***********************************************************************
[function]: 
                      callback:         send a command to ctpm.
[parameters]:
			  btcmd[in]:       command code;
			  btPara1[in]:     parameter 1;    
			  btPara2[in]:     parameter 2;    
			  btPara3[in]:     parameter 3;    
                      num[in]:         the valid input parameter numbers, 
                                           if only command code needed and no 
                                           parameters followed,then the num is 1;    
[return]:
			  FTS_TRUE:      success;
			  FTS_FALSE:     io fail;
************************************************************************/
static bool cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
    u8 write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(write_cmd, num);
}




/***********************************************************************
[function]: 
                      callback:         write a byte data  to ctpm;
[parameters]:
			  buffer[in]:       write buffer;
			  length[in]:      the size of write data;    
[return]:
			  FTS_TRUE:      success;
			  FTS_FALSE:     io fail;
************************************************************************/
static bool byte_write(u8* buffer, int length)
{
    
    return i2c_write_interface(buffer, length);
}




/***********************************************************************
[function]: 
                      callback:         read a byte data  from ctpm;
[parameters]:
			  buffer[in]:       read buffer;
			  length[in]:      the size of read data;    
[return]:
			  FTS_TRUE:      success;
			  FTS_FALSE:     io fail;
************************************************************************/
static bool byte_read(u8* buffer, int length)
{
    return i2c_read_interface(buffer, length);
}





#define    FTS_PACKET_LENGTH        128

static unsigned char CTPM_FW[]=
{
#include "ft_app.i"
};



/***********************************************************************
[function]: 
                        callback:          burn the FW to ctpm.
[parameters]:
			    pbt_buf[in]:     point to Head+FW ;
			    dw_lenth[in]:   the length of the FW + 6(the Head length);    
[return]:
			    ERR_OK:          no error;
			    ERR_MODE:      fail to switch to UPDATE mode;
			    ERR_READID:   read id fail;
			    ERR_ERASE:     erase chip fail;
			    ERR_STATUS:   status error;
			    ERR_ECC:        ecc error.
************************************************************************/
E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(u8* pbt_buf, int dw_lenth)
{
    u8  cmd,reg_val[2] = {0};
    u8  packet_buf[FTS_PACKET_LENGTH + 6];
    u8  auc_i2c_write_buf[10];
    u8  bt_ecc;
	
    int  j,temp,lenght,i_ret,packet_number, i = 0;
    int  i_is_new_protocol = 0;
	

    /******write 0xaa to register 0xfc******/
    cmd=0xaa;
    fts_register_write(0xfc,&cmd);
    mdelay(50);
	
     /******write 0x55 to register 0xfc******/
    cmd=0x55;
    fts_register_write(0xfc,&cmd);
    printk("[TSP] Step 1: Reset CTPM test\n");
   
    mdelay(10);   


    /*******Step 2:Enter upgrade mode ****/
    printk("\n[TSP] Step 2:enter new update mode\n");
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = fts_i2c_txdata(auc_i2c_write_buf, 2);
        mdelay(5);
    }while(i_ret <= 0 && i < 10 );

    if (i > 1)
    {
        i_is_new_protocol = 1;
    }

    /********Step 3:check READ-ID********/        
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
    {
        printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        return ERR_READID;
        //i_is_new_protocol = 1;
    }
    

     /*********Step 4:erase app**********/
    if (i_is_new_protocol)
    {
        cmd_write(0x61,0x00,0x00,0x00,1);
    }
    else
    {
        cmd_write(0x60,0x00,0x00,0x00,1);
    }
    mdelay(1500);
    printk("[TSP] Step 4: erase. \n");



    /*Step 5:write firmware(FW) to ctpm flash*/
    bt_ecc = 0;
    printk("[TSP] Step 5: start upgrade. \n");
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
        mdelay(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
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
        mdelay(20);
    }

    /***********send the last six byte**********/
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
        mdelay(20);
    }

    /********send the opration head************/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    /*******Step 7: reset the new FW**********/
    cmd_write(0x07,0x00,0x00,0x00,1);

    return ERR_OK;
}




int fts_ctpm_fw_upgrade_with_i_file(void)
{
   u8*     pbt_buf = FTS_NULL;
   int i_ret;
    
   pbt_buf = CTPM_FW;
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
   
   return i_ret;
}

unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
	
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
        return 0xff; 
 
}

#endif

static int fts_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    //omap4_commix_vaux2_power(1);
    struct FTS_TS_DATA_T *ft5x0x_ts;
    struct input_dev *input_dev;
    int err = 0;
    unsigned char reg_value=0;
    unsigned char reg_version=0;
    int i;

#if 1
	struct ft5x0x_platform_data *pdata = client->dev.platform_data;
	global_pdata = pdata;
	_sui_irq_num = pdata->irq;
	global_pdata->rst();
#else
    _sui_irq_num = 6;/*IRQ_EINT(6);*/
#endif

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        err = -ENODEV;
        goto exit_check_functionality_failed;
    }

    ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
    if (!ft5x0x_ts)    {
        err = -ENOMEM;
        goto exit_alloc_data_failed;
    }

    this_client = client;
    i2c_set_clientdata(client, ft5x0x_ts);

    INIT_WORK(&ft5x0x_ts->pen_event_work, fts_work_func);

    ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
    if (!ft5x0x_ts->ts_workqueue) {
        err = -ESRCH;
        goto exit_create_singlethread;
    }

	
   /***wait CTP to bootup normally***/
    msleep_interruptible(200);
    
    fts_register_read(FT5X0X_REG_FIRMID, &reg_version,1);
    printk("[TSP] firmware version = 0x%2x\n", reg_version);
    fts_register_read(FT5X0X_REG_REPORT_RATE, &reg_value,1);
    printk("[TSP]firmware report rate = %dHz\n", reg_value*10);
    fts_register_read(FT5X0X_REG_THRES, &reg_value,1);
    printk("[TSP]firmware threshold = %d\n", reg_value * 4);
    fts_register_read(FT5X0X_REG_NOISE_MODE, &reg_value,1);
    printk("[TSP]nosie mode = 0x%2x\n", reg_value);

#ifdef upgrade_tp_firmware
    if (1 || fts_ctpm_get_upg_ver() != reg_version)  //upgrade touch screen firmware if not the same with the firmware in host flashs
    {
        printk("[TSP] start upgrade new verison 0x%2x\n", fts_ctpm_get_upg_ver());
        msleep(200);
        err =  fts_ctpm_fw_upgrade_with_i_file();
        if (err == 0)
        {
            printk("[TSP] ugrade successfuly.\n");
            msleep(300);
            //fts_read_reg(FT5X0X_REG_FIRMID, &reg_value);
            fts_register_read(FT5X0X_REG_FIRMID, &reg_value, 1);
            printk("FTS_DBG from old version 0x%2x to new version = 0x%2x\n", reg_version, reg_value);
        }
        else
        {
            printk("[TSP]  ugrade fail err=%d, line = %d.\n",
                err, __LINE__);
        }
        msleep(4000);
    }
#endif
    err = request_irq(_sui_irq_num, fts_ts_irq, IRQF_TRIGGER_FALLING, "qt602240_ts", ft5x0x_ts);

    if (err < 0) {
        dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
        goto exit_irq_request_failed;
    }
    disable_irq(_sui_irq_num);

    input_dev = input_allocate_device();
    if (!input_dev) {
        err = -ENOMEM;
        dev_err(&client->dev, "failed to allocate input device\n");
        goto exit_input_dev_alloc_failed;
    }
    
    ft5x0x_ts->input_dev = input_dev;

    /***setup coordinate area******/
    set_bit(EV_ABS, input_dev->evbit);
    set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
    set_bit(ABS_MT_POSITION_X, input_dev->absbit);
    set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
//    set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	
    /****** for multi-touch *******/
    for (i=0; i<CFG_MAX_POINT_NUM; i++)   
        _st_finger_infos[i].u2_pressure = -1;
	input_mt_init_slots(input_dev, 5);
    input_set_abs_params(input_dev,
                 ABS_MT_POSITION_X, 0, SCREEN_MAX_X - 1, 0, 0);
    input_set_abs_params(input_dev,
                 ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y - 1, 0, 0);
//    input_set_abs_params(input_dev,
  //               ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
    //input_set_abs_params(input_dev,
    //            ABS_MT_WIDTH_MAJOR, 0, 30, 0, 0);
   // input_set_abs_params(input_dev,
     //            ABS_MT_TRACKING_ID, 0, 30, 0, 0);
    /*****setup key code area******/
    set_bit(EV_SYN, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);
//    set_bit(BTN_TOUCH, input_dev->keybit);
    input_dev->keycode = tsp_keycodes;
    for(i = 0; i < CFG_NUMOFKEYS; i++)
    {
        input_set_capability(input_dev, EV_KEY, ((int*)input_dev->keycode)[i]);
        tsp_keystatus[i] = KEY_RELEASE;
    }

    input_dev->name        = FT5X0X_NAME;
    err = input_register_device(input_dev);
    if (err) {
        dev_err(&client->dev,
        "fts_ts_probe: failed to register input device: %s\n",
        dev_name(&client->dev));
        goto exit_input_register_device_failed;
    }


    enable_irq(_sui_irq_num);    

#ifdef CONFIG_HAS_EARLYSUSPEND
	pdata->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	pdata->early_suspend.suspend = fts_ts_early_suspend;
	pdata->early_suspend.resume = fts_ts_late_resume;
	register_early_suspend(&pdata->early_suspend);
#endif

    printk("[TSP] file(%s), function (%s), line =%d --------------------------------------------- end\n", __FILE__, __func__, __LINE__);
    return 0;

exit_input_register_device_failed:
    input_free_device(input_dev);
exit_input_dev_alloc_failed:
    free_irq(_sui_irq_num, ft5x0x_ts);
exit_irq_request_failed:
    cancel_work_sync(&ft5x0x_ts->pen_event_work);
    destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
    printk("[TSP] ==singlethread error =\n");
    i2c_set_clientdata(client, NULL);
    kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
    return err;
}



static int __devexit fts_ts_remove(struct i2c_client *client)
{
    struct FTS_TS_DATA_T *ft5x0x_ts;
    
    ft5x0x_ts = (struct FTS_TS_DATA_T *)i2c_get_clientdata(client);
    free_irq(_sui_irq_num, ft5x0x_ts);
    input_unregister_device(ft5x0x_ts->input_dev);
    kfree(ft5x0x_ts);
    cancel_work_sync(&ft5x0x_ts->pen_event_work);
    destroy_workqueue(ft5x0x_ts->ts_workqueue);
    i2c_set_clientdata(client, NULL);
    return 0;
}

static int fts_ts_suspend(struct device *dev)
{
	printk("debug : fts_ts_suspend\n");
	return 0;
}

static int fts_ts_resume(struct device *dev)
{
	printk("debug : fts_ts_resume\n");
	return 0; 
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void omap4_commix_vaux2_power(int action)
{
	static struct regulator *omap4_commix_vaux2_reg = NULL;

	if (IS_ERR_OR_NULL(omap4_commix_vaux2_reg)) {
		omap4_commix_vaux2_reg = regulator_get(NULL, "s7-tp-power");
		if (IS_ERR_OR_NULL(omap4_commix_vaux2_reg)) {
			pr_err("Can't get s7-tp-power for system!\n");
			return;
		}
	}

	if (action < 0) {
		regulator_put(omap4_commix_vaux2_reg);
		omap4_commix_vaux2_reg = NULL;
	} else if (0 < action)
		regulator_enable(omap4_commix_vaux2_reg);
	else regulator_disable(omap4_commix_vaux2_reg);
}

static void fts_ts_early_suspend(struct early_suspend *h)
{
#if 1
	u8 cmd=0x03;
	fts_register_write(0xA5,&cmd);
#else
	omap4_commix_vaux2_power(0);
#endif
}

#include <linux/gpio.h>
static void fts_ts_late_resume(struct early_suspend *h)
{
	struct FTS_TS_DATA_T *data = i2c_get_clientdata(this_client);
	int i=0;
	printk("global_i=%x\n",global_i);
	for (i=0;i<5;i++) {
		if(global_i & (1<<i)) {
			input_mt_slot(data->input_dev, i);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,false);
		}
	}
	global_i=0;

#if 1
	global_pdata->rst();
#else
	omap4_commix_vaux2_power(1);
#endif
}
#endif

static const struct i2c_device_id ft5x0x_ts_id[] = {
    { FT5X0X_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);


static SIMPLE_DEV_PM_OPS(fts_ts_pm_ops,
			 fts_ts_suspend, fts_ts_resume);


static struct i2c_driver fts_ts_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.pm	= &fts_ts_pm_ops,
	},
    .probe        = fts_ts_probe,
    .remove        = __devexit_p(fts_ts_remove),
    .id_table    = ft5x0x_ts_id,
    .driver    = {
        .name = FT5X0X_NAME,
    },
};

static int __init fts_ts_init(void)
{
    return i2c_add_driver(&fts_ts_driver);
}


static void __exit fts_ts_exit(void)
{
    i2c_del_driver(&fts_ts_driver);
}



module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("<duxx@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");

