/*
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <linux/power/hhcn_charger.h>
#include "oz8806.h"

/*-------------------------------------------------------------------------*/
struct struct_batt_data batt_info = {
	0, ((20*107)/100), 5, 391, 250, 40, 6800, 345, 0, 0, 0, 0, 0, 0, 0, 68*10, 4140, 0, 5, 0, 100, 100, 1400, 0, 0, 3775, 100
};
EXPORT_SYMBOL(batt_info);

// Addresses to scan, in 7-bit presentation
//I2C_CLIENT_INSMOD_1(OZ8806);

static int FullCount = 0;

struct OZ8806_data {
	struct i2c_client	*myclient;
	struct mutex		update_lock;

	u32					valid;
	unsigned long		last_updated;
	u8					control;
	u16					aout16;
};

static struct OZ8806_data *the_OZ8806;

int		sECxMAH[4]		= {0,0,0,0};		//tables of EOC adjustment point
int		fStateDtm		= 10;				//current below this value, state => idle
u8		yOZState, yPrevOZState;
int		iVoltAdjHigh	= 3800;				//mV
int		iEODLowVolt		= 3500;				//mV, low voltage of battery, base on battery spec
bool	bHadFullEOD		= false;			//Record EOD process

/*-------------------------------------------------------------------------*/

//
//declaration of temperature table
//
#define NPoints		31
struct RTPoint
{
	int			oC;
	int			Ohm;
	int			mV;
};
//value of 1st point depends on HW connection, demo board using 120Kohm+1.8V, oC can be any value
struct RTPoint			RTtable[NPoints] = {
				{ 1800, 120000, 1800},
				{ -20, 69204, 0},	{ -15, 54464, 0},	{ -10, 43139, 0},
				{  -5, 34386, 0},	{   0, 27580, 0},	{   5, 22257, 0},
				{  10, 18067, 0},	{  15, 14752, 0},	{  20, 12112, 0},
				{  25, 10000, 0},	{  30,  8300, 0},	{  35,  6924, 0},
				{  40,  5806, 0},	{  45,  4891, 0},	{  50,  4140, 0},
				{  55,  3521, 0},	{  60,  3007, 0},	{  65,  2578, 0},
				{  70,  2221, 0},	{  75,  1920, 0},	{  80,  1666, 0},
				{  85,  1451, 0},	{  90,  1269, 0},	{  95,  1113, 0},
				{ 100,   980, 0},	{ 105,   865, 0},	{ 110,   766, 0},
				{ 115,   681, 0},	{ 120,   606, 0},	{ 125,   542, 0}};


#define	OCVNPoints	21
struct SOCVPoint
{
	int			iVoltage;
	int			iRSOC;				//this value save in xx%, really value * 100
};

//To Customer: OCVtable[] is used when OS goes into Suspend and OZ8806 ever did SleepOCV detection
//When OS resume from Suspend Mode, battery driver checks OZ8806 SleepOCV value, and use SleepOCV voltage to re-initialize battery capacity.
struct SOCVPoint	OCVtable[OCVNPoints] = {
				{3000, 00},	{3400, 05},	{3475, 10},	{3518, 15},	{3557, 20},
				{3590, 25},	{3616, 30},	{3632, 35},	{3647, 40},	{3660, 45},
				{3690, 50},	{3721, 55},	{3760, 60},	{3800, 65},	{3838, 70},
				{3878, 75},	{3935, 80},	{3990, 85},	{4035, 90},	{4100, 95},
				{4160, 100}};
/*
				{3000, 00},	{3400, 05},	{3475, 10},	{3518, 15},	{3557, 20},
				{3587, 25},	{3608, 30},	{3624, 35},	{3638, 40},	{3655, 45},
				{3675, 50},	{3705, 55},	{3743, 60},	{3786, 65},	{3832, 70},
				{3882, 75},	{3933, 80},	{3990, 85},	{4050, 90},	{4114, 95},
				{4160, 100}};
*/
// Loading table for initialization of battery
//declaration of RC table
struct SOCVPoint	PoOntable[OCVNPoints] = {
				{2930, 00},	{3330, 05},	{3405, 10},	{3450, 15},	{3490, 20},
				{3517, 25},	{3540, 30},	{3554, 35},	{3568, 40},	{3585, 45},
				{3605, 50},	{3634, 55},	{3673, 60},	{3716, 65},	{3762, 70},
				{3812, 75},	{3863, 80},	{3920, 85},	{3980, 90},	{4045, 95},
				{4130, 100}};

//
#define ZAxis		8
#define YAxis		3
#define XAxis		25

// RC table X Axis value
int				XAxisElement[XAxis] = { 3000, 3070, 3145, 3205, 3240, 3275, 3305, 3335,
										3360, 3385, 3410, 3435, 3460, 3485, 3510, 3530,
										3555, 3580, 3605, 3625, 3645, 3670, 3700, 3735,
										3775};	//mV
// RC table Y Axis value
int				YAxisElement[YAxis] = {  1000, 2000, 3000};	//10000C (1C=DesignCapacity)
// RC table Z Axis value, in 10*'C format
int			ZAxisElement[ZAxis] = { -75, -25,  50, 150, 250, 350, 450, 550};

// contents of RC table, its unit is 10000C, 1C = DesignCapacity
int				RCtable[YAxis*ZAxis][XAxis]={
{3696,3804,3976,4154,4281,4433,4593,4801,5005,5235,5511,5827,6177,6565,6948,7208,7454,7623,7724,7791,7873,7992,8143,8293,8415},		//-7.5'C
{4230,4423,4728,4979,5166,5411,5660,5937,6203,6495,6813,7145,7441,7663,7813,7893,7970,8066,8199,8322,8453,8620,8816,9024,9211},
{4198,4368,4729,5217,5600,6037,6456,6899,7274,7611,7871,8051,8161,8241,8331,8427,8579,8754,8941,9094,9292,9542,9784,9983,10000},
{2454,2542,2684,2830,2934,3057,3188,3358,3537,3757,4026,4343,4700,5093,5503,5826,6206,6549,6846,7055,7246,7464,7700,7934,8134},		//-2.5'C
{2835,2983,3219,3422,3573,3774,3994,4264,4534,4841,5182,5547,5916,6266,6586,6820,7085,7331,7569,7751,7927,8140,8385,8640,8868},
{2845,2985,3269,3647,3960,4348,4741,5178,5564,5947,6308,6637,6929,7193,7441,7634,7872,8106,8334,8513,8717,8976,9361,9735,9974},
{1338,1408,1522,1640,1723,1821,1925,2060,2217,2428,2691,3008,3372,3769,4204,4584,5083,5584,6056,6392,6682,6988,7303,7611,7881},		//5'C
{1580,1689,1862,2022,2141,2302,2497,2760,3033,3355,3716,4111,4545,5009,5484,5855,6289,6671,7002,7237,7455,7709,7996,8295,8560},
{611,669,766,864,934,1016,1102,1216,1358,1563,1821,2139,2507,2906,3358,3775,4352,4956,5542,5961,6315,6679,7044,7401,7716},	//15'C
{764,846,979,1110,1209,1343,1521,1781,2056,2386,2761,3176,3652,4191,4765,5227,5770,6241,6633,6903,7147,7428,7744,8070,8360},
{837,932,1102,1317,1525,1840,2193,2622,3025,3478,3988,4538,5099,5638,6120,6457,6823,7144,7434,7651,7865,8123,8437,8761,9038},
{295,348,437,527,591,666,744,848,984,1187,1443,1761,2132,2532,2990,3423,4035,4682,5318,5773,6155,6544,6931,7309,7644},	//25'C
{409,479,595,714,804,926,1098,1355,1631,1966,2346,2769,3264,3836,4453,4954,5545,6054,6473,6758,7013,7305,7634,7973,8273},
{493,580,731,918,1108,1411,1757,2184,2590,3054,3590,4178,4786,5371,5893,6255,6643,6979,7279,7504,7719,7976,8278,8593,8872},
{157,209,294,381,442,514,589,689,822,1023,1279,1597,1968,2369,2830,3270,3897,4564,5221,5692,6086,6486,6882,7269,7613},	//35'C
{254,320,428,542,627,745,913,1170,1447,1783,2165,2592,3096,3681,4318,4835,5447,5973,6403,6695,6955,7252,7586,7930,8235},
{344,427,569,744,927,1224,1567,1994,2401,2870,3417,4022,4649,5255,5795,6167,6565,6907,7212,7439,7655,7913,8209,8520,8800},
{98,148,232,317,377,448,521,619,751,952,1208,1525,1897,2298,2761,3204,3837,4512,5179,5657,6056,6460,6861,7252,7600},	//45'C
{187,251,355,467,551,667,833,1090,1366,1703,2087,2516,3022,3614,4259,4783,5404,5938,6373,6667,6929,7229,7565,7912,8218},
{279,360,499,669,848,1143,1485,1911,2319,2791,3342,3954,4590,5205,5752,6129,6531,6876,7183,7411,7627,7885,8180,8488,8768},
{72,122,205,290,349,419,492,589,721,921,1177,1494,1866,2267,2731,3175,3811,4490,5161,5641,6042,6449,6852,7245,7594},	//55'C
{158,221,324,435,518,632,799,1055,1331,1669,2053,2482,2991,3585,4233,4761,5386,5922,6360,6655,6918,7219,7556,7904,8211},
{250,332,468,636,814,1107,1449,1875,2283,2756,3309,3925,4564,5183,5734,6112,6516,6862,7171,7399,7615,7873,8167,8474,8755},
{5422,5463,5581,5847,6146,6486,6862,7181,7522,7836,8103,8332,8510,8686,8843,8954,9056,9145,9223,9292,9354,9379,9420,9459,9470}};
/*-------------------------------------------------------------------------*/

static void OZ8806_TemperatureInit(void)
{
	int				i;
	struct RTPoint			*pRTPoint;

	for (i=1;i<NPoints;i++)
	{
		pRTPoint = &RTtable[i];
		pRTPoint->mV = RTtable[0].oC * pRTPoint->Ohm;
		pRTPoint->mV = pRTPoint->mV / (RTtable[0].Ohm + pRTPoint->Ohm);
	}
}

int OZ8806_TemperaturemV2oC(int mVvalue)
{
	int		i,j;
	int	res;

	for (j=0;j<NPoints;j++)
	{
	   if (RTtable[j].Ohm==0)
			break;
	}
	for (i=NPoints-1; i>0; i--)
	{
		if (mVvalue < RTtable[i].mV) // increase
		{
			break;
		}
	}
	if (i==0)
	{
		res = RTtable[i].oC;
		res = res - (((RTtable[i].mV-mVvalue)*RTtable[i].oC)/RTtable[i].mV);
	}
	else if(i==j-1)
	{
		res = (mVvalue+RTtable[i-1].mV)*(RTtable[i-1].oC)/(RTtable[i-1].mV);
	}
	else if (mVvalue==RTtable[i-1].mV)
	{
		res=RTtable[i-1].oC;
	}
	else
	{
		res=((mVvalue-RTtable[i-1].mV)*(RTtable[i].oC-RTtable[i-1].oC)/(RTtable[i].mV-RTtable[i-1].mV) + RTtable[i-1].oC);
	}

	return res;
}

/*-------------------------------------------------------------------------*/

static int OZ8806_LoadingVoltToRC(void)
{
	int j;
	int res;

	for (j=0;j<OCVNPoints;j++)
	{
		if (PoOntable[j].iVoltage==batt_info.fVolt)
		{
			res = PoOntable[j].iRSOC;
			return res;
		}
		if(PoOntable[j].iVoltage > batt_info.fVolt)
			break;
	}
	if(j == 0)
		res = PoOntable[j].iRSOC;
	else if(j == OCVNPoints)
		res = PoOntable[j-1].iRSOC;
	else
	{
		res = ((batt_info.fVolt-PoOntable[j-1].iVoltage)*
				(PoOntable[j].iRSOC-PoOntable[j-1].iRSOC));
		res = res / (PoOntable[j].iVoltage-PoOntable[j-1].iVoltage);
		res += PoOntable[j-1].iRSOC;
	}

	return res;
}
//EXPORT_SYMBOL(OZ8806_LoadingVoltToRC);

static int OZ8806_PowerOnVoltToRC(void)
{
	int j;
	int res;

	for (j=0;j<OCVNPoints;j++)
	{
		if (OCVtable[j].iVoltage==batt_info.fVolt)
		{
			res = OCVtable[j].iRSOC;
			return res;
		}
		if(OCVtable[j].iVoltage > batt_info.fVolt)
			break;
	}
	if(j == 0)
		res = OCVtable[j].iRSOC;
	else if(j == OCVNPoints)
		res = OCVtable[j-1].iRSOC;
	else
	{
		res = ((batt_info.fVolt-OCVtable[j-1].iVoltage)*
				(OCVtable[j].iRSOC-OCVtable[j-1].iRSOC));
		res = res / (OCVtable[j].iVoltage-OCVtable[j-1].iVoltage);
		res += OCVtable[j-1].iRSOC;
	}

	return res;
}
//EXPORT_SYMBOL(OZ8806_PowerOnVoltToRC);

/*-------------------------------------------------------------------------*/

static u8 OZ8806_control_register(struct i2c_client *client, int op, u8 indata)
{
	struct		OZ8806_data *data = i2c_get_clientdata(client);
	u8			bRet = 0;

	data->control	= ControlStatus;
	data->aout16	= indata;

	if(op == 1)
	{
		bRet =i2c_smbus_write_word_data(client, data->control, data->aout16);
	}
	else
	{
		bRet = i2c_smbus_read_byte_data(client, data->control);
	}

	return bRet;
}
//EXPORT_SYMBOL(OZ8806_control_register);

static void OZ8806_sleep_control(struct i2c_client *client, int sleepEn, int sleepOCV)
{
	u8			data8;
	data8= 0x05;
	if(sleepEn != 0)
	{
		data8 = data8 | CtBitSleepMode;
		if(sleepOCV != 0)
		{
			data8 = data8 | CtBitSleepOCVEn;
		}
	}
	else
	{
		data8 = data8 & ~CtBitSleepMode;
	}
	OZ8806_control_register(client, 1, data8);				//write to control register
}
//EXPORT_SYMBOL(OZ8806_sleep_control);

static void OZ8806_CAR_Write(struct i2c_client *client)
{
	int			tmpVl;
	struct		OZ8806_data *data = i2c_get_clientdata(client);

	tmpVl = (((batt_info.fRC)*batt_info.fRsense)/batt_info.dbCARLSB);		//transfer to CAR

	data->control	= CellCARLow;
	data->aout16	= (u16)tmpVl;
	i2c_smbus_write_word_data(client, data->control, data->aout16);
	batt_info.fRC = ((data->aout16*batt_info.dbCARLSB)/batt_info.fRsense);
	//synchronize with CAR, cause LSB of CAR is 0.25mAhr
															//skip meaningless decimal point
															//if fRC = 10.0125, make it = 10.00
}
//EXPORT_SYMBOL(OZ8806_CAR_Write);

static void OZ8806_CAR_Reset(struct i2c_client *client)
{
	batt_info.fRC = batt_info.fRSOC * batt_info.fFCC / 100;
	OZ8806_CAR_Write(client);
	//Reserved Capacity should not used by OS
	batt_info.fRSOC = ((batt_info.fRC - batt_info.fReserved) * 100) / (batt_info.fFCC - batt_info.fReserved);
	if(batt_info.fRSOC >= 100)		batt_info.fRSOC = 100;
	if(batt_info.fRSOC <= 0)		batt_info.fRSOC = 0.0;
}
//EXPORT_SYMBOL(OZ8806_CAR_Reset);

/*-------------------------------------------------------------------------*/
void OZ8806_init_chip(struct i2c_client *client)
{
	struct OZ8806_data *data = i2c_get_clientdata(client);
	u16 ADCValue;
	u8  bRet = 0, tmpADC;

	// Initialize OZ8806 chip
	// Called when we have found OZ8806.
	data->control = CellOCVLow;
	ADCValue = i2c_smbus_read_word_data(client, data->control);
	if((ADCValue > 0))				//Power On OCV detect
	{
		if(ADCValue & CellOCVSleepMask)
		{
			bRet = 2;
			printk("%s: OVC sleep mode is activated ",__func__);
		}
		else if(ADCValue & CellOCVPoOnMask)
		{
			bRet = 1;
			printk("%s: Power on mode is activated ",__func__);
		}
		else
		{
			bRet = 0;
			printk("%s: In electrical mode is activated ",__func__);
		}
		ADCValue=ADCValue >> 4;									//if no OCV flag, use old value for pretended initialization
		ADCValue=ADCValue & CellOCVMASK;
		batt_info.fOCVVolt = ((int)ADCValue * batt_info.fVoltLSB) / 100;
		printk(" fOCVVolt = %d\n",batt_info.fOCVVolt);
	}
	OZ8806_sleep_control(client, 0, 0);				//wake up OZ8806 into FullPower mode
	data->control = ControlStatus;
	tmpADC = i2c_smbus_read_byte_data(client, data->control);
	printk("%s: Fuel gauge status values = %x\n",__func__,tmpADC);
	if((tmpADC & CtBitChargeON) != 0)
		batt_info.PowerStatus = 1;
	else
		batt_info.PowerStatus = 0;
	data->control = CellVoltLow;
	ADCValue = i2c_smbus_read_word_data(client, data->control);
	if(ADCValue > 0)
	{
		ADCValue=ADCValue >> 4;
		ADCValue=ADCValue & CellVoltMASK;
		batt_info.fVolt = ((int)ADCValue * batt_info.fVoltLSB) / 100;
	}
	else
		batt_info.fVolt = 3900;										//just for case
	if(bRet == 2)
	{
		if(batt_info.fOCVVolt == 0)
		{
			if(batt_info.fVolt == 0)
				batt_info.fOCVVolt = 3900;
			batt_info.fOCVVolt = batt_info.fVolt;
		}
		batt_info.fVolt = batt_info.fOCVVolt;
		batt_info.fPerFromOCV = OZ8806_PowerOnVoltToRC();					//use PoOCV to initial
		batt_info.fRSOC = batt_info.fPerFromOCV;
		OZ8806_CAR_Reset(client);
	}

	else if((bRet == 1) && (batt_info.PowerStatus == 0))					//system goes normal booting
	{
		msleep(2500);
		data->control = CellVoltLow;
		ADCValue = i2c_smbus_read_word_data(client, data->control);
		if(ADCValue > 0)
		{
			ADCValue=ADCValue >> 4;
			ADCValue=ADCValue & CellVoltMASK;
			batt_info.fVolt = ((int)ADCValue * batt_info.fVoltLSB) / 100;
		}
		else
			batt_info.fVolt = 3900;
		batt_info.fVolt += 20;
		batt_info.fPerFromOCV = OZ8806_PowerOnVoltToRC();					//use PoOCV to initial
		batt_info.fRSOC = batt_info.fPerFromOCV;					//use fRSOC from OCV table to initialize CAR
		OZ8806_CAR_Reset(client);
	}
	else if((bRet == 1) && (batt_info.PowerStatus == 1))			//AC-in and boot up system
	{
		if(batt_info.fVolt == 0)
			batt_info.fVolt = 3900;
		batt_info.fPerFromOCV = OZ8806_PowerOnVoltToRC();					//use PoOCV to initial
		batt_info.fRSOC = batt_info.fPerFromOCV;					//use fRSOC from OCV table to initialize CAR
		OZ8806_CAR_Reset(client);
	}
	else
	{
		//batt_info.fPerFromOCV = OZ8806_LoadingVoltToRC();					//use CellVolt to initial
		//batt_info.fRSOC = batt_info.fPerFromOCV;					//use fRSOC from OCV table to initialize CAR
		//printk("--->> fPerFromOCV = %d  fOCVVolt = %d\n",batt_info.fPerFromOCV,batt_info.fOCVVolt);
		//OZ8806_CAR_Reset(client);
	}
	data->control	= BoardOffsetLow;
	data->aout16	= 20;
	bRet =i2c_smbus_write_word_data(client, data->control, data->aout16);
	//initial sCaMAH equals to fRC
	printk("%s: fVolt = %d  fOCVVolt = %d fRSOC = %d\n",__func__,batt_info.fVolt,batt_info.fOCVVolt,OZ8806_PowerOnVoltToRC());
	batt_info.sCaMAH = batt_info.fRC;

}
EXPORT_SYMBOL(OZ8806_init_chip);

bool OZ8806_PollingLoop(struct pltdata_charger *pd)
{
	struct OZ8806_data *data = i2c_get_clientdata(the_OZ8806->myclient);
	u16	ADCValue;
	u16 Value;
	//int	val;

	batt_info.fPrevCurr = batt_info.fCurr;							//saved previous current
	batt_info.fRCPrev = batt_info.fRC;

	mutex_lock(&data->update_lock);
	data->control = CellTempLow;
	ADCValue = i2c_smbus_read_word_data(the_OZ8806->myclient, data->control);
	ADCValue=ADCValue >> 4;
	ADCValue=ADCValue & CellTempMASK;
	batt_info.fCellTemp = ADCValue * batt_info.fVoltLSB;		//fVoltLSB = 250 (2.5 mV)
	batt_info.fCellTemp = OZ8806_TemperaturemV2oC(batt_info.fCellTemp/100);

	data->control = CellVoltLow;
	ADCValue = i2c_smbus_read_word_data(the_OZ8806->myclient, data->control);
	ADCValue=ADCValue >> 4;
	ADCValue=ADCValue & CellTempMASK;
	batt_info.fVolt = (ADCValue * batt_info.fVoltLSB) / 100;	//fVoltLSB = 250 (2.5 mV)

	data->control = CellCurrLow;
	ADCValue = i2c_smbus_read_word_data(the_OZ8806->myclient, data->control);
	ADCValue=ADCValue & CellCurrMASK;
	batt_info.fCurr = (short)ADCValue * batt_info.dbCurrLSB;	//dbCurrLSB = 391 (3.90625 mA)
	batt_info.fCurr = (batt_info.fCurr / batt_info.fRsense) / 100;
	batt_info.fCurr /= 2;

	data->control = CellCARLow;
	ADCValue = i2c_smbus_read_word_data(the_OZ8806->myclient, data->control);
	ADCValue=ADCValue & CellCARMASK;
	Value = ADCValue;
	if((short)Value < 0)
	{
		int OverCount = 10;

		printk("%s: CAR abnormal overflow = %d",__func__,(int)Value);
		while(OverCount)
		{
			OverCount--;
			msleep(200);
			ADCValue = i2c_smbus_read_word_data(the_OZ8806->myclient, data->control);
			ADCValue=ADCValue & CellCARMASK;
			Value = ADCValue;
			if((short)Value > 0)
			{
				batt_info.fRC = batt_info.fRCPrev;
				printk("%s: CAR value of hard to read finally! fRC = %d Count = %d\n",__func__,batt_info.fRC,OverCount);
				break;
			}
		}
	}
	else
	{
		batt_info.fRC = (short)(ADCValue) * batt_info.dbCARLSB;
		batt_info.fRC = batt_info.fRC / batt_info.fRsense;
	}
	//val = (batt_info.fRC - 0) * 100;
	//val = val / (batt_info.fFCC - 0);
	batt_info.fRSOC = (batt_info.fRC - batt_info.fReserved) * 100;
	batt_info.fRSOC = batt_info.fRSOC / (batt_info.fFCC - batt_info.fReserved);
	if(batt_info.fRSOC >= 100)
        batt_info.fRSOC = 100;
	if(batt_info.fRSOC <= 0)
        batt_info.fRSOC = 0;
	if((batt_info.fCurr < batt_info.fEOC)
		&& (batt_info.fVolt >= batt_info.fVoltFCHG))
	{
        //EOC condition, this may be different during each project
		if(((batt_info.fPrevCurr < batt_info.fCurr+2) && (batt_info.fCurr >= 0)) || \
		   ((pd->charger_sts()) && (batt_info.fRSOC >= 99) && (batt_info.fCurr <= 0)))
		{
			if(FullCount >= 5)
			{
				batt_info.fRSOC = 101;
				OZ8806_CAR_Reset(the_OZ8806->myclient);
				FullCount = 0;
			}
			else
				++FullCount;
		}
	}
//printk("--->> nr = %d ns = %d nc = %d nu = %d nv = %d\n",val,batt_info.fRSOC,batt_info.fRC,batt_info.fCurr,batt_info.fVolt);
	mutex_unlock(&data->update_lock);

	return true;
}
EXPORT_SYMBOL(OZ8806_PollingLoop);

/*-------------------------------------------------------------------------*/

void OZ8806_EOCXSet(void)
{
	int i;

	for(i=4;i>0;i--)
	{
		if(batt_info.fCurr > (i+1)*batt_info.fEOC)
		{
			sECxMAH[i-1] = -1;
		}
		else
		{
			if(sECxMAH[i-1] == -1)
			{
				sECxMAH[i-1] = batt_info.fRC;
			}
		}
	}
}

void OZ8806_EOCBlend(void)
{
	int		l=0,h=3;
	int		fTemp;

	while(l<=3)
	{
		if(sECxMAH[l] > 0)
			break;
		l++;
	}

	while(h>=0)
	{
		if(sECxMAH[h] > 0)
			break;
		h--;
	}

	if(h>=0)
	{
		int fraction;

		fraction = ((batt_info.fCurr - batt_info.fEOC) * 100) / (batt_info.fEOC * (h+1));
		fTemp = fraction * (batt_info.fRC) + ((100-fraction)*batt_info.sCfMAH);
		if(fTemp > 100*(batt_info.sCaMAH + 2*batt_info.fRCDelta))					//prevent fRSOC jump
		{
			batt_info.sCaMAH += 2*batt_info.fRCDelta;
		}
		else
		{
			batt_info.sCaMAH = fTemp / 100;
		}
	}
}

void OZ8806_ProcessEndCharging(void)
{
	//sCrMAH = sCrpMAH;
	if(yOZState  & STATEFULLEOC)
	{
		if(bHadFullEOD)														//do Fully EOD first then do Fully EOC
		{
			batt_info.fFCC		= batt_info.fRC;
			batt_info.sCfMAH	= batt_info.fFCC;							//learning cycle, update sCfMAH
			batt_info.sCaMAH	= batt_info.sCfMAH;
		}
		else
		{
			batt_info.fRC		= batt_info.sCaMAH;
			OZ8806_CAR_Write(the_OZ8806->myclient);										//synchronize CAR
			batt_info.sCaMAH	= batt_info.fRC;							//LSB of CAR is 0.25mAhr, skip meaningless decimal point
		}
	}
	else if (yOZState & STATEEOC)											//not Fully EOC
	{
		if(batt_info.sCaMAH != (batt_info.fRC))								//had done EOC blending, but not Fully EOC
		{
			batt_info.fRC		= batt_info.sCaMAH;
			OZ8806_CAR_Write(the_OZ8806->myclient);
			batt_info.sCaMAH	= batt_info.fRC;							//LSB of CAR is 0.25mAhr, skip meaningless decimal point
		}
	}
	bHadFullEOD = false;
}

void OZ8806_ProcessEndDischarging(void)
{
	//int		RC1;

	if(yOZState & STATEFULLEOD)
	{
		batt_info.sCrMAH		= batt_info.fRC;
		batt_info.sCaMAH		= batt_info.sCrMAH;
		bHadFullEOD = true;
	}
	else if(yOZState & STATEEOD)
	{
		if(batt_info.fRSOC < 98)
		{
			batt_info.sCrMAH	= (batt_info.fRC*100 - batt_info.sCfMAH * batt_info.fRSOC) / (100-batt_info.fRSOC);
			batt_info.sCrMAH	/= 100;
		}
		batt_info.sCaMAH		= batt_info.fRC;
		bHadFullEOD = false;
	}
}


int OZ8806_ma2c10k(int infCurr)
{
	return (int)((infCurr*10000)/batt_info.fFCC);
}


int OZ8806_c10k2mah(int capIn10kC)
{
	return (int)((capIn10kC*batt_info.fFCC) / 10000);
}


bool OZ8806_LookUpRCTable(int infTemp, int infCurr, int infVolt, int *infCal)
{
	bool	bRet = true;
	int		indexX, indexY, indexZ;
	long	fRCtemp1, fRCtemp2, fRCInter1, fRCInter2, flongCal;

	for(indexX=1;indexX<XAxis;indexX++)
	{
		if((XAxisElement[indexX-1] <= infVolt) && (XAxisElement[indexX] > infVolt))
		{
			break;
		}
	}
	for(indexY=0;indexY<YAxis;indexY++)
	{
		if(YAxisElement[indexY] >= infCurr)
		{
			break;
		}
	}
	for(indexZ=0;indexZ<ZAxis;indexZ++)
	{
		if(ZAxisElement[indexZ] >= infTemp)
		{
			break;
		}
	}

	if((indexY != 0) && (indexZ !=0) && (indexY != YAxis) && (indexZ != ZAxis))
	{
		fRCtemp1 = (long)(RCtable[(indexY-1)+(indexZ-1)*YAxis][indexX-1])*100;
		fRCtemp1 +=((long)(infVolt - XAxisElement[indexX-1])*100 /
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY-1)+(indexZ-1)*YAxis][indexX] - RCtable[(indexY-1)+(indexZ-1)*YAxis][indexX-1]);
		fRCtemp2 = (long)(RCtable[(indexY)+(indexZ-1)*YAxis][indexX-1])*100;
		fRCtemp2 +=((long)(infVolt - XAxisElement[indexX-1])*100 /
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ-1)*YAxis][indexX] - RCtable[(indexY)+(indexZ-1)*YAxis][indexX-1]);
		fRCInter1 = fRCtemp1;
		fRCInter1 +=(long)((infCurr - YAxisElement[indexY-1]) /
					(long)(YAxisElement[indexY] - YAxisElement[indexY-1])) *
					(fRCtemp2 - fRCtemp1);

		fRCtemp1 = (long)(RCtable[(indexY-1)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp1 +=((long)(infVolt - XAxisElement[indexX-1])*100 /
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY-1)+(indexZ)*YAxis][indexX] - RCtable[(indexY-1)+(indexZ)*YAxis][indexX-1]);
		fRCtemp2 = (long)(RCtable[(indexY)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp2 +=((long)(infVolt - XAxisElement[indexX-1])*100 /
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ)*YAxis][indexX] - RCtable[(indexY)+(indexZ)*YAxis][indexX-1]);
		fRCInter2 = fRCtemp1;
		fRCInter2 +=(long)((infCurr - YAxisElement[indexY-1]) /
					(long)(YAxisElement[indexY] - YAxisElement[indexY-1])) *
					(fRCtemp2 - fRCtemp1);
		flongCal = fRCInter1;
		flongCal +=((long)(infTemp - ZAxisElement[indexZ-1]) /
				(long)(ZAxisElement[indexZ] - ZAxisElement[indexZ-1])) *
				(fRCInter2 - fRCInter1);
		*infCal = (int)(flongCal/100);
	}
	else if(indexY == 0)						//current is too low, but no matter temperature is below
	{
		fRCInter1 = (long)OZ8806_PowerOnVoltToRC()*100;
		//fRCInter1 *= 10000;
		fRCInter1 *= 100;
		fRCtemp1 = (long)(RCtable[(indexY)+(indexZ-1)*YAxis][indexX-1])*100;
		fRCtemp1 +=((long)(infVolt - XAxisElement[indexX-1])*100 /
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ-1)*YAxis][indexX] - RCtable[(indexY)+(indexZ-1)*YAxis][indexX-1]);
		fRCtemp2 = (long)(RCtable[(indexY)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp2 +=((long)(infVolt - XAxisElement[indexX-1])*100 /
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ)*YAxis][indexX] - RCtable[(indexY)+(indexZ)*YAxis][indexX-1]);
		fRCInter2 = fRCtemp1;
		fRCInter2 +=(long)((infTemp - ZAxisElement[indexZ-1]) /
					(long)(ZAxisElement[indexZ] - ZAxisElement[indexZ-1])) *
					(fRCtemp2 - fRCtemp1);
		flongCal = fRCInter1;
		flongCal +=((long)(infCurr - batt_info.fDisCOCV) /
				(long)(YAxisElement[indexY] - batt_info.fDisCOCV)) *
				(fRCInter2 - fRCInter1);
		*infCal = (int)(flongCal/100);
	}
	else if((indexZ == 0) || (indexZ == ZAxis))
	{
		if(indexZ >= ZAxis)			indexZ = ZAxis - 1;
		fRCtemp1 = (long)(RCtable[(indexY-1)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp1 +=((long)(infVolt - XAxisElement[indexX-1])*100 /
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY-1)+(indexZ)*YAxis][indexX] - RCtable[(indexY-1)+(indexZ)*YAxis][indexX-1]);
		fRCtemp2 = (long)(RCtable[(indexY)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp2 +=((long)(infVolt - XAxisElement[indexX-1]) /
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ)*YAxis][indexX] - RCtable[(indexY)+(indexZ)*YAxis][indexX-1]);
		fRCInter1 = fRCtemp1;
		fRCInter1 +=(long)((infCurr - YAxisElement[indexY-1]) /
					(long)(YAxisElement[indexY] - YAxisElement[indexY-1])) *
					(fRCtemp2 - fRCtemp1);
		*infCal = (int)(fRCInter1/100);
	}
	else if(indexY == YAxis)
	{
		indexY--;
		fRCtemp1 = (long)(RCtable[(indexY-1)+(indexZ-1)*YAxis][indexX-1])*100;
		fRCtemp1 +=((long)(infVolt - XAxisElement[indexX-1])*100 /
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY-1)+(indexZ-1)*YAxis][indexX] - RCtable[(indexY-1)+(indexZ-1)*YAxis][indexX-1]);
		fRCtemp2 = (long)(RCtable[(indexY)+(indexZ-1)*YAxis][indexX-1])*100;
		fRCtemp2 +=((long)(infVolt - XAxisElement[indexX-1]) /
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ-1)*YAxis][indexX] - RCtable[(indexY)+(indexZ-1)*YAxis][indexX-1]);
		fRCInter1 = fRCtemp1;
		fRCInter1 +=(long)((infCurr - YAxisElement[indexY-1]) /
					(long)(YAxisElement[indexY] - YAxisElement[indexY-1])) *
					(fRCtemp2 - fRCtemp1);

		fRCtemp1 = (long)(RCtable[(indexY-1)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp1 +=((long)(infVolt - XAxisElement[indexX-1])*100 /
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY-1)+(indexZ)*YAxis][indexX] - RCtable[(indexY-1)+(indexZ)*YAxis][indexX-1]);
		fRCtemp2 = (long)(RCtable[(indexY)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp2 +=((long)(infVolt - XAxisElement[indexX-1])*100 /
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ)*YAxis][indexX] - RCtable[(indexY)+(indexZ)*YAxis][indexX-1]);
		fRCInter2 = fRCtemp1;
		fRCInter2 +=(long)((infCurr - YAxisElement[indexY-1]) /
					(long)(YAxisElement[indexY] - YAxisElement[indexY-1])) *
					(fRCtemp2 - fRCtemp1);
		flongCal = fRCInter1;
		flongCal +=((long)(infTemp - ZAxisElement[indexZ-1]) /
				(long)(ZAxisElement[indexZ] - ZAxisElement[indexZ-1])) *
				(fRCInter2 - fRCInter1);
		*infCal = (int)(flongCal/100);
	}




	return bRet;
}

bool OZ8806_GaugeAdjustion(void)
{
	struct OZ8806_data *data = i2c_get_clientdata(the_OZ8806->myclient);

	mutex_lock(&data->update_lock);

	if(batt_info.fCurr > fStateDtm)											//charging
	{
		yPrevOZState = yOZState;
		yOZState = STATECHARGING;
		if(yPrevOZState == STATEDISCHARGING)								//end discharging then charge immediately
		{
			yOZState |= STATEEOD;
			OZ8806_ProcessEndDischarging();									//calculate EOD first, then do CHG process
			yOZState &= (~STATEEOD);
		}
		OZ8806_EOCXSet();
		batt_info.sCaMAH += batt_info.fRCDelta;
		if((batt_info.fVolt >= VOLTCVMODE) && (batt_info.fCurr >= batt_info.fEOC))
		{
			OZ8806_EOCBlend();
		}
		if((batt_info.fCurr < batt_info.fEOC) && (batt_info.fVolt >= batt_info.fVoltFCHG))						//EOC condition, this may be different during each project
		{		//EOC condition, this may be different during each project
			if((batt_info.fPrevCurr < batt_info.fCurr+2) && (batt_info.fCurr >= 0))
			{
				if(!(yOZState & STATEFULLEOC))
				{
					yOZState |= STATEFULLEOC;
					OZ8806_ProcessEndCharging();
//					yOZState = STATEIDLE;
				}
			}
		}
		batt_info.fRSOC = (batt_info.sCaMAH - batt_info.sCrMAH) / (batt_info.sCfMAH - batt_info.sCrMAH);		//get RSOC
		if(batt_info.fRSOC >= 100)		batt_info.fRSOC = 100;
		if(batt_info.fRSOC <= 0)		batt_info.fRSOC = 0;
	}
	else if(batt_info.fCurr < (-fStateDtm))									//discharging
	{
		yPrevOZState = yOZState;
		yOZState = STATEDISCHARGING;
		if(yPrevOZState == STATECHARGING)									//end charging then discharge immediately
		{
			yOZState |= STATEEOC;
			OZ8806_ProcessEndCharging();									//calculate EOC first then do DSG process
			yOZState &= (~STATEEOC);
		}
		if(batt_info.fVolt > iVoltAdjHigh)									//voltage higher than adjust
		{
			batt_info.sCaMAH += batt_info.fRCDelta;							//do coulomb counting
			//sCeodMAH = sCaMAH;
			if(batt_info.sCaMAH > batt_info.sCfMAH+50)
			{
				batt_info.sCaMAH = batt_info.sCfMAH - 1;					//set to 99%
				batt_info.fRCPrev = batt_info.fRC = batt_info.sCaMAH;		//synchronize variables
				OZ8806_CAR_Write(the_OZ8806->myclient);											//write to CAR register
			}
		}
		else
		{
			if(batt_info.fVolt > iEODLowVolt)								//higher than Low Volt
			{																//calculate sCa by RC table reference
				int		RC1, RC2;
				OZ8806_LookUpRCTable(batt_info.fCellTemp*10,
									-1*(OZ8806_ma2c10k(batt_info.fCurr)),
									batt_info.fVolt, &RC1);
				OZ8806_LookUpRCTable(batt_info.fCellTemp*10,
									-1*(OZ8806_ma2c10k(batt_info.fCurr)),
									batt_info.fVoltAdjLow, &RC2);
				batt_info.sCeodMAH = batt_info.sCrMAH + OZ8806_c10k2mah(RC1 - RC2);
				if(batt_info.sCaMAH > batt_info.sCeodMAH)					//if greater than table calculated value, faster to close it
				{
					batt_info.sCaMAH += (int)((150*batt_info.fRCDelta)/100);
				}
				else
				{
					batt_info.sCaMAH += batt_info.fRCDelta;
				}
				//batt_info.sCaMAH += batt_info.fRCDelta;
			}
			else															//under voltage stage
			{
				if(!(yOZState & STATEFULLEOD))
				{
					yOZState |= STATEFULLEOD;
					OZ8806_ProcessEndDischarging();
//					yOZState = STATEIDLE;
				}
			}
		}
		//get RSOC
		batt_info.fRSOC = (batt_info.sCaMAH - batt_info.sCrMAH) / (batt_info.sCfMAH - batt_info.sCrMAH);
		if(batt_info.fRSOC >= 100)		batt_info.fRSOC = 100;
		if(batt_info.fRSOC <= 1)		batt_info.fRSOC = 0;				//below 1%=> fully EndOfDischarge
		{
			if(yOZState & STATEFULLEOD)
			{
				yOZState |= STATEFULLEOD;
				OZ8806_ProcessEndDischarging();
//				yOZState = STATEIDLE;
			}
		}
	}
	else																	//idle
	{
		yPrevOZState = yOZState;
		yOZState = STATEIDLE;
		//get RSOC
		if(yPrevOZState & STATECHARGING)									//going idle from CHG,
		{
			yOZState |= STATEEOC;
			OZ8806_ProcessEndCharging();
			yOZState &= (~STATEEOC);
		}
		else if(yPrevOZState & STATEDISCHARGING)							//going idle from DSG
		{
			yOZState |= STATEEOD;
			OZ8806_ProcessEndDischarging();
			yOZState &= (~STATEEOD);
		}
		else
		{
			batt_info.sCaMAH = batt_info.fRC;								//synchronize sCaMAH with coulomb counting
		}
		batt_info.fRSOC = (batt_info.sCaMAH - batt_info.sCrMAH) / (batt_info.sCfMAH - batt_info.sCrMAH);
		if(batt_info.fRSOC >= 100)		batt_info.fRSOC = 100;
		if(batt_info.fRSOC <= 0)		batt_info.fRSOC = 0;
	}

	mutex_unlock(&data->update_lock);

	return true;
}
EXPORT_SYMBOL(OZ8806_GaugeAdjustion);

/*-------------------------------------------------------------------------*/

static int OZ8806_detect(struct i2c_client *client,struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA
				     | I2C_FUNC_SMBUS_READ_BYTE))
		return -ENODEV;

	strlcpy(info->type, MYDRIVER, I2C_NAME_SIZE);

	return 0;
}

static int OZ8806_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct OZ8806_data *data;

	if (!(data = kzalloc(sizeof(struct OZ8806_data), GFP_KERNEL)))
		return -ENOMEM;

	//Note that mainboard definition file, ex: arch/arm/mach-msm/board-xxx.c, must has declared
	// static struct i2c_board_info xxx_i2c_devs[] __initdata = {....}
	// and it must add including this "I2C_BOARD_INFO("OZ8806", 0x2F),"
	// otherwise, probe will occur error
	// string is matching with definition in OZ8806_id id table

	// Init real i2c_client
	i2c_set_clientdata(client, data);

	the_OZ8806 = data;
	data->myclient = client;

	mutex_init(&data->update_lock);

	// Init OZ8806 chip
	mutex_lock(&data->update_lock);
	OZ8806_init_chip(client);
	mutex_unlock(&data->update_lock);
	OZ8806_TemperatureInit();

	return 0;					//return Ok
}

static int OZ8806_remove(struct i2c_client *client)
{
	struct OZ8806_data *data = i2c_get_clientdata(client);

	kfree(data);
	return 0;
}

#ifdef CONFIG_PM
static int OZ8806_suspend(struct device *dev)
{
	struct OZ8806_data *data = the_OZ8806;

	mutex_lock(&data->update_lock);
	FullCount = 5;
	OZ8806_sleep_control(data->myclient, 1, 1);
	mutex_unlock(&data->update_lock);

	return 0;
}

static int OZ8806_resume(struct device *dev)
{
	struct OZ8806_data *data = the_OZ8806;
	u16			ADCValue;
	int				tmpfRSOC;

	mutex_lock(&data->update_lock);
	data->control = CellOCVLow;
	ADCValue = i2c_smbus_read_word_data(data->myclient, data->control);
	if(ADCValue & CellOCVSleepMask)							//Sleep OCV detect
	{
		ADCValue=ADCValue >> 4;								//if no OCV flag, use old value for pretended initialization
		ADCValue=ADCValue & CellOCVMASK;
		batt_info.fOCVVolt = (ADCValue * batt_info.fVoltLSB) / 100;
		batt_info.fVolt = batt_info.fOCVVolt;
		tmpfRSOC= OZ8806_PowerOnVoltToRC();
		if(tmpfRSOC < batt_info.fRSOC)
		{
			batt_info.fRSOC = tmpfRSOC;
			OZ8806_CAR_Reset(data->myclient);
			//sCaMAH = fRCPrev = fRC;
		}
		printk("%s: Suspend wake-up from the OCV mode OCV = %d\n",__func__,batt_info.fOCVVolt);
	}
	else													//no Sleep OCV detection, use current CAR
	{
		data->control = CellCARLow;
		ADCValue = i2c_smbus_read_word_data(data->myclient, data->control);
		ADCValue=ADCValue & CellCARMASK;
		batt_info.fRC = (int)(ADCValue)*batt_info.dbCARLSB;
		batt_info.fRC = batt_info.fRC / batt_info.fRsense;
		batt_info.fRSOC = (batt_info.fRC - batt_info.fReserved) * 100;
		batt_info.fRSOC = batt_info.fRSOC / (batt_info.fFCC - batt_info.fReserved);
		if(batt_info.fRSOC >= 100)		batt_info.fRSOC = 100;
		if(batt_info.fRSOC <= 0)		batt_info.fRSOC = 0;
		printk("%s: Suspend wake-up from the CAR mode RSOC = %d\n",__func__,batt_info.fRSOC);
	}
	OZ8806_sleep_control(data->myclient, 0, 0);
	mutex_unlock(&data->update_lock);

	return 0;
}


static const struct dev_pm_ops oz8806_pm_ops = {
		.suspend = OZ8806_suspend,
		.resume = OZ8806_resume,
};
#endif

static void OZ8806_shutdown(struct i2c_client *client)
{
	struct OZ8806_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->update_lock);
	OZ8806_sleep_control(client, 1, 1);
	mutex_unlock(&data->update_lock);
}

static int oz8806_proc_read(char* page, char** start, off_t off, int count,int* eof, void* data)
{
    struct i2c_client *client = the_OZ8806->myclient;

    if (!client)
        return count;
    data = (void *)page;
    //OZ8806_PollingLoop();

	page += sprintf(page,"\t%-4d\n",batt_info.fCurr);
	page += sprintf(page,"\t%-4d\n",batt_info.fVolt);
	page += sprintf(page,"\t%-4d\n",batt_info.fOCVVolt);
    page += sprintf(page,"\t%-4d\n",batt_info.fRSOC);
    count = 64;

    return ((page += sprintf(page, "\n")) - (char*)data);
}

static int oz8806_proc_write(struct file* file, const char* buffer,unsigned long count, void* data) {
#define MAX_BUFLEN  16
    u8 reg;
    u16 val = MAX_BUFLEN - 1;
    char *ptr, tmp_buf[MAX_BUFLEN];
    struct i2c_client *client = the_OZ8806->myclient;

    if (!client)
        return count;
     if (count < MAX_BUFLEN)
         val = count - 1;
     tmp_buf[val] = 0;
     if (copy_from_user(tmp_buf, buffer, val))
         return -EFAULT;
     for (ptr = tmp_buf; isspace(*ptr); ++ptr);
     reg = simple_strtoul(ptr, &ptr, 16);
     if (!(reg < 0x19)) {
         printk(KERN_DEBUG "wrong register no %d, max %x\n",reg, 0x19);
         return count;
     }
     while (isspace(*ptr))
         ++ptr;
     val = simple_strtoul(ptr, &ptr, 16);
     i2c_smbus_write_word_data(client, reg, val);

     return count;
}


#define OZ8806_PROC_ENTRY      "driver/oz8806"

static int __init oz8806_proc_init(void) {
    struct proc_dir_entry* oz8806;
    if (!(oz8806 = create_proc_entry(OZ8806_PROC_ENTRY,S_IRUGO | S_IWUSR, NULL))) {
        printk("Proc-FS interface for oz8806 failed\n");
        return -ENOMEM;
    }
    oz8806->read_proc  = oz8806_proc_read;
    oz8806->write_proc = oz8806_proc_write;
    oz8806->data   = NULL;
    return 0;
}

static void __exit oz8806_proc_exit(void) {
    remove_proc_entry(OZ8806_PROC_ENTRY, NULL);
}

/*-------------------------------------------------------------------------*/

static const struct i2c_device_id OZ8806_id[] = {
	{ MYDRIVER, 0 },							//string, id??
	{ }
};
MODULE_DEVICE_TABLE(i2c, OZ8806_id);

static struct i2c_driver OZ8806_driver = {
	.driver = {
		.name	= MYDRIVER,
#ifdef CONFIG_PM
		.pm		= &oz8806_pm_ops,
#endif
		.owner  = THIS_MODULE,
	},
	.probe			= OZ8806_probe,
	.remove			= OZ8806_remove,
	.id_table		= OZ8806_id,
	.shutdown		= OZ8806_shutdown,

	//auto-detection function
	.class			= I2C_CLASS_HWMON,			// Nearest choice
	.detect			= OZ8806_detect,
};

/*-------------------------------------------------------------------------*/

static int __init OZ8806_init(void)
{
    int ret = i2c_add_driver(&OZ8806_driver);
    oz8806_proc_init();
    return ret;
}

static void __exit OZ8806_exit(void)
{
    oz8806_proc_exit();
    i2c_del_driver(&OZ8806_driver);
}

/*-------------------------------------------------------------------------*/

#define	DRIVER_VERSION	"24 June 2009"
#define	DRIVER_NAME	(OZ8806_driver.driver.name)

MODULE_DESCRIPTION("OZ8806 Battery Monitor IC Driver");
MODULE_LICENSE("O2Micro");

late_initcall_sync(OZ8806_init);
module_exit(OZ8806_exit);

