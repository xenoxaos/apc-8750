/*++ 
 * linux/drivers/video/wmt/govm.c
 * WonderMedia video post processor (VPP) driver
 *
 * Copyright c 2010  WonderMedia  Technologies, Inc.
 *
 * This program is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 2 of the License, or 
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * WonderMedia Technologies, Inc.
 * 4F, 533, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C
--*/

/*
 * ChangeLog
 *
 * 2010-08-05  Sam Shen
 *     * Add License declaration and ChangeLog
 */

#define HDMI_C

// #define DEBUG
/*----------------------- DEPENDENCE -----------------------------------------*/
#include "hdmi.h"
#include "vout.h"

#ifdef CONFIG_CRYPTO
// #define CONFIG_HDCP_CIPHER
#endif
#ifdef CONFIG_HDCP_CIPHER
#include "../../crypto/wmt-cipher-comm.h"
#include "../../crypto/wmt-cipher-core.h"
#include "../../crypto/wmt-cipher.h"
#endif

/*----------------------- PRIVATE MACRO --------------------------------------*/

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define HDMI_XXXX    1     *//*Example*/
//#define CONFIG_HDMI_INFOFRAME_DISABLE
//#define CONFIG_HDMI_EDID_DISABLE
#define CONFIG_HDMI_HDCP_DISABLE

#define HDMI_I2C_FREQ	80000
#define HDMI_HDCP_CNT	336			// 328 align 16
/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx hdmi_xxx_t; *//*Example*/
typedef enum {
	HDMI_FIFO_SLOT_AVI = 0,
	HDMI_FIFO_SLOT_AUDIO = 1,
	HDMI_FIFO_SLOT_CONTROL = 2,
	HDMI_FIFO_SLOT_MAX = 15
} hdmi_fifo_slot_t;

/*----------EXPORTED PRIVATE VARIABLES are defined in hdmi.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  hdmi_xxx;        *//*Example*/

char hdmi_hdcp_key_encode[HDMI_HDCP_CNT] __attribute__((aligned(4))) = {
	0xc6, 0x45, 0x27, 0x2a, 0x01, 0xc7, 0xa5, 0xc4, 0xaf, 0x02,
	0x36, 0x92, 0xc3, 0x98, 0x85, 0xf1, 0x1a, 0x36, 0x14, 0x23,
	0xe0, 0xc7, 0x14, 0x6e, 0xc9, 0xcd, 0x3f, 0x33, 0xa7, 0xc6,
	0xf0, 0x6f, 0x8c, 0x2a, 0xff, 0xf0, 0xcb, 0x68, 0xa6, 0x87,
	0xd3, 0x3a, 0x5f, 0xbe, 0x77, 0xc1, 0xd4, 0x44, 0x81, 0x35,
	0x9b, 0xfd, 0x53, 0x65, 0x00, 0xf1, 0x8d, 0x09, 0x76, 0x85,
	0x4c, 0xf3, 0xd5, 0x59, 0xc0, 0x49, 0x5a, 0xe0, 0x04, 0x47,
	0x30, 0x45, 0xe0, 0x2e, 0x54, 0x27, 0x6a, 0x88, 0xc6, 0x62,
	0x13, 0x6d, 0x04, 0x60, 0x50, 0x1f, 0x5a, 0x76, 0x41, 0x30,
	0x18, 0x89, 0x96, 0xeb, 0xfe, 0x62, 0xb9, 0x01, 0x09, 0x32,
	0xb1, 0x6d, 0xb9, 0xf2, 0x90, 0x30, 0x32, 0xcc, 0x45, 0x35,
	0xad, 0x04, 0xf1, 0x9e, 0x9d, 0x0c, 0x65, 0x09, 0xf2, 0x43,
	0xfe, 0x98, 0xb6, 0x05, 0x1d, 0xd3, 0xde, 0x3d, 0x4b, 0x61,
	0xc9, 0x69, 0x95, 0x5d, 0xf6, 0x05, 0x4d, 0xc8, 0x6d, 0x15,
	0x0c, 0x2a, 0xee, 0x94, 0x41, 0x13, 0x07, 0x08, 0xb9, 0x63,
	0xf8, 0xd2, 0x66, 0xe6, 0x78, 0xd8, 0x56, 0x96, 0x10, 0xa1,
	0x08, 0x7a, 0xa2, 0x41, 0x5a, 0x97, 0x48, 0x69, 0xac, 0xcf,
	0x35, 0xec, 0xa3, 0x84, 0xe0, 0xe6, 0xbf, 0x21, 0x31, 0xfa,
	0x42, 0xf2, 0x74, 0x17, 0x6f, 0xdc, 0x3a, 0x00, 0xc2, 0x0d,
	0xe2, 0xd4, 0xe2, 0xa4, 0x8b, 0x1e, 0xaf, 0xd4, 0x40, 0x9e,
	0xb5, 0xfc, 0x9e, 0xdc, 0x68, 0x3f, 0xa1, 0x4f, 0x80, 0x6e,
	0x87, 0x4f, 0xda, 0x79, 0x2c, 0xdf, 0x82, 0x9e, 0xde, 0x60,
	0xa1, 0x97, 0x8f, 0x19, 0x16, 0xe7, 0x48, 0xef, 0xf5, 0xa9,
	0xc5, 0x93, 0x7e, 0x8f, 0xd4, 0x01, 0x71, 0x64, 0x32, 0x29,
	0x90, 0x01, 0xd3, 0xca, 0x49, 0x54, 0x05, 0x31, 0x5d, 0x21,
	0xdc, 0x6b, 0xe0, 0x96, 0x27, 0x2d, 0xe4, 0x35, 0xa2, 0x90,
	0xa0, 0xf6, 0x1d, 0x05, 0x89, 0xaf, 0xc6, 0xc3, 0xc2, 0x2f,
	0x41, 0x96, 0xa1, 0x06, 0x9e, 0xfb, 0x36, 0xdc, 0x2b, 0x95,
	0xb7, 0x95, 0x14, 0x24, 0x76, 0xad, 0x48, 0xcb, 0x45, 0xb4,
	0x7f, 0xc8, 0xa3, 0x06, 0xf7, 0x6f, 0x27, 0x09, 0x2f, 0x5c,
	0x76, 0x94, 0x2b, 0xc4, 0x5d, 0x6f, 0x97, 0x94, 0x5a, 0xea,
	0x00, 0xe5, 0x92, 0x37, 0x77, 0xff, 0x38, 0x55, 0xf2, 0x8c,
	0x18, 0x25, 0x1b, 0x73, 0x2f, 0xdb, 0x8c, 0x37, 0x10, 0xcf,
	0xcc, 0xc9, 0xbe, 0xad, 0xf9, 0x18,
};

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void hdmi_xxx(void); *//*Example*/
#ifdef CONFIG_HDCP_CIPHER
extern int Cypher_Action( IN OUT cypher_base_cfg_t *cypher_base );
extern int Cypher_Action_core( IN OUT cypher_base_cfg_t *cypher_base );	
extern int Cypher_Action_port( IN OUT cypher_base_cfg_t *cypher_base );
#endif

/*----------------------- Function Body --------------------------------------*/
/*---------------------------- HDMI COMMON API -------------------------------*/
unsigned char hdmi_ecc(unsigned char *buf,int bit_cnt)
{
	#define HDMI_CRC_LEN	9
	
	int crc[HDMI_CRC_LEN],crc_o[HDMI_CRC_LEN];
	int i,j;
	int input,result,result_rev = 0;

	for(i=0;i<HDMI_CRC_LEN;i++){
		crc[i] = 0;
	}

	for(i=0;i<bit_cnt;i++){
		for(j=0;j<HDMI_CRC_LEN;j++){
			crc_o[j] = crc[j];
		}
		input = (buf[i/8] & (1<<(i%8)))? 1:0;
		crc[0] = crc_o[7] ^ input;
		crc[1] = crc_o[0];
		crc[2] = crc_o[1];
		crc[3] = crc_o[2];
		crc[4] = crc_o[3];
		crc[5] = crc_o[4];
		crc[6] = crc_o[5] ^ crc_o[7] ^ input;
		crc[7] = crc_o[6] ^ crc_o[7] ^ input;
		crc[8] = crc_o[7];

		result     = 0;
		result_rev = 0;
		for (j=0;j<HDMI_CRC_LEN-1;j++){
			result     += (crc[j]<<j);
			result_rev += (crc[j]<<(HDMI_CRC_LEN-2-j));
		}
	}

//	DPRINT("[HDMI] crc 0x%x, %x %x %x %x %x %x %x\n",result_rev,buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6]);
	
	return result_rev;	
}

unsigned char hdmi_checksum(unsigned char *header,unsigned char *buf,int cnt)
{
	unsigned char sum;
	int i;

	for(i=0,sum=0;i<cnt;i++){
		sum += buf[i];
	}
	for(i=0;i<3;i++){
		sum += header[i];
	}
	return (0 - sum);
}

#ifdef WMT_FTBLK_HDMI
unsigned int hdmi_reg32_write(U32 offset, U32 mask, U32 shift, U32 val)
{
	unsigned int new_val;

	new_val = (inl(offset) & ~(mask)) | (((val) << (shift)) & mask);
	if( offset == REG_HDMI_GENERAL_CTRL ){
		new_val &= ~( BIT24 | BIT25 | BIT26 );
		new_val |= g_vpp.hdmi_ctrl;
		DBGMSG("[HDMI] reg32_wr 0x%x ctrl 0x%x\n",new_val,g_vpp.hdmi_ctrl);
	}
	
	outl(new_val, offset);
	return new_val;
}

/*---------------------------- HDMI HAL --------------------------------------*/
void hdmi_set_enable(vpp_flag_t enable)
{
	hdmi_reg32_write(HDMI_ENABLE,enable);
}

void hdmi_set_dvi_enable(vpp_flag_t enable)
{
	hdmi_reg32_write(HDMI_DVI_MODE_ENABLE,enable);
}

void hdmi_set_HDCP_enable(vpp_flag_t enable)
{
	vppif_reg32_write(HDMI_HDCP_ENABLE,enable);
}

void hdmi_set_sync_low_active(vpp_flag_t hsync,vpp_flag_t vsync)
{
	hdmi_reg32_write(HDMI_HSYNC_LOW_ACTIVE,hsync);
	hdmi_reg32_write(HDMI_VSYNC_LOW_ACTIVE,vsync);
}

void hdmi_set_output_colfmt(vdo_color_fmt colfmt)
{
	unsigned int val;

	switch(colfmt){
		default:
		case VDO_COL_FMT_ARGB:
			val = 0;
			break;
		case VDO_COL_FMT_YUV444:
			val = 1;
			break;
		case VDO_COL_FMT_YUV422H:
		case VDO_COL_FMT_YUV422V:
			val = 2;
			break;
	}
	hdmi_reg32_write(HDMI_CONVERT_YUV422,(val==2)?1:0);
	hdmi_reg32_write(HDMI_OUTPUT_FORMAT,val);
}

vdo_color_fmt hdmi_get_output_colfmt(void)
{
	unsigned int val;

	val = vppif_reg32_read(HDMI_OUTPUT_FORMAT);
	switch(val){
		default:
		case 0: return VDO_COL_FMT_ARGB;
		case 1: return VDO_COL_FMT_YUV444;
		case 2: return VDO_COL_FMT_YUV422H;
	}
	return VDO_COL_FMT_ARGB;
}

int hdmi_get_plugin(void)
{
	return vppif_reg32_read(HDMI_HOTPLUG_IN);
}

int hdmi_get_plug_status(void)
{
	int reg;

	reg = vppif_reg32_in(REG_HDMI_HOTPLUG_DETECT);
	return (reg & 0x3000000);
}

void hdmi_clear_plug_status(void)
{
	vppif_reg32_write(HDMI_HOTPLUG_IN_STS,1);
	vppif_reg32_write(HDMI_HOTPLUG_OUT_STS,1);	
}

void hdmi_enable_plugin(int enable)
{
	vppif_reg32_write(HDMI_HOTPLUG_OUT_INT,enable);
	vppif_reg32_write(HDMI_HOTPLUG_IN_INT,enable);
}

void hdmi_write_fifo(hdmi_fifo_slot_t no,unsigned int *buf,int cnt)
{
	int i;

	if( no > HDMI_FIFO_SLOT_MAX ) return;
#ifdef DEBUG
{
	char *ptr;

	DPRINT("[HDMI] AVI info package %d,cnt %d",no,cnt);
	ptr = (char *) buf;
	for(i=0;i<cnt;i++){
		if( (i % 4)==0 ) DPRINT("\n %02d :",i);
		DPRINT(" 0x%02x",ptr[i]);
	}
	DPRINT("\n[HDMI] AVI info package end\n");	
}	
#endif

	vppif_reg32_out(REG_HDMI_FIFO_CTRL,(no << 8));
	cnt = (cnt+3)/4;
	for(i=0;i<cnt;i++){
		vppif_reg32_out(REG_HDMI_WR_FIFO_ADDR+4*i,buf[i]);
	}
	vppif_reg32_write(HDMI_INFOFRAME_WR_STROBE,1);
}

void hdmi_read_fifo(hdmi_fifo_slot_t no,unsigned int *buf,int cnt)
{
	int i;
	
	if( no > HDMI_FIFO_SLOT_MAX ) return;
	
	vppif_reg32_out(REG_HDMI_FIFO_CTRL,(no << 8));	
	vppif_reg32_write(HDMI_INFOFRAME_RD_STROBE,1);
	for(i=0;i<cnt;i++){
		buf[i] = vppif_reg32_in(REG_HDMI_RD_FIFO_ADDR+4*i);
	}
}

#define HDMI_STATUS_START		BIT16
#define HDMI_STATUS_STOP		BIT17
#define HDMI_STATUS_WR_AVAIL	BIT18
#define HDMI_STATUS_HDCP_USE	BIT19
#define HDMI_STATUS_SW_READ		BIT25
int hdmi_DDC_check_status(unsigned int checkbits,int condition)
{
    int status = 1;
    unsigned int i = 0, maxloop = 50;

	if( condition ){ // wait 1 --> 0
		while((vppif_reg32_in(REG_HDMI_I2C_CTRL2) & checkbits) && (i<maxloop)){
			udelay(20); // delay
            if(++i == maxloop) status = 0;
		}
	}
	else { // wait 0 --> 1
		while(!(vppif_reg32_in(REG_HDMI_I2C_CTRL2) & checkbits) && (i<maxloop)){
			udelay(20); // delay
            if(++i == maxloop) status = 0;
		}
	}
	
	if( status == 0 ){
		unsigned int reg;

		reg = vppif_reg32_in(REG_HDMI_I2C_CTRL2);
		DBGMSG("[HDMI] status timeout check 0x%x,wait to %s\n",checkbits,(condition)? "0":"1");
		DBGMSG("[HDMI] 0x%x,sta %d,stop %d,wr %d,rd %d,hdcp %d\n",reg,(reg & HDMI_STATUS_START)? 1:0,
			(reg & HDMI_STATUS_STOP)? 1:0,(reg & HDMI_STATUS_WR_AVAIL)? 1:0,(reg & HDMI_STATUS_SW_READ)? 1:0,
			(reg & HDMI_STATUS_HDCP_USE)? 1:0);
	}
	
    return status;
}

void hdmi_DDC_set_freq(unsigned int hz)
{
	unsigned int clock;
	unsigned int div;

	clock = 25000000*15/100;	// RTC clock source 
	div = clock / hz;
	
	vppif_reg32_write(HDMI_I2C_CLK_DIVIDER,div);
	DBGMSG("[HDMI] set freq(%d,clk %d,div %d)\n",hz,clock,div);
}

int hdmi_DDC_read(char addr,int index,char *buf,int length)
{
    int status = 1;
    unsigned int i = 0;
	int err_cnt = 0;

	DBGMSG("[HDMI] read DDC(index 0x%x,len %d),reg 0x%x\n",index,length,vppif_reg32_in(REG_HDMI_I2C_CTRL2));

#ifdef CONFIG_HDMI_EDID_DISABLE
	return status;
#endif

//	hdmi_DDC_set_freq(g_vpp.hdmi_i2c_freq);
//	vppif_reg32_write(HDMI_I2C_ENABLE,1);

	// enhanced DDC read
	if( index >= 256 ){
		vppif_reg32_write(REG_HDMI_I2C_CTRL2,BIT18+BIT16,16,0x5);	// sw start, write data avail	
		status = hdmi_DDC_check_status(HDMI_STATUS_START+HDMI_STATUS_WR_AVAIL, 1);	// wait start & wr data avail
		if( status == 0 ){
			DBGMSG("[HDMI] *E* start\n");
			err_cnt++;
		}

		// Slave address
		vppif_reg32_write(HDMI_WR_DATA,0x60);
		vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
		status = hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);// wait wr data avail
		if( status == 0 ){
			DBGMSG("[HDMI] *E* slave addr 0x%x\n",addr);
			err_cnt++;
		}

		// Offset
		vppif_reg32_write(HDMI_WR_DATA,0x1);
		vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
		status = hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);// wait wr data avail
		if( status == 0 ){
			DBGMSG("[HDMI] *E* index 0x%x\n",index);
			err_cnt++;
		}
		index -= 256;
	}

	// START
//	vppif_reg32_write(HDMI_SW_START_REQ,1);					// sw start
//	vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
	vppif_reg32_write(REG_HDMI_I2C_CTRL2,BIT18+BIT16,16,0x5);	// sw start, write data avail	
	status = hdmi_DDC_check_status(HDMI_STATUS_START+HDMI_STATUS_WR_AVAIL, 1);	// wait start & wr data avail
	if( status == 0 ){
		DBGMSG("[HDMI] *E* start\n");
		err_cnt++;
	}

	// Slave address
	vppif_reg32_write(HDMI_WR_DATA,addr);
	vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
	status = hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);// wait wr data avail
	if( status == 0 ){
		DBGMSG("[HDMI] *E* slave addr 0x%x\n",addr);
		err_cnt++;
	}

	// Offset
	vppif_reg32_write(HDMI_WR_DATA,index);
	vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
	status = hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);// wait wr data avail
	if( status == 0 ){
		DBGMSG("[HDMI] *E* index 0x%x\n",index);
		err_cnt++;
	}

    // START
//	vppif_reg32_write(HDMI_SW_START_REQ,1);					// sw start
//	vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
	vppif_reg32_write(REG_HDMI_I2C_CTRL2,BIT18+BIT16,16,0x5);	// sw start, write data avail	
	status = hdmi_DDC_check_status(HDMI_STATUS_START+HDMI_STATUS_WR_AVAIL, 1);		// wait start & wr data avail
	if( status == 0 ){
		DBGMSG("[HDMI] *E* restart\n");
		err_cnt++;
	}

    // Slave Address + 1
	vppif_reg32_write(HDMI_WR_DATA,addr+1);
	vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
	status = hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);// wait wr data avail
	if( status == 0 ){
		DBGMSG("[HDMI] *E* slave addr 0x%x\n",addr+1);
		err_cnt++;
	}

    // Read Data
    for(i = 0; i < length; i++)
    {
		vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
		udelay(g_vpp.hdmi_i2c_udelay); // delay
		status = hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);// wait wr data avail
		if( status == 0 ){
			DBGMSG("[HDMI] *E* wr avail(%d)\n",i);
			err_cnt++;
			break;
		}

		status = hdmi_DDC_check_status(HDMI_STATUS_SW_READ, 0);	// wait sw read not set
		if( status == 0 ){
			DBGMSG("[HDMI] *E* read avail(%d)\n",i);
			err_cnt++;
			break;
		}

        *buf++ = vppif_reg32_read(HDMI_RD_DATA);
		udelay(g_vpp.hdmi_i2c_udelay); // delay
		vppif_reg32_write(HDMI_SW_READ,0);						// sw read from HDCP
		udelay(g_vpp.hdmi_i2c_udelay);
    }

    // STOP
//	vppif_reg32_write(HDMI_SW_STOP_REQ,1);					// sw stop
//	vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
	vppif_reg32_write(REG_HDMI_I2C_CTRL2,BIT18+BIT17,17,3);	// sw stop, write data avail
	status = hdmi_DDC_check_status(HDMI_STATUS_STOP+HDMI_STATUS_WR_AVAIL+HDMI_STATUS_HDCP_USE, 1);	// wait start & wr data avail 
	if( status == 0 ){
		DBGMSG("[HDMI] *E* stop\n");
		err_cnt++;
	}

	if( err_cnt ){
		DBGMSG("[HDMI] *E* read DDC %d\n",err_cnt);
	}
	else {
		DBGMSG("[HDMI] read DDC OK\n");
	}
    return (err_cnt)? 1:0;
}

void hdmi_audio_enable(vpp_flag_t enable)
{
	vppif_reg32_write(HDMI_AUD_ENABLE,enable);
}

void hdmi_audio_mute(vpp_flag_t enable)
{
	vppif_reg32_write(HDMI_AUD_MUTE,enable);
}

/*----------------------- HDMI API --------------------------------------*/
void hdmi_write_packet(unsigned int header,unsigned char *packet,int cnt)
{
	unsigned char buf[36];
	int i;
	hdmi_fifo_slot_t no;

#ifdef CONFIG_HDMI_INFOFRAME_DISABLE
	return;
#endif
	memcpy(&buf[0],&header,3);
	buf[3] = hdmi_ecc((unsigned char *)&header,24);
	for( i=0; i<cnt/7; i++ ){
		memcpy(&buf[4+8*i],&packet[7*i],7);
		buf[11+8*i] = hdmi_ecc(&packet[7*i],56);
	}

	switch(header & 0xFF){
		case HDMI_PACKET_INFOFRAME_AVI:
			no = HDMI_FIFO_SLOT_AVI;
			break;
		case HDMI_PACKET_INFOFRAME_AUDIO:
			no = HDMI_FIFO_SLOT_AUDIO;
			break;
		default:
			no = HDMI_FIFO_SLOT_CONTROL;
			break;
	}
	hdmi_write_fifo(no,(unsigned int *)buf,(4+8*(cnt/7)));
}

void hdmi_tx_null_packet(void)
{
	hdmi_write_packet(HDMI_PACKET_NULL,0,0);
}

void hdmi_tx_general_control_packet(int mute)
{
	unsigned char buf[7];
	memset(buf,0x0,7);
	buf[0] = (mute)? 0x01:0x10;
	buf[1] = HDMI_COLOR_DEPTH_24 | ( HDMI_PHASE_4 << 4);
	hdmi_write_packet(HDMI_PACKET_GENERAL_CTRL,buf,7);
}

void hdmi_tx_avi_infoframe_packet(vdo_color_fmt colfmt,hdmi_video_code_t vic)
{
	unsigned int header;
	unsigned char buf[28];
	unsigned char temp;

	memset(buf,0x0,28);
	header = HDMI_PACKET_INFOFRAME_AVI + (0x2 << 8) + (0x0d << 16);
	buf[1] = HDMI_SI_NO_DATA + (HDMI_BI_V_H_VALID << 2) + (HDMI_AF_INFO_NO_DATA << 4);
	switch( colfmt ){
		case VDO_COL_FMT_YUV422H:
		case VDO_COL_FMT_YUV422V:
			temp = HDMI_OUTPUT_YUV422;
			break;
		case VDO_COL_FMT_YUV444:
			temp = HDMI_OUTPUT_YUV444;
			break;
		case VDO_COL_FMT_ARGB:
		default:
			temp = HDMI_OUTPUT_RGB;
			break;
	}
	buf[1] += (temp << 5);
	buf[2] = HDMI_ASPECT_RATIO_PIC + (HDMI_PIC_ASPECT_16_9 << 4) + (HDMI_COLORIMETRY_ITU709 << 6);
	buf[3] = 0x84;	
	buf[4] = vic;
	switch( vic ){
		case HDMI_1440x480i60_16x9:
		case HDMI_1440x576i50_16x9:
			buf[5] = HDMI_PIXEL_REP_2;
			break;
		default:
			buf[5] = HDMI_PIXEL_REP_NO;
			break;
	}
	buf[0] = hdmi_checksum((unsigned char *)&header,buf,28);
	hdmi_write_packet(header,buf,28);
}

void hdmi_tx_audio_infoframe_packet(int channel,int freq)
{
	unsigned int header;
	unsigned char buf[28];

	memset(buf,0x0,28);
	header = HDMI_PACKET_INFOFRAME_AUDIO + (0x1 << 8) + (0x0a << 16);
	buf[1] = g_vpp.hdmi_audio_pb1; // channel + (HDMI_AUD_TYPE_REF_STM << 4);
	buf[2] = 0x0;	// HDMI_AUD_SAMPLE_24 + (freq << 2);
	buf[3] = 0x00;
	buf[4] = g_vpp.hdmi_audio_pb4; // 0x0;
	buf[5] = 0x0;	// 0 db
	buf[0] = hdmi_checksum((unsigned char *)&header,buf,28);
	hdmi_write_packet(header,buf,28);
}

void hdmi_set_audio_n_cts(unsigned int freq)
{
	unsigned int n,cts;

	n = 128 * freq / 1000;
	cts = vpp_get_base_clock(VPP_MOD_GOVRH) / 1000;
	vppif_reg32_write(HDMI_AUD_N_20BITS,n);
	vppif_reg32_write(HDMI_AUD_ACR_RATIO,cts-2);	
#if 1	// auto detect CTS
	vppif_reg32_write(HDMI_AUD_CTS_SELECT,0);
	cts = 0;
#else
	vppif_reg32_write(HDMI_AUD_CTS_SELECT,1);
#endif
	vppif_reg32_write(HDMI_AUD_CTS_LOW_12BITS,cts & 0xFFF);
	vppif_reg32_write(HDMI_AUD_CTS_HI_8BITS,(cts & 0xFF000)>>12);

	DBGMSG("[HDMI] set audio freq %d,n %d,cts %d\n",freq,n,cts);
}
void hdmi_config_audio(vout_audio_t *info)
{
	unsigned int freq;

	hdmi_tx_audio_infoframe_packet(info->channel-1,info->sample_rate);
	hdmi_audio_enable(VPP_FLAG_DISABLE);
//	vppif_reg32_out(REG_HDMI_AUD_MODE,0);
//	vppif_reg32_write(HDMI_AUD_LAYOUT,(info->channel==8)? 1:0);

	switch(info->sample_rate){
		case 32000: freq = 0x3; break;
		case 44100: freq = 0x0; break;
		case 88200: freq = 0x8; break;
		case 176400: freq = 0xC; break;
		default:
		case 48000: freq = 0x2; break;
		case 96000: freq = 0xA; break;
		case 192000: freq = 0xE; break;
		case 768000: freq = 0x9; break;
	}
	vppif_reg32_out(REG_HDMI_AUD_CHAN_STATUS0,(freq << 24) + 0x4);
	vppif_reg32_out(REG_HDMI_AUD_CHAN_STATUS1,0x0);
	vppif_reg32_out(REG_HDMI_AUD_CHAN_STATUS2,0xb);
	vppif_reg32_out(REG_HDMI_AUD_CHAN_STATUS3,0x0);
	vppif_reg32_out(REG_HDMI_AUD_CHAN_STATUS4,0x0);
	vppif_reg32_out(REG_HDMI_AUD_CHAN_STATUS5,0x0);

	hdmi_set_audio_n_cts(info->sample_rate);
	vppif_reg32_write(HDMI_AUD_ACR_ENABLE,VPP_FLAG_ENABLE);	// 
	vppif_reg32_write(HDMI_AUD_AIPCLK_RATE,0);
	vppif_reg32_out(AUDREGF_BASE_ADDR+0x18c,0x76543210);
	hdmi_audio_enable(VPP_FLAG_ENABLE);
}

void hdmi_config_video(hdmi_info_t *info)
{
	hdmi_set_output_colfmt(info->outfmt);
	hdmi_tx_avi_infoframe_packet(info->outfmt,info->vic);
}

void hdmi_config(hdmi_info_t *info)
{
	vout_audio_t audio_info;
	int h_porch;
	int hdcp_delay_cfg;

	vppif_reg32_write(HDMI_INFOFRAME_SELECT,0);
	vppif_reg32_write(HDMI_INFOFRAME_FIFO1_RDY,0);
	hdmi_config_video(info);

	h_porch = vppif_reg32_read(GOVRH_H_ALLPXL) - vppif_reg32_read(GOVRH_ACTPX_END) - 2; // fp
	hdcp_delay_cfg = 47 - h_porch;
	if( hdcp_delay_cfg <= 0 ) hdcp_delay_cfg = 1;
	h_porch = vppif_reg32_read(GOVRH_ACTPX_BG) + 2;	// bp
	h_porch = (h_porch - (hdcp_delay_cfg+1) - 26) / 32;
	if( h_porch <= 0 ) h_porch = 1;
	if( h_porch >= 8 ) h_porch = 0;
	hdmi_reg32_write(HDMI_HDCP_DELAY,hdcp_delay_cfg);
	vppif_reg32_write(HDMI_HORIZ_BLANK_MAX_PCK,h_porch);
	DBGMSG("[HDMI] H blank max pck %d,hdcp delay %d\n",h_porch,hdcp_delay_cfg);

	audio_info.fmt = 16;
	audio_info.channel = info->channel;
	audio_info.sample_rate = info->freq;
	hdmi_config_audio(&audio_info);

	vppif_reg32_write(HDMI_INFOFRAME_FIFO1_ADDR,0);
	vppif_reg32_write(HDMI_INFOFRAME_FIFO1_LEN,1);
	vppif_reg32_write(HDMI_INFOFRAME_FIFO1_RDY,1);
}

/*----------------------- Module API --------------------------------------*/
int hdmi_hdcp_check_int(void)
{
	unsigned int reg;
	int ret = 0;

	reg = vppif_reg32_in(REG_HDMI_HDCP_INT);
	if( reg & BIT28 ){
		int sts;

		sts = (reg & 0xE0000000) >> 29;
		ret = 1;
		switch( sts ){
			case 0x0:
				DPRINT("[HDCP] auth fail\n");
				break;
			case 0x1:
				DPRINT("[HDCP] RI fail\n");
				break;
			case 0x2:
				DPRINT("[HDCP] PJ fail\n");
				break;
			case 0x3:
				DPRINT("[HDCP] KSV not ready\n");
				break;
			case 0x4:
				DPRINT("[HDCP] BKSV invalid\n");
				break;
			case 0x5:
				DPRINT("[HDCP] vcheck fail\n");
				break;
			case 0x6:
				DPRINT("[HDCP] RI vry timeout\n");
				break;
			default:
			case 0x7:
				DPRINT("[HDCP] auth ok\n");
				ret = 0;
				break;
		}
		vppif_reg32_write(HDMI_HDCP_INT_CLR,1);
	}
	return ret;
}

int hdmi_check_plugin(int hotplug)
{
	int plugin;
	int option = 0;

	plugin = hdmi_get_plugin();
	if( hotplug ){
		hdmi_clear_plug_status();
		hdmi_set_enable(plugin);
		if( plugin ){
#if 0
			option = EDID_OPT_HDMI + EDID_OPT_AUDIO;
#else
			unsigned char *buf;

			buf = vout_get_edid(VOUT_HDMI);
			option = edid_parse_option(buf);
#endif			
			hdmi_set_dvi_enable((option & EDID_OPT_HDMI)? VPP_FLAG_DISABLE:VPP_FLAG_ENABLE);
			hdmi_audio_enable((option & EDID_OPT_AUDIO)? VPP_FLAG_ENABLE:VPP_FLAG_DISABLE);
			if( g_vpp.hdmi_hdcp_enable )
				hdmi_set_HDCP_enable((option & EDID_OPT_HDMI)? VPP_FLAG_ENABLE:VPP_FLAG_DISABLE);
		}
		else {
			hdmi_set_HDCP_enable(VPP_FLAG_DISABLE);
			hdmi_audio_enable(VPP_FLAG_DISABLE);
		}
	}
	DPRINT("[HDMI] HDMI plug%s,option(8-HDMI,6-AUDIO) 0x%x\n",(plugin)?"in":"out",option);
	return plugin;
}

void hdmi_reg_dump(void)
{
	DPRINT("========== HDMI register dump ==========\n");
	vpp_reg_dump(REG_HDMI_BEGIN,REG_HDMI_END-REG_HDMI_BEGIN);
	
	DPRINT("---------- HDMI common ----------\n");
	DPRINT("enable %d,reset %d,dvi %d\n",vppif_reg32_read(HDMI_ENABLE),vppif_reg32_read(HDMI_RESET),vppif_reg32_read(HDMI_DVI_MODE_ENABLE));
	DPRINT("colfmt %d,conv 422 %d,hsync low %d,vsync low %d\n",vppif_reg32_read(HDMI_OUTPUT_FORMAT),vppif_reg32_read(HDMI_CONVERT_YUV422),vppif_reg32_read(HDMI_HSYNC_LOW_ACTIVE),vppif_reg32_read(HDMI_VSYNC_LOW_ACTIVE));
	DPRINT("dbg bus sel %d,state mach %d\n",vppif_reg32_read(HDMI_DBG_BUS_SELECT),vppif_reg32_read(HDMI_STATE_MACHINE_STATUS));	
	DPRINT("eep reset %d,encode %d,eess %d\n",vppif_reg32_read(HDMI_EEPROM_RESET),vppif_reg32_read(HDMI_ENCODE_ENABLE),vppif_reg32_read(HDMI_EESS_ENABLE));
	DPRINT("verify pj %d,auth test %d,cipher %d\n",vppif_reg32_read(HDMI_VERIFY_PJ_ENABLE),vppif_reg32_read(HDMI_AUTH_TEST_KEY),vppif_reg32_read(HDMI_CIPHER_1_1));	
	DPRINT("preamble %d\n",vppif_reg32_read(HDMI_PREAMBLE));	
	DPRINT("wdt divid %d\n",vppif_reg32_read(HDMI_WDT_DIVIDER));	

	DPRINT("---------- HDMI hotplug ----------\n");
	DPRINT("plug %s\n",vppif_reg32_read(HDMI_HOTPLUG_IN)? "in":"out");
	DPRINT("plug in enable %d, status %d\n",vppif_reg32_read(HDMI_HOTPLUG_IN_INT),vppif_reg32_read(HDMI_HOTPLUG_IN_STS));
	DPRINT("plug out enable %d, status %d\n",vppif_reg32_read(HDMI_HOTPLUG_OUT_INT),vppif_reg32_read(HDMI_HOTPLUG_OUT_STS));
	DPRINT("debounce detect %d,sample %d\n",vppif_reg32_read(HDMI_DEBOUNCE_DETECT),vppif_reg32_read(HDMI_DEBOUNCE_SAMPLE));

	DPRINT("---------- HDCP ----------\n");
	DPRINT("enable %d,key encode %d,key path %d\n",vppif_reg32_read(HDMI_HDCP_ENABLE),vppif_reg32_read(HDMI_HDCP_KEY_ENCODE),vppif_reg32_read(HDMI_CPU_WR_HDCP_KEY_PATH));
	DPRINT("key req %d,key read %d,key last %d\n",vppif_reg32_read(HDMI_HDCP_KEY_REQ),vppif_reg32_read(HDMI_HDCP_KEY_READ),vppif_reg32_read(HDMI_HDCP_KEY_LAST));
	DPRINT("src sel %d,ksv list avail %d,ksv ver done %d\n",vppif_reg32_read(HDMI_HDCP_SRC_SEL),vppif_reg32_read(HDMI_KSV_LIST_AVAIL),vppif_reg32_read(HDMI_KSV_VERIFY_DONE));
	DPRINT("delay %d\n",vppif_reg32_read(HDMI_HDCP_DELAY));
	DPRINT("key shuffle mode %d,key debug mode %d\n",vppif_reg32_read(HDMI_HDCP_KEY_SHUFFLE_MODE),vppif_reg32_read(HDMI_HDCP_KEY_DEBUG_MODE));
	DPRINT("TMDS test format %d,enable %d\n",vppif_reg32_read(HDMI_TMDS_TST_FORMAT),vppif_reg32_read(HDMI_TMDS_TST_ENABLE));
	DPRINT("status 0x%x\n",vppif_reg32_in(HDMI_BASE_ADDR+0x11c));
	DPRINT("HDCP auth pass %d, done %d\n",vppif_reg32_read(HDMI_HDCP_AUTH_PASS),vppif_reg32_read(HDMI_HDCP_AUTH_DONE));

	DPRINT("---------- I2C ----------\n");
	DPRINT("enable %d,exit FSM %d,key read %d\n",vppif_reg32_read(HDMI_I2C_ENABLE),vppif_reg32_read(HDMI_FORCE_EXIT_FSM),vppif_reg32_read(HDMI_KEY_READ_WORD));
	DPRINT("clk divid %d,rd data 0x%x,wr data 0x%x\n",vppif_reg32_read(HDMI_I2C_CLK_DIVIDER),vppif_reg32_read(HDMI_RD_DATA),vppif_reg32_read(HDMI_WR_DATA));
	DPRINT("start %d,stop %d,wr avail %d\n",vppif_reg32_read(HDMI_SW_START_REQ),vppif_reg32_read(HDMI_SW_STOP_REQ),vppif_reg32_read(HDMI_WR_DATA_AVAIL));
	DPRINT("status %d,sw read %d,sw i2c req %d\n",vppif_reg32_read(HDMI_I2C_STATUS),vppif_reg32_read(HDMI_SW_READ),vppif_reg32_read(HDMI_SW_I2C_REQ));

	DPRINT("---------- AUDIO ----------\n");
	DPRINT("enable %d,sub pck %d,spflat %d\n",vppif_reg32_read(HDMI_AUD_ENABLE),vppif_reg32_read(HDMI_AUD_SUB_PACKET),vppif_reg32_read(HDMI_AUD_SPFLAT));	
	DPRINT("aud pck insert reset %d,enable %d,delay %d\n",vppif_reg32_read(HDMI_AUD_PCK_INSERT_RESET),vppif_reg32_read(HDMI_AUD_PCK_INSERT_ENABLE),vppif_reg32_read(HDMI_AUD_INSERT_DELAY));
	DPRINT("avmute set %d,clr %d,pixel repete %d\n",vppif_reg32_read(HDMI_AVMUTE_SET_ENABLE),vppif_reg32_read(HDMI_AVMUTE_CLR_ENABLE),vppif_reg32_read(HDMI_AUD_PIXEL_REPETITION));
	DPRINT("acr ratio %d,acr enable %d,mute %d\n",vppif_reg32_read(HDMI_AUD_ACR_RATIO),vppif_reg32_read(HDMI_AUD_ACR_ENABLE),vppif_reg32_read(HDMI_AUD_MUTE));	
	DPRINT("layout %d,pwr save %d,n 20bits %d\n",vppif_reg32_read(HDMI_AUD_LAYOUT),vppif_reg32_read(HDMI_AUD_PWR_SAVING),vppif_reg32_read(HDMI_AUD_N_20BITS));
	DPRINT("cts low 12 %d,hi 8 %d,cts sel %d\n",vppif_reg32_read(HDMI_AUD_CTS_LOW_12BITS),vppif_reg32_read(HDMI_AUD_CTS_HI_8BITS),vppif_reg32_read(HDMI_AUD_CTS_SELECT));
	DPRINT("aipclk rate %d\n",vppif_reg32_read(HDMI_AUD_AIPCLK_RATE));

	DPRINT("---------- INFOFRAME ----------\n");
	DPRINT("sel %d,hor blank pck %d\n",vppif_reg32_read(HDMI_INFOFRAME_SELECT),vppif_reg32_read(HDMI_HORIZ_BLANK_MAX_PCK));
	DPRINT("fifo1 ready %d,addr 0x%x,len %d\n",vppif_reg32_read(HDMI_INFOFRAME_FIFO1_RDY),vppif_reg32_read(HDMI_INFOFRAME_FIFO1_ADDR),vppif_reg32_read(HDMI_INFOFRAME_FIFO1_LEN));
	DPRINT("fifo2 ready %d,addr 0x%x,len %d\n",vppif_reg32_read(HDMI_INFOFRAME_FIFO2_RDY),vppif_reg32_read(HDMI_INFOFRAME_FIFO2_ADDR),vppif_reg32_read(HDMI_INFOFRAME_FIFO2_LEN));
	DPRINT("wr strobe %d,rd strobe %d,fifo addr %d\n",vppif_reg32_read(HDMI_INFOFRAME_WR_STROBE),vppif_reg32_read(HDMI_INFOFRAME_RD_STROBE),vppif_reg32_read(HDMI_INFOFRAME_FIFO_ADDR));

	DPRINT("---------- HDMI test ----------\n");
	DPRINT("ch0 enable %d, data 0x%x\n",vppif_reg32_read(HDMI_CH0_TEST_MODE_ENABLE),vppif_reg32_read(HDMI_CH0_TEST_DATA));
	DPRINT("ch1 enable %d, data 0x%x\n",vppif_reg32_read(HDMI_CH1_TEST_MODE_ENABLE),vppif_reg32_read(HDMI_CH1_TEST_DATA));
	DPRINT("ch2 enable %d, data 0x%x\n",vppif_reg32_read(HDMI_CH2_TEST_MODE_ENABLE),vppif_reg32_read(HDMI_CH2_TEST_DATA));
}

#ifdef CONFIG_PM
static unsigned int *hdmi_pm_bk;
static unsigned int hdmi_pm_enable;
void hdmi_suspend(int sts)
{
	switch( sts ){
		case 0:	// disable module
			hdmi_pm_enable = vppif_reg32_read(HDMI_ENABLE);
			hdmi_reg32_write(HDMI_ENABLE,0);
			break;
		case 1: // disable tg
			break;
		case 2:	// backup register
			hdmi_pm_bk = vpp_backup_reg(REG_HDMI_BEGIN,(REG_HDMI_END-REG_HDMI_BEGIN));
			break;
		default:
			break;
	}
}

void hdmi_resume(int sts)
{
	switch( sts ){
		case 0:	// restore register
			vpp_restore_reg(REG_HDMI_BEGIN,(REG_HDMI_END-REG_HDMI_BEGIN),hdmi_pm_bk);
			hdmi_pm_bk = 0;
			break;
		case 1:	// enable module
			hdmi_reg32_write(HDMI_ENABLE,0);
			break;
		case 2: // enable tg
			break;
		default:
			break;
	}
}
#else
#define hdmi_suspend NULL
#define hdmi_resume NULL
#endif

void hdmi_write_key(unsigned int *buf)
{
#ifdef CONFIG_HDCP_CIPHER
	cypher_base_cfg_t cypher;

	DPRINT("[HDMI] decode HDCP key by cipher\n");

	memset(&cypher,0,sizeof(cypher_base_cfg_t));
	cypher.algo_mode = CYPHER_ALGO_AES;
	cypher.input_addr = __pa(buf);
	cypher.output_addr = REG_HDMI_HDCP_FIFO_ADDR;
	cypher.text_length = 328;
	cypher.dec_enc = CYPHER_DECRYPT;
	cypher.op_mode = CYPHER_OP_ECB_HW_KEY;
	cypher.INC = 0;
	Cypher_Action_port(&cypher);
#endif
}

void hdmi_hdcp_init(void)
{
	vppif_reg32_write(HDMI_HDCP_DELAY,0x40);
	vppif_reg32_write(HDMI_EEPROM_RESET,0x1);
	vppif_reg32_write(HDMI_EESS_ENABLE,0x1);
	vppif_reg32_out(REG_HDMI_WDT_CTRL,0x160000);	// 14.318M/wdt = 1/100ms, wdt --> 2^20 == 0x100000
	vppif_reg32_write(HDMI_HDCP_KEY_DEBUG_MODE,0x1);
	vppif_reg32_write(HDMI_HDCP_INT_ENABLE,1);
	hdmi_write_key((unsigned int *)&hdmi_hdcp_key_encode[0]);
}

void hdmi_init(void)
{
	hdmi_info_t hdmi_info;

	g_vpp.hdmi_i2c_freq = HDMI_I2C_FREQ;
	g_vpp.hdmi_i2c_udelay = 0;

	vppif_reg32_write(LVDS_VBG_SEL,2);
	vppif_reg32_write(LVDS_DRV_PDMODE,1);
	hdmi_set_enable(VPP_FLAG_ENABLE);
	hdmi_set_dvi_enable(VPP_FLAG_DISABLE);
	lvds_set_enable(1);
	lvds_set_enable(0);

	hdmi_info.outfmt = VDO_COL_FMT_ARGB;
	hdmi_info.vic = HDMI_1280x720p60_16x9;
	hdmi_info.channel = 2;
	hdmi_info.freq = 48000;

	hdmi_config(&hdmi_info);
	lvds_set_enable(1);
	lvds_set_enable(0);
	vppif_reg32_out(HDMI_BASE_ADDR+0x3ec,0x0);
	vppif_reg32_out(HDMI_BASE_ADDR+0x3e8,0x0);

	vppif_reg32_write(HDMI_INFOFRAME_SELECT,0);
	vppif_reg32_write(HDMI_INFOFRAME_FIFO1_RDY,0);

	g_vpp.hdmi_ctrl = 0x1000000;
	g_vpp.hdmi_audio_pb4 = 0x0;
	g_vpp.hdmi_audio_pb1 = 0x0;
	vppif_reg32_write(HDMI_AUD_LAYOUT,1);
	hdmi_DDC_set_freq(g_vpp.hdmi_i2c_freq);
	vppif_reg32_write(HDMI_I2C_ENABLE,1);

	hdmi_hdcp_init();
}
#endif /* WMT_FTBLK_HDMI */

