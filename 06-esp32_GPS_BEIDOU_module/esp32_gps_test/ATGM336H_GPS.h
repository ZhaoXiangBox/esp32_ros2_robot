/***********************************************************  
  GPS、北斗(BDS)双模模块
  通信接口：
    VCC:5.0V  或 3.3V
    GND:0V
    RX: 串口接收接口
    Tx: 串口发送接口

    波特率：9600
    定位精度：2.5m (开阔地)
    首次定位时间：32s
    串口输出协议： NMEA0183 ，参见《CASIC多模卫星导航接收机协议规范》

***********************************************************/
#define GPS_BDS_Serial  Serial2

// 定义的GPS信息结构体类型，用来存储接收的模组定位信息
struct GPS_BDS_DATA
{
	char GPS_Buffer[80];
	bool isGetData;		    //是否获取到GPS数据
	bool isParseData;	    //是否解析完成
	char UTCTime[11];		  //UTC时间
	char latitude[11];		//纬度
	char N_S[2];		      //N/S
	char longitude[12];		//经度
	char E_W[2];		      //E/W
	bool isUsefull;		    //定位信息是否有效
} Save_Data;

const unsigned int gpsRxBufferLength = 600;
char gpsRxBuffer[gpsRxBufferLength];
unsigned int ii = 0;

void GPS_BDS_Init()
{
	GPS_BDS_Serial.begin(9600);			// 模块的波特率决定
  Save_Data.isGetData = false;
	Save_Data.isParseData = false;
	Save_Data.isUsefull = false;
}

void clrGpsRxBuffer(void)
{
	memset(gpsRxBuffer, 0, gpsRxBufferLength);      //清空
	ii = 0;
}

// 然后根据$GNRMC 格式的定义，将其中的：定位状态、经纬度、东西半球、南北半球信息提取出来。
void parseGpsBuffer()
{
	char *subString;
	char *subStringNext;
	if (Save_Data.isGetData)
	{
		Save_Data.isGetData = false;

		for (int i = 0 ; i <= 6 ; i++)
		{
			if (i == 0)
			{
				if ((subString = strstr(Save_Data.GPS_Buffer, ",")) == NULL)
        {
					// return; //解析错误
        }
			}
			else
			{
				subString++;
				if ((subStringNext = strstr(subString, ",")) != NULL)
				{
					char usefullBuffer[2]; 
					switch(i)
					{
						case 1:memcpy(Save_Data.UTCTime, subString, subStringNext - subString);break;	//获取UTC时间
						case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break;	//获取UTC时间
						case 3:memcpy(Save_Data.latitude, subString, subStringNext - subString);break;	//获取纬度信息
						case 4:memcpy(Save_Data.N_S, subString, subStringNext - subString);break;	//获取N/S
						case 5:memcpy(Save_Data.longitude, subString, subStringNext - subString);break;	//获取纬度信息
						case 6:memcpy(Save_Data.E_W, subString, subStringNext - subString);break;	//获取E/W

						default:break;
					}

					subString = subStringNext;
					Save_Data.isParseData = true;
					if(usefullBuffer[0] == 'A')
						Save_Data.isUsefull = true;
					else if(usefullBuffer[0] == 'V')
						Save_Data.isUsefull = false;
				}
				else
				{
					// return;	//解析错误
				}
			}
		}
	}
}

// 通过串口读取 定位模组输出的定位信息，并根据帧头作为判断条件，只保存 $GNRMC 格式的数据帧
void gpsRead() {
	while (GPS_BDS_Serial.available())
	{
		gpsRxBuffer[ii++] = GPS_BDS_Serial.read();
		if (ii == gpsRxBufferLength)
      clrGpsRxBuffer();
	}

	char* GPS_BufferHead;
	char* GPS_BufferTail;
	if ((GPS_BufferHead = strstr(gpsRxBuffer, "$GPRMC,")) != NULL || (GPS_BufferHead = strstr(gpsRxBuffer, "$GNRMC,")) != NULL )
	{
		if (((GPS_BufferTail = strstr(GPS_BufferHead, "\r\n")) != NULL) && (GPS_BufferTail > GPS_BufferHead))
		{
			memcpy(Save_Data.GPS_Buffer, GPS_BufferHead, GPS_BufferTail - GPS_BufferHead);
			Save_Data.isGetData = true;

			clrGpsRxBuffer();
		}
	}
}

