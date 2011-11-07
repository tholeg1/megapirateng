// Support for Remzibi OSD
// Based on file OSD_RemzibiV2.pde from WWW.DIYDRONES.COM

#if OSD_PROTOCOL == OSD_PROTOCOL_REMZIBI

#define SendSer		Serial1.print
#define SendSerln	Serial1.println

/*
byte oldOSDSwitchPosition = 1;

void read_osd_switch()
{
	byte osdSwitchPosition = readOSDSwitch();
	if (oldOSDSwitchPosition != osdSwitchPosition){
		
		switch(osdSwitchPosition)
		{
			case 1: // First position
			set_osd_mode(1);
			break;

			case 2:
			set_osd_mode(0);
			break;
		}

		oldOSDSwitchPosition = osdSwitchPosition;
	}
}

byte readOSDSwitch(void){
  	int osdPulseWidth = APM_RC.InputCh(OSD_MODE_CHANNEL - 1);
	if (osdPulseWidth >= 1450)  return 2;	// Off
	return 1;
}
*/
void osd_init(){
	Serial1.begin(57600);
	set_osd_mode(1);
}

void osd_heartbeat_10Hz(void)
{
        double nMult=0;
        int nMeters=0; //Change this to 1 for meters, 0 for feet

        if (nMeters==1) {
          nMult=1;
        } else {
          nMult=3.2808399;
        }

	SendSer("$A,");
	SendSer((float)current_loc.lat/10000000,5); //Latitude
	SendSer(",");
	SendSer((float)current_loc.lng/10000000,5); //Longitude
	SendSer(",");
	SendSer(g_gps->num_sats,DEC); //Satellite Count
	SendSer(",");
  SendSer((int)(wp_distance*nMult)); //Distance to Waypoint
	SendSer(",");
	SendSer(current_loc.alt*nMult/10,DEC); //Altitude
	SendSer(",");
	SendSer(g_gps->ground_speed/100,DEC); //Ground Speed
	SendSer(",");
  SendSer(get_bearing(&current_loc,&home)/100,DEC);
	SendSer(",");
	SendSer(",");
	//SendSer(pitch_sensor/100,DEC); //Pitch
	//SendSer(",");
	//SendSer((roll_sensor/100) * -1,DEC); //Roll
	//SendSer(",");
	SendSer(g_gps->date,DEC); //Date
  //SendSer(""); //Date
	SendSer(",");
	SendSer(g_gps->time,DEC); //Time
	SendSerln();

	#if BATTERY_EVENT == ENABLED
    if(battery_voltage < LOW_VOLTAGE)
    {
        SendSer("$M");
        SendSer("97");
        SendSer("02");
        SendSer("D7");
        SendSer("00");
        SendSer(battery_voltage,1);
        SendSerln();
        
        SendSer("$M");
        SendSer("87"); // Hex Column (01h - 1Eh) for small fonts add 80h (81h - 9Eh) 
        SendSer("0B"); // Hex Row (01h - 0Dh for NTSC, 01h - 10h for PAL) 
        SendSer("D7"); // Hex address of First Character
        SendSer("D7"); // Hex address of Last Character
        SendSer("LOW VOLTAGE ALERT");
        SendSerln();                  
    }
    else
    {
        SendSer("$M");
        SendSer("97");
        SendSer("02");
        SendSer("D5");
        SendSer("00");
        SendSer(battery_voltage,1);
        SendSerln();         
    }
  #endif
  
  SendSer("$M");
  SendSer("82"); // Hex Column (01h - 1Eh) for small fonts add 80h (81h - 9Eh) 
  SendSer("0C"); // Hex Row (01h - 0Dh for NTSC, 01h - 10h for PAL) 
  SendSer("E9"); // Hex address of First Character
  SendSer("00"); // Hex address of Last Character

	switch (control_mode){
		case STABILIZE:
			SendSer("STABILIZE       ");
      break;
		case ACRO:
			SendSer("ACRO            ");
      break;
		case ALT_HOLD:
			SendSer("ALT_HOLD        ");
      break;
		case AUTO:
			SendSer("WP");
      SendSer((int)(wp_distance*nMult));
      SendSer("   ");
      break;
		case GUIDED:
			SendSer("GUIDED          ");
      break;
		case LOITER:
			SendSer("LOITER          ");
      break;
		case RTL:
			SendSer("RTL:");
      SendSer((int)(wp_distance*nMult));
      SendSer("   ");
      break;
		case CIRCLE:
			SendSer("CIRCLE          ");
      break;
		case POSITION:
			SendSer("POSITION        ");
      break;
	}
  SendSerln("");
}

void osd_heartbeat_50Hz(void)
{
	SendSer("$I,");
	SendSer(dcm.roll_sensor/100,DEC); //Roll
	SendSer(",");
	SendSer(dcm.pitch_sensor/100,DEC); //Pitch
	SendSer(",");
	SendSerln();
} 

void osd_init_home(void)
{
	SendSer("$SH");
	SendSerln();
	SendSer("$CLS");
	SendSerln(); 
}

void set_osd_mode(int mode){
		switch(mode)
		{
			case 1: // On
				SendSerln("$CLS");
        SendSerln("$L1");
			break;

			case 0: // Off
				SendSerln("$L0");
        SendSerln("$CLS");
			break;
		}
}

#endif
