Android App Readme
***************************************************
Sourse file are located at: \Capacitance Measurement\Source\src\com\dsi\ant\capacitance\measurement


Xml file are loacted at: Capacitance Measurement\Source\res\layout
****************************************************


UI structure
************************************************
Push button
************************************************
Master on
    Create Ant channel, start master broadcasting
Slave off
    No function attached
Delete data
    Delete all data saved on local drive, also wiped out the list    	on screen
Data output
    Ouput data file based on user's choice
Set	
    Set paraemter to master broadcast
***********************************************
Listview
*************************************************
Device List
    contain the list of device have been received by master.
    NOTE: when a device is disconnected, it will not be wiped 	from the list
Data list
    contains the list of data for selected device in device list.
    NOTE: This list will update only when whole data package(2 or 	3)was reviced by master
*********************************************************
Toggle button
********************************************************
Temp ON: enable RTD unit on chip
Temp OFF: disable RTD unit on chip
NOTE: enable the RTD will add one more ANT broadcast data to the data package.

