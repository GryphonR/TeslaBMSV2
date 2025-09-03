#include "NextionDisplay.h"
#include "globals.h"
#include "pinouts.h"

void dashupdate()
{
    Serial2.write("stat.txt=");
    Serial2.write(0x22);
    if (settings.ESSmode == 1)
    {
        switch (bmsstatus)
        {
        case (BMS_STATUS_BOOT):
            Serial2.print(" Active ");
            break;
        case (BMS_STATUS_ERROR):
            Serial2.print(" Error ");
            break;
        }
    }
    else
    {
        switch (bmsstatus)
        {
        case (BMS_STATUS_BOOT):
            Serial2.print(" Boot ");
            break;

        case (BMS_STATUS_READY):
            Serial2.print(" Ready ");
            break;

        case (BMS_STATUS_PRECHARGE):
            Serial2.print(" Precharge ");
            break;

        case (BMS_STATUS_DRIVE):
            Serial2.print(" Drive ");
            break;

        case (BMS_STATUS_CHARGE):
            Serial2.print(" Charge ");
            break;

        case (BMS_STATUS_ERROR):
            Serial2.print(" Error ");
            break;
        }
    }
    Serial2.write(0x22);
    Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("soc.val=");
    Serial2.print(SOC);
    Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("soc1.val=");
    Serial2.print(SOC);
    Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("current.val=");
    Serial2.print(currentact / 100, 0);
    Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("temp.val=");
    Serial2.print(bms.getAvgTemperature(), 0);
    Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("templow.val=");
    Serial2.print(bms.getLowTemperature(), 0);
    Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("temphigh.val=");
    Serial2.print(bms.getHighTemperature(), 0);
    Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("volt.val=");
    Serial2.print(bms.getPackVoltage() * 10, 0);
    Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("lowcell.val=");
    Serial2.print(bms.getLowCellVolt() * 1000, 0);
    Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("highcell.val=");
    Serial2.print(bms.getHighCellVolt() * 1000, 0);
    Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("firm.val=");
    Serial2.print(firmver);
    Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("celldelta.val=");
    Serial2.print((bms.getHighCellVolt() - bms.getLowCellVolt()) * 1000, 0);
    Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("cellbal.val=");
    Serial2.print(bms.getBalancing());
    Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
}
