package org.firstinspires.ftc.teamcode;

import android.content.Context;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.List;
import java.util.ArrayList;

public class AutoConfig
{
  Context context;
  OpMode opMode;

  public static int LOCATION_ITEMS = 6;
  public static int NON_LOCATION_ITEMS = 3;
  public static int MENU_ITEMS = LOCATION_ITEMS + NON_LOCATION_ITEMS;

  public class Param {
    public boolean redAlliance = false;
    public int delayInSec = 0;
    public int options = 0;
    public List<AutoMenuItem> menuItems = new ArrayList<>(LOCATION_ITEMS);
  }

  public int currentMenuIndex;
  public Param autoOptions;

  // variables used during the configuration process
  AutoMenuItem currentMenuItem;
  boolean prev;
  boolean x1;
  boolean b1;
  boolean next;
  boolean lastPrev;
  boolean lastX1;
  boolean lastB1;
  boolean lastNext;
  private String configFileName="GFORCE.txt";

  public AutoConfig() {
    autoOptions = new Param();

    // Setup the menu file
    for (int item = 0; item < LOCATION_ITEMS; item++)
    {
      if (item == 0)
        autoOptions.menuItems.add(new AutoMenuItem("Start", true));
      else
        autoOptions.menuItems.add(new AutoMenuItem("Loc " + item, false));
    }
  }

  public void init(Context context, OpMode opMode) {

    this.context = context;
    this.opMode = opMode;

    // Get the current auto configuration
    currentMenuIndex = 0;
    readConfig();

    // setup initial toggle memory states for buttons used
    lastPrev =false;
    lastX1 =false;
    lastB1 =false;
    lastNext =false;
  }



  public void init_loop() {

    // read the gamepad state
    prev = opMode.gamepad1.dpad_up;
    x1 = opMode.gamepad1.dpad_left;
    b1 = opMode.gamepad1.dpad_right;
    next = opMode.gamepad1.dpad_down;

    // Remember that there are two MORE menu items than there are FieldLocation items.
    //  FULL Menu Items go from 0 to N
    //  FieldLocation Menu Items go from 2 to N
    //  Locations ListArray Index goes from 0 to N-2

    // checking to see if we are switching to the next menu item.
    if (next && !lastNext) {
      // move to next menu item
      currentMenuIndex = (currentMenuIndex + 1 ) % MENU_ITEMS;
    }
    // checking to see if we are switching to the prev menu item.
    else if (prev && !lastPrev) {
      // move to prev menu item
      currentMenuIndex = (currentMenuIndex + MENU_ITEMS - 1 ) % MENU_ITEMS;
    }
    // checking if we are moving to the next menu item.
    else if (b1 && !lastB1) {
      // select next option
      if (currentMenuIndex == 0) {
        autoOptions.redAlliance = !autoOptions.redAlliance;
      }
      else if (currentMenuIndex == 1) {
        autoOptions.delayInSec++;
      }
      else if (currentMenuIndex == 2) {
        autoOptions.options++;
      }
      else {
        autoOptions.menuItems.get(currentMenuIndex - NON_LOCATION_ITEMS).next();
      }
      saveConfig();
    }
    // checking to see if we are moving to prev menu item
    else if (x1 && !lastX1) {
          // select prev option
      if (currentMenuIndex == 0) {
        autoOptions.redAlliance = !autoOptions.redAlliance;
      }
      else if (currentMenuIndex == 1) {
        if (autoOptions.delayInSec > 0)
          autoOptions.delayInSec--;
      }
      else if (currentMenuIndex == 2) {
        if (autoOptions.options > 0)
          autoOptions.options--;
      }
      else {
        autoOptions.menuItems.get(currentMenuIndex - NON_LOCATION_ITEMS).prev();
      }
      saveConfig();
    }

    updateMenu();

    // update toggle memory for next call
    lastPrev = prev;
    lastX1 = x1;
    lastB1 = b1;
    lastNext  = next;
    opMode.telemetry.update();
  }


  public void saveConfig() {
    try {
      OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput(configFileName, Context.MODE_PRIVATE));

      // write each configuration parameter as a string on its own line
      outputStreamWriter.write(Boolean.toString(autoOptions.redAlliance)  + "\n");
      outputStreamWriter.write(Integer.toString(autoOptions.delayInSec)   + "\n");
      outputStreamWriter.write(Integer.toString(autoOptions.options)      + "\n");
      for (int item = 0; item < LOCATION_ITEMS; item++ )
        outputStreamWriter.write(autoOptions.menuItems.get(item).getSelection() + "\n");

      outputStreamWriter.close();
    }
    catch (IOException e) {
      opMode.telemetry.addData("Exception", "Auto Settings file write failed: " + e.toString());
    }
  }

  public void readConfig() {
    // read configuration data from file
    try
    {
      InputStream inputStream = context.openFileInput(configFileName);

      if (inputStream != null)
      {
        InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
        BufferedReader bufferedReader = new BufferedReader(inputStreamReader);

        autoOptions.redAlliance = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.delayInSec  = Integer.valueOf(bufferedReader.readLine());
        autoOptions.options     = Integer.valueOf(bufferedReader.readLine());

        for (int item = 0; item < LOCATION_ITEMS; item++)
        {
          String option = bufferedReader.readLine();
          for (FieldLocation loc : FieldLocation.values())
          {
            if (loc.name().equals(option))
            {
              autoOptions.menuItems.get(item).setSelection(loc);
            }
          }
        }
        inputStream.close();
      }
    } catch (Exception e)
    {
      opMode.telemetry.addData("Exception", "Auto settings file does not exist: " + e.toString());
    }
  }

  public void updateMenu ()
  {
    opMode.telemetry.clearAll();
    opMode.telemetry.addData((currentMenuIndex == 0) ? "0 > Color"   : "0   Color", autoOptions.redAlliance ? "RED" : "BLUE");
    opMode.telemetry.addData((currentMenuIndex == 1) ? "1 > Delay"   : "1   Delay", autoOptions.delayInSec);
    opMode.telemetry.addData((currentMenuIndex == 2) ? "2 > Option"  : "3   Option", autoOptions.options);
    for (int item = NON_LOCATION_ITEMS; item < MENU_ITEMS; item++)
    {
      String itemName;
      if (currentMenuIndex == item){
        itemName = item + " > " + autoOptions.menuItems.get(item - NON_LOCATION_ITEMS).getName();
      } else {
        itemName = item + "   " + autoOptions.menuItems.get(item - NON_LOCATION_ITEMS).getName();
      }

      opMode.telemetry.addData(itemName , autoOptions.menuItems.get(item - NON_LOCATION_ITEMS).getSelection().toString());
    }
    opMode.telemetry.update();
  }
}
