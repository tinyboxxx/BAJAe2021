  cli.setOnError(errorCallback); // Set error Callback
  Command turnoffwifi_Cmd = cli.addCmd(         "wifioff", turnoffwifi);
  Command turnonwifi_Cmd = cli.addCmd(          "wifion", turnonwifi);
  Command printip_cmd = cli.addCmd(             "printip", printip);
  Command loralowpower_Cmd = cli.addCmd(        "loralowpower", loralowpower);
  Command lorahighpower_Cmd = cli.addCmd(       "lorahighpower", lorahighpower);
  Command Scanner_Cmd = cli.addCmd(             "i2cscan", i2cscan);
  Command reboot_Cmd = cli.addCmd(              "reboot", reboot);
  Command printGpsTime_Cmd = cli.addCmd(        "gpstime", cmd_gpstime);
  Command settime_cmd = cli.addSingleArgCmd(    "settime", settime);
  Command sendtele = cli.addSingleArgCmd(       "teleon", cmd_sendtele);
  Command telepoff = cli.addSingleArgCmd(       "teleoff", cmd_teleoff);
  Command fpson = cli.addCmd(                   "fpson", cmd_fpson);
  Command fpsoff = cli.addCmd(                  "fpsoff", cmd_fpsoff);
  Command fps_calc_on = cli.addCmd(             "calcpson", cmd_calcpson);
  Command fps_calc_off = cli.addCmd(            "calcpsoff", cmd_calcpsoff);
  Command setfps_tick = cli.addSingleArgCmd(    "setfps", cmd_setfps);