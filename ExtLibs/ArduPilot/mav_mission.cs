﻿using System;
using System.Collections.Generic;
using System.Reflection;
using log4net;
using MissionPlanner.Utilities;

namespace MissionPlanner.ArduPilot
{
    public class mav_mission
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        public static void upload(MAVLinkInterface port, MAVLink.MAV_MISSION_TYPE type, List<Locationwp> commandlist, MAVLink.MAV_FRAME frame = MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT)
        {
            try
            {
                if (!port.BaseStream.IsOpen)
                {
                    throw new Exception("Please connect first!");
                }

                int a;

                bool use_int = (port.MAV.cs.capabilities & (uint)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_INT) > 0;

                port.setWPTotal((ushort)commandlist.Count);

                // process commandlist to the mav
                for (a = 0; a < commandlist.Count; a++)
                {
                    var temp = commandlist[a];

                    // handle current wp upload number
                    int uploadwpno = a;

                    // try send the wp
                    MAVLink.MAV_MISSION_RESULT ans = port.setWP(temp, (ushort)(uploadwpno), (MAVLink.MAV_FRAME)temp.frame, 0, 1, use_int, type);

                    // we timed out while uploading wps/ command wasnt replaced/ command wasnt added
                    if (ans == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_ERROR)
                    {
                        // resend for partial upload
                        port.setWPPartialUpdate((ushort)(uploadwpno), (ushort)commandlist.Count, type);
                        // reupload this point.
                        ans = port.setWP(temp, (ushort)(uploadwpno), frame, 0, 1, use_int, type);
                    }

                    if (ans == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE)
                    {
                        throw new Exception("Upload failed, please reduce the number of wp's");
                    }
                    if (ans == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_INVALID)
                    {
                        throw new Exception(
                            "Upload failed, mission was rejected by the Mav,\n item had a bad option wp# " + a + " " +
                            ans);
                    }
                    if (ans == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE)
                    {
                        // invalid sequence can only occur if we failed to see a response from the apm when we sent the request.
                        // or there is io lag and we send 2 mission_items and get 2 responces, one valid, one a ack of the second send

                        // the ans is received via mission_ack, so we dont know for certain what our current request is for. as we may have lost the mission_request

                        // get requested wp no - 1;
                        try
                        {
                            a = port.getRequestedWPNo() - 1;
                        }
                        catch
                        {
                            // resend for partial upload
                            port.setWPPartialUpdate((ushort)(uploadwpno), (ushort)commandlist.Count, type);
                            // reupload this point.
                        }

                        continue;
                    }
                    if (ans != MAVLink.MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED)
                    {
                        throw new Exception("Upload wps failed " + Enum.Parse(typeof(MAVLink.MAV_CMD), temp.id.ToString()) +
                                            " " + Enum.Parse(typeof(MAVLink.MAV_MISSION_RESULT), ans.ToString()));
                    }
                }

                port.setWPACK();

            }
            catch (Exception ex)
            {
                log.Error(ex);
                throw;
            }
        }
    }
}