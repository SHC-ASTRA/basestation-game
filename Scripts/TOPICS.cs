global using static Globals.TOPICS;
global using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace Globals
{
    record struct TOPICS
    {
        public static class CONTROL
        {
            public const string
            CORESTATE = "/core/control/state",
            CORETWIST = "/core/control/cmd_vel",
            PTZCONTROL = "/ptz/control",

            ARMCONTROLMANUAL = "/arm/control/manual",
            ARMCONTROLSTATE = "/arm/control/state",

            VACUUMCONTROL = "/bio/control/vacuum",
            CITADELCONTROL = "/bio/control/citadel",
            TUBECONTROL = "/bio/control/test_tube",
            LANCECONTROL = "/bio/control/lance",
            LIBSTOPIC = "/bio/libs/fire";
        }


        public static class FEEDBACK
        {
            public const string
            COREMAIN = "/core/feedback/main",
            COREGPS = "/core/feedback/gps/fix",
            COREIMU = "/core/feedback/imu/data",
            BATTERY = "/core/feedback/battery",
            F_PTZ = "/ptz/feedback",

            ARMMAIN = "/arm/feedback/main",

            AUTOMACULA = "/auto/macula",

            BIOMAIN = "/bio/feedback/main";
        }
    }
}
