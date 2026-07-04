// 2026 RCJ Rescue Maze SuperTeam - Robot B (chef) wireless comms.
//
// The Giga R1 WiFi hosts its own access point; Robot A joins the network and
// sends the active order over UDP during the order handoff.
//
// Protocol (ASCII, one message per UDP packet, no trailing newline needed):
//   A -> B : "ORDER:<sum>"   sum in [-2, 2]  (a bare "<sum>" is also accepted)
//   B -> A : "ACK:<sum>"     order accepted and stored
//   B -> A : "ERR"           malformed packet, resend
// UDP has no delivery guarantee: Robot A must resend ORDER (e.g. every 500 ms)
// until it hears the matching ACK. Duplicate ORDERs are simply re-ACKed.
//
// Robot A connection settings (same names in Robot A's code):
//   KBSsid "SuperTeamsBVTandTHS", KBPass "SP12345678", UDP to KBIp 192.168.4.1 : KBPort 4210

#include <WiFi.h>
#include <WiFiUdp.h>

// ---- access point / UDP configuration ----
// Names and values shared with Robot A's code (KB = "Kitchen / Robot B"):
// Robot A connects to KBSsid/KBPass and sends UDP to KBIp:KBPort.
const char KBSsid[] = "SuperTeamsBVTandTHS";
const char KBPass[] = "SP12345678";       // WPA2 requires >= 8 chars
const int  SUPERTEAM_CHANNEL = 1;         // pick a quiet channel at the venue (1/6/11)
const unsigned int KBPort = 4210;         // UDP listen port
const IPAddress KBIp(192, 168, 4, 1);

WiFiUDP superteamUdp;
bool superteamCommsUp = false; // AP + UDP started successfully

// last peer that sent us a valid packet (Robot A), for ACKs and outbound sends
IPAddress robotA_ip;
unsigned int robotA_port = 0;
volatile bool robotA_known = false;

// order state: written by the comms thread, consumed by the main loop
volatile bool orderPending = false; // a new order arrived and hasn't been taken yet
volatile int  orderSumValue = 0;    // SUM of the latest order, -2..2

// ingredient bitmask (matches the five single-colour ingredient targets)
#define ING_RED    0x01
#define ING_YELLOW 0x02
#define ING_GREEN  0x04
#define ING_BLUE   0x08
#define ING_BLACK  0x10

// comms gets its own thread like cameraThread/pauseThread; 8 KB stack because
// the WiFi stack runs deeper call chains than the default 4 KB allows for.
rtos::Thread superteamThread(osPriorityNormal, 8192);

// SUM -> required ingredient set (2026 SuperTeam menu):
// -2 Tteokbokki  R+Y+K | -1 Sujebi Y+G+B | 0 Bibimbap R+Y+G
// +1 Doenjang jjigae G+B+K | +2 Galbitang R+B+K
uint8_t ingredientsForSum(int sum) {
  switch (sum) {
    case -2: return ING_RED    | ING_YELLOW | ING_BLACK;
    case -1: return ING_YELLOW | ING_GREEN  | ING_BLUE;
    case  0: return ING_RED    | ING_YELLOW | ING_GREEN;
    case  1: return ING_GREEN  | ING_BLUE   | ING_BLACK;
    case  2: return ING_RED    | ING_BLUE   | ING_BLACK;
    default: return 0;
  }
}

// accepts "ORDER:<n>" or a bare "<n>", n in [-2,2]
static bool parseOrderPacket(const char* buf, int& sumOut) {
  const char* p = buf;
  if (strncmp(p, "ORDER:", 6) == 0) p += 6;
  char* end;
  long v = strtol(p, &end, 10);
  if (end == p) return false;      // no digits at all
  if (v < -2 || v > 2) return false;
  sumOut = (int)v;
  return true;
}

void superteamCommsTask() {
  char buf[64];
  while (true) {
    int packetSize = superteamUdp.parsePacket();
    if (packetSize > 0) {
      int len = superteamUdp.read(buf, sizeof(buf) - 1);
      if (len < 0) len = 0;
      buf[len] = '\0';

      IPAddress fromIp = superteamUdp.remoteIP();
      unsigned int fromPort = superteamUdp.remotePort();

      int sum;
      if (parseOrderPacket(buf, sum)) {
        robotA_ip = fromIp;
        robotA_port = fromPort;
        robotA_known = true;
        orderSumValue = sum;
        orderPending = true;

        char ack[16];
        snprintf(ack, sizeof(ack), "ACK:%d", sum);
        superteamUdp.beginPacket(fromIp, fromPort);
        superteamUdp.write((const uint8_t*)ack, strlen(ack));
        superteamUdp.endPacket();

        Serial.print("superteam: order received, SUM=");
        Serial.println(sum);
      } else {
        superteamUdp.beginPacket(fromIp, fromPort);
        superteamUdp.write((const uint8_t*)"ERR", 3);
        superteamUdp.endPacket();
        Serial.print("superteam: bad packet: ");
        Serial.println(buf);
      }
    }
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(20));
  }
}

// Call once from setup(). Non-fatal on failure: the robot still runs the maze,
// it just can't hear Robot A (superteamCommsUp stays false).
void initSuperteamComms() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("superteam: WiFi module not found");
    lcdPrint("wifi: no module");
    return;
  }

  WiFi.config(KBIp); // fixed AP address so Robot A can hardcode it
  int status = WiFi.beginAP(KBSsid, KBPass, SUPERTEAM_CHANNEL);
  if (status != WL_AP_LISTENING) {
    Serial.print("superteam: beginAP failed, status=");
    Serial.println(status);
    lcdPrint("wifi: AP failed");
    return;
  }

  superteamUdp.begin(KBPort);
  superteamCommsUp = true;
  superteamThread.start(superteamCommsTask);

  Serial.print("superteam: AP up, listening on ");
  Serial.print(KBIp);
  Serial.print(":");
  Serial.println(KBPort);
  lcdPrint("wifi: AP up");
}

// ---- accessors for the main loop ----

bool superteamOrderAvailable() {
  return orderPending;
}

// returns the SUM of the pending order and clears the pending flag
int superteamTakeOrder() {
  orderPending = false;
  return orderSumValue;
}

// send a message back to Robot A (e.g. "DISH_READY"). Only works after A has
// sent us at least one valid packet, since that's how we learn its address.
bool superteamSend(const char* msg) {
  if (!superteamCommsUp || !robotA_known) return false;
  superteamUdp.beginPacket(robotA_ip, robotA_port);
  superteamUdp.write((const uint8_t*)msg, strlen(msg));
  superteamUdp.endPacket();
  return true;
}
