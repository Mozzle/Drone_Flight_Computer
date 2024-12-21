
// Receiver			ESP32
// Channel #	      Pin #
#define CH1			34		// ROLL LEFT RIGHT
#define CH2			35		// PITCH UP DOWN
#define CH3			32		// THROTTLE
#define CH4			33		// YAW LEFT RIGHT
//#define CH5			25		
//#define CH6			26
#define CH7			27		// MODE SELECT
#define CH8			13		// MODE SELECT
#define CH9			4		// MODE SELECT
//#define CH10		16

struct ReceiverData {
  int ch1Value;
  int ch2Value;
  uint16_t ch3Value;
  int ch4Value;
  //int ch5Value;
  //int ch6Value;
  bool ch7Value;
  bool ch8Value;
  int ch9Value;
  //bool ch10Value;
};