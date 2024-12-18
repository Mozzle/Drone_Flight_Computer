
// Receiver			ESP32
// Channel #	      Pin #
#define CH1			34
#define CH2			35
#define CH3			32
#define CH4			33
#define CH5			25
#define CH6			26
#define CH7			27
#define CH8			13
#define CH9			4
#define CH10		16

struct ReceiverData {
  int ch1Value;
  int ch2Value;
  int ch3Value;
  int ch4Value;
  int ch5Value;
  int ch6Value;
  bool ch7Value;
  bool ch8Value;
  int ch9Value;
  bool ch10Value;
};