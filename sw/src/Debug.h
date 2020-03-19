
#ifndef DEBUG_H
#define DEBUG_H

#ifdef __cplusplus
	extern "C" {
#endif

#define DEBUG

//when debbuging on Arduino
//#define DEBUG_MSG(msg) Serial.Print(msg);Serial.Print(" on ");Serial.Print(__func__);Serial.Print(":");Serial.Print(__FILE__);Serial.Print(":");Serial.Print(__LINE__);

//when debbuging on PC
#define DEBUG_MSG(msg) printf("%s on %s:%s:%d", msg, __func__, __FILE__, __LINE__);

#ifdef __cplusplus
	}
#endif

#endif
